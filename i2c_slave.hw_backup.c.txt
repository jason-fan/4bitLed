/**
 * i2c_slave.c - STM8S I2C Slave Driver Implementation (Object-Oriented)
 *
 * Hardware notes (STM8S103F3P6):
 *   - I2C pins: PB4 = SDA, PB5 = SCL (hardware I2C, open-drain)
 *   - Slave address is 7-bit, programmed into OARL = addr7 << 1
 *   - Bus speed is provided by the master; CCR is not programmed in
 *     slave mode (kept at reset value, documented for clarity)
 *   - The ISR body runs in the I2C vector (IRQ 19) defined in main.c;
 *     call i2c.irq(&i2c) from that vector stub.
 *
 * Transaction state machine:
 *   IDLE     - waiting for address match
 *   RX_DATA  - master is writing bytes to us (accumulate in rx_buf)
 *   TX_DATA  - master is reading bytes from us (stream tx_buf)
 *
 * A full 4-byte write payload (display digits) triggers onWrite at the
 * STOP condition, so the application sees one consistent update per
 * I2C transaction. Master-read returns the current tx_buf (initialised
 * to the displayed digits), followed by 0x00 padding until NACK.
 */

#include <stdint.h>
#include "i2c_slave.h"

/* ========== Internal state codes ========== */
#define I2CS_STATE_IDLE      0
#define I2CS_STATE_RX_DATA   1
#define I2CS_STATE_TX_DATA   2

/* ========== I2C Slave Method Implementations ========== */

static void i2cs_init(I2CS_t *self)
{
    /* Peripheral clock: bit5 = I2CEN. uart_init() already wrote 0xFF,
     * but keep the explicit gate so this module is self-contained. */
    CLK_PCKENR1 |= CLK_PCKENR1_I2CEN;

    /* PB4 = SDA, PB5 = SCL: open-drain outputs (CR1=0). The STM8S I2C
     * alternate function needs DDR=1 to reach the pin; external 4.7k
     * pull-ups are required on both lines. */
    GPIO_PB_DDR |= (I2C_SDA_PIN | I2C_SCL_PIN);
    GPIO_PB_CR1 &= (uint8_t)~(I2C_SDA_PIN | I2C_SCL_PIN); /* open-drain */
    GPIO_PB_CR2 |= (I2C_SDA_PIN | I2C_SCL_PIN);           /* 10MHz slew */

    /* Software-reset the I2C peripheral first (clean state). */
    I2C_CR1 |= I2C_CR1_SWRST;
    I2C_CR1 &= (uint8_t)~I2C_CR1_SWRST;

    /* Input clock frequency = 16MHz (HSI), FREQ[5:0] = 0x10. */
    I2C_FREQR = 0x10;

    /* 7-bit slave address: OARL = addr7 << 1, OARH bit7 (ADDMODE) = 0.
     * Example: 0x50 -> OARL = 0xA0. */
    I2C_OARH = 0x00;                        /* 7-bit mode */
    I2C_OARL = (uint8_t)(self->addr7 << 1);

    /* No general call, no SMBus alert. */
    I2C_CR1 &= (uint8_t)~(I2C_CR1_ENGC);

    /* Enable event + buffer + error interrupts. */
    I2C_CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);

    /* Enable error interrupt sources (bus error / arbitration lost /
     * NACK / overrun), so AF (master NACK at end of read) is serviced. */
    I2C_ITCR |= (I2C_ITCR_BERRIE | I2C_ITCR_ARLOIE |
                 I2C_ITCR_AFIE | I2C_ITCR_OVRIE);

    /* Enable the peripheral and slave ACK. */
    I2C_CR1 |= (I2C_CR1_PE | I2C_CR1_ACK);

    /* Clear residual flags by reading SR1/SR3. */
    (void)I2C_SR1;
    (void)I2C_SR2;
    (void)I2C_SR3;

    /* Reset transaction state. */
    self->state        = I2CS_STATE_IDLE;
    self->rx_len       = 0;
    self->tx_pos       = 0;
    self->tx_len       = I2C_DISP_LEN;   /* master reads 4 display digits */
    self->write_pending = 0;
}

static void i2cs_irq(I2CS_t *self)
{
    uint8_t sr1 = I2C_SR1;

    /* ---- 1. Error handling (read SR3, clear via ITR) ---- */
    if (I2C_SR3 & (I2C_SR3_BERR | I2C_SR3_ARLO | I2C_SR3_AF | I2C_SR3_OVR)) {
        uint8_t err = I2C_SR3;
        if (err & I2C_SR3_AF) {
            /* Master NACK: read transfer finished (or address NACK). */
            if (self->state == I2CS_STATE_TX_DATA) {
                self->state = I2CS_STATE_IDLE;
            }
            I2C_ITR = I2C_ITR_AF;   /* write 1 clears AF */
        }
        if (err & I2C_SR3_BERR) {
            self->state = I2CS_STATE_IDLE;
            I2C_ITR = I2C_ITR_BERR;
        }
        if (err & I2C_SR3_ARLO) {
            self->state = I2CS_STATE_IDLE;
            I2C_ITR = I2C_ITR_ARLO;
        }
        if (err & I2C_SR3_OVR) {
            I2C_ITR = I2C_ITR_OVR;
        }
    }

    /* ---- 2. Address matched (EV1) ---- */
    if (sr1 & I2C_SR1_ADDR) {
        (void)I2C_SR2;              /* read SR2 clears ADDR */
        self->rx_len = 0;
        self->tx_pos = 0;

        if (I2C_SR2 & I2C_SR2_TX) {
            /* Master reads: we transmit. Give the app a chance to
             * refresh tx_buf, then start with the first byte. */
            self->state = I2CS_STATE_TX_DATA;
            if (self->onRequest) {
                self->onRequest(self);
            }
            if (self->tx_len == 0) {
                self->tx_len = I2C_DISP_LEN;   /* safety default */
            }
            if (I2C_SR1 & I2C_SR1_TXE) {       /* TXE already set? */
                I2C_DR = self->tx_buf[0];
                self->tx_pos = 1;
            }
        } else {
            /* Master writes: we receive. */
            self->state = I2CS_STATE_RX_DATA;
        }
    }

    /* ---- 3. TXE: transmit register empty, master is reading ---- */
    if (sr1 & I2C_SR1_TXE) {
        if (self->state == I2CS_STATE_TX_DATA) {
            if (self->tx_pos < self->tx_len) {
                I2C_DR = self->tx_buf[self->tx_pos++];
            } else {
                I2C_DR = 0x00;      /* payload exhausted: pad until NACK */
            }
        }
    }

    /* ---- 4. RXNE: receive register not empty, master is writing ---- */
    if (sr1 & I2C_SR1_RXNE) {
        uint8_t b = I2C_DR;         /* reading DR clears RXNE */
        if (self->state == I2CS_STATE_RX_DATA) {
            if (self->rx_len < I2C_BUF_SIZE) {
                self->rx_buf[self->rx_len++] = b;
            }
            if (self->rx_len >= I2C_DISP_LEN) {
                self->write_pending = 1;   /* full display payload seen */
            }
        }
        /* In any other state: byte consumed and discarded. */
    }

    /* ---- 5. STOPF: master sent STOP, transaction finished ---- */
    if (sr1 & I2C_SR1_STOPF) {
        self->state = I2CS_STATE_IDLE;
        if (self->write_pending && self->onWrite) {
            self->onWrite(self);    /* consistent 4-digit update */
        }
        self->write_pending = 0;
        self->rx_len = 0;
        /* Clear STOPF: read SR1 (done above), then set STOP bit in CR1
         * (in slave mode this clears the flag, does not emit STOP). */
        I2C_CR1 |= I2C_CR1_STOP;
        I2C_CR1 &= (uint8_t)~I2C_CR1_STOP;
    }
}

/* ========== I2C Slave Constructor ========== */

I2CS_t I2CS_Create(uint16_t base, uint8_t addr7)
{
    I2CS_t i2c;

    i2c.base  = base;
    i2c.addr7 = addr7;

    i2c.state         = I2CS_STATE_IDLE;
    i2c.rx_len        = 0;
    i2c.tx_pos        = 0;
    i2c.tx_len        = I2C_DISP_LEN;
    i2c.write_pending = 0;

    /* Default TX payload: ASCII "0000" — mirrors the boot-demo start
     * character '0' (main.c demo_apply() keeps it in sync while the demo
     * rolls; an I2C write replaces it with the host payload). */
    i2c.tx_buf[0] = '0';
    i2c.tx_buf[1] = '0';
    i2c.tx_buf[2] = '0';
    i2c.tx_buf[3] = '0';
    i2c.tx_buf[4] = 0;
    i2c.tx_buf[5] = 0;
    i2c.tx_buf[6] = 0;
    i2c.tx_buf[7] = 0;

    /* Bind methods. */
    i2c.init      = i2cs_init;
    i2c.irq       = i2cs_irq;

    /* No callbacks by default; app binds after Create(). */
    i2c.onWrite   = 0;
    i2c.onRequest = 0;

    return i2c;
}
