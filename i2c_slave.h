/**
 * i2c_slave.h - STM8S I2C Slave Driver (Object-Oriented)
 *
 * Provides I2C register access macros and a polymorphic I2CS_t
 * abstraction with method binding, mirroring the OO style of
 * project2 (LED_t / UART_t). Supports 7-bit slave address mode.
 *
 * Pin mapping (STM8S103F3P6 TSSOP20):
 *   PB4 = I2C_SDA, PB5 = I2C_SCL  (hardware I2C, open-drain + external pull-up)
 *
 * Protocol (register-file style slave):
 *   - Master write: slave accumulates bytes into rx_buf; when 4 bytes
 *     (display length) have been received the onWrite callback fires
 *     (also fires on STOP if any bytes arrived).
 *   - Master read : slave streams tx_buf bytes back (initialised to the
 *     current 4-digit display values), then sends 0x00 until NACK.
 *
 * Usage:
 *   #include "i2c_slave.h"
 *   I2CS_t i2c = I2CS_Create(I2C_BASE, 0x50);
 *   i2c.init(&i2c);
 *   i2c.onWrite = &my_write_handler;   // called from I2C ISR (keep it non-blocking)
 */

#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include <stdint.h>

/* ========== STM8S103 I2C Register Addresses ==========
 * Base 0x5210 (verified against STM8S103 datasheet map):
 *   +0: CR1    Control Register 1
 *   +1: CR2    Control Register 2
 *   +2: FREQR Frequency Register
 *   +3: OARL   Own Address Register Low
 *   +4: OARH   Own Address Register High
 *   +5: DR     Data Register
 *   +6: CCRL   Clock Control Register Low
 *   +7: CCRH   Clock Control Register High
 *   +8: SR1    Status Register 1 (events)
 *   +9: SR2    Status Register 2 (direction / master)
 *   +A: SR3    Status Register 3 (errors)
 *   +B: ITR    Interrupt Register (error flag clear)
 *   +C: ITCR   Interrupt Control Register
 */
#define I2C_BASE     0x5210
#define I2C_CR1     (*(volatile uint8_t *)0x5210)
#define I2C_CR2     (*(volatile uint8_t *)0x5211)
#define I2C_FREQR   (*(volatile uint8_t *)0x5212)
#define I2C_OARL    (*(volatile uint8_t *)0x5213)
#define I2C_OARH    (*(volatile uint8_t *)0x5214)
#define I2C_DR      (*(volatile uint8_t *)0x5215)
#define I2C_CCRL    (*(volatile uint8_t *)0x5216)
#define I2C_CCRH    (*(volatile uint8_t *)0x5217)
#define I2C_SR1     (*(volatile uint8_t *)0x5218)
#define I2C_SR2     (*(volatile uint8_t *)0x5219)
#define I2C_SR3     (*(volatile uint8_t *)0x521A)
#define I2C_ITR     (*(volatile uint8_t *)0x521B)
#define I2C_ITCR    (*(volatile uint8_t *)0x521C)

/* CR1 bits */
#define I2C_CR1_SWRST  0x80    /* software reset */
#define I2C_CR1_PE     0x40    /* peripheral enable */
#define I2C_CR1_ENGC   0x20    /* enable general call */
#define I2C_CR1_START  0x10    /* generate START */
#define I2C_CR1_STOP   0x08    /* generate STOP / clear STOPF in slave */
#define I2C_CR1_ACK    0x04    /* acknowledge enable */
#define I2C_CR1_POS    0x02    /* acknowledge position (2-byte reception) */

/* CR2 bits */
#define I2C_CR2_ITBUFEN 0x40   /* buffer interrupt enable (RXNE/TXE) */
#define I2C_CR2_ITEVTEN  0x20  /* event interrupt enable (ADDR/STOPF) */
#define I2C_CR2_ITERREN  0x10  /* error interrupt enable */

/* SR1 bits (events, read-only) */
#define I2C_SR1_BUSY   0x80    /* bus busy */
#define I2C_SR1_TXE    0x40    /* transmit data register empty */
#define I2C_SR1_RXNE   0x20    /* receive data register not empty */
#define I2C_SR1_STOPF  0x10    /* stop detection flag (slave) */
#define I2C_SR1_ADD10  0x08    /* 10-bit header sent */
#define I2C_SR1_BTF    0x04    /* byte transfer finished */
#define I2C_SR1_ADDR   0x02    /* address matched (slave) */
#define I2C_SR1_SB     0x01    /* start bit (master) */

/* SR2 bits */
#define I2C_SR2_MST    0x01    /* master mode */
#define I2C_SR2_TX     0x02    /* transmit direction (1 = master reads / slave transmits) */
#define I2C_SR2_GENCALL 0x04   /* general call address received */

/* SR3 bits (errors, read-only; clear via ITR) */
#define I2C_SR3_BERR   0x01    /* bus error */
#define I2C_SR3_ARLO   0x02    /* arbitration lost */
#define I2C_SR3_AF     0x04    /* acknowledge failure (NACK) */
#define I2C_SR3_OVR    0x08    /* overrun/underrun */

/* ITR bits (write 1 to clear the matching error flag) */
#define I2C_ITR_BERR   0x01
#define I2C_ITR_ARLO   0x02
#define I2C_ITR_AF     0x04
#define I2C_ITR_OVR    0x08

/* ITCR bits (error interrupt enables) */
#define I2C_ITCR_BERRIE 0x01
#define I2C_ITCR_ARLOIE 0x02
#define I2C_ITCR_AFIE   0x04
#define I2C_ITCR_OVRIE  0x08

/* ========== GPIOB Register Addresses (PB4 = SDA, PB5 = SCL) ==========
 * PB base 0x5005: ODR+0, IDR+1, DDR+2, CR1+3, CR2+4.
 * I2C lines are open-drain: DDR=1 (output), CR1=0 (open-drain),
 * CR2=1 (10MHz slew) + external pull-up resistors required.
 */
#define GPIO_PB_ODR   (*(volatile uint8_t *)0x5005)
#define GPIO_PB_IDR   (*(volatile uint8_t *)0x5006)
#define GPIO_PB_DDR   (*(volatile uint8_t *)0x5007)
#define GPIO_PB_CR1   (*(volatile uint8_t *)0x5008)
#define GPIO_PB_CR2   (*(volatile uint8_t *)0x5009)
#define I2C_SDA_PIN   0x10     /* PB4 bit mask */
#define I2C_SCL_PIN   0x20     /* PB5 bit mask */

/* ========== Clock Register Addresses ========== */
#define CLK_CKDIVR            (*(volatile uint8_t *)0x50C6)
#define CLK_PCKENR1           (*(volatile uint8_t *)0x50C7)
#define CLK_PCKENR1_I2CEN     0x20   /* bit5 = I2C clock enable (STM8S103) */

/* ========== Buffer / Protocol Constants ========== */
#define I2C_BUF_SIZE   8       /* RX/TX software buffer (power of two not required) */
#define I2C_DISP_LEN   4       /* 4-digit LED display payload length */

/* ========== I2C Slave Class Definition ========== */

typedef struct I2CS I2CS_t;

struct I2CS {
    uint16_t base;              /* I2C base address (0x5210) */
    uint8_t  addr7;             /* 7-bit slave address (e.g. 0x50) */

    /* --- Transaction state (managed by i2c_slave.c, do not touch) --- */
    uint8_t  state;             /* 0=IDLE 1=RX_DATA 2=TX_DATA */
    uint8_t  rx_buf[I2C_BUF_SIZE];  /* bytes written by master */
    uint8_t  tx_buf[I2C_BUF_SIZE];  /* bytes read by master */
    uint8_t  rx_len;            /* valid bytes in rx_buf */
    uint8_t  tx_pos;            /* next TX index */
    uint8_t  tx_len;            /* valid bytes in tx_buf */
    uint8_t  write_pending;     /* 1 = rx_buf holds a full display payload */

    /* --- Public methods --- */
    void (*init)  (I2CS_t *self);                 /* configure slave, enable IRQ */
    void (*irq)   (I2CS_t *self);                 /* I2C event/error ISR body */

    /* --- Callbacks (called from I2C ISR — keep them non-blocking) ---
     * onWrite: master wrote bytes; rx_buf[0..rx_len-1] holds the payload.
     * onRequest: master is about to read; fill tx_buf/tx_len here if desired. */
    void (*onWrite)  (I2CS_t *self);
    void (*onRequest)(I2CS_t *self);
};

/* ========== Constructor ========== */

/**
 * Create an I2C slave object bound to the I2C peripheral and a 7-bit
 * slave address. Uses static storage — safe for single-instance use
 * on STM8S103. The constructor does NOT configure the peripheral;
 * call init() first.
 *
 * @param base   I2C base address (use I2C_BASE on STM8S103)
 * @param addr7  7-bit slave address (e.g. 0x50)
 * @return       fully initialized I2CS_t object
 */
I2CS_t I2CS_Create(uint16_t base, uint8_t addr7);

#endif /* I2C_SLAVE_H */
