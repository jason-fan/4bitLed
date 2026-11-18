/**
 * i2c_slave.c - STM8S I2C Slave, SOFTWARE BIT-BANG (polled) implementation
 *
 * Why bit-bang?
 *   Both project3 (slave) and project4 (master) sit on STM8S103F3P6 with
 *   PB4=SCL / PB5=SDA — 与 STM8S103 硬件 I2C 外设的默认映射完全一致。 The hardware I2C block on these parts is mapped via
 *   the AFR option byte; when it is remapped to PC1/PC2 (not bonded out on
 *   TSSOP20) the block keeps PB4/PB5 as plain GPIO. The old init() did
 *   DDR=1 + CR1=0 + ODR(reset)=0, i.e. it turned both lines into open-drain
 *   outputs driving 0 -> the slave held SCL/SDA low forever and the master
 *   could only ever report TIMEOUT. This driver never touches the I2C
 *   peripheral at all.
 *
 * Pin usage (bit-bang，功能与硬件 I2C 定义一致：PB4=SCL / PB5=SDA):
 *   PB5 SDA : open-drain OUTPUT (DDR=1, CR1=0, CR2=0).
 *             ODR=1 -> released (external 4.7k pulls high); ODR=0 -> drive low.
 *   PB4 SCL : pure INPUT (DDR=0, CR1=0 = floating, external 4.7k pull-up).
 *             A slave must never drive the clock, so a stuck SCL is
 *             physically impossible with this configuration.
 *   Reading GPIO_PB_IDR always returns the real bus level, so the open-drain
 *   SDA can be sampled even while configured as an output.
 *
 * Timing model:
 *   There are no external interrupts involved. i2cs_poll() is called from the
 *   main loop as fast as possible and derives every event from SCL/SDA level
 *   changes (rising edge -> sample, falling edge -> change SDA). At 16MHz HSI
 *   one poll costs only a few instructions. The TIM1 1ms display ISR does
 *   steal some cycles, therefore the master (project4) is configured to
 *   ~25kHz SCL (BB_HALF_PERIOD = 28), which leaves ample margin.
 *
 * Transaction state machine (phases, driven purely by bus edges):
 *   IDLE      - waiting for START
 *   ADDR      - shifting in 7-bit address + R/W bit
 *   ADDR_ACK  - drive ACK low on the falling edge
 *   ADDR_ACK2 - ack clock finished: release, load first byte if master reads
 *   RX        - shifting in a data byte
 *   RX_ACK    - drive ACK low on the falling edge
 *   RX_ACK2   - ack clock finished: release, next byte
 *   TX        - driving a data byte (MSB first, changes on falling edges)
 *   TX_ACK    - sampling the master ACK (low) / NACK (high)
 *   WAIT_STOP - not addressed or NACKed: ignore everything until STOP/START
 */

#include <stdint.h>
#include "i2c_slave.h"

/* ========== 关于 MCU HOLD 的修复（已定位并根治，勿回退） ==========
 * 根因不在硬件、也不在 bit-bang 采样路径，而是 PB4/PB5 的 CR2（GPIO 外部
 * 中断使能）被置 1：向量表 int4（Port B）原为 0x000000，任何边沿都会把 PC
 * 打进空向量死锁。现已（a）挂上 PB_EXTI_IRQHandler 兜底、（b）两脚 CR2 强制
 * 清零。隔离测试开关 BB_PB5_RX_OFF 已移除。
 * ================================================================= */
/* Suppress SDCC warning 110 (unsupported #pragma) coming from the device header */
#pragma disable_warning 110

/* ========== Transaction state codes (same as the hardware version) ========== */
#define I2CS_STATE_IDLE      0
#define I2CS_STATE_RX_DATA   1
#define I2CS_STATE_TX_DATA   2

/* ========== Bit-bang phases ========== */
#define BB_PH_IDLE       0u
#define BB_PH_ADDR       1u
#define BB_PH_ADDR_ACK   2u
#define BB_PH_ADDR_ACK2  3u
#define BB_PH_RX         4u
#define BB_PH_RX_ACK     5u
#define BB_PH_RX_ACK2    6u
#define BB_PH_TX         7u
#define BB_PH_TX_ACK     8u
#define BB_PH_WAIT_STOP  9u

/* ========== Line helpers ========== */
#define SDA_HIGH()     (GPIO_PB_ODR |=  (uint8_t)I2C_SDA_PIN)
#define SDA_LOW()      (GPIO_PB_ODR &= (uint8_t)~(uint8_t)I2C_SDA_PIN)

/* Module-private state (single instance on STM8S103). */
static uint8_t bb_ph;      /* current phase */
static uint8_t bb_bit;     /* bits handled in this phase (0..8) */
static uint8_t bb_sh;      /* shift register */
static uint8_t bb_rd;      /* 1 = master reads (slave transmits) */
static uint8_t bb_pcl;     /* SCL level of the previous poll */
static uint8_t bb_pda;     /* SDA level of the previous poll */

/* ---- TEMP DIAG: bus observability counters (read by main.c) ----
 * g_bb_scl / g_bb_sda : last sampled bus levels (1 = high)
 * g_bb_edges          : number of SCL transitions seen (any edge)
 * g_bb_starts         : number of START conditions recognised
 * These tell apart "slave cannot see the bus at all" (edges stuck at 0,
 * wiring / pull-up problem) from "edges arrive but START never detected"
 * (SDA sampled stuck high) from "frame received, address mismatch". */
volatile uint8_t  g_bb_scl    = 0;
volatile uint8_t  g_bb_sda    = 0;
volatile uint16_t g_bb_edges  = 0;
volatile uint16_t g_bb_starts = 0;

/* ========== Byte transfer helpers ========== */

/* Load the next byte into the shift register. When the payload is
 * exhausted, 0x00 is sent as padding until the master answers NACK. */
static void bb_tx_load(I2CS_t *self)
{
    if (self->tx_pos < self->tx_len) {
        bb_sh = self->tx_buf[self->tx_pos++];
    } else {
        bb_sh = 0x00;
    }
}

/* Drive the MSB onto SDA and advance the shift register. Called on a
 * falling edge so the level is stable when the master samples it. */
static void bb_tx_put_bit(void)
{
    if (bb_sh & 0x80u) {
        SDA_HIGH();
    } else {
        SDA_LOW();
    }
    bb_sh = (uint8_t)(bb_sh << 1);
    bb_bit++;
}

/* START (or repeated START): resynchronise and expect a new address. */
static void bb_start(I2CS_t *self)
{
    g_bb_starts++;
    SDA_HIGH();
    bb_ph  = BB_PH_ADDR;
    bb_bit = 0;
    bb_sh  = 0;
    bb_rd  = 0;
    self->rx_len = 0;
    self->tx_pos = 0;
    self->state  = I2CS_STATE_IDLE;
}

/* STOP: the transaction is complete; hand the payload to the application. */
static void bb_stop(I2CS_t *self)
{
    SDA_HIGH();
    if (self->write_pending && self->onWrite) {
        self->onWrite(self);      /* one consistent 4-digit update */
    }
    self->write_pending = 0;
    self->rx_len        = 0;
    self->tx_pos        = 0;
    self->state         = I2CS_STATE_IDLE;
    bb_ph               = BB_PH_IDLE;
    bb_bit              = 0;
    bb_sh               = 0;
}

/* ========== I2C Slave Method Implementations ========== */

static void i2cs_init(I2CS_t *self)
{
    /* 1) Release both lines BEFORE enabling any output driver. ODR resets to
     *    0x00, so setting DDR first would immediately pull SDA/SCL low and
     *    stall the master -- exactly the failure seen with the HW version. */
    GPIO_PB_ODR |= (uint8_t)(I2C_SDA_PIN | I2C_SCL_PIN);

    /* 2) SDA open-drain output, SCL plain input (see file header). */
    GPIO_PB_CR1 &= (uint8_t)~(uint8_t)(I2C_SDA_PIN | I2C_SCL_PIN);
    GPIO_PB_DDR |=  (uint8_t)I2C_SDA_PIN;                /* PB5 SDA = 开漏输出 */
    GPIO_PB_DDR &= (uint8_t)~(uint8_t)I2C_SCL_PIN;       /* PB4 SCL = 纯输入，从机不驱动时钟 */

    /* 3) CR2 = GPIO 外部中断使能位（与 DDR 无关）。本工程中断向量表里
     *    int3 = PB 端口 = 0x000000，一旦 PB4/PB5 的 CR2 被置 1，总线或
     *    静电产生的边沿就会把 PC 打进 0x000000 -> MCU 死锁(HOLD)（已实战复现并验证）。
     *    因此两脚的 CR2 一律保持 0，绝不允许开输入中断。 */
    GPIO_PB_CR2 &= (uint8_t)~(uint8_t)(I2C_SDA_PIN | I2C_SCL_PIN);

    /* 3) The I2C peripheral is not used and its clock is left ungated: with
     *    PE=0 (reset value) the block can never fight the GPIO driver, no
     *    matter how the AFR option byte maps it. */

    /* 4) Reset the polling state machine from the real bus levels. */
    bb_ph  = BB_PH_IDLE;
    bb_bit = 0;
    bb_sh  = 0;
    bb_rd  = 0;
    bb_pcl = (GPIO_PB_IDR & I2C_SCL_PIN) ? 1u : 0u;
    bb_pda = (GPIO_PB_IDR & I2C_SDA_PIN) ? 1u : 0u;

    self->state         = I2CS_STATE_IDLE;
    self->rx_len        = 0;
    self->tx_pos        = 0;
    self->tx_len        = I2C_DISP_LEN;   /* master reads 4 display digits */
    self->write_pending = 0;
}

/* Legacy hook: the hardware I2C vector never fires any more. Kept so that
 * main.c's I2C_IRQHandler() still links against a valid symbol. */
static void i2cs_irq(I2CS_t *self)
{
    (void)self;
}

/**
 * Poll the bus. Must be called in a tight loop from main(); never call wfi,
 * otherwise the START condition and all clock edges are missed.
 *
 * Each call: read both lines, detect START/STOP (SDA moves while SCL high),
 * then handle an SCL rising edge (sample) or falling edge (drive SDA).
 */
static void i2cs_poll(I2CS_t *self)
{
    uint8_t idr = GPIO_PB_IDR;
    uint8_t scl = (idr & I2C_SCL_PIN) ? 1u : 0u;
    uint8_t sda = (idr & I2C_SDA_PIN) ? 1u : 0u;
    /* Classify this poll explicitly instead of testing chained booleans:
     *   st = (scl << 1) | bb_pcl -> 3 = high&high, 2 = rising, 1 = falling,
     *                                0 = low&low (nothing to do)
     * An explicit code stops SDCC rewriting the short-circuit chain
     * (warning 110) and documents the edge detection. */
    uint8_t st = (uint8_t)((scl << 1) | bb_pcl);

    /* TEMP DIAG: publish the levels and count real SCL transitions. */
    g_bb_scl = scl;
    g_bb_sda = sda;
    if (st == 2u || st == 1u) {
        g_bb_edges++;
    }

    /* ---------- START / repeated START / STOP ---------- */
    if (st == 3u) {
        if (bb_pda && !sda) {                 /* SDA fell while SCL high */
            bb_start(self);
            bb_pcl = scl;
            bb_pda = sda;
            return;
        }
        if (!bb_pda && sda) {                 /* SDA rose while SCL high */
            bb_stop(self);
            bb_pcl = scl;
            bb_pda = sda;
            return;
        }
    }

    if (st == 2u) {
        /* ===== SCL rising edge: the bus data is valid, sample it ===== */
        switch (bb_ph) {
        case BB_PH_ADDR:
            bb_sh = (uint8_t)((bb_sh << 1) | sda);
            if (++bb_bit >= 8u) {
                bb_bit = 0;
                bb_rd  = (uint8_t)(bb_sh & 0x01u);      /* R/W bit */
                if ((uint8_t)(bb_sh >> 1) == self->addr7) {
                    self->rx_len = 0;
                    self->tx_pos = 0;
                    bb_ph = BB_PH_ADDR_ACK;
                } else {
                    SDA_HIGH();                     /* not for us, no ACK */
                    bb_ph = BB_PH_WAIT_STOP;
                }
            }
            break;

        case BB_PH_RX:
            bb_sh = (uint8_t)((bb_sh << 1) | sda);
            if (++bb_bit >= 8u) {
                bb_bit = 0;
                if (self->rx_len < I2C_BUF_SIZE) {
                    self->rx_buf[self->rx_len++] = bb_sh;
                }
                if (self->rx_len >= I2C_DISP_LEN) {
                    self->write_pending = 1;        /* full display payload */
                }
                bb_sh = 0;
                bb_ph = BB_PH_RX_ACK;               /* ACK on next fall */
            }
            break;

        case BB_PH_TX_ACK:
            if (sda) {                              /* NACK: payload done */
                SDA_HIGH();
                bb_ph = BB_PH_WAIT_STOP;
            } else {                                /* ACK: send next byte */
                bb_bit = 0;
                bb_tx_load(self);
                bb_ph = BB_PH_TX;
            }
            break;

        default:
            /* ADDR_ACK / ADDR_ACK2 / RX_ACK / RX_ACK2 / TX: nothing to
             * sample on this edge. */
            break;
        }
    } else if (st == 1u) {
        /* ===== SCL falling edge: change SDA for the next bit ===== */
        switch (bb_ph) {
        case BB_PH_ADDR_ACK:
            SDA_LOW();                              /* ACK the address */
            bb_ph = BB_PH_ADDR_ACK2;
            break;

        case BB_PH_ADDR_ACK2:
            SDA_HIGH();                             /* ack clock finished */
            if (bb_rd) {
                self->state = I2CS_STATE_TX_DATA;
                /* NOTE: onRequest runs here and MUST be very short (fill
                 * tx_buf only) -- the master's first data clock follows
                 * immediately. Unbound in project3, so no risk. */
                if (self->onRequest) {
                    self->onRequest(self);
                }
                bb_bit = 0;
                bb_tx_load(self);
                bb_tx_put_bit();                    /* MSB straight away */
                bb_ph = BB_PH_TX;
            } else {
                self->state = I2CS_STATE_RX_DATA;
                bb_sh  = 0;
                bb_bit = 0;
                bb_ph  = BB_PH_RX;
            }
            break;

        case BB_PH_RX_ACK:
            SDA_LOW();                              /* ACK the received byte */
            bb_ph = BB_PH_RX_ACK2;
            break;

        case BB_PH_RX_ACK2:
            SDA_HIGH();
            bb_sh  = 0;
            bb_bit = 0;
            bb_ph  = BB_PH_RX;
            break;

        case BB_PH_TX:
            if (bb_bit < 8u) {
                bb_tx_put_bit();
            } else {
                SDA_HIGH();                         /* 8 bits done: let the
                                                     * master drive ACK */
                bb_ph = BB_PH_TX_ACK;
            }
            break;

        case BB_PH_WAIT_STOP:
        default:
            break;
        }
    }

    bb_pcl = scl;
    bb_pda = sda;
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

    /* Default TX payload: ASCII "0000" -- mirrors the boot-demo start
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
    i2c.poll      = i2cs_poll;

    /* No callbacks by default; app binds after Create(). */
    i2c.onWrite   = 0;
    i2c.onRequest = 0;

    return i2c;
}
