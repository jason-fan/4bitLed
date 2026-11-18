/**
 * STM8S103F3P6 I2C Slave + 4-Digit 7-Segment LED Display (Object-Oriented)
 *
 * Target MCU:   STM8S103F3P6
 * Compiler:     SDCC (Small Device C Compiler)
 * Clock:        HSI 16MHz
 *
 * Peripherals / pins (TSSOP20):
 *   I2C      PB4=SCL PB5=SDA  -> bit-bang I2C slave (功能同硬件 I2C 定义), 7-bit addr 0x50
 *                               (open-drain + external 4.7k pull-ups)
 *   7-SEG    segments a..g,dp -> PA1 PA2 PA3 PC4 PC3 PC5 PC7 PC6 (active high)
 *            digits 1..4      -> PD3 PD2 PD1 PD4 (common cathode, active low)
 *            (12 GPIO total; I2C PB4/PB5 and UART PD5/PD6 avoided;
 *             PD0 is NOT brought out on TSSOP20; PD1 is shared with
 *             SWIM — using it disables SWIM debug)
 *   UART     PD5 TX / PD6 RX, 115200 8N1 debug output
 *
 * Boot demo: after reset the display fills up from the RIGHT (low digit
 * first), one digit per step. Digit numbering here is "digit N" = digit
 * index N-1 (digits[3] = digit 4 = rightmost = low digit; digits[0] =
 * digit 1 = leftmost = high digit). Step 1 lights digit 4 only, step 2
 * lights digits 3-4, step 3 lights digits 2-4 and step 4 lights all four,
 * so the screen reads
 *        8.  ->   8.8.  ->  8.8.8.  ->  8.8.8.8.
 * A lit digit is driven with DEMO_SEG_8DP (0xFF = '8' segments 0x7F plus
 * the dp 0x80), a dark digit with DEMO_SEG_OFF (0x00, all segments off).
 * Steps 1..3 are each held for DEMO_STEP_MS (default 500ms); the final
 * step 8.8.8.8. is held noticeably longer, DEMO_LAST_HOLD_MS (default
 * 2000ms), so the completed pattern stays readable. The animation is a
 * one-shot: it plays exactly once and never restarts. Stepping is timed by
 * the TIM1 1ms tick (non-blocking, no delay inside the ISR).
 * segled's SEGCODE_Count()/SEGCODE_CharAt() are NOT used by the demo any
 * more; they stay in the module for the I2C character mapping.
 *
 * DP marquee: when the last step (8.8.8.8.) has been held for a full
 * DEMO_LAST_HOLD_MS it does NOT stop — it hands over to the decimal-point marquee.
 * Exactly one of the four digits drives the dp segment (0x80) while the
 * other three are blank (0x00); the lit position rotates from the low
 * digit to the high one (right to left):
 * 4 -> 3 -> 2 -> 1 -> 4 ... forever, once every MARQUEE_STEP_MS
 * (default 300ms). The marquee is clocked by the same TIM1 1ms slice
 * (non-blocking, no delay in the ISR) and mirrors itself into
 * i2c.tx_buf as '.' at the lit position and ' ' elsewhere, so a master
 * read always matches the screen.
 *
 * Event loop: TIM1 hard interrupt drives seg.onTime() dynamic scan
 * (1ms per digit), the boot-demo rollover and, afterwards, the dp
 * marquee; the I2C vector (IRQ 19)
 * services the hardware I2C slave and, on a complete 4-byte write,
 * updates the display through the onWrite callback. The main loop
 * drains the UART TX ring buffer and echoes UART RX bytes.
 *
 * I2C protocol (ASCII payload):
 *   Master writes 4 bytes -> each byte is mapped by segled's
 *   SEGCODE_FromChar(): recognized characters ('0'-'9','A','b','C',
 *   'd','E','F','G','H','I','J','L','n','O','P','q','r','S','t','U',
 *   'y','-','.') display their segment pattern, any other byte maps
 *   to 0x00 (blank / space); UART prints "I2C WR ...". Any I2C write
 *   immediately cancels the boot demo and the dp marquee (permanently,
 *   no auto-resume) and the host payload controls the display from
 *   then on.
 *   Master reads 4 bytes  -> returns the current displayed characters.
 */

#include <stdint.h>
#include "i2c_slave.h"
#include "segled.h"
#include "uart.h"

/* ========== User Configuration ========== */

#define GPIO_PA         0x5000
#define GPIO_PB         0x5005
#define GPIO_PC         0x500A
#define GPIO_PD         0x500F

/* I2C slave address (7-bit, e.g. 0x50 -> register 0xA0 on the bus) */
#define I2C_SLAVE_ADDR  0x50

/* Boot demo (left-to-right fill animation) before any host data arrives.
 * DEMO_STEP_MS      : how long fill steps 1..3 are held (digit count 1..3).
 * DEMO_LAST_HOLD_MS : how long the final step (all four digits, 8.8.8.8.)
 *                     is held before the dp marquee takes over — longer
 *                     than the other steps so the completed pattern is
 *                     readable.
 * DEMO_SEG_8DP      : segment code of a lit digit = '8' (0x7F) + dp (0x80).
 * DEMO_SEG_OFF      : segment code of a dark digit = every segment off.
 * Change these two hold times here to speed the animation up or slow it down. */
#define DEMO_STEP_MS      500u
#define DEMO_LAST_HOLD_MS 2000u
#define DEMO_SEG_8DP      0xFFu
#define DEMO_SEG_OFF      0x00u

/* DP marquee (decimal-point runner) after the boot demo has finished.
 * MARQUEE_STEP_MS : how long one digit keeps its dp lit before the lit
 *                   position moves to the next lower digit (4 -> 3 -> 2 -> 1).
 * MARQUEE_SEG_DP  : segment code with only the dp segment on (0x80).
 * MARQUEE_SEG_OFF : segment code with every segment off (0x00, blank). */
#define MARQUEE_STEP_MS 300u
#define MARQUEE_SEG_DP  0x80u
#define MARQUEE_SEG_OFF 0x00u

/* ========== STM8S103 Register Definitions ========== */

#define CLK_CKDIVR      (*(volatile uint8_t *)0x50C6)

/* ========== Clock Configuration ========== */

static void clock_setup(void)
{
    CLK_CKDIVR = 0x00;  /* HSI 16MHz, no division */
}

/* ========== Global Objects (file-level, accessible from ISR) ========== */

static I2CS_t   i2c;                    /* hardware I2C slave (PB4/PB5) */
static SEGLED_t seg;                    /* 4-digit common-cathode 7-seg */
static UART_t   uart;                   /* UART1 debug link (PD5/PD6) */

/* ---- TEMP DIAG: bus observability counters published by i2c_slave.c ---- */
extern volatile uint8_t  g_bb_scl;      /* SCL level at last poll  */
extern volatile uint8_t  g_bb_sda;      /* SDA level at last poll  */
extern volatile uint16_t g_bb_edges;    /* SCL transitions seen    */
extern volatile uint16_t g_bb_starts;   /* START conditions seen   */


/* Global 1ms tick, incremented by TIM1 ISR */
volatile uint16_t g_tick = 0;

/* ========== Boot Demo State ==========
 * After reset the 4 digits run a one-shot right-to-left fill animation
 * (low digit first): the lit '8.' area grows by one digit per step
 * (digit 4, then digits 3-4, then digits 2-4, then all four). Steps 1..3
 * are held for DEMO_STEP_MS, the final step for DEMO_LAST_HOLD_MS.
 *
 *   g_demo_mode     1 = demo running, 0 = demo finished OR cancelled
 *   g_demo_step     current fill step: 0..3 meaning "every digit whose
 *                   index is >= (3 - g_demo_step) is lit; the digits to
 *                   the left of that range stay dark"
 *   g_demo_ms       1ms accumulator; compared against DEMO_STEP_MS while
 *                   filling (steps 0..2) and against DEMO_LAST_HOLD_MS on
 *                   the final step (3)
 *
 * The animation is played exactly once; it neither wraps nor restarts.
 * A complete I2C write immediately cancels the demo (g_demo_mode = 0
 * in i2c_on_write()) and the demo never auto-resumes.
 *
 * After the last step (8.8.8.8.) has been held for a full
 * DEMO_LAST_HOLD_MS the demo switches over to the DP marquee (see below).
 */
volatile uint8_t  g_demo_mode = 1;
volatile uint8_t  g_demo_step = 0;
volatile uint16_t g_demo_ms   = 0;

/* ========== DP Marquee State ==========
 * Runs after the boot demo finished: only one digit at a time lights its
 * dp segment (MARQUEE_SEG_DP), the other three are blank
 * (MARQUEE_SEG_OFF), and the lit position travels from the low digit to
 * the high one (right to left):
 * 4 -> 3 -> 2 -> 1 -> 4 ... without end.
 *
 *   g_marquee_mode  1 = marquee running, 0 = idle (before it starts or
 *                   after an I2C write took over the display)
 *   g_marquee_pos   position with the lit dp: 0..3 (0 = digit 1 = left /
 *                   high, 3 = digit 4 = right / low); it is decremented
 *                   by one every step, so the dp runs right to left
 *   g_marquee_ms    1ms accumulator; at MARQUEE_STEP_MS the lit
 *                   position moves one digit to the left (wrapping)
 *
 * Both counters are advanced from the TIM1 1ms ISR only, so no locking
 * is needed; the ISR never blocks and never delays.
 * A complete I2C write cancels the marquee immediately (same handling
 * as the demo): g_marquee_mode = 0 in i2c_on_write(), and the host
 * payload owns the display from then on.
 */
volatile uint8_t  g_marquee_mode = 0;
volatile uint8_t  g_marquee_pos  = 0;
volatile uint16_t g_marquee_ms   = 0;

/* ========== I2C Write Callback ==========
 * Runs inside the I2C IRQ: only non-blocking actions are allowed.
 * uart.printf() only enqueues into the TX ring buffer (safe in ISR),
 * seg.setDigit() only updates the display buffer (scanned by TIM1).
 */
/* ========== ASCII -> Segment Code ==========
 * Thin wrapper over segled's SEGCODE_FromChar(): the character table
 * lives in the segled module (Flash) and covers '0'-'9', 'A', 'b',
 * 'C', 'd', 'E', 'F', 'G', 'H', 'I', 'J', 'L', 'n', 'O', 'P', 'q',
 * 'r', 'S', 't', 'U', 'y', '-', '.'. Any other byte maps to 0x00
 * (blank / space).
 */
static uint8_t ch_to_disp(uint8_t ch)
{
    return SEGCODE_FromChar((char)ch);
}

/* ========== Demo Display Apply ==========
 * Called from the TIM1 ISR (once per hold time: DEMO_STEP_MS for the fill
 * steps, DEMO_LAST_HOLD_MS for the last one) and from main (to show step 1
 * right after power-on). Renders the current fill step, growing from the
 * LOW digit (rightmost, index 3) towards the HIGH digit (leftmost,
 * index 0):
 *   g_demo_step = 0 -> digit 4 lit            ("    8.")
 *   g_demo_step = 1 -> digits 3-4 lit         ("  8.8.")
 *   g_demo_step = 2 -> digits 2-4 lit         ("8.8.8.")
 *   g_demo_step = 3 -> all four digits lit    ("8.8.8.8.")
 * A lit digit gets DEMO_SEG_8DP (0xFF = '8' + dp), a dark digit gets
 * DEMO_SEG_OFF (0x00). i2c.tx_buf mirrors the same state as ASCII ('8'
 * on the lit digits, ' ' on the dark ones), so a master read always
 * matches the screen. Non-blocking: setDigit() only writes the scan
 * buffer and tx_buf is a plain RAM array. */
static void demo_apply(void)
{
    uint8_t i;
    /* Leftmost index that is already lit: 3, 2, 1, 0 for step 0..3. */
    uint8_t first_lit = (uint8_t)(3u - g_demo_step);

    for (i = 0; i < 4; i++) {
        if (i >= first_lit) {
            seg.setDigit(&seg, i, DEMO_SEG_8DP);    /* lit: '8' + dp */
            i2c.tx_buf[i] = (uint8_t)'8';
        } else {
            seg.setDigit(&seg, i, DEMO_SEG_OFF);    /* dark: blank  */
            i2c.tx_buf[i] = (uint8_t)' ';
        }
    }
}

/* ========== DP Marquee Apply ==========
 * Called from the TIM1 ISR when the demo ends and then once every
 * MARQUEE_STEP_MS. Exactly one digit drives the dp segment
 * (MARQUEE_SEG_DP = 0x80), the other three output 0x00 (all segments
 * off), so the decimal point appears to run across the 4 digits from the
 * LOW digit to the HIGH one (right to left):
 *   digit4 -> digit3 -> digit2 -> digit1 -> digit4 ...
 * Nothing here blocks: setDigit() only stores into the scan buffer and
 * the tx_buf writes keep a master read consistent with the screen
 * ('.' on the lit position, ' ' on the three dark ones). */
static void marquee_apply(void)
{
    uint8_t i;

    for (i = 0; i < 4; i++) {
        if (i == g_marquee_pos) {
            seg.setDigit(&seg, i, MARQUEE_SEG_DP);      /* dp only */
            i2c.tx_buf[i] = (uint8_t)'.';
        } else {
            seg.setDigit(&seg, i, MARQUEE_SEG_OFF);     /* blank   */
            i2c.tx_buf[i] = (uint8_t)' ';
        }
    }
}

/* ========== 主机字符流 -> 数码管 ==========
 * 主机把 UART 收到的每个字符原样转发过来，帧格式
 *   { CMD_SET_DIGIT(0x10), pos, ch, 0x00 }   ch = ASCII 字符
 * pos 字段从机一律忽略，排位完全由从机自己维护：
 *   1) 复位后第一次收到字符：停掉 demo/marquee 动画，清空 4 位显示缓冲，
 *      字符显示在最右（最低）位 digit4 (index 3)；
 *   2) 之后的字符是 '.'：只把 dp 段叠加到刚才那一位（最右位），不移动内容；
 *   3) 之后是可显示字符（SEGCODE_FromChar() 命中字符索引表）：
 *      原内容整体右移一位 digit1<-digit2<-digit3<-digit4，新字符进最右位；
 *   4) '\r'（回车）：清屏，4 位全灭，光标回到最右位 digit4，后续字符
 *      重新从最右位开始排；
 *   5) 其余字符表外的字符（空格 / LF 等，映射为 0x00）：忽略，显示不动。
 * 所有动作只写扫描缓冲，函数内不阻塞。 */

#define DISP_LAST    (I2C_DISP_LEN - 1u)   /* 最右（最低）位 digit4 */
#define SEG_DP_BIT   0x80u

static uint8_t g_disp_host = 0;            /* 0 = 主机尚未接管显示 */

/* 清空 4 位显示缓冲：段码全灭，回读缓冲同步写空格。
 * 只写 RAM 缓冲，不阻塞，可在 I2C/定时器上下文里调用。 */
static void disp_clear(void)
{
    uint8_t i;

    for (i = 0; i < I2C_DISP_LEN; i++) {
        seg.digits[i] = DEMO_SEG_OFF;      /* 清空显示缓冲区 */
        i2c.tx_buf[i] = (uint8_t)' ';      /* 主机回读同步 */
    }
}

/* 首次接管：停动画 + 清显示缓冲 */
static void disp_take_over(void)
{
    g_demo_mode    = 0;
    g_demo_step    = 0;
    g_demo_ms      = 0;
    g_marquee_mode = 0;
    g_marquee_pos  = 0;
    g_marquee_ms   = 0;

    disp_clear();

    g_disp_host = 1;
}

static void disp_push_char(uint8_t ch)
{
    uint8_t code;
    uint8_t i;

    if (!g_disp_host) {
        disp_take_over();
    }

    if (ch == (uint8_t)'\r') {             /* 回车：清屏，光标回最右位 */
        disp_clear();
        return;
    }

    if (ch == (uint8_t)'.') {              /* 小数点：叠加到刚显示的那一位 */
        seg.digits[DISP_LAST] |= SEG_DP_BIT;
        return;
    }

    code = ch_to_disp(ch);
    if (code == 0x00u) {                   /* 不在字符索引表：忽略 */
        return;
    }

    for (i = 0; i + 1u < I2C_DISP_LEN; i++) {   /* 整体右移一位 */
        seg.digits[i] = seg.digits[i + 1u];
        i2c.tx_buf[i] = i2c.tx_buf[i + 1u];
    }
    seg.digits[DISP_LAST] = code;          /* 新字符进最右位 */
    i2c.tx_buf[DISP_LAST] = ch;
}

static void i2c_on_write(I2CS_t *self)
{
    uint8_t cmd = self->rx_buf[0];

    if (cmd == 0x10u) {                    /* 主机转发的一个字符 */
        if (self->rx_len >= 3u) {
            uint8_t ch = self->rx_buf[2];

            /* 一律按原始 ASCII 处理。这里曾经有一段"旧协议数值 0..15
             * 转回 '0'..'F'"的兼容分支，会把 0x0D('\r') 转成 'A'+3='D'、
             * 把 0x0A('\n') 转成 'A'，导致清屏控制符被当成字符 'D' 显示。
             * 旧主机已全部改发 ASCII，故该分支删除。 */
            disp_push_char(ch);
            /* '\r' 是清屏控制符，不回显控制字符，换成换行便于观察 */
            uart.putc(&uart, (ch == (uint8_t)'\r') ? '\n' : (char)ch);
        }
        return;
    }

    if (cmd == 0x01u) {                    /* PING：绝不碰显示 */
        self->tx_buf[0] = (uint8_t)'P';
        self->tx_buf[1] = (uint8_t)'I';
        self->tx_buf[2] = (uint8_t)'N';
        self->tx_buf[3] = (uint8_t)'G';
        return;
    }

    /* 未知帧：保持显示完全不变 */
}

/* ========== TIM1 1ms Time-Slice Timer (STM8S103) ========== */

#define TIM1_CR1    (*(volatile uint8_t *)0x5250)
#define TIM1_IER    (*(volatile uint8_t *)0x5254)
#define TIM1_SR1    (*(volatile uint8_t *)0x5255)
#define TIM1_EGR    (*(volatile uint8_t *)0x5257)
#define TIM1_PSCRH  (*(volatile uint8_t *)0x5260)
#define TIM1_PSCRL  (*(volatile uint8_t *)0x5261)
#define TIM1_ARRH   (*(volatile uint8_t *)0x5262)
#define TIM1_ARRL   (*(volatile uint8_t *)0x5263)
#define CLK_PCKENR1 (*(volatile uint8_t *)0x50C7)

/* HSI 16MHz, prescaler /16 -> 1MHz counter, ARR=1000 -> 1ms update */
static void tim1_setup(void)
{
    CLK_PCKENR1 |= 0x80;   /* bit7 = TIM1 clock gate, explicit enable */
    TIM1_PSCRH = 0x00;   /* prescaler high byte (16-bit prescaler = 16) */
    TIM1_PSCRL = 15;     /* /16 */
    TIM1_ARRH  = 0x03;   /* auto-reload 1000 (0x03E8) */
    TIM1_ARRL  = 0xE8;
    TIM1_EGR  |= 0x01;   /* UG: generate update to load prescaler/ARR */
    TIM1_IER  |= 0x01;   /* UIE: enable update interrupt */
    TIM1_CR1  |= 0x01;   /* CEN: start counter */
}

/* ========== TIM1 Update ISR ==========
 * STM8S103F3 TIM1_UPD_OVF_TRG_BRK vector = 0x8034 (IRQ 11)
 * -> SDCC __interrupt(11). Drives the non-blocking 1ms time slices. */
void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(11)
{
    TIM1_SR1 &= (uint8_t)~0x01;      /* clear UIF (must do before exit) */

    g_tick++;                        /* advance global 1ms tick */

    /* 1ms dynamic scan: one digit lit per call, full refresh every 4ms. */
    seg.onTime(&seg, g_tick);

    /* Boot demo: non-blocking one-shot fill animation (no delay inside the
     * ISR). It starts at step 0 (demo_apply() in main: only digit 4, the
     * rightmost / LOW digit, is lit) and advances one step every
     * DEMO_STEP_MS, so the lit area grows towards the left:
     * digit 4 -> digits 3-4 -> digits 2-4 -> all four digits. The fill
     * steps (0..2) share DEMO_STEP_MS, while the final step 3
     * (8.8.8.8.) uses its own, longer DEMO_LAST_HOLD_MS before the demo
     * ends and the dp marquee takes over; an I2C write cancels both
     * earlier (see i2c_on_write()). */
    if (g_demo_mode) {
        /* Hold time depends on the step we are currently showing. */
        uint16_t hold = (g_demo_step < 3u) ? DEMO_STEP_MS
                                          : DEMO_LAST_HOLD_MS;

        g_demo_ms++;
        if (g_demo_ms >= hold) {
            g_demo_ms = 0;
            if (g_demo_step < 3u) {
                g_demo_step++;
                demo_apply();
            } else {
                /* All four digits lit and held long enough: hand the
                 * display over to the dp marquee starting at digit 4 (the
                 * LOW digit that the fill animation ended on) so it keeps
                 * running. */
                g_demo_mode    = 0;
                g_marquee_mode = 1;
                g_marquee_pos  = 3;
                g_marquee_ms   = 0;
                marquee_apply();
            }
        }
    }
    /* DP marquee: shift the lit decimal point one digit to the left, i.e.
     * towards the HIGH digits (4 -> 3 -> 2 -> 1) every MARQUEE_STEP_MS,
     * wrapping 1 -> 4, forever (no delay here). */
    else if (g_marquee_mode) {
        g_marquee_ms++;
        if (g_marquee_ms >= MARQUEE_STEP_MS) {
            g_marquee_ms  = 0;
            /* -1 with wrap: (pos + 3) & 0x03 gives 3,2,1,0,3,... */
            g_marquee_pos = (uint8_t)((g_marquee_pos + 3u) & 0x03u);
            marquee_apply();
        }
    }
}

/* ========== UART1 RX ISR Stub (must live in the main unit!) ==========
 * SDCC 4.5.0 stm8 后端只在"含 main 的编译单元"生成 __interrupt_vect 向量表。
 * UART1_RX vector = 0x8050 (IRQ 18) -> __interrupt(18)。
 * 向量壳放在本文件，实际处理委托给 uart 模块的 uart_rx_irq_handler()。
 */
void UART1_RX_IRQHandler(void) __interrupt(18)
{
    uart_rx_irq_handler();
}

/* ========== I2C ISR (unused) ==========
 * STM8S103F3 I2C vector = 0x8054 (IRQ 19) -> __interrupt(19).
 * The slave is now pure GPIO bit-bang (i2c_slave.c): the hardware I2C block
 * stays disabled (PE=0, clock ungated), so this vector never fires. The
 * stub is kept because the bit-bang slave polls the bus from the main loop
 * instead of running inside an interrupt. */
void I2C_IRQHandler(void) __interrupt(19)
{
    i2c.irq(&i2c);              /* no-op stub */
}

/* ========== 兜底：PB 端口外部中断（int3） ==========================
 * 向量表原本把 int3 填成 0x000000：只要 PB4/PB5 的 CR2（外部中断使能）
 * 被置 1，总线抖动/静电/碰触金属产生的边沿就会把 PC 打进 0x000000，
 * 表现为整机死锁（数码管停在一位、串口静默、只能断电恢复）。
 * 这里挂一个只做 iret 的空处理程序，从根上消除"跳 0 死机"这条路径；
 * 同时 PB4/PB5 的 CR2 已在 i2cs_init / pin_selftest 里强制清零。
 * 编号说明：SDCC 的 __interrupt(N) 槽地址 = 0x8008 + 4N，而 STM8 的 IRQ n
 * 在 0x800C + 4n，故 N = n + 1。Port B = EXTI1 = IRQ3 -> __interrupt(4)
 * （同法可验：TIM1_UPD = IRQ10 -> 11、UART1_RX = IRQ17 -> 18、I2C = IRQ18 -> 19）。 */
void PB_EXTI_IRQHandler(void) __interrupt(4)
{
    /* 不读 IDR、不清任何标志，直接返回。EXTI 无锁存标志位。 */
}

/* ========== TEMP DIAG: PB4/PB5 pin controllability self-test ==========
 * 接线、共地、4.7k 上拉三项均已确认无误，但从机侧 g_bb_edges 每次事务
 * 只 +2，说明 PB5(SCL) 的电平翻转根本没有被采到。必须先区分两种可能：
 *   (a) 引脚受 GPIO 控制，只是主机信号没到  -> 主动拉低能读回 0
 *   (b) 引脚不受 GPIO 控制（选项字节 AFR 把 I2C 复用走了，或引脚仍被
 *       I2C 外设占住）                      -> 主动拉低后读回仍是 1
 * 做法：把 SCL 临时设为开漏输出并驱动低，忙等一会儿后读 IDR；再释放
 * 读一次（验外部上拉是否真的存在）。SDA 同理。
 * 打印格式：#SELFTEST scl_low=x scl_rel=x sda_low=x sda_rel=x
 *   *_low=0 -> 拉低成功（引脚可控）    *_low=1 -> 拉不低（引脚不可控/短路到 3V3）
 *   *_rel=1 -> 释放后回高（上拉有效）  *_rel=0 -> 释放后仍低（被别处拉死）
 */
static uint8_t g_st_scl_low, g_st_scl_rel, g_st_sda_low, g_st_sda_rel;

static void pin_selftest(void)
{
    volatile uint16_t d;

    /* ---- SCL / PB4 (第12脚): switch to open-drain output, drive low ----
     * CR2 是 GPIO 外部中断使能位，此处必须清零（&= ~），绝不能置 1：
     * 它是整机 HOLD 的根因 —— 置 1 后任何边沿都会打进空的 Port B 向量。 */
    GPIO_PB_ODR &= (uint8_t)~I2C_SCL_PIN;
    GPIO_PB_DDR |=  I2C_SCL_PIN;
    GPIO_PB_CR1 &= (uint8_t)~I2C_SCL_PIN;
    GPIO_PB_CR2 &= (uint8_t)~I2C_SCL_PIN;
    for (d = 0u; d < 40000u; d++) { ; }
    g_st_scl_low = (GPIO_PB_IDR & I2C_SCL_PIN) ? 1u : 0u;

    /* release and read back, then restore pure-input configuration */
    GPIO_PB_ODR |= I2C_SCL_PIN;
    for (d = 0u; d < 40000u; d++) { ; }
    g_st_scl_rel = (GPIO_PB_IDR & I2C_SCL_PIN) ? 1u : 0u;
    GPIO_PB_DDR &= (uint8_t)~I2C_SCL_PIN;
    GPIO_PB_CR2 &= (uint8_t)~I2C_SCL_PIN;

    /* ---- SDA / PB5 (第11脚): already open-drain output, just toggle ODR ---- */
    GPIO_PB_CR2 &= (uint8_t)~I2C_SDA_PIN;   /* 不许开 PB5 输入中断 */
    GPIO_PB_ODR &= (uint8_t)~I2C_SDA_PIN;
    for (d = 0u; d < 40000u; d++) { ; }
    g_st_sda_low = (GPIO_PB_IDR & I2C_SDA_PIN) ? 1u : 0u;

    GPIO_PB_ODR |= I2C_SDA_PIN;
    for (d = 0u; d < 40000u; d++) { ; }
    g_st_sda_rel = (GPIO_PB_IDR & I2C_SDA_PIN) ? 1u : 0u;
}

/* ========== Main Entry ========== */

int main(void)
{
    /* Create objects (bind methods + defaults) */
    i2c  = I2CS_Create(I2C_BASE, I2C_SLAVE_ADDR);
    seg  = SEGLED_Create();
    uart = UART_Create(UART1_BASE);

    clock_setup();                  /* HSI 16MHz */

    /* Init peripherals */
    uart.init(&uart);               /* UART1 115200 8N1, TX PD5 / RX PD6 */
    seg.init(&seg);                 /* 12 GPIO outputs for 4x7-seg */
    i2c.init(&i2c);                 /* I2C slave 0x50, PB4=SCL PB5=SDA */

    /* Bind I2C write callback (display update + debug print). */
    i2c.onWrite = &i2c_on_write;

    tim1_setup();                   /* TIM1: 1ms update interrupt */
    __asm__("rim");                 /* enable global interrupts */

    /* Print the ready banner first so it is not delayed by the demo's
     * per-step UART trace (demo_apply() enqueues one char on UART). */
    uart.puts(&uart, "SLAVE ready\r\n");

    pin_selftest();

    /* Start the boot demo immediately: g_demo_step = 0 -> only digit 4, the
     * rightmost / LOW digit, is lit ("    8."). Every DEMO_STEP_MS the TIM1
     * ISR lights one more digit to the LEFT (8.8. / 8.8.8. / 8.8.8.8.); the
     * completed pattern is held for DEMO_LAST_HOLD_MS (2s) and then hands
     * over to the endless dp marquee (one dp segment running digit4 ->
     * digit1, i.e. low to high, every MARQUEE_STEP_MS). */
    demo_apply();

    /* Main loop: the I2C slave is a SOFTWARE bit-bang implementation and
     * has to be polled continuously, so wfi/halt must NOT be used here --
     * either one would sleep through the START condition and every SCL
     * edge. UART service is throttled to every 16th poll so the polling
     * rate stays high enough for the bit timing. */
    {
        uint8_t svc = 0;

        while (1) {
            i2c.poll(&i2c);         /* sample SCL/SDA, run the bit state machine */

            if (++svc >= 16u) {
                svc = 0;

                /* Drain pending TX bytes */
                uart.tx_service(&uart);
            }
        }
    }

    // return 0;
}
