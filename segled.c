/**
 * segled.c - STM8S 4-Digit 7-Segment LED Driver Implementation
 *
 * Dynamic scanning: onTime() is called every 1ms from the TIM1 ISR
 * and lights exactly one digit (segment code + digit select low for
 * common-cathode). A full 4-digit refresh takes 4ms (~250Hz per digit),
 * which is flicker-free for human eyes.
 *
 * Common-cathode wiring notes:
 *   - Segment pins drive the segment ANODES through current-limiting
 *     resistors (e.g. 220R per segment), active-HIGH to light a segment.
 *   - Digit pins drive the shared CATHODES, active-LOW to select a digit.
 *     For higher brightness / larger displays buffer the digit select
 *     with NPN transistors; direct GPIO is fine for small displays.
 */

#include <stdint.h>
#include "segled.h"

/* ========== GPIO Port Bases (STM8S103F3P6) ========== */
#define GPIO_PA_BASE   0x5000
#define GPIO_PB_BASE   0x5005
#define GPIO_PC_BASE   0x500A
#define GPIO_PD_BASE   0x500F

/* ========== Character -> Segment Code Lookup Table ==========
 * const table stored in Flash (CODE segment on STM8/SDCC).
 * bit0..bit6 = a..g, bit7 = dp. Active-HIGH segment outputs.
 * Recognized characters: '0'-'9', 'A','b','C','d','E','F','G','H',
 * 'I','J','L','n','O','P','q','r','S','t','U','y','-','.'.
 * Any other character maps to 0x00 (blank / space).
 */
typedef struct {
    char    ch;
    uint8_t code;
} SEGCODE_ENTRY_t;

static const SEGCODE_ENTRY_t seg_code_lookup[] = {
    { '0', 0x3F }, { '1', 0x06 }, { '2', 0x5B }, { '3', 0x4F },
    { '4', 0x66 }, { '5', 0x6D }, { '6', 0x7D }, { '7', 0x07 },
    { '8', 0x7F }, { '9', 0x6F },
    { 'A', 0x77 }, { 'b', 0x7C }, { 'C', 0x39 }, { 'd', 0x5E },
    { 'E', 0x79 }, { 'F', 0x71 }, { 'G', 0x3D }, { 'H', 0x76 },
    { 'I', 0x06 }, { 'J', 0x1E }, { 'L', 0x38 }, { 'n', 0x54 },
    { 'O', 0x3F }, { 'P', 0x73 }, { 'q', 0x67 }, { 'r', 0x50 },
    { 'S', 0x6D }, { 't', 0x78 }, { 'U', 0x3E }, { 'y', 0x6E },
    { '-', 0x40 }, { '.', 0x80 },
};
#define SEGCODE_LOOKUP_SIZE (sizeof(seg_code_lookup) / sizeof(seg_code_lookup[0]))

/* Resolve one ASCII character to its 8-segment code.
 * Unrecognized characters return 0x00 (blank / space). */
uint8_t SEGCODE_FromChar(char ch)
{
    uint8_t i;
    for (i = 0; i < SEGCODE_LOOKUP_SIZE; i++) {
        if (seg_code_lookup[i].ch == ch) {
            return seg_code_lookup[i].code;
        }
    }
    return 0x00;   /* blank / space */
}

/* Number of entries in the character table: the single source of truth
 * for callers that iterate over every displayable character. */
uint8_t SEGCODE_Count(void)
{
    return (uint8_t)SEGCODE_LOOKUP_SIZE;
}

/* Character at index idx (0-based); out-of-range -> ' ' (blank). */
char SEGCODE_CharAt(uint8_t idx)
{
    if (idx < SEGCODE_LOOKUP_SIZE) {
        return seg_code_lookup[idx].ch;
    }
    return ' ';   /* blank / space */
}

/* ========== SEGLED Method Implementations ========== */

static void segled_init(SEGLED_t *self)
{
    uint8_t i;

    /* Segments: push-pull outputs, all off (low). */
    for (i = 0; i < 8; i++) {
        uint16_t port = self->seg_port[i];
        uint8_t  mask = self->seg_mask[i];
        GPIO_DDR(port) |=  mask;
        GPIO_CR1(port) |=  mask;   /* push-pull */
        GPIO_CR2(port) &= ~mask;   /* 2MHz slew (default) */
        GPIO_ODR(port) &= ~mask;   /* segment off */
    }

    /* Digits: push-pull outputs, all inactive (high = cathode released
     * for common-cathode displays, i.e. no digit selected). */
    for (i = 0; i < 4; i++) {
        uint16_t port = self->dig_port[i];
        uint8_t  mask = self->dig_mask[i];
        GPIO_DDR(port) |=  mask;
        GPIO_CR1(port) |=  mask;   /* push-pull */
        GPIO_CR2(port) &= ~mask;
        GPIO_ODR(port) |=  mask;   /* digit off (high) */
    }

    self->cur_digit = 0;
}

static void segled_setDigit(SEGLED_t *self, uint8_t pos, uint8_t value)
{
    if (pos < 4) {
        self->digits[pos] = value;
    }
}

static void segled_show(SEGLED_t *self, const char *str)
{
    uint8_t i;
    for (i = 0; i < 4 && str && str[i]; i++) {
        self->digits[i] = SEGCODE_FromChar(str[i]);
    }
}

/* Scan one digit per call. Non-blocking, ISR-safe. */
static void segled_onTime(SEGLED_t *self, uint16_t tick)
{
    (void)tick;                     /* fixed 1ms cadence from TIM1 */
    uint8_t i;
    uint8_t code;
    uint8_t cur;

    /* 1. Blank all digits (all digit selects high). */
    for (i = 0; i < 4; i++) {
        GPIO_ODR(self->dig_port[i]) |= self->dig_mask[i];
    }

    /* 2. Drive segment pattern for the current digit. */
    cur = self->cur_digit;
    code = self->digits[cur];   /* digits[] holds the segment code directly */
    for (i = 0; i < 8; i++) {
        if (code & (1u << i)) {
            GPIO_ODR(self->seg_port[i]) |= self->seg_mask[i];
        } else {
            GPIO_ODR(self->seg_port[i]) &= (uint8_t)~self->seg_mask[i];
        }
    }

    /* 3. Select the current digit (cathode low). */
    GPIO_ODR(self->dig_port[cur]) &= (uint8_t)~self->dig_mask[cur];

    /* 4. Advance to the next digit. */
    self->cur_digit = (uint8_t)((cur + 1) & 0x03);
}

/* ========== SEGLED Constructor ========== */

SEGLED_t SEGLED_Create(void)
{
    SEGLED_t seg;

    /* Default pin map (TSSOP20 actual GPIO, 12 pins):
     *   segments PA1 PA2 PA3 PC4 PC3 PC5 PC7 PC6   (a b c d e f g dp)
     *   digits   PD3 PD2 PD1 PD4                  (DIG1..DIG4)
     * PD0 is NOT brought out on TSSOP20; PD1 = SWIM (shared) and is
     * used as DIG3, so SWIM in-circuit debug is disabled. This is the
     * only feasible 12-GPIO mapping
     * that coexists with I2C (PB4/PB5) and UART (PD5/PD6). */
    static const uint16_t seg_ports[8] = {
        GPIO_PA_BASE, GPIO_PA_BASE, GPIO_PA_BASE, GPIO_PC_BASE,
        GPIO_PC_BASE, GPIO_PC_BASE, GPIO_PC_BASE, GPIO_PC_BASE
    };
    static const uint8_t  seg_pins[8]  = { 1, 2, 3, 4, 3, 5, 7, 6 };
    static const uint16_t dig_ports[4] = {
        GPIO_PD_BASE, GPIO_PD_BASE, GPIO_PD_BASE, GPIO_PD_BASE
    };
    static const uint8_t  dig_pins[4]  = { 3, 2, 1, 4 };

    uint8_t i;

    for (i = 0; i < 8; i++) {
        seg.seg_port[i] = seg_ports[i];
        seg.seg_pin[i]  = seg_pins[i];
        seg.seg_mask[i] = (uint8_t)(1 << seg_pins[i]);
    }
    for (i = 0; i < 4; i++) {
        seg.dig_port[i] = dig_ports[i];
        seg.dig_pin[i]  = dig_pins[i];
        seg.dig_mask[i] = (uint8_t)(1 << dig_pins[i]);
    }

    /* Power-on display: "8888" (all segments self-test). */
    seg.digits[0] = SEGCODE_FromChar('8');
    seg.digits[1] = SEGCODE_FromChar('8');
    seg.digits[2] = SEGCODE_FromChar('8');
    seg.digits[3] = SEGCODE_FromChar('8');
    seg.cur_digit = 0;

    /* Bind methods. */
    seg.init     = segled_init;
    seg.setDigit = segled_setDigit;
    seg.show     = segled_show;
    seg.onTime   = segled_onTime;

    return seg;
}
