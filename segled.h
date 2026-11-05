/**
 * segled.h - STM8S 4-Digit 7-Segment LED Driver (Object-Oriented)
 *
 * Drives a 4-digit common-cathode 8-segment (a..g + dp) LED display
 * using 12 GPIO pins (8 segment + 4 digit select) with software
 * dynamic scanning. Each module exposes a SEGLED_t abstraction with
 * method binding, mirroring the OO style of project2.
 *
 * Pin mapping (TSSOP20 actual GPIO, common-cathode, active-low digit):
 *   SEG_A PA1   SEG_B PA2   SEG_C PA3   SEG_D PC4
 *   SEG_E PC3   SEG_F PC5   SEG_G PC7   SEG_DP PC6
 *   DIG1 PD3   DIG2 PD2    DIG3 PD1*   DIG4 PD4
 * (*) PD1 is shared with SWIM debug (used here as DIG3): using it
 *     disables SWIM in-circuit debug; keep the SWIM connector free
 *     if SWIM is needed.
 *     This is the only feasible 12-GPIO mapping that coexists with
 *     I2C (PB4/PB5) and UART (PD5/PD6) on the 20-pin package.
 * (all 12 pins deliberately avoid I2C PB4/PB5 and UART PD5/PD6)
 *
 * Usage:
 *   #include "segled.h"
 *   SEGLED_t seg = SEGLED_Create();          // default pin map above
 *   seg.init(&seg);
 *   seg.setDigit(&seg, 0, SEGCODE_FromChar('1')); // show "1" on digit 0
 *   seg.show(&seg, "1234");                  // convenience (4 chars)
 *   seg.onTime(&seg, tick);                  // call every 1ms (TIM1 ISR)
 */

#ifndef SEGLED_H
#define SEGLED_H

#include <stdint.h>

/* ========== GPIO Register Offset Macros ==========
 * STM8S GPIO registers layout per port (same as led.h in project2):
 *   +0: ODR    Output Data Register
 *   +1: IDR    Input Data Register
 *   +2: DDR    Data Direction Register
 *   +3: CR1    Control Register 1
 *   +4: CR2    Control Register 2
 */
#define GPIO_REG(port, off)     (*(volatile uint8_t *)((port) + (off)))
#define GPIO_ODR(port)          GPIO_REG(port, 0)
#define GPIO_IDR(port)          GPIO_REG(port, 1)
#define GPIO_DDR(port)          GPIO_REG(port, 2)
#define GPIO_CR1(port)          GPIO_REG(port, 3)
#define GPIO_CR2(port)          GPIO_REG(port, 4)

/* ========== Character -> Segment Code Mapping ==========
 * SEGCODE_FromChar() resolves one ASCII character to its 8-segment
 * code (bit0..bit6 = a..g, bit7 = dp, active-high). The lookup table
 * lives in Flash (segled.c) and covers:
 *   '0'-'9'                        digits
 *   'A','b','C','d','E','F','G','H','I','J','L','n','O','P',
 *   'q','r','S','t','U','y'        letters
 *   '-', '.'                       minus / decimal point
 * Any other character maps to 0x00 (blank / space). The digits[]
 * buffers store segment codes directly, so no second translation is
 * needed at scan time.
 */
uint8_t SEGCODE_FromChar(char ch);

/* Accessors over the same lookup table, so callers (e.g. the boot demo)
 * can walk every displayable character without duplicating the table.
 * SEGCODE_Count()    number of entries in the table (currently 32).
 * SEGCODE_CharAt(i)  character at index i (0-based); out-of-range
 *                    requests return ' ' (blank). */
uint8_t SEGCODE_Count(void);
char    SEGCODE_CharAt(uint8_t idx);

/* ========== 4-Digit LED Class Definition ========== */

typedef struct SEGLED SEGLED_t;

struct SEGLED {
    uint16_t seg_port[8];       /* GPIO base per segment a..g,dp */
    uint8_t  seg_pin[8];        /* pin number per segment */
    uint8_t  seg_mask[8];       /* precomputed bit masks */
    uint16_t dig_port[4];       /* GPIO base per digit select */
    uint8_t  dig_pin[4];        /* pin number per digit */
    uint8_t  dig_mask[4];       /* precomputed bit masks */

    /* --- Display state --- */
    uint8_t  digits[4];         /* current segment codes (bit0..6=a..g, bit7=dp) */
    uint8_t  cur_digit;         /* digit currently being scanned (0..3) */

    /* --- Public methods --- */
    void (*init)   (SEGLED_t *self);                     /* configure 12 GPIO outputs */
    void (*setDigit)(SEGLED_t *self, uint8_t pos, uint8_t code);  /* store segment code into one digit buffer */
    void (*show)   (SEGLED_t *self, const char *str);    /* set 4 digits from string */
    void (*onTime) (SEGLED_t *self, uint16_t tick);      /* 1ms dynamic scan (TIM1 ISR) */
};

/* ========== Constructor ========== */

/**
 * Create a 4-digit LED object with the default pin map:
 *   segments PA1 PA2 PA3 PC4 PC3 PC5 PC7 PC6 ; digits PD3 PD2 PD1 PD4.
 * Uses static storage — safe for single-instance use on STM8S103.
 * The constructor does NOT configure the GPIOs; call init() first.
 *
 * @return  fully initialized SEGLED_t object
 */
SEGLED_t SEGLED_Create(void);

#endif /* SEGLED_H */
