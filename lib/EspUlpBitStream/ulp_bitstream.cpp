#include "ulp_bitstream.hpp"

#include <driver/rtc_io.h>
#include <esp32/ulp.h>
#include <ulp_common.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/sens_reg.h>
#include <soc/rtc_periph.h>
//#include "soc/soc.h"

#include <esp_log.h>

namespace ulp_bitstream {

/* ---------- Convenience: access RTC slow memory from main core ---------- */
#define RTC_WORD(offset)    (RTC_SLOW_MEM[(offset)] & 0xFFFF)
#define RTC_WRITE(offset, val) \
    do { RTC_SLOW_MEM[(offset)] = (uint32_t)(val) & 0xFFFF; } while(0)

/* ---------- RTC memory layout ---------- *
 *   [0..max_prog_size-1]       = ULP program
 *   [max_prog_size]            = read_pos   (ULP writes, main reads)
 *   [max_prog_size+1]          = write_pos  (main writes, ULP reads)
 *   [max_prog_size+2 .. max_prog_size+2+buf_len-1] = buf_a
 *   [max_prog_size+2+buf_len .. max_prog_size+2+2*buf_len-1] = buf_b
 *
 */
static constexpr uint16_t MAX_PROG_SIZE = 40; // must be known at compile time to calculate variable addresses
static constexpr uint16_t OFF_READ_POS  = MAX_PROG_SIZE;
static constexpr uint16_t OFF_WRITE_POS = MAX_PROG_SIZE + 1;

/* ---------- Module state (set once in init, read by other functions) ----- */
static struct {
    uint16_t buf_len;
    uint16_t off_buf_a;
    uint16_t off_buf_b;
    uint16_t off_program;
} s_cfg;

/* ---------- Label enumeration ---------- */
enum {
    LBL_A_LOW = 1,
    LBL_B,
    LBL_B_LOW,
    LBL_ADVANCE,
    LBL_NO_WRAP,
    LBL_HALT,
};

/* ---------- Init & start ---------- */
int init(const config_t &cfg) {
    /* Resolve RTC GPIO numbers */
    int rtc_a = rtc_io_number_get(cfg.gpio_a);
    int rtc_b = rtc_io_number_get(cfg.gpio_b);
    if (rtc_a < 0 || rtc_b < 0) return -1;

    uint16_t buf_len = cfg.buf_len;

    /* Compute memory layout */
    uint16_t off_program = 0;
    uint16_t off_buf_a   = OFF_WRITE_POS + 1;
    uint16_t off_buf_b   = off_buf_a + buf_len;
    if(off_buf_b + buf_len > CONFIG_ULP_COPROC_RESERVE_MEM/sizeof(uint32_t)) {
        ESP_LOGW("ULP", "ULP mem overflows COPROC_RESERVE_MEM");
    }

    /* Store config for other API functions */
    s_cfg.buf_len    = buf_len;
    s_cfg.off_buf_a  = off_buf_a;
    s_cfg.off_buf_b  = off_buf_b;
    s_cfg.off_program = off_program;

    /* Configure RTC GPIOs for output */
    ESP_ERROR_CHECK(rtc_gpio_init(cfg.gpio_a));
    ESP_ERROR_CHECK(rtc_gpio_set_direction(cfg.gpio_a, RTC_GPIO_MODE_OUTPUT_ONLY));
    ESP_ERROR_CHECK(rtc_gpio_init(cfg.gpio_b));
    ESP_ERROR_CHECK(rtc_gpio_set_direction(cfg.gpio_b, RTC_GPIO_MODE_OUTPUT_ONLY));

    /* Zero out control words and both buffers */
    RTC_WRITE(OFF_READ_POS,  0);
    RTC_WRITE(OFF_WRITE_POS, 0);
    for (int i = 0; i < buf_len; i++) {
        RTC_WRITE(off_buf_a + i, 0);
        RTC_WRITE(off_buf_b + i, 0);
    }

    /* Build ULP program with runtime parameters */
    const ulp_insn_t program[] = {
        /* ---- Load positions ---- */
        I_MOVI(R3, 0),     // R3 = 0, base address
        I_LD(R1, R3, OFF_READ_POS),         /* R1 = read_pos  */
        I_LD(R2, R3, OFF_WRITE_POS),        /* R2 = write_pos */

        /* ---- If read_pos==write_pos (buffer empty) then halt ---- */
        I_SUBR(R0, R1, R2),                 /* R0 = read_pos - write_pos */
        M_BL(LBL_HALT, 1),                  /* if R0 < 1 (== 0) goto halt  */

        /* ---- Output bit from buf_a[read_pos] ---- */
        I_ADDI(R0, R1, off_buf_a),
        I_LD(R0, R0, 0),                     // R0 = buf_a[read_pos]
        M_BL(LBL_A_LOW, 1),                  // if R0<1 goto A_LOW

        I_WR_REG(RTC_GPIO_OUT_REG,           // output 1
                 RTC_GPIO_OUT_DATA_S + (unsigned)rtc_a,
                 RTC_GPIO_OUT_DATA_S + (unsigned)rtc_a, 1),
        M_BX(LBL_B),

        M_LABEL(LBL_A_LOW),
        I_WR_REG(RTC_GPIO_OUT_REG,           // output 0
                 RTC_GPIO_OUT_DATA_S + (unsigned)rtc_a,
                 RTC_GPIO_OUT_DATA_S + (unsigned)rtc_a, 0),

        /* ---- Output bit from buf_b[read_pos] ---- */
        M_LABEL(LBL_B),
        I_ADDI(R0, R1, off_buf_b),           // R0 = off_buf_b + read_pos
        I_LD(R0, R0, 0),                     // R0 = buf_b[read_pos]
        M_BL(LBL_B_LOW, 1),                  // if R0<1 goto B_LOW

        I_WR_REG(RTC_GPIO_OUT_REG,           // output 1
                 RTC_GPIO_OUT_DATA_S + (unsigned)rtc_b,
                 RTC_GPIO_OUT_DATA_S + (unsigned)rtc_b, 1),
        M_BX(LBL_ADVANCE),

        M_LABEL(LBL_B_LOW),
        I_WR_REG(RTC_GPIO_OUT_REG,           // output 0
                 RTC_GPIO_OUT_DATA_S + (unsigned)rtc_b,
                 RTC_GPIO_OUT_DATA_S + (unsigned)rtc_b, 0),

        /* ---- Advance read_pos with wrap ---- */
        M_LABEL(LBL_ADVANCE),
        I_ADDI(R1, R1, 1),            // R1 += 1

        I_MOVI(R2, buf_len),          // R2 = buf_len
        I_SUBR(R0, R2, R1),           // R0 = buf_len - read_pos
        M_BGE(LBL_NO_WRAP, 1),        // if R0>=1 goto NO_WRAP

        I_MOVI(R1, 0),                // R1 = 0

        M_LABEL(LBL_NO_WRAP),
        I_ST(R1, R3, OFF_READ_POS),   // read_pos = R1

        M_LABEL(LBL_HALT),
        I_HALT(),                     // halt until next tick
    };

    /* Load ULP program */
    size_t prog_size = sizeof(program) / sizeof(ulp_insn_t);
    assert(prog_size <= MAX_PROG_SIZE);

    ESP_ERROR_CHECK(ulp_process_macros_and_load(off_program, program, &prog_size));

    /* Set ULP wakeup timer */
    ESP_ERROR_CHECK(ulp_set_wakeup_period(0, cfg.timer_us));

    return 0;
}

void start(void) {
    ESP_ERROR_CHECK(ulp_run(s_cfg.off_program));
}

void stop(void) {
    /* Disable ULP timer to stop re-triggering */
    CLEAR_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
}

/* ---------- Read current ULP read position ---------- */
volatile uint16_t get_read_pos(void) {
    return (uint16_t)RTC_WORD(OFF_READ_POS);
}

uint16_t get_write_pos(void) {
    return (uint16_t)RTC_WORD(OFF_WRITE_POS);
}

uint16_t buf_len(void) {
    return s_cfg.buf_len;
}

volatile size_t available(void) {
    size_t read_pos  = get_read_pos();
    size_t write_pos = get_write_pos();
    size_t buf_len   = s_cfg.buf_len;
    /* One slot is always kept empty to distinguish full from empty */
    return (read_pos - write_pos - 1 + buf_len) % buf_len;
}

void write(size_t len, const uint8_t *bits_a, const uint8_t *bits_b) {
    size_t pos     = get_write_pos();
    size_t buf_len = s_cfg.buf_len;

    for (size_t i = 0; i < len; i++) {
        size_t idx = (pos + i) % buf_len;
        RTC_WRITE(s_cfg.off_buf_a + idx, bits_a[i] & 1);
        RTC_WRITE(s_cfg.off_buf_b + idx, bits_b[i] & 1);
    }

    /* Advance write_pos and publish to RTC memory for ULP */
    pos = (pos + len) % buf_len;
    RTC_WRITE(OFF_WRITE_POS, pos);
}

} // namespace ulp_bitstream
