#ifndef HW_MINI4_BH_H_
#define HW_MINI4_BH_H_

#include "utils_sys.h"
#include "conf_general.h"
#include "hw_mini4_bh_core.inc"
#include "hw_mini4_bh_policy.inc"

static inline void bh_rtos_sleep(float seconds) {
    if (seconds <= 0.0f) return;
    uint32_t us = (uint32_t)(seconds * 1000000.0f + 0.5f);
    if (us == 0U) us = 1U;
    chThdSleepMicroseconds(us);
}

#define timer_sleep(seconds) bh_rtos_sleep(seconds)
#include "hw_mini4_bh_measure.inc"
#include "hw_mini4_bh_measure_v2.inc"

#define bh_run_one bh_run_one_legacy
#define bh_print_metrics bh_print_metrics_legacy
#include "hw_mini4_bh_capture.inc"
#undef bh_run_one
#undef bh_print_metrics

#define bh_run_one bh_run_one_v2_base
#define bh_print_metrics bh_print_metrics_v2_base
#include "hw_mini4_bh_capture_v2.inc"
#undef bh_run_one
#undef bh_print_metrics
#undef timer_sleep

static bool bh_run_one(float freq, bool sine_wave, bool print_raw, bh_metrics_t *m);
static void bh_print_metrics(float freq, const bh_metrics_t *m, bool sweep);

#define bh_init_commands bh_init_commands_legacy
#include "hw_mini4_bh_terminal.inc"
#undef bh_init_commands

static bool bh_fast_live_mode;
static bool bh_fast_live_plot(float freq);
static volatile bool bh_fast_active;
static volatile bool bh_fast_external_stop;

#define timer_sleep(seconds) bh_rtos_sleep(seconds)
#include "hw_mini4_bh_live.inc"
#undef timer_sleep

static volatile float bh_fast_last_h;
static volatile float bh_fast_last_b;

#define BH_FAST_PLOT_PERIOD_MS          4U
#define BH_FAST_DIAG_REPORT_MS          1000U
#define BH_FAST_DUTY_LINE_LIMIT_A       5.0f

static float bh_fast_duty_pk = 0.0f;
static volatile float bh_fast_last_duty_cmd = 0.0f;
static bool bh_fast_plot_started;
static unsigned bh_fast_plot_elapsed_ms;

static float bh_fast_diag_vs_sum = 0.0f;
static unsigned bh_fast_diag_vs_count = 0U;
static float bh_fast_diag_b_min_work = 0.0f;
static float bh_fast_diag_b_max_work = 0.0f;
static bool bh_fast_diag_have_b = false;
static volatile unsigned bh_fast_diag_cycle = 0U;
static volatile float bh_fast_diag_i_wrap = 0.0f;
static volatile float bh_fast_diag_duty_wrap = 0.0f;
static volatile float bh_fast_diag_b_close = 0.0f;
static volatile float bh_fast_diag_b_min = 0.0f;
static volatile float bh_fast_diag_b_max = 0.0f;
static volatile float bh_fast_diag_vs_mean = 0.0f;
static unsigned bh_fast_diag_report_elapsed_ms = 0U;
static unsigned bh_fast_diag_last_reported_cycle = 0U;

static inline void bh_fast_diag_reset_work(void) {
    bh_fast_diag_vs_sum = 0.0f;
    bh_fast_diag_vs_count = 0U;
    bh_fast_diag_b_min_work = 0.0f;
    bh_fast_diag_b_max_work = 0.0f;
    bh_fast_diag_have_b = false;
}

static inline void bh_fast_apply_keeper_current(float i_line) {
    if (fabsf(i_line) < 0.001f) {
        bh_fast_plot_started = false;
        bh_fast_plot_elapsed_ms = 0U;
        bh_fast_diag_report_elapsed_ms = 0U;
        bh_fast_diag_last_reported_cycle = 0U;
        bh_fast_diag_cycle = 0U;
        bh_fast_last_duty_cmd = 0.0f;
        bh_fast_diag_reset_work();
        i_line = 0.010f;
    }
    bh_apply_current(i_line);
    mc_interface_unlock();
}

static bool bh_fast_plot_started = false;
static unsigned bh_fast_plot_elapsed_ms = 0U;
static inline void bh_fast_worker_plot_sleep(unsigned sleep_ms) {
    if (bh_fast_active) {
        if (!bh_fast_plot_started) {
            commands_init_plot("H (A/m)", "B relative (T)");
            commands_plot_add_graph("B-H fast live");
            commands_plot_set_graph(0);
            commands_printf("BH_FAST_PLOT duty baseline, budget=%u points/s",
                    (unsigned)(1000U / BH_FAST_PLOT_PERIOD_MS));
            bh_fast_plot_started = true;
            bh_fast_plot_elapsed_ms = 0U;
        }

        bh_fast_plot_elapsed_ms += sleep_ms;
        if (bh_fast_plot_elapsed_ms >= BH_FAST_PLOT_PERIOD_MS) {
            bh_fast_plot_elapsed_ms = 0U;
            commands_send_plot_points(bh_fast_last_h, bh_fast_last_b);
        }

        bh_fast_diag_report_elapsed_ms += sleep_ms;
        if (bh_fast_diag_report_elapsed_ms >= BH_FAST_DIAG_REPORT_MS) {
            bh_fast_diag_report_elapsed_ms = 0U;
            unsigned cy = bh_fast_diag_cycle;
            if (cy != 0U && cy != bh_fast_diag_last_reported_cycle) {
                float vs_mean = bh_fast_diag_vs_mean;
                commands_printf("BH_FAST_CYCLE n=%u Iwrap=%+.5fA Dwrap=%+.6f Bclose=%+.7fT Bmin=%+.7fT Bmax=%+.7fT Vsmean=%+.3fuV A1mean=%+.3fmV",
                        cy,
                        (double)bh_fast_diag_i_wrap,
                        (double)bh_fast_diag_duty_wrap,
                        (double)bh_fast_diag_b_close,
                        (double)bh_fast_diag_b_min,
                        (double)bh_fast_diag_b_max,
                        (double)(vs_mean * 1.0e6f),
                        (double)(vs_mean * BH_INA_GAIN * 1.0e3f));
                bh_fast_diag_last_reported_cycle = cy;
            }
        }
    }
    chThdSleepMilliseconds(sleep_ms);
}

static inline void bh_fast_keep_foc_running(float i_axis_unused, float phase_unused) {
    (void)i_axis_unused;
    (void)phase_unused;
    mcpwm_foc_set_openloop_phase(0.010f * BH_TWO_BY_SQRT3, BH_FOC_PHASE_DEG);
}

static void terminal_bh_live_fast_duty(int argc, const char **argv);

static inline void bh_fast_register_duty_command(const char *command,
        const char *help, const char *arg_names,
        void(*cbf)(int argc, const char **argv)) {
    (void)help;
    (void)arg_names;
    (void)cbf;
    terminal_register_command_callback(command,
            "Duty B-H tracer.",
            "<D> <f> <path> <area>",
            terminal_bh_live_fast_duty);
}

static void bh_fast_pwm_callback(void);
static void bh_sample_scope_pwm_cb(void);
static inline void bh_fast_install_pwm_callback(void) {
    mc_interface_set_pwm_callback(bh_fast_pwm_callback);
}
static inline void bh_fast_restore_pwm_callback(void) {
    mc_interface_set_pwm_callback(bh_sample_scope_pwm_cb);
}
#define BH_FAST_CB_DISPATCH_bh_fast_pwm_callback() bh_fast_install_pwm_callback()
#define BH_FAST_CB_DISPATCH_0() bh_fast_restore_pwm_callback()
#define BH_FAST_CB_DISPATCH_I(x) BH_FAST_CB_DISPATCH_##x()
#define BH_FAST_CB_DISPATCH(x) BH_FAST_CB_DISPATCH_I(x)

#define bh_apply_current(i_line) bh_fast_apply_keeper_current(i_line)
#define mc_interface_lock() ((void)0)
#define mc_interface_set_pwm_callback(p_func) BH_FAST_CB_DISPATCH(p_func)
#define mcpwm_foc_set_openloop_phase(i_axis, phase) \
        bh_fast_keep_foc_running((i_axis), (phase))
#define terminal_register_command_callback(command, help, arg_names, cbf) \
        bh_fast_register_duty_command((command), (help), (arg_names), (cbf))
#undef chThdSleepMilliseconds
#define chThdSleepMilliseconds(ms) bh_fast_worker_plot_sleep((unsigned)(ms))
#include "hw_mini4_bh_fast.inc"
#undef chThdSleepMilliseconds
#define chThdSleepMilliseconds(msec) chThdSleep(MS2ST(msec))
#undef terminal_register_command_callback
#undef mcpwm_foc_set_openloop_phase
#undef mc_interface_set_pwm_callback
#undef mc_interface_lock
#undef bh_apply_current
#undef BH_FAST_CB_DISPATCH
#undef BH_FAST_CB_DISPATCH_I
#undef BH_FAST_CB_DISPATCH_0
#undef BH_FAST_CB_DISPATCH_bh_fast_pwm_callback

static void bh_fast_pwm_callback(void) {
    bh_sample_scope_pwm_cb();

    if (!bh_fast_active) return;

    if (fabsf(bh_coil_current()) > BH_FAST_DUTY_LINE_LIMIT_A) {
        bh_foc_svm_override_alpha = 0.0f;
        bh_foc_svm_override_beta = 0.0f;
        bh_fast_last_duty_cmd = 0.0f;
        bh_fast_overcurrent_abort = true;
        bh_fast_active = false;
        return;
    }

    if (bh_fast_diag_div_count != 0U) {
        bh_fast_diag_div_count--;
        return;
    }
    bh_fast_diag_div_count = BH_FAST_DIAG_DIV - 1U;

    if (mc_interface_get_state() != MC_STATE_RUNNING) {
        bh_fast_external_stop = true;
        bh_fast_active = false;
        return;
    }

    if (bh_stop_requested || mc_interface_get_fault() != FAULT_CODE_NONE) {
        bh_fast_abort_drive();
        return;
    }

    if (!bh_fast_acquire_sample()) return;

    if (bh_fast_have_vmed) {
        float b = bh_fast_b;
        bh_fast_diag_vs_sum += bh_fast_last_vmed;
        bh_fast_diag_vs_count++;
        if (!bh_fast_diag_have_b) {
            bh_fast_diag_b_min_work = b;
            bh_fast_diag_b_max_work = b;
            bh_fast_diag_have_b = true;
        } else {
            if (b < bh_fast_diag_b_min_work) bh_fast_diag_b_min_work = b;
            if (b > bh_fast_diag_b_max_work) bh_fast_diag_b_max_work = b;
        }
    }

    bh_fast_sample_n++;
    float next_phase = bh_fast_phase + bh_fast_phase_step;
    if (next_phase >= 1.0f) {
        bh_fast_last_amp = 0.5f * (bh_fast_cycle_i_max - bh_fast_cycle_i_min);
        bh_fast_last_center = 0.5f * (bh_fast_cycle_i_max + bh_fast_cycle_i_min);
        bh_fast_cycle_n++;
        bh_fast_cycle_i_max = -1.0e30f;
        bh_fast_cycle_i_min = 1.0e30f;
        bh_fast_sample_n = 0;
        next_phase -= 1.0f;

        bh_fast_diag_cycle = (unsigned)bh_fast_cycle_n;
        bh_fast_diag_i_wrap = fabsf(bh_fast_h_scale) > 1.0e-9f ?
                bh_fast_last_h / bh_fast_h_scale : bh_coil_current();
        bh_fast_diag_duty_wrap = bh_fast_last_duty_cmd;
        bh_fast_diag_b_close = bh_fast_b;
        bh_fast_diag_b_min = bh_fast_diag_have_b ? bh_fast_diag_b_min_work : 0.0f;
        bh_fast_diag_b_max = bh_fast_diag_have_b ? bh_fast_diag_b_max_work : 0.0f;
        bh_fast_diag_vs_mean = bh_fast_diag_vs_count > 0U ?
                bh_fast_diag_vs_sum / (float)bh_fast_diag_vs_count : 0.0f;
        bh_fast_diag_reset_work();

        bh_fast_b = 0.0f;
        bh_fast_last_b = 0.0f;
        bh_fast_have_vmed = false;
    }
    bh_fast_phase = next_phase;

    bh_fast_set_current_ref(bh_i_pk * bh_fast_triangle(bh_fast_phase));
}

#include "hw_mini4_bh_worker_v2.inc"

static void terminal_bh_live_fast_duty(int argc, const char **argv) {
    if (bh_is_busy()) {
        commands_printf("BH busy; use bh_stop");
        return;
    }

    /* Reuse this callback slot for the A/B test. Turning the LUT off only
     * clears its valid flag; the measured arrays stay in RAM. Enabling is
     * allowed only if the retained current axis still looks like a valid table,
     * so a failed bh_measure_rl (which clears the arrays) cannot resurrect one.
     */
    if (argc == 3 && strcmp(argv[1], "lut") == 0) {
        int en = atoi(argv[2]) != 0;
        if (!en) {
            bh_sense_pwm_lut_valid = false;
        } else if (bh_sense_pwm_i_lut[0] < bh_sense_pwm_i_lut[10]) {
            bh_sense_pwm_lut_valid = true;
        }
        commands_printf("BH_A1_LUT %d", bh_sense_pwm_lut_valid ? 1 : 0);
        return;
    }

    if (argc != 5) {
        commands_printf("Use: bh_live_fast D f path area | lut 0/1");
        return;
    }

    float duty_pk = strtof(argv[1], 0);
    float freq = strtof(argv[2], 0);
    float path_mm = strtof(argv[3], 0);
    float area_mm2 = strtof(argv[4], 0);

    if (!bh_rl_valid) {
        commands_printf("Run bh_measure_rl first");
        return;
    }
    if (!(duty_pk >= 0.0f && duty_pk <= 1.0f) ||
            !(freq >= BH_MIN_FREQ_HZ && freq <= BH_MAX_FREQ_HZ) ||
            !(path_mm > 0.0f) || !(area_mm2 > 0.0f)) {
        commands_printf("Bad duty/frequency/geometry");
        return;
    }

    bh_fast_duty_pk = duty_pk;
    bh_i_pk = BH_FAST_DUTY_LINE_LIMIT_A / 1.5f;
    bh_path_m = path_mm * 1.0e-3f;
    bh_area_m2 = area_mm2 * 1.0e-6f;
    bh_cycles = 1;
    bh_freq = freq;
    bh_fast_live_mode = true;
    bh_live_mode = false;
    bh_live_suppress_metric = false;
    if (!bh_queue_job(BH_JOB_RUN)) {
        bh_fast_live_mode = false;
        return;
    }

    commands_printf("BH duty D=%.6f f=%.3f Iabort=%.1fA",
            (double)duty_pk, (double)freq, (double)BH_FAST_DUTY_LINE_LIMIT_A);
}

#define bh_init_commands bh_init_commands_v2_base
#include "hw_mini4_bh_terminal_v2.inc"
#undef bh_init_commands
static void bh_init_commands(void) {
    bh_init_commands_v2_base();
    bh_live_init_commands();
    bh_fast_init_commands();
}

#undef bh_set_measurement_gains
#undef bh_set_run_gains
#undef bh_release_locked
#undef BH_FAST_DUTY_LINE_LIMIT_A
#undef BH_FAST_DIAG_REPORT_MS
#undef BH_FAST_PLOT_PERIOD_MS

#endif /* HW_MINI4_BH_H_ */