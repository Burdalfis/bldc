#ifndef HW_MINI4_BH_H_
#define HW_MINI4_BH_H_

#include "utils_sys.h"
#include "conf_general.h"
#include "hw_mini4_bh_core.inc"
#include "hw_mini4_bh_policy.inc"

/*
 * timer_sleep() in VESC is a high-resolution busy-wait intended for very
 * short delays. The B-H tracer uses timing waits continuously for tens or
 * hundreds of milliseconds, and doing those as busy-waits from the tracer
 * thread can starve lower-priority housekeeping long enough to trip IWDG.
 *
 * Keep the high-resolution timer for timestamping, but make all waits in the
 * measurement/capture loops proper ChibiOS sleeps so the scheduler can run.
 */
static inline void bh_rtos_sleep(float seconds) {
    if (seconds <= 0.0f) {
        return;
    }

    uint32_t us = (uint32_t)(seconds * 1000000.0f + 0.5f);
    if (us == 0U) {
        us = 1U;
    }

    chThdSleepMicroseconds(us);
}

#define timer_sleep(seconds) bh_rtos_sleep(seconds)

/* Keep the current R/L and calibration workers. The fast B-H path below is
 * intentionally returned to the simple known-good per-cycle estimator.
 */
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

#define BH_FAST_PLOT_PERIOD_MS 4U
static bool bh_fast_plot_started = false;
static unsigned bh_fast_plot_elapsed_ms = 0U;

/* Give fast-mode startup a harmless nonzero target so FOC is already RUNNING
 * before its callback takes over. Once the callback owns the current reference,
 * unlock ordinary mc_interface input so VESC Tool Stop/release can reach FOC.
 */
static inline void bh_fast_apply_keeper_current(float i_line) {
    if (fabsf(i_line) < 0.001f) {
        bh_fast_plot_started = false;
        bh_fast_plot_elapsed_ms = 0U;
        i_line = 0.010f;
    }
    bh_apply_current(i_line);
    mc_interface_unlock();
}

/* Keep Experiment Plot worker-side and lossy, with the same 250 point/s budget
 * as the known-good baseline. No estimator state is updated from this worker.
 */
static inline void bh_fast_worker_plot_sleep(unsigned sleep_ms) {
    if (bh_fast_active) {
        if (!bh_fast_plot_started) {
            commands_init_plot("H (A/m)", "B relative (T)");
            commands_plot_add_graph("B-H fast live");
            commands_plot_set_graph(0);
            commands_printf("BH_FAST_PLOT simple per-cycle baseline, budget=%u points/s",
                    (unsigned)(1000U / BH_FAST_PLOT_PERIOD_MS));
            bh_fast_plot_started = true;
            bh_fast_plot_elapsed_ms = 0U;
        }

        bh_fast_plot_elapsed_ms += sleep_ms;
        if (bh_fast_plot_elapsed_ms >= BH_FAST_PLOT_PERIOD_MS) {
            bh_fast_plot_elapsed_ms = 0U;
            float h = bh_fast_last_h;
            float b = bh_fast_last_b;
            commands_send_plot_points(h, b);
        }
    }

    chThdSleepMilliseconds(sleep_ms);
}

/* Preserve the useful full-rate phase-C A1 Sampled Data overlay while fast mode
 * owns VESC's single PWM callback. This is diagnostic-only and does not modify
 * the B-H estimator. Restore the standalone overlay when fast mode exits.
 */
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
#undef chThdSleepMilliseconds
#define chThdSleepMilliseconds(ms) bh_fast_worker_plot_sleep((unsigned)(ms))
#include "hw_mini4_bh_fast.inc"
#undef chThdSleepMilliseconds
#define chThdSleepMilliseconds(msec) chThdSleep(MS2ST(msec))
#undef mc_interface_set_pwm_callback
#undef mc_interface_lock
#undef bh_apply_current
#undef BH_FAST_CB_DISPATCH
#undef BH_FAST_CB_DISPATCH_I
#undef BH_FAST_CB_DISPATCH_0
#undef BH_FAST_CB_DISPATCH_bh_fast_pwm_callback

/* Known-good estimator architecture:
 *   - full-rate VESC FOC/current PI;
 *   - tracer work at /2 cadence;
 *   - median measured current -> H;
 *   - hard-off-zero-referenced A1 -> median -> trapezoidal B integration;
 *   - B integration restarts at every complete excitation cycle.
 *
 * No A1 LUT, rolling DC servo, phase-bin H averaging, B-center estimator or
 * analog-chain de-embedding is active in this fast path.
 */
static void bh_fast_pwm_callback(void) {
    bh_sample_scope_pwm_cb();

    if (!bh_fast_active) {
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

    if (!bh_fast_acquire_sample()) {
        return;
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

        /* Make every B-H cycle independent. This is the key behavior from the
         * pre-regression tracer and prevents any DC integration error from
         * accumulating into later cycles.
         */
        bh_fast_b = 0.0f;
        bh_fast_last_b = 0.0f;
        bh_fast_have_vmed = false;
    }
    bh_fast_phase = next_phase;

    bh_fast_set_current_ref(bh_i_pk * bh_fast_triangle(bh_fast_phase));
}

#include "hw_mini4_bh_worker_v2.inc"

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
#undef BH_FAST_PLOT_PERIOD_MS

#endif /* HW_MINI4_BH_H_ */
