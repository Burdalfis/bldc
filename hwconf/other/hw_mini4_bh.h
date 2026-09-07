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

/* Shared zero/search-coil helpers plus the v2 R/L worker. The obsolete legacy
 * R/L worker has been removed from hw_mini4_bh_measure.inc to save MINI4 flash.
 */
#include "hw_mini4_bh_measure.inc"
#include "hw_mini4_bh_measure_v2.inc"

#define bh_run_one bh_run_one_legacy
#define bh_print_metrics bh_print_metrics_legacy
#include "hw_mini4_bh_capture.inc"
#undef bh_run_one
#undef bh_print_metrics

/* Keep capture_v2 as the base implementation, then let the live tracer wrap
 * bh_run_one/bh_print_metrics without disturbing the inductance-sweep path.
 */
#define bh_run_one bh_run_one_v2_base
#define bh_print_metrics bh_print_metrics_v2_base
#include "hw_mini4_bh_capture_v2.inc"
#undef bh_run_one
#undef bh_print_metrics

#undef timer_sleep

static bool bh_run_one(float freq, bool sine_wave, bool print_raw, bh_metrics_t *m);
static void bh_print_metrics(float freq, const bh_metrics_t *m, bool sweep);

/* Keep the original terminal command callbacks/registration as the base. The
 * obsolete legacy tracer worker has been removed from this include; queue_job
 * dispatches to worker_v2 through the bh_thread forward declaration in core.
 */
#define bh_init_commands bh_init_commands_legacy
#include "hw_mini4_bh_terminal.inc"
#undef bh_init_commands

/* bh_live's wrapper dispatches the optional fast path as well. */
static bool bh_fast_live_mode;
static bool bh_fast_live_plot(float freq);
static volatile bool bh_fast_active;
static volatile bool bh_fast_external_stop;

/* The live wrapper needs the command parser/queue helpers above and must be
 * visible before worker_v2 calls bh_run_one(). Keep its 4 kHz timing loop
 * scheduler-friendly just like the finite capture paths.
 */
#define timer_sleep(seconds) bh_rtos_sleep(seconds)
#include "hw_mini4_bh_live.inc"
#undef timer_sleep

/* The decimated acquisition include defines these with initializers below.
 * Tentative declarations let the worker-side plot wrapper snapshot them without
 * moving any transport work into the FOC callback.
 */
static volatile float bh_fast_last_h;
static volatile float bh_fast_last_b;

#define BH_FAST_PLOT_PERIOD_MS 4U
static bool bh_fast_plot_started = false;
static unsigned bh_fast_plot_elapsed_ms = 0U;

/* Frozen startup calibration state:
 *   cycle 1: estimate one residual volt-second bias; no plot
 *   cycle 2: integrate with that fixed bias removed and measure Bmin/Bmax; no plot
 *   cycle 3+: plot using the fixed B center from cycle 2
 *
 * Neither the A1 zero correction nor B center changes once plotting starts. This
 * makes a stationary magnetic loop remain stationary instead of allowing the
 * estimator itself to slowly reshape or reverse the apparent hysteresis.
 */
static volatile bool bh_fast_bias_ready = false;
static volatile bool bh_fast_center_ready = false;
static float bh_fast_cycle_vsum = 0.0f;
static unsigned bh_fast_cycle_vcount = 0U;
static float bh_fast_b_center = 0.0f;
static float bh_fast_cycle_b_min = 1.0e30f;
static float bh_fast_cycle_b_max = -1.0e30f;

/* Give fast-mode startup a harmless nonzero target so FOC is already RUNNING
 * before its callback takes over. Once the callback owns the current reference,
 * unlock ordinary mc_interface input so VESC Tool Stop/release can reach FOC.
 * Reset plot/calibration state here as this startup-only path runs once per
 * fast-live job, before the PWM callback is installed.
 */
static inline void bh_fast_apply_keeper_current(float i_line) {
    if (fabsf(i_line) < 0.001f) {
        bh_fast_plot_started = false;
        bh_fast_plot_elapsed_ms = 0U;
        bh_fast_bias_ready = false;
        bh_fast_center_ready = false;
        bh_fast_cycle_vsum = 0.0f;
        bh_fast_cycle_vcount = 0U;
        bh_fast_b_center = 0.0f;
        bh_fast_cycle_b_min = 1.0e30f;
        bh_fast_cycle_b_max = -1.0e30f;
        i_line = 0.010f;
    }
    bh_apply_current(i_line);
    mc_interface_unlock();
}

/* The fast-live worker already sleeps for 1 ms on every loop. Wrap that sleep
 * while hw_mini4_bh_fast.inc is expanded and use it as the plot pacer. This is
 * intentionally lossy: if the worker is delayed we send one current snapshot,
 * never a catch-up burst. With the normal 1 ms loop and a 4 ms period the hard
 * transport budget is approximately 250 H/B points per second.
 *
 * Do not initialize or send the plot until both hidden calibration cycles have
 * completed. The first visible point therefore already uses the frozen
 * volt-second bias estimate and fixed B origin.
 *
 * This helper is defined while ChibiOS's original sleep macro is still active,
 * so its final statement preprocesses to chThdSleep(MS2ST(sleep_ms)).
 */
static inline void bh_fast_worker_plot_sleep(unsigned sleep_ms) {
    if (bh_fast_active && bh_fast_center_ready) {
        if (!bh_fast_plot_started) {
            commands_init_plot("H (A/m)", "B relative (T)");
            commands_plot_add_graph("B-H fast live");
            commands_plot_set_graph(0);
            commands_printf("BH_FAST_PLOT enabled after frozen two-cycle bias/center calibration, budget=%u points/s",
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

/* Fast mode temporarily owns VESC's single global PWM callback. Multiplex the
 * ordinary phase-C Sampled Data overlay inside bh_fast_pwm_callback(), then
 * restore the standalone overlay when fast mode exits instead of clearing the
 * callback to NULL.
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

/* The worker locks normal motor input while configuring the fixture. Fast mode
 * deliberately unlocks once its callback owns the reference so VESC Tool Stop
 * can release the motor. Do not immediately re-lock it in the 1-ms live loop;
 * worker_v2's hard-off cleanup locks again before restoring the user's config.
 */
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

/* VESC's FOC/current PI continues running at its normal full cadence. The A1
 * Sampled Data overlay also stays full-rate. Only the tracer reference/H-B
 * acquisition work is decimated by BH_FAST_DIAG_DIV.
 */
static void bh_fast_pwm_callback(void) {
    /* Phase C is physically unused by the fixture. Preserve the existing
     * Sampled Data convention at every FOC callback: phase-C current displays
     * raw INA282 OUT relative to 1.65 V, where 1 displayed A means 1 V.
     */
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

    /* Calibration work is only active during the first two cycles. Once both
     * values are frozen, the callback stops accumulating these statistics.
     */
    if (bh_fast_have_vmed) {
        if (!bh_fast_bias_ready) {
            bh_fast_cycle_vsum += bh_fast_last_vmed;
            bh_fast_cycle_vcount++;
        } else if (!bh_fast_center_ready) {
            if (bh_fast_b < bh_fast_cycle_b_min) {
                bh_fast_cycle_b_min = bh_fast_b;
            }
            if (bh_fast_b > bh_fast_cycle_b_max) {
                bh_fast_cycle_b_max = bh_fast_b;
            }
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

        if (!bh_fast_bias_ready) {
            if (bh_fast_cycle_vcount > 0U && fabsf(bh_fast_vs_scale) > 1.0e-9f) {
                float correction_vs = bh_fast_cycle_vsum /
                        (float)bh_fast_cycle_vcount;

                /* Freeze one residual volt-second correction. Move all stored
                 * voltage history into the same new reference so the median and
                 * trapezoid filters do not see an artificial cycle-boundary step.
                 */
                bh_fast_sense_zero += correction_vs / bh_fast_vs_scale;
                bh_fast_last_vmed -= correction_vs;
                bh_fast_last_good_vs -= correction_vs;
                bh_fast_vhist[0] -= correction_vs;
                bh_fast_vhist[1] -= correction_vs;
                bh_fast_vhist[2] -= correction_vs;

                /* Cycle 1 is calibration only. Start the relative-B integral
                 * once in the corrected reference, then never alter A1 zero again.
                 */
                bh_fast_b = 0.0f;
                bh_fast_last_b = 0.0f;
                bh_fast_have_vmed = false;
                bh_fast_bias_ready = true;
                bh_fast_cycle_b_min = 1.0e30f;
                bh_fast_cycle_b_max = -1.0e30f;
            }
        } else if (!bh_fast_center_ready &&
                bh_fast_cycle_b_max > bh_fast_cycle_b_min) {
            /* Cycle 2 establishes the arbitrary integration constant exactly
             * once. No visible plot exists yet, so this fixed midpoint creates
             * no discontinuity and cannot later morph the apparent loop shape.
             */
            bh_fast_b_center = 0.5f *
                    (bh_fast_cycle_b_max + bh_fast_cycle_b_min);
            bh_fast_last_b = bh_fast_b - bh_fast_b_center;
            bh_fast_center_ready = true;
        }

        bh_fast_cycle_vsum = 0.0f;
        bh_fast_cycle_vcount = 0U;
        if (!bh_fast_center_ready) {
            bh_fast_cycle_b_min = 1.0e30f;
            bh_fast_cycle_b_max = -1.0e30f;
        }
    }
    bh_fast_phase = next_phase;

    bh_fast_set_current_ref(bh_i_pk * bh_fast_triangle(bh_fast_phase));
}

#include "hw_mini4_bh_worker_v2.inc"

/* Keep terminal_v2 intact: rename its initializer, then wrap it so the live
 * and fast-live commands are registered too. The old zero-state diagnostic
 * commands were useful while debugging timeout/PWM behavior, but leaving them
 * linked costs precious MINI4 flash.
 */
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
