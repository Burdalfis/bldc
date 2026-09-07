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

/* Give fast-mode startup a harmless nonzero target so FOC is already RUNNING
 * before its callback takes over. Once the callback owns the current reference,
 * unlock ordinary mc_interface input so VESC Tool Stop/release can reach FOC.
 */
static inline void bh_fast_apply_keeper_current(float i_line) {
    if (fabsf(i_line) < 0.001f) {
        i_line = 0.010f;
    }
    bh_apply_current(i_line);
    mc_interface_unlock();
}

/* CPU-load diagnostic: the full H/B acquisition implementation was removed
 * from hw_mini4_bh_fast.inc for this test. The only extra PWM callback is the
 * stripped current-reference/sanity path defined below.
 */
static void bh_fast_pwm_callback(void);

/* The worker locks normal motor input while configuring the fixture. Fast mode
 * deliberately unlocks once its callback owns the reference so VESC Tool Stop
 * can release the motor. Do not immediately re-lock it in the 1-ms live loop;
 * worker_v2's hard-off cleanup locks again before restoring the user's config.
 */
#define bh_apply_current(i_line) bh_fast_apply_keeper_current(i_line)
#define mc_interface_lock() ((void)0)
#include "hw_mini4_bh_fast.inc"
#undef mc_interface_lock
#undef bh_apply_current

/* CPU-load diagnostic callback. VESC's FOC/current PI continues running at its
 * normal full cadence, but this tracer-specific work runs only every
 * BH_FAST_DIAG_DIV callbacks. At div=2 and ~15 kHz FOC that is ~7.5 kHz.
 *
 * Built-in VESC current/fault protection remains full-rate. This diagnostic's
 * additional state/fault/overcurrent checks have at most one skipped FOC period
 * of extra latency at div=2.
 */
static void bh_fast_pwm_callback(void) {
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

    /* Raw measured current is sufficient for this CPU diagnostic. The previous
     * 3-sample median was useful for H/B acquisition but needlessly spent CPU
     * while we are only tracking amplitude and enforcing a coarse sanity limit.
     */
    float i_now = bh_coil_current();
    if (i_now > bh_fast_cycle_i_max) bh_fast_cycle_i_max = i_now;
    if (i_now < bh_fast_cycle_i_min) bh_fast_cycle_i_min = i_now;

    if (fabsf(i_now) > bh_fast_oc_lim) {
        bh_fast_overcurrent_count++;
        if (bh_fast_overcurrent_count >= BH_FAST_OVERCURRENT_SAMPLES) {
            bh_fast_overcurrent_abort = true;
            bh_fast_abort_drive();
            return;
        }
    } else {
        bh_fast_overcurrent_count = 0;
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

#endif /* HW_MINI4_BH_H_ */
