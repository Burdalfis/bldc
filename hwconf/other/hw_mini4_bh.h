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

/* Keep the original implementations compiled as fallbacks, then provide the
 * v2 policy-driven R/L worker and the diagnostic current/frequency sweep.
 */
#define bh_measure_rl_locked bh_measure_rl_locked_legacy
#include "hw_mini4_bh_measure.inc"
#undef bh_measure_rl_locked
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

/* The legacy terminal file contains a renamed, unused worker body that still
 * references the final bh_run_one()/bh_print_metrics() names. Declare those
 * wrappers before compiling that body so GCC does not create implicit int
 * declarations and later reject the real static definitions in bh_live.inc.
 */
static bool bh_run_one(float freq, bool sine_wave, bool print_raw, bh_metrics_t *m);
static void bh_print_metrics(float freq, const bh_metrics_t *m, bool sweep);

/* Keep the legacy command callbacks, but rename its worker. The queue helper
 * in capture.inc was compiled earlier against the original bh_thread forward
 * declaration, so it will dispatch to the fixed worker defined below.
 */
#define bh_thread bh_thread_legacy
#define bh_init_commands bh_init_commands_legacy
#include "hw_mini4_bh_terminal.inc"
#undef bh_init_commands
#undef bh_thread

/* bh_live's wrapper dispatches the optional PWM-rate path as well. Forward
 * declare the fast-mode state/function, compile the ordinary live wrapper,
 * then compile the implementation once the ordinary live globals exist.
 */
static bool bh_fast_live_mode;
static bool bh_fast_live_plot(float freq);
static volatile bool bh_fast_active;
static volatile bool bh_fast_external_stop;

/* The live wrapper needs the legacy command parser/queue helpers above and
 * must be visible before worker_v2 calls bh_run_one(). Keep its 4 kHz timing
 * loop scheduler-friendly just like the finite capture paths.
 */
#define timer_sleep(seconds) bh_rtos_sleep(seconds)
#include "hw_mini4_bh_live.inc"
#undef timer_sleep

/* Give the fast-mode startup call a harmless nonzero target so FOC is already
 * in MC_STATE_RUNNING before its PWM callback takes over the reference. Once
 * the callback owns the current reference, unlock ordinary mc_interface input
 * so VESC Tool's Stop/release command can actually reach FOC. Any accidental
 * non-stop motor command is overwritten again on the very next PWM callback.
 */
static inline void bh_fast_apply_keeper_current(float i_line) {
    if (fabsf(i_line) < 0.001f) {
        i_line = 0.010f;
    }
    bh_apply_current(i_line);
    mc_interface_unlock();
}

/* Fast mode sees raw phase-current samples at PWM cadence. Isolated switching
 * spikes are large enough to distort both H and peak measurements. A causal
 * 3-sample median removes one-sample outliers with only one FOC-sample delay.
 */
static inline float bh_fast_coil_current_median(void) {
    static float h0 = 0.0f;
    static float h1 = 0.0f;
    static float h2 = 0.0f;
    static unsigned n = 0U;

    float x = bh_coil_current();
    h0 = h1;
    h1 = h2;
    h2 = x;
    if (n < 3U) {
        n++;
        return x;
    }

    float a = h0, b = h1, c = h2;
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

/* CPU-load diagnostic: intentionally suppress every fast-mode Experiment Plot
 * data packet. The next diagnostic layer below also bypasses fast H/B
 * acquisition in the PWM callback, leaving only the triangle current-reference
 * generator, current tracking/sanity check, Stop handling and normal VESC FOC.
 */
static inline void bh_fast_send_plot_point(float x, float y) {
    (void)x;
    (void)y;
}

/* Run the extra B-H callback at the full FOC callback cadence. */
static inline float bh_fast_effective_sample_hz(void) {
    return mc_interface_get_sampling_frequency_now();
}

/* During the CPU diagnostic, do not also run the INA282 Sampled Data overlay
 * from the fast callback. It is restored when fast mode exits.
 */
static void bh_fast_pwm_callback(void);
static void bh_fast_pwm_callback_core(void);
static void bh_sample_scope_pwm_cb(void);
static void bh_fast_pwm_callback_mux(void) {
    bh_fast_pwm_callback();
}
static inline void bh_fast_set_pwm_callback_mux(void (*p_func)(void)) {
    if (p_func) {
        mc_interface_set_pwm_callback(bh_fast_pwm_callback_mux);
    } else {
        mc_interface_set_pwm_callback(bh_sample_scope_pwm_cb);
    }
}

/* The worker locks normal motor input while configuring the fixture. Fast mode
 * deliberately unlocks once its PWM callback owns the reference so VESC Tool
 * Stop can release the motor. Do not immediately re-lock it in the 1-ms live
 * loop; worker_v2's hard-off cleanup locks again before restoring the user's
 * configuration.
 */
#define bh_apply_current(i_line) bh_fast_apply_keeper_current(i_line)
#define bh_coil_current() bh_fast_coil_current_median()
#define commands_send_plot_points(x, y) bh_fast_send_plot_point((x), (y))
#define mc_interface_get_sampling_frequency_now() bh_fast_effective_sample_hz()
#define mc_interface_set_pwm_callback(p_func) bh_fast_set_pwm_callback_mux(p_func)
#define mc_interface_lock() ((void)0)
#define bh_fast_pwm_callback bh_fast_pwm_callback_core
#include "hw_mini4_bh_fast.inc"
#undef bh_fast_pwm_callback
#undef mc_interface_lock
#undef mc_interface_set_pwm_callback
#undef mc_interface_get_sampling_frequency_now
#undef commands_send_plot_points
#undef bh_coil_current
#undef bh_apply_current

/* CPU-load diagnostic callback: keep only the PWM-rate triangle reference and
 * measured-current safety/tracking. The original bh_fast_pwm_callback_core()
 * remains compiled above but is intentionally not called in this build.
 */
static void bh_fast_pwm_callback(void) {
    if (!bh_fast_active) {
        return;
    }

    if (mc_interface_get_state() != MC_STATE_RUNNING) {
        bh_fast_external_stop = true;
        bh_fast_active = false;
        return;
    }

    if (bh_stop_requested || mc_interface_get_fault() != FAULT_CODE_NONE) {
        bh_fast_abort_drive();
        return;
    }

    float i_now = bh_fast_coil_current_median();
    if (i_now > bh_fast_cycle_i_max) bh_fast_cycle_i_max = i_now;
    if (i_now < bh_fast_cycle_i_min) bh_fast_cycle_i_min = i_now;

    float ai = fabsf(i_now);
    float oc_lim = fmaxf(bh_i_pk * 1.50f, bh_i_pk + 0.75f);
    if (ai > oc_lim) {
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
 * and PWM-rate live commands are registered too. The old zero-state diagnostic
 * commands were useful while debugging timeout/PWM behavior, but that issue is
 * now understood and fixed; leaving them linked costs precious MINI4 flash.
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