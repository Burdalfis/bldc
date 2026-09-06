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

/* The live wrapper needs the legacy command parser/queue helpers above and
 * must be visible before worker_v2 calls bh_run_one(). Keep its 4 kHz timing
 * loop scheduler-friendly just like the finite capture paths.
 */
#define timer_sleep(seconds) bh_rtos_sleep(seconds)
#include "hw_mini4_bh_live.inc"
#undef timer_sleep

/* bh_live_fast owns the actual bridge voltage below VESC's current controller,
 * but FOC's housekeeping still looks at its ordinary current target when
 * deciding whether the motor should remain MC_STATE_RUNNING. A literal 0-A
 * target can therefore make FOC auto-release even while the raw SVM override
 * is producing the requested fixture voltage; bh_live_fast then mistakes that
 * state exit for VESC Tool Stop.
 *
 * Give only the fast-mode startup call a harmless 10 mA line-current keeper.
 * The SVM override still completely determines the bridge waveform, while the
 * dormant current loop now has a nonzero target that prevents idle auto-off.
 * A real Stop/release command overwrites that target, so external Stop remains
 * detectable. If the SVM override ever failed to engage, the fallback command
 * is only 10 mA and is therefore intrinsically gentle.
 */
static inline void bh_fast_apply_keeper_current(float i_line) {
    if (fabsf(i_line) < 0.001f) {
        i_line = 0.010f;
    }
    bh_apply_current(i_line);
}

/* Fast mode sees raw phase-current samples at PWM cadence. Isolated switching
 * spikes were large enough to fool the cycle peak tracker even when the useful
 * triangle was well behaved. A causal 3-sample median removes one-sample
 * outliers without materially changing a 10..200 Hz waveform at 30 kHz.
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

/* Experiment Plot packets are comparatively expensive. The fast acquisition
 * buffers hundreds of points per selected cycle; sending them as one tight
 * burst can monopolize the USB/packet path for long enough to starve Sampled
 * Data and even the motor timeout. Keep roughly one quarter of the already
 * decimated plot points and yield 2 ms between transmitted points. Acquisition
 * and B integration remain at the full PWM rate; this affects display traffic
 * only. About 90 points per loop at 10 Hz is still plenty to see loop shape.
 */
static unsigned bh_fast_plot_tx_decim = 0U;
static inline void bh_fast_send_plot_point(float x, float y) {
    if ((bh_fast_plot_tx_decim++ & 3U) == 0U) {
        commands_send_plot_points(x, y);
        chThdSleepMilliseconds(2);
    }
}

#define bh_apply_current(i_line) bh_fast_apply_keeper_current(i_line)
#define bh_coil_current() bh_fast_coil_current_median()
#define commands_send_plot_points(x, y) bh_fast_send_plot_point((x), (y))
#include "hw_mini4_bh_fast.inc"
#undef commands_send_plot_points
#undef bh_coil_current
#undef bh_apply_current

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