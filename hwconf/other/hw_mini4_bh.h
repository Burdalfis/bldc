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

/* Keep the legacy command callbacks, but rename its worker. The queue helper
 * in capture.inc was compiled earlier against the original bh_thread forward
 * declaration, so it will dispatch to the fixed worker defined below.
 */
#define bh_thread bh_thread_legacy
#define bh_init_commands bh_init_commands_legacy
#include "hw_mini4_bh_terminal.inc"
#undef bh_init_commands
#undef bh_thread

/* The live wrapper needs the legacy command parser/queue helpers above and
 * must be visible before worker_v2 calls bh_run_one().
 */
#include "hw_mini4_bh_live.inc"
#include "hw_mini4_bh_worker_v2.inc"
#include "hw_mini4_bh_zero_diag.inc"

/* Keep terminal_v2 intact: rename its initializer, then wrap it so the new
 * live-loop command and static bridge-state diagnostics are registered too.
 */
#define bh_init_commands bh_init_commands_v2_base
#include "hw_mini4_bh_terminal_v2.inc"
#undef bh_init_commands
static void bh_init_commands(void) {
    bh_init_commands_v2_base();
    bh_live_init_commands();
    bh_zero_diag_init_commands();
}

#undef bh_set_measurement_gains
#undef bh_set_run_gains
#undef bh_release_locked

#endif /* HW_MINI4_BH_H_ */
