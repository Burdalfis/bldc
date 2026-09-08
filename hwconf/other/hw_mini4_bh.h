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
 * Tentative declarations let the worker-side plot/analog helpers use them before
 * hw_mini4_bh_fast.inc is expanded. bh_fast_dt and bh_fast_b_step_scale are
 * initialized by fast_reset_state before the keeper-current setup uses them.
 */
static volatile float bh_fast_last_h;
static volatile float bh_fast_last_b;
static float bh_fast_dt;
static float bh_fast_b_step_scale;

#define BH_FAST_PLOT_PERIOD_MS 4U
#define BH_FAST_STARTUP_CYCLES 16U
#define BH_FAST_A1_BIAS_CYCLES 16U
#define BH_FAST_H_PHASE_BINS 64U
#define BH_FAST_H_PHASE_ALPHA 0.125f

/* Full analog measurement-chain de-embedding.
 *
 * Physical differential network, per the current fixture:
 *   each leg: 91R -> node 1 -> 91R -> INA input
 *   node 1 differential C: 100 nF + 680 nF = 780 nF
 *   INA-node differential C: 100 nF
 *   INA-node common-mode C: 470 nF from each input to ground
 *   INA282 differential input resistance: approximately 6 kohm
 *
 * Under differential/odd-mode excitation the half-circuit is therefore:
 *   R1 = R2 = 91 ohm
 *   C1 = 2 * 780 nF = 1.56 uF
 *   C2 = 2 * 100 nF + 470 nF = 0.67 uF
 *   Rload = 6 kohm / 2 = 3 kohm
 *
 * Including that finite load, the external network is:
 *   Hext(s) = G0 / (1 + Aext*s + Bext*s^2)
 *   G0   = 0.9428032684
 *   Aext = 252.8656065 us
 *   Bext = 8.160246260e-9 s^2
 *
 * Approximate the INA282's roughly 10 kHz signal bandwidth as one first-order
 * pole, tau = 1/(2*pi*10 kHz) = 15.91549431 us. Multiplying that pole by Hext
 * creates a third-order denominator. Across the tracer's <=200 Hz range its
 * cubic term is negligible, so retain only the first two dynamic coefficients:
 *   A = Aext + tau        = 268.7811008 us
 *   B = Bext + Aext*tau   = 1.218472738e-8 s^2
 *
 * The resulting second-order approximation differs from the full three-pole
 * model by less than about 0.025 percent in complex response at 200 Hz while
 * avoiding a noisy third derivative in the realtime inverse.
 *
 * Search voltage is integrated to obtain B, and integration commutes with this
 * LTI chain, so de-embed measured B with:
 *   Btrue = (1/G0) * (Bmeas + A*dBmeas/dt + B*d2Bmeas/dt2).
 * dB/dt comes directly from the measured search voltage; d2B/dt2 is obtained
 * from a second-order backward difference of that already median-filtered voltage.
 */
#define BH_FAST_ANALOG_INV_DC_GAIN 1.0606666667f
#define BH_FAST_ANALOG_INV_A_S     0.000268781101f
#define BH_FAST_ANALOG_INV_B_S2    1.21847274e-8f

static bool bh_fast_plot_started = false;
static unsigned bh_fast_plot_elapsed_ms = 0U;

/* Startup deliberately produces no Experiment Plot points. First let the
 * 16-cycle A1-DC estimator and phase-synchronous H profile settle. At the next
 * cycle boundary reset only the B integrator, preserving all analog/filter
 * history. Integrate one complete clean cycle, use its mean de-embedded B as a
 * fixed display origin, then begin plotting. The B origin is never moved again
 * during that run; any remaining integrator velocity must be corrected at A1.
 */
static volatile bool bh_fast_center_ready = false;
static bool bh_fast_center_cycle_active = false;
static unsigned bh_fast_startup_cycles_done = 0U;
static float bh_fast_cycle_b_sum = 0.0f;
static unsigned bh_fast_cycle_b_count = 0U;
static float bh_fast_b_center = 0.0f;

/* Diagnostic A/B switch for the RAM-only current-indexed A1 feedthrough LUT.
 * bh_measure_rl normally makes bh_sense_pwm_lut_valid true. The command below
 * can temporarily clear that validity bit while retaining the table contents,
 * then restore it without re-running the R/L measurement.
 */
static bool bh_fast_a1_lut_saved_valid = false;

/* Analog inverse runtime state. Coefficients are derived once after
 * fast_reset_state has established dt and the B integration scale, so the
 * PWM-rate path only performs multiplies/adds. bh_fast_rc_d2_coeff multiplies
 * the second-order backward-difference numerator (3*v[n]-4*v[n-1]+v[n-2]).
 */
static float bh_fast_rc_d1_coeff = 0.0f;
static float bh_fast_rc_d2_coeff = 0.0f;
static float bh_fast_rc_v_prev1 = 0.0f;
static float bh_fast_rc_v_prev2 = 0.0f;
static unsigned bh_fast_rc_v_count = 0U;
static float bh_fast_b_deembedded = 0.0f;

/* Give fast-mode startup a harmless nonzero target so FOC is already RUNNING
 * before its callback takes over. Once the callback owns the current reference,
 * unlock ordinary mc_interface input so VESC Tool Stop/release can reach FOC.
 * Reset plot/center/filter state here. bh_fast_reset_state() has already run by
 * the time this startup-only zero-current call is made, so dt/B scale are valid.
 */
static inline void bh_fast_apply_keeper_current(float i_line) {
    if (fabsf(i_line) < 0.001f) {
        bh_fast_plot_started = false;
        bh_fast_plot_elapsed_ms = 0U;
        bh_fast_center_ready = false;
        bh_fast_center_cycle_active = false;
        bh_fast_startup_cycles_done = 0U;
        bh_fast_cycle_b_sum = 0.0f;
        bh_fast_cycle_b_count = 0U;
        bh_fast_b_center = 0.0f;

        bh_fast_rc_v_prev1 = 0.0f;
        bh_fast_rc_v_prev2 = 0.0f;
        bh_fast_rc_v_count = 0U;
        bh_fast_b_deembedded = 0.0f;
        if (bh_fast_dt > 1.0e-9f) {
            float inv_na = 2.0f * bh_fast_b_step_scale / bh_fast_dt;
            bh_fast_rc_d1_coeff = BH_FAST_ANALOG_INV_A_S * inv_na;
            bh_fast_rc_d2_coeff = BH_FAST_ANALOG_INV_B_S2 * inv_na /
                    (2.0f * bh_fast_dt);
        } else {
            bh_fast_rc_d1_coeff = 0.0f;
            bh_fast_rc_d2_coeff = 0.0f;
        }

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
 * Plotting is suppressed for 16 estimator-settling cycles plus one dedicated
 * B-center cycle. Once enabled, the B display center is fixed for the run.
 */
static inline void bh_fast_worker_plot_sleep(unsigned sleep_ms) {
    if (bh_fast_active && bh_fast_center_ready) {
        if (!bh_fast_plot_started) {
            commands_init_plot("H (A/m)", "B relative (T)");
            commands_plot_add_graph("B-H fast live");
            commands_plot_set_graph(0);
            commands_printf("BH_FAST_PLOT startup=%ucy+1center A1dc=%ucy Havg=%ubin/~8cy fixed-B-center budget=%u points/s analog-deembed=on",
                    (unsigned)BH_FAST_STARTUP_CYCLES,
                    (unsigned)BH_FAST_A1_BIAS_CYCLES,
                    (unsigned)BH_FAST_H_PHASE_BINS,
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

    if (bh_fast_have_vmed) {
        /* Track the residual corrected-A1/search-voltage center separately from
         * B. This is the quantity that causes integrator velocity, so removing
         * its whole-cycle rolling mean fixes drift at the source. Accumulate the
         * pre-servo median here; bh_fast_acquire_sample subtracts the previous
         * cycle's frozen bh_fast_vs_bias before integrating.
         */
        bh_fast_cycle_vs_sum += bh_fast_last_vmed_raw;
        bh_fast_cycle_vs_count++;

        /* Undo the loaded external RC network plus the INA282 bandwidth on B.
         * bh_fast_last_vmed is the current median-filtered, DC-servoed search
         * voltage after INA gain and current-indexed feedthrough correction.
         */
        float v_now = bh_fast_last_vmed;
        float b_deembed = bh_fast_b + bh_fast_rc_d1_coeff * v_now;

        if (bh_fast_rc_v_count == 1U) {
            /* First derivative estimate available: use first-order backward
             * difference. Factor 2 converts the stored 1/(2*dt) coefficient.
             */
            b_deembed += 2.0f * bh_fast_rc_d2_coeff *
                    (v_now - bh_fast_rc_v_prev1);
        } else if (bh_fast_rc_v_count >= 2U) {
            b_deembed += bh_fast_rc_d2_coeff *
                    (3.0f * v_now - 4.0f * bh_fast_rc_v_prev1 +
                     bh_fast_rc_v_prev2);
        }

        /* The INA input resistance causes a real DC attenuation as well as
         * changing the RC poles, so restore the full chain's 1/G0 gain after
         * the dynamic inverse terms have been reconstructed.
         */
        b_deembed *= BH_FAST_ANALOG_INV_DC_GAIN;

        bh_fast_rc_v_prev2 = bh_fast_rc_v_prev1;
        bh_fast_rc_v_prev1 = v_now;
        if (bh_fast_rc_v_count < 2U) {
            bh_fast_rc_v_count++;
        }

        bh_fast_b_deembedded = b_deembed;
        bh_fast_last_b = b_deembed - bh_fast_b_center;

        /* Only the one dedicated center cycle contributes to the fixed display
         * origin. Warm-up cycles and plotted cycles never move B center.
         */
        if (bh_fast_center_cycle_active) {
            bh_fast_cycle_b_sum += b_deembed;
            bh_fast_cycle_b_count++;
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

        /* Commit one current-vs-phase profile per completed magnetic cycle. This
         * is plot-only smoothing; protection and control have already used raw
         * measured current during the cycle that just ended.
         */
        bh_fast_h_phase_commit_cycle();

        if (bh_fast_cycle_vs_count > 0U) {
            float cycle_mean_vs = bh_fast_cycle_vs_sum /
                    (float)bh_fast_cycle_vs_count;

            if (bh_fast_vs_bias_hist_count < BH_FAST_A1_BIAS_CYCLES) {
                bh_fast_vs_bias_hist[bh_fast_vs_bias_hist_next] = cycle_mean_vs;
                bh_fast_vs_bias_hist_sum += cycle_mean_vs;
                bh_fast_vs_bias_hist_count++;
            } else {
                bh_fast_vs_bias_hist_sum -=
                        bh_fast_vs_bias_hist[bh_fast_vs_bias_hist_next];
                bh_fast_vs_bias_hist[bh_fast_vs_bias_hist_next] = cycle_mean_vs;
                bh_fast_vs_bias_hist_sum += cycle_mean_vs;
            }

            bh_fast_vs_bias_hist_next++;
            if (bh_fast_vs_bias_hist_next >= BH_FAST_A1_BIAS_CYCLES) {
                bh_fast_vs_bias_hist_next = 0U;
            }

            float new_vs_bias = bh_fast_vs_bias_hist_sum /
                    (float)bh_fast_vs_bias_hist_count;
            float delta_vs_bias = new_vs_bias - bh_fast_vs_bias;
            bh_fast_vs_bias = new_vs_bias;

            /* The bias is frozen for the entire next cycle. Shift only states
             * expressed in the post-bias reference so the cycle-boundary update
             * does not create a fake trapezoid or analog-inverse derivative impulse.
             * vhist and last_good_vs remain intentionally pre-bias.
             */
            if (bh_fast_have_vmed) {
                bh_fast_last_vmed -= delta_vs_bias;
            }
            if (bh_fast_rc_v_count >= 1U) {
                bh_fast_rc_v_prev1 -= delta_vs_bias;
            }
            if (bh_fast_rc_v_count >= 2U) {
                bh_fast_rc_v_prev2 -= delta_vs_bias;
            }
        }

        /* No B points are plotted while the A1-DC and H estimators fill. At the
         * end of the 16th warm-up cycle, throw away only the accumulated B state
         * and begin one dedicated clean center cycle. Preserve A1/median/analog
         * history so this reset cannot create a fresh filter transient.
         */
        if (!bh_fast_center_ready) {
            if (bh_fast_center_cycle_active) {
                if (bh_fast_cycle_b_count > 0U) {
                    bh_fast_b_center = bh_fast_cycle_b_sum /
                            (float)bh_fast_cycle_b_count;
                    bh_fast_center_ready = true;
                    bh_fast_center_cycle_active = false;
                    bh_fast_last_b = bh_fast_b_deembedded - bh_fast_b_center;
                }
            } else {
                bh_fast_startup_cycles_done++;
                if (bh_fast_startup_cycles_done >= BH_FAST_STARTUP_CYCLES) {
                    bh_fast_b = 0.0f;
                    bh_fast_b_deembedded = 0.0f;
                    bh_fast_last_b = 0.0f;
                    bh_fast_cycle_b_sum = 0.0f;
                    bh_fast_cycle_b_count = 0U;
                    bh_fast_center_cycle_active = true;
                }
            }
        }

        bh_fast_cycle_vs_sum = 0.0f;
        bh_fast_cycle_vs_count = 0U;
        bh_fast_cycle_b_sum = 0.0f;
        bh_fast_cycle_b_count = 0U;
    }
    bh_fast_phase = next_phase;

    bh_fast_set_current_ref(bh_i_pk * bh_fast_triangle(bh_fast_phase));
}

#include "hw_mini4_bh_worker_v2.inc"

/* RAM-only diagnostic switch for A/B testing the current-indexed A1 LUT after
 * bh_measure_rl has established both R/L and the table. Disabling does not touch
 * bh_rl_valid or erase the LUT arrays, so it can be restored immediately.
 */
static void terminal_bh_a1_lut(int argc, const char **argv) {
    if (bh_is_busy()) {
        commands_printf("A B-H job is running or queued. Use bh_stop before changing A1 LUT state.");
        return;
    }

    if (argc == 1) {
        commands_printf("BH_A1_LUT %s saved=%s",
                bh_sense_pwm_lut_valid ? "enabled" : "disabled",
                bh_fast_a1_lut_saved_valid ? "yes" : "no");
        return;
    }

    if (argc != 2 || (strcmp(argv[1], "0") != 0 && strcmp(argv[1], "1") != 0)) {
        commands_printf("Usage: bh_a1_lut [0|1]");
        commands_printf("  0 = bypass current-indexed A1 feedthrough LUT; 1 = restore it");
        return;
    }

    if (argv[1][0] == '0') {
        if (bh_sense_pwm_lut_valid) {
            bh_fast_a1_lut_saved_valid = true;
        }
        bh_sense_pwm_lut_valid = false;
        commands_printf("BH_A1_LUT disabled; R/L state retained and LUT samples left in RAM");
    } else {
        if (bh_sense_pwm_lut_valid) {
            bh_fast_a1_lut_saved_valid = true;
            commands_printf("BH_A1_LUT enabled");
        } else if (bh_fast_a1_lut_saved_valid) {
            bh_sense_pwm_lut_valid = true;
            commands_printf("BH_A1_LUT enabled from retained RAM table");
        } else {
            commands_printf("BH_A1_LUT cannot enable: no retained valid LUT; run bh_measure_rl first");
        }
    }
}

/* Keep terminal_v2 intact: rename its initializer, then wrap it so the live,
 * fast-live and A1-LUT diagnostic commands are registered too.
 */
#define bh_init_commands bh_init_commands_v2_base
#include "hw_mini4_bh_terminal_v2.inc"
#undef bh_init_commands
static void bh_init_commands(void) {
    bh_init_commands_v2_base();
    bh_live_init_commands();
    bh_fast_init_commands();
    terminal_register_command_callback(
            "bh_a1_lut",
            "Enable/bypass the retained current-indexed A1 feedthrough LUT.",
            "[0|1]",
            terminal_bh_a1_lut);
}

#undef bh_set_measurement_gains
#undef bh_set_run_gains
#undef bh_release_locked
#undef BH_FAST_ANALOG_INV_B_S2
#undef BH_FAST_ANALOG_INV_A_S
#undef BH_FAST_ANALOG_INV_DC_GAIN
#undef BH_FAST_STARTUP_CYCLES
#undef BH_FAST_PLOT_PERIOD_MS
#undef BH_FAST_H_PHASE_ALPHA
#undef BH_FAST_H_PHASE_BINS
#undef BH_FAST_A1_BIAS_CYCLES

#endif /* HW_MINI4_BH_H_ */
