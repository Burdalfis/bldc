#ifndef HW_MINI4_BH_H_
#define HW_MINI4_BH_H_

#include "utils_sys.h"
#include "conf_general.h"
#include "hw_mini4_bh_core.inc"
#include "hw_mini4_bh_policy.inc"

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
static float bh_fast_dt;
static float bh_fast_b_step_scale;

#define BH_FAST_PLOT_PERIOD_MS 4U
#define BH_FAST_STARTUP_CYCLES 16U
#define BH_FAST_A1_BIAS_CYCLES 16U
#define BH_FAST_H_PHASE_BINS 64U
#define BH_FAST_H_PHASE_ALPHA 0.125f

/* Full analog measurement-chain inverse. The external differential network is
 * 91R -> (780 nF differential) -> 91R -> INA input on each leg, with 100 nF
 * differential and 470 nF from each INA input to ground at the second node.
 * Including the INA282 ~6 kohm differential input resistance gives
 * G0=0.9428032684, Aext=252.8656065 us, Bext=8.160246260e-9 s^2. Folding the
 * INA's ~10 kHz first-order bandwidth pole into the <=200 Hz second-order
 * approximation gives A=268.7811008 us and B=1.218472738e-8 s^2.
 */
#define BH_FAST_ANALOG_INV_DC_GAIN 1.0606666667f
#define BH_FAST_ANALOG_INV_A_S     0.000268781101f
#define BH_FAST_ANALOG_INV_B_S2    1.21847274e-8f

static bool bh_fast_plot_started = false;
static unsigned bh_fast_plot_elapsed_ms = 0U;

/* Let A1-DC and phase-H estimates settle for 16 cycles, reset only B, integrate
 * one clean center cycle, then freeze the B display origin for the run.
 */
static volatile bool bh_fast_center_ready = false;
static bool bh_fast_center_cycle_active = false;
static unsigned bh_fast_startup_cycles_done = 0U;
static float bh_fast_cycle_b_sum = 0.0f;
static unsigned bh_fast_cycle_b_count = 0U;
static float bh_fast_b_center = 0.0f;

/* RAM-only A/B switch. Clearing LUT-valid bypasses the table while this flag
 * remembers that the retained RAM table can be restored without remeasuring R/L.
 */
static bool bh_fast_a1_lut_saved_valid = false;

static float bh_fast_rc_d1_coeff = 0.0f;
static float bh_fast_rc_d2_coeff = 0.0f;
static float bh_fast_rc_v_prev1 = 0.0f;
static float bh_fast_rc_v_prev2 = 0.0f;
static unsigned bh_fast_rc_v_count = 0U;
static float bh_fast_b_deembedded = 0.0f;

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

static inline void bh_fast_worker_plot_sleep(unsigned sleep_ms) {
    if (bh_fast_active && bh_fast_center_ready) {
        if (!bh_fast_plot_started) {
            commands_init_plot("H (A/m)", "B relative (T)");
            commands_plot_add_graph("B-H fast live");
            commands_plot_set_graph(0);
            commands_printf("BH_FAST_PLOT warm=%ucy A1=%ucy H=%ubin fixedB analog=on",
                    (unsigned)BH_FAST_STARTUP_CYCLES,
                    (unsigned)BH_FAST_A1_BIAS_CYCLES,
                    (unsigned)BH_FAST_H_PHASE_BINS);
            bh_fast_plot_started = true;
            bh_fast_plot_elapsed_ms = 0U;
        }
        bh_fast_plot_elapsed_ms += sleep_ms;
        if (bh_fast_plot_elapsed_ms >= BH_FAST_PLOT_PERIOD_MS) {
            bh_fast_plot_elapsed_ms = 0U;
            commands_send_plot_points(bh_fast_last_h, bh_fast_last_b);
        }
    }
    chThdSleepMilliseconds(sleep_ms);
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

    if (bh_fast_have_vmed) {
        bh_fast_cycle_vs_sum += bh_fast_last_vmed_raw;
        bh_fast_cycle_vs_count++;

        float v_now = bh_fast_last_vmed;
        float b_deembed = bh_fast_b + bh_fast_rc_d1_coeff * v_now;
        if (bh_fast_rc_v_count == 1U) {
            b_deembed += 2.0f * bh_fast_rc_d2_coeff *
                    (v_now - bh_fast_rc_v_prev1);
        } else if (bh_fast_rc_v_count >= 2U) {
            b_deembed += bh_fast_rc_d2_coeff *
                    (3.0f * v_now - 4.0f * bh_fast_rc_v_prev1 +
                     bh_fast_rc_v_prev2);
        }
        b_deembed *= BH_FAST_ANALOG_INV_DC_GAIN;

        bh_fast_rc_v_prev2 = bh_fast_rc_v_prev1;
        bh_fast_rc_v_prev1 = v_now;
        if (bh_fast_rc_v_count < 2U) {
            bh_fast_rc_v_count++;
        }

        bh_fast_b_deembedded = b_deembed;
        bh_fast_last_b = b_deembed - bh_fast_b_center;
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

/* Reuse bh_live_fast's existing callback slot for the LUT diagnostic. */
static void terminal_bh_live_fast_with_lut(int argc, const char **argv) {
    if (argc >= 2 && strcmp(argv[1], "lut") == 0) {
        if (bh_is_busy()) {
            commands_printf("BH_A1_LUT busy");
            return;
        }
        if (argc == 2) {
            commands_printf("BH_A1_LUT %d/%d",
                    bh_sense_pwm_lut_valid ? 1 : 0,
                    bh_fast_a1_lut_saved_valid ? 1 : 0);
            return;
        }
        if (argc != 3 || argv[2][1] != '\0' ||
                (argv[2][0] != '0' && argv[2][0] != '1')) {
            commands_printf("Usage: bh_live_fast lut [0|1]");
            return;
        }
        if (argv[2][0] == '0') {
            if (bh_sense_pwm_lut_valid) {
                bh_fast_a1_lut_saved_valid = true;
            }
            bh_sense_pwm_lut_valid = false;
            commands_printf("BH_A1_LUT off");
        } else if (bh_sense_pwm_lut_valid || bh_fast_a1_lut_saved_valid) {
            bh_sense_pwm_lut_valid = true;
            bh_fast_a1_lut_saved_valid = true;
            commands_printf("BH_A1_LUT on");
        } else {
            commands_printf("BH_A1_LUT no table");
        }
        return;
    }

    terminal_bh_live_fast(argc, argv);
}

#define bh_init_commands bh_init_commands_v2_base
#include "hw_mini4_bh_terminal_v2.inc"
#undef bh_init_commands
static void bh_init_commands(void) {
    bh_init_commands_v2_base();
    bh_live_init_commands();
    bh_fast_init_commands();
    /* Re-registering the same command replaces its callback without consuming
     * another terminal callback slot. MINI4 is already at the 40-slot limit.
     */
    terminal_register_command_callback(
            "bh_live_fast", "Fast B-H; 'bh_live_fast lut [0|1]' toggles A1 LUT.",
            "<Ipk_A> <freq_Hz> <path_mm> <area_mm2>",
            terminal_bh_live_fast_with_lut);
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
