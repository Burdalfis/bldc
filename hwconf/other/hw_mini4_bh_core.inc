
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "terminal.h"
#include "commands.h"
#include "timeout.h"
#include "mempools.h"
#include "timer.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// MINI4 B-H tracer fixture:
//   Excitation winding: 20 turns, phase output 1 to phase output 2
//   Search winding:     10 turns -> INA282 -> COMM A1 / ADC_EXT1
//   INA282 gain:        50 V/V, REF1=0 V, REF2=3.3 V (about 1.65 V center)
//
// For an A-B two-terminal winding current I:
//   Ia = +I, Ib = -I, Ic = 0
// The Clarke-vector angle is -30 degrees and Id = 2/sqrt(3) * I.
// At that same fixed angle Vab = sqrt(3) * Vd, therefore the FOC d-axis
// plant sees exactly half the physical A-B line resistance and inductance.

#define BH_EXC_TURNS                 20.0f
#define BH_SENSE_TURNS               10.0f
#define BH_INA_GAIN                  50.0f
#define BH_FOC_PHASE_DEG             (-30.0f)
#define BH_SQRT3                     1.73205080757f
#define BH_TWO_BY_SQRT3              1.15470053838f

#define BH_SAMPLE_HZ                 4000.0f
#define BH_RL_SAMPLE_HZ              2000.0f
#define BH_PLOT_HZ                   100.0f
#define BH_CURRENT_BW_HZ             250.0f
#define BH_MAX_CURRENT_A             10.0f
#define BH_MIN_FREQ_HZ               0.2f
#define BH_MAX_FREQ_HZ               200.0f
#define BH_MAX_CYCLES                100
#define BH_MAX_SWEEP_POINTS          512
#define BH_MAX_HARMONIC              9
#define BH_SETTLE_CYCLES             1
#define BH_OFFSET_CYCLES             1

// Use a large but very short current pulse for the R fit. This gives much more
// V=IR signal than the old 0.25 A test without appreciably heating the 26 AWG
// fixture winding. The AC L measurement remains small-signal to avoid saturation.
#define BH_RL_DEFAULT_CURRENT_A      5.0f
#define BH_RL_AC_CURRENT_A           0.25f
#define BH_RL_DEFAULT_FREQ_HZ        50.0f
#define BH_RL_MIN_CURRENT_A          0.10f
#define BH_RL_MAX_CURRENT_A          5.0f
#define BH_RL_MIN_FREQ_HZ            5.0f
#define BH_RL_MAX_FREQ_HZ            200.0f

#define BH_A1_RAW_MIN                40
#define BH_A1_RAW_MAX                4055
#define BH_A1_HEADROOM_MIN           150
#define BH_A1_HEADROOM_MAX           3945
#define BH_A1_MAX_EVENTS_PER_SEC     10
#define BH_A1_EVENT_SLOTS            (BH_A1_MAX_EVENTS_PER_SEC + 1)

#define BH_KP_MAX                    0.20f
#define BH_KI_MAX                    200.0f


typedef enum {
	BH_JOB_NONE = 0,
	BH_JOB_MEASURE_RL,
	BH_JOB_RUN,
	BH_JOB_SWEEP_LOG,
	BH_JOB_SWEEP_LINEAR
} bh_job_t;

typedef struct {
	float h_pk;
	float b_pk;
	float b_r;
	float h_c;
	float loop_j_m3;
	float r_ac;
	float l_ac;
	float thd;
	float sense_mean_v;
	float harm[BH_MAX_HARMONIC + 1];
	int max_harmonic;
	bool valid;
} bh_metrics_t;

typedef struct {
	uint32_t stamp[BH_A1_EVENT_SLOTS];
	uint8_t next;
	uint8_t count;
} bh_a1_rate_guard_t;

// Float formatting, configuration changes and the analysis helpers can nest
// fairly deeply. Give this diagnostic thread ample stack so a test cannot turn
// a marginal stack into a hard fault/reset.
static THD_WORKING_AREA(bh_thread_wa, 4096);
static thread_t *bh_tp = 0;
static volatile bool bh_running = false;
static volatile bool bh_stop_requested = false;
static volatile bh_job_t bh_job = BH_JOB_NONE;

static float bh_i_pk = 0.0f;
static float bh_freq = 0.0f;
static float bh_fmin = 0.0f;
static float bh_fmax = 0.0f;
static float bh_sweep_step = 0.0f;
static float bh_path_m = 0.0f;
static float bh_area_m2 = 0.0f;
static int bh_cycles = 0;

static float bh_rl_current = BH_RL_DEFAULT_CURRENT_A;
static float bh_rl_freq = BH_RL_DEFAULT_FREQ_HZ;
static float bh_r_line_ohm = 0.0f;
static float bh_l_line_h = 0.0f;
static bool bh_rl_valid = false;

static bh_a1_rate_guard_t bh_a1_hard_guard;
static bh_a1_rate_guard_t bh_a1_headroom_guard;

static THD_FUNCTION(bh_thread, arg);

static inline float bh_triangle(float phase01) {
	phase01 -= floorf(phase01);
	if (phase01 < 0.25f) {
		return phase01 * 4.0f;
	} else if (phase01 < 0.75f) {
		return 2.0f - phase01 * 4.0f;
	} else {
		return phase01 * 4.0f - 4.0f;
	}
}

static inline float bh_wave(float phase01, bool sine_wave) {
	phase01 -= floorf(phase01);
	return sine_wave ? sinf(2.0f * M_PI * phase01) : bh_triangle(phase01);
}

static inline float bh_coil_current(void) {
	// Phase-shunt currents are already offset-corrected and scaled to amperes.
	return 0.5f * (ADC_curr_norm_value[0] - ADC_curr_norm_value[1]);
}

static inline float bh_sense_out_v(void) {
	return ADC_VOLTS(ADC_IND_EXT);
}

static inline float bh_line_voltage(void) {
	// Fixed -30 degree d-axis vector: Vab = sqrt(3) * Vd.
	return BH_SQRT3 * mcpwm_foc_get_vd();
}

static inline void bh_a1_rate_guard_reset(bh_a1_rate_guard_t *guard) {
	memset(guard, 0, sizeof(*guard));
}

/* Return true only when more than BH_A1_MAX_EVENTS_PER_SEC out-of-range
 * samples have occurred inside a rolling one-second span. One-off ADC spikes
 * therefore do not kill a run, while sustained rail contact still does.
 */
static inline bool bh_a1_rate_guard_record(bh_a1_rate_guard_t *guard,
		uint16_t raw, uint16_t lo, uint16_t hi) {
	if (raw >= lo && raw <= hi) {
		return false;
	}

	uint32_t now = timer_time_now();
	guard->stamp[guard->next] = now;
	guard->next++;
	if (guard->next >= BH_A1_EVENT_SLOTS) {
		guard->next = 0;
	}
	if (guard->count < BH_A1_EVENT_SLOTS) {
		guard->count++;
	}
	if (guard->count < BH_A1_EVENT_SLOTS) {
		return false;
	}

	// Once full, next points at the oldest of the last 11 excursions.
	return timer_calc_diff(guard->stamp[guard->next], now) <= 1.0f;
}

static inline bool bh_adc_sample_hard_rail(uint16_t raw) {
	return raw < BH_A1_RAW_MIN || raw > BH_A1_RAW_MAX;
}

static inline void bh_adc_clip_reset(void) {
	bh_a1_rate_guard_reset(&bh_a1_hard_guard);
}

static inline bool bh_adc_clipped(void) {
	return bh_a1_rate_guard_record(&bh_a1_hard_guard,
			ADC_Value[ADC_IND_EXT], BH_A1_RAW_MIN, BH_A1_RAW_MAX);
}

static inline void bh_adc_headroom_reset(void) {
	bh_a1_rate_guard_reset(&bh_a1_headroom_guard);
}

static inline bool bh_adc_headroom_exceeded(uint16_t raw) {
	return bh_a1_rate_guard_record(&bh_a1_headroom_guard,
			raw, BH_A1_HEADROOM_MIN, BH_A1_HEADROOM_MAX);
}

static inline void bh_apply_current(float i_line) {
	timeout_reset();
	mc_interface_lock_override_once();
	mc_interface_set_openloop_phase(i_line * BH_TWO_BY_SQRT3, BH_FOC_PHASE_DEG);
}

static void bh_release_locked(void) {
	mc_interface_lock_override_once();
	mc_interface_release_motor();
}

static void bh_common_cfg(mc_configuration *cfg, float max_line_current) {
	cfg->motor_type = MOTOR_TYPE_FOC;
	cfg->foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
	cfg->foc_cc_decoupling = FOC_CC_DECOUPLING_DISABLED;
	cfg->foc_mtpa_mode = MTPA_MODE_OFF;
	cfg->foc_fw_current_max = 0.0f;
	cfg->foc_motor_ld_lq_diff = 0.0f;
	cfg->foc_motor_flux_linkage = 1e-6f;
	cfg->foc_f_zv = 30000.0f;
	cfg->foc_control_sample_mode = FOC_CONTROL_SAMPLE_MODE_V0_V7;
	cfg->cc_min_current = 0.0f;

	float id_lim = max_line_current * BH_TWO_BY_SQRT3;
	cfg->l_current_max = fmaxf(cfg->l_current_max, id_lim * 1.25f);
	cfg->l_current_min = fminf(cfg->l_current_min, -id_lim * 1.25f);
	cfg->l_abs_current_max = fmaxf(cfg->l_abs_current_max, id_lim * 1.5f);
}

static void bh_set_measurement_gains(mc_configuration *cfg, float r_line_hint) {
	// Similar spirit to VESC's detector startup gains: intentionally gentle and
	// independent of an unknown winding L. Once R is measured, the integrator is
	// raised enough for a useful low-frequency AC impedance measurement.
	cfg->foc_current_kp = 0.020f;
	float ki = 0.5f * fmaxf(r_line_hint, 0.010f) * (2.0f * M_PI * 100.0f);
	cfg->foc_current_ki = fminf(fmaxf(ki, 1.0f), 50.0f);
}

static void bh_set_run_gains(mc_configuration *cfg) {
	// A-B line load transforms to Rd=Rline/2 and Ld=Lline/2.
	float r_d = 0.5f * bh_r_line_ohm;
	float l_d = 0.5f * bh_l_line_h;
	float wc = 2.0f * M_PI * BH_CURRENT_BW_HZ;
	float kp = l_d * wc;
	float ki = r_d * wc;

	cfg->foc_motor_r = r_d;
	cfg->foc_motor_l = l_d;
	cfg->foc_current_kp = fminf(fmaxf(kp, 0.00001f), BH_KP_MAX);
	cfg->foc_current_ki = fminf(fmaxf(ki, 0.1f), BH_KI_MAX);
}

static bool bh_wait_and_check(float seconds) {
	uint32_t start = timer_time_now();
	while (timer_seconds_elapsed_since(start) < seconds) {
		if (bh_stop_requested) {
			return false;
		}
		mc_fault_code fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			commands_printf("BH_ABORT fault=%d (%s)", (int)fault,
					mc_interface_fault_to_string(fault));
			return false;
		}
		chThdSleepMilliseconds(1);
	}
	return true;
}

static bool bh_ramp_current(float from, float to, float seconds) {
	const float dt = 0.001f;
	int steps = (int)(seconds / dt);
	if (steps < 1) {
		steps = 1;
	}
	for (int i = 1; i <= steps; i++) {
		if (bh_stop_requested) {
			return false;
		}
		float a = (float)i / (float)steps;
		bh_apply_current(from + (to - from) * a);
		mc_fault_code fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			commands_printf("BH_ABORT fault=%d (%s)", (int)fault,
					mc_interface_fault_to_string(fault));
			return false;
		}
		chThdSleepMilliseconds(1);
	}
	return true;
}

static bool bh_measure_dc_point(float target, float *i_avg, float *v_avg) {
	// The old low-current routine dwelled for almost a second per point. With a
	// 5 A R measurement that would self-heat this tiny winding and bias the fit.
	// These short ramps/holds give the current loop ample time to settle while
	// keeping each high-current pulse below about 0.1 s.
	if (!bh_ramp_current(0.0f, target, 0.020f)) {
		return false;
	}
	if (!bh_wait_and_check(0.020f)) {
		return false;
	}

	double isum = 0.0;
	double vsum = 0.0;
	const int samples = 80;
	for (int i = 0; i < samples; i++) {
		if (bh_stop_requested) {
			return false;
		}
		bh_apply_current(target);
		isum += bh_coil_current();
		vsum += bh_line_voltage();
		mc_fault_code fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			commands_printf("BH_ABORT fault=%d (%s)", (int)fault,
					mc_interface_fault_to_string(fault));
			return false;
		}
		chThdSleepMicroseconds(500);
	}

	*i_avg = (float)(isum / (double)samples);
	*v_avg = (float)(vsum / (double)samples);
	return bh_ramp_current(target, 0.0f, 0.010f);
}