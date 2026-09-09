/* PWM-rate B-H live mode.
 *
 * VESC's FOC machinery continues at full rate while the MINI4 raw-SVM hook
 * supplies a fixed A-B voltage vector. A1/current acquisition, median impulse
 * rejection, H calculation and per-cycle B integration run at /2 cadence.
 */

#define BH_FAST_OVERCURRENT_SAMPLES       1
#define BH_FAST_HARD_CLIPS_PER_SEC        10
#define BH_FAST_DIAG_DIV                  2U
#define BH_FAST_A1_ZERO_SAMPLES           40U

/* Actual search-input chain:
 *   91R -> (100nF || 680nF differential) -> 91R
 *       -> 100nF differential + 1uF from EACH INA input to ground
 *   INA282 Rin ~= 6k differential, BW ~= 10kHz.
 * Odd-mode half-circuit: C1=1.56uF, C2=1.20uF, Rload=3k.
 * Including the INA pole and dropping only the negligible cubic term:
 *   H(s)=G0/(1+A*s+B*s^2)
 */
#define BH_FAST_A1_INV_GAIN       1.0606666667f
#define BH_FAST_A1_INV_A_S        0.000359723904f
#define BH_FAST_A1_INV_B_S2       2.00872472e-8f

volatile bool bh_foc_svm_override_active = false;
volatile float bh_foc_svm_override_alpha = 0.0f;
volatile float bh_foc_svm_override_beta = 0.0f;

static bool bh_fast_live_mode = false;
static volatile bool bh_fast_active = false;
static volatile bool bh_fast_external_stop = false;
static volatile bool bh_fast_clip_abort = false;
static volatile bool bh_fast_overcurrent_abort = false;

static volatile float bh_fast_last_amp = 0.0f;
static volatile float bh_fast_last_center = 0.0f;
static volatile float bh_fast_last_iref = 0.0f;
static volatile float bh_fast_last_h = 0.0f;
static volatile float bh_fast_last_b = 0.0f;
static volatile unsigned bh_fast_hard_clips = 0;

static float bh_fast_fs = 30000.0f;
static float bh_fast_update_hz = 15000.0f;
static float bh_fast_dt = 1.0f / 15000.0f;
static float bh_fast_phase = 0.0f;
static float bh_fast_phase_step = 0.0f;
static float bh_fast_oc_lim = 1.0f;
static float bh_fast_sense_zero = 1.65f;
static float bh_fast_vs_scale = 0.02f;
static float bh_fast_h_scale = 1.0f;
static float bh_fast_b_step_scale = 0.0f;
static int bh_fast_clip_window_limit = 15000;
static unsigned bh_fast_diag_div_count = 0U;
static int bh_fast_cycle_n = 0;
static int bh_fast_sample_n = 0;
static float bh_fast_cycle_i_max = -1.0e30f;
static float bh_fast_cycle_i_min = 1.0e30f;
static int bh_fast_overcurrent_count = 0;

static unsigned bh_fast_center_cycle_seen = 0U;
static float bh_fast_display_b_center = 0.0f;

static float bh_fast_vhist[3] = {0.0f, 0.0f, 0.0f};
static float bh_fast_ihist[3] = {0.0f, 0.0f, 0.0f};
static int bh_fast_hist_n = 0;
static float bh_fast_b = 0.0f;
static float bh_fast_last_vmed = 0.0f;
static bool bh_fast_have_vmed = false;
static float bh_fast_last_good_vs = 0.0f;
static int bh_fast_clip_window_samples = 0;
static int bh_fast_clip_window_events = 0;

/* Inverse-filter state. v1/v2 are median-filtered measured search voltage.
 * The direct A1 inverse needs dV/dt and d2V/dt2, so do it only after the
 * three-sample median. The same corrected voltage is integrated into B and is
 * exposed on Sampled Data phase C as an INA-output-equivalent voltage.
 */
static float bh_fast_de_v1 = 0.0f;
static float bh_fast_de_v2 = 0.0f;
static float bh_fast_de_k1 = 0.0f;
static float bh_fast_de_k2 = 0.0f;
static unsigned bh_fast_de_n = 0U;
static volatile float bh_fast_scope_a1_v = 0.0f;

static inline float bh_fast_triangle(float p) {
	if (p < 0.25f) return 4.0f * p;
	if (p < 0.75f) return 2.0f - 4.0f * p;
	return 4.0f * p - 4.0f;
}

static inline float bh_fast_median3(float a, float b, float c) {
	if (a > b) { float t = a; a = b; b = t; }
	if (b > c) { float t = b; b = c; c = t; }
	if (a > b) { float t = a; a = b; b = t; }
	return b;
}

static inline float bh_fast_deembed_vs(float v) {
	float out = v;
	if (bh_fast_de_n == 1U) {
		out += 2.0f * bh_fast_de_k1 * (v - bh_fast_de_v1);
	} else if (bh_fast_de_n >= 2U) {
		out += bh_fast_de_k1 *
				(3.0f * v - 4.0f * bh_fast_de_v1 + bh_fast_de_v2);
		out += bh_fast_de_k2 *
				(v - 2.0f * bh_fast_de_v1 + bh_fast_de_v2);
	}
	out *= BH_FAST_A1_INV_GAIN;
	bh_fast_de_v2 = bh_fast_de_v1;
	bh_fast_de_v1 = v;
	if (bh_fast_de_n < 2U) bh_fast_de_n++;
	return out;
}

/* -1..+1 triangle directly scales A-B line duty. */
static inline void bh_fast_set_current_ref(float i_line) {
	float i_axis = i_line * BH_TWO_BY_SQRT3;
	mcpwm_foc_set_openloop_phase(i_axis, BH_FOC_PHASE_DEG);

	float norm = fabsf(bh_i_pk) > 1.0e-9f ? i_line / bh_i_pk : 0.0f;
	if (norm > 1.0f) norm = 1.0f;
	if (norm < -1.0f) norm = -1.0f;
	float signed_duty = bh_fast_duty_pk * norm;

	bh_foc_svm_override_alpha = 0.5f * signed_duty;
	bh_foc_svm_override_beta = -0.288675134595f * signed_duty;
	bh_foc_svm_override_active = true;
	bh_fast_last_duty_cmd = signed_duty;
	bh_fast_last_iref = i_line;
}

static inline void bh_fast_abort_drive(void) {
	bh_fast_set_current_ref(0.0f);
	bh_fast_active = false;
}

static bool bh_fast_measure_active_zero(float *zero_v, float *i_avg) {
	bh_fast_set_current_ref(0.0f);
	chThdSleepMicroseconds(12000);

	float vsum = 0.0f;
	float isum = 0.0f;
	for (unsigned n = 0U; n < BH_FAST_A1_ZERO_SAMPLES; n++) {
		timeout_reset();
		if (mc_interface_get_state() != MC_STATE_RUNNING ||
				mc_interface_get_fault() != FAULT_CODE_NONE) return false;
		uint16_t raw = ADC_Value[ADC_IND_EXT];
		if (bh_adc_sample_hard_rail(raw)) return false;

		float i_now = bh_coil_current();
		float a1 = bh_sense_out_v();
		if (bh_sense_pwm_lut_valid) a1 -= bh_sense_pwm_lut_error_v(i_now);
		vsum += a1;
		isum += i_now;
		chThdSleepMicroseconds(250);
	}

	*zero_v = vsum / (float)BH_FAST_A1_ZERO_SAMPLES;
	*i_avg = isum / (float)BH_FAST_A1_ZERO_SAMPLES;
	return isfinite(*zero_v) && *zero_v > 0.1f && *zero_v < 3.2f &&
			isfinite(*i_avg) && fabsf(*i_avg) < 0.25f;
}

static inline bool bh_fast_acquire_sample(void) {
	float i_now = bh_coil_current();
	uint16_t raw = ADC_Value[ADC_IND_EXT];
	bool hard = bh_adc_sample_hard_rail(raw);

	bh_fast_clip_window_samples++;
	if (hard) {
		bh_fast_hard_clips++;
		bh_fast_clip_window_events++;
	}
	if (bh_fast_clip_window_events > BH_FAST_HARD_CLIPS_PER_SEC) {
		bh_fast_clip_abort = true;
		bh_fast_abort_drive();
		return false;
	}
	if (bh_fast_clip_window_samples >= bh_fast_clip_window_limit) {
		bh_fast_clip_window_samples = 0;
		bh_fast_clip_window_events = 0;
	}

	float vs = bh_fast_last_good_vs;
	if (!hard) {
		float vout = bh_sense_out_v() - bh_fast_sense_zero;
		if (bh_sense_pwm_lut_valid) vout -= bh_sense_pwm_lut_error_v(i_now);
		vs = vout * bh_fast_vs_scale;
		bh_fast_last_good_vs = vs;
	}

	bh_fast_vhist[0] = bh_fast_vhist[1];
	bh_fast_vhist[1] = bh_fast_vhist[2];
	bh_fast_vhist[2] = vs;
	bh_fast_ihist[0] = bh_fast_ihist[1];
	bh_fast_ihist[1] = bh_fast_ihist[2];
	bh_fast_ihist[2] = i_now;
	if (bh_fast_hist_n < 3) {
		bh_fast_hist_n++;
		return true;
	}

	float imed = bh_fast_median3(
			bh_fast_ihist[0], bh_fast_ihist[1], bh_fast_ihist[2]);
	float vmed = bh_fast_median3(
			bh_fast_vhist[0], bh_fast_vhist[1], bh_fast_vhist[2]);
	vmed = bh_fast_deembed_vs(vmed);
	bh_fast_scope_a1_v = fabsf(bh_fast_vs_scale) > 1.0e-9f ?
			vmed / bh_fast_vs_scale : 0.0f;

	if (imed > bh_fast_cycle_i_max) bh_fast_cycle_i_max = imed;
	if (imed < bh_fast_cycle_i_min) bh_fast_cycle_i_min = imed;

	if (fabsf(imed) > bh_fast_oc_lim) {
		bh_fast_overcurrent_count++;
		if (bh_fast_overcurrent_count >= BH_FAST_OVERCURRENT_SAMPLES) {
			bh_fast_overcurrent_abort = true;
			bh_fast_abort_drive();
			return false;
		}
	} else {
		bh_fast_overcurrent_count = 0;
	}

	if (bh_fast_have_vmed) {
		bh_fast_b += (bh_fast_last_vmed + vmed) * bh_fast_b_step_scale;
	}
	bh_fast_last_vmed = vmed;
	bh_fast_have_vmed = true;

	if (bh_fast_diag_cycle != bh_fast_center_cycle_seen) {
		bh_fast_center_cycle_seen = bh_fast_diag_cycle;
		bh_fast_display_b_center = 0.5f *
				(bh_fast_diag_b_min + bh_fast_diag_b_max);
	}

	bh_fast_last_h = imed * bh_fast_h_scale;
	bh_fast_last_b = bh_fast_b - bh_fast_display_b_center;
	return true;
}

static bool bh_fast_install_tight_current_limits(void) {
	mc_configuration *cfg = mempools_alloc_mcconf();
	if (!cfg) {
		commands_printf("BH_FAST_ABORT alloc limits");
		return false;
	}
	*cfg = *mc_interface_get_configuration();

	float axis_req = bh_i_pk * BH_TWO_BY_SQRT3;
	float axis_lim = fmaxf(1.0f, axis_req * 1.60f);
	cfg->l_current_max = fmaxf(cfg->l_current_max, axis_lim);
	cfg->l_current_min = fminf(cfg->l_current_min, -axis_lim);
	mc_interface_set_configuration(cfg);
	mempools_free_mcconf(cfg);
	return true;
}

static void bh_fast_reset_state(float freq, float sense_zero) {
	bh_foc_svm_override_active = false;
	bh_foc_svm_override_alpha = 0.0f;
	bh_foc_svm_override_beta = 0.0f;
	bh_fast_active = false;
	bh_fast_external_stop = false;
	bh_fast_clip_abort = false;
	bh_fast_overcurrent_abort = false;
	bh_fast_last_amp = 0.0f;
	bh_fast_last_center = 0.0f;
	bh_fast_last_iref = 0.0f;
	bh_fast_last_h = 0.0f;
	bh_fast_last_b = 0.0f;
	bh_fast_hard_clips = 0;
	bh_fast_phase = 0.0f;
	bh_fast_cycle_n = 0;
	bh_fast_sample_n = 0;
	bh_fast_cycle_i_max = -1.0e30f;
	bh_fast_cycle_i_min = 1.0e30f;
	bh_fast_overcurrent_count = 0;
	bh_fast_diag_div_count = 0U;
	bh_fast_center_cycle_seen = 0U;
	bh_fast_display_b_center = 0.0f;
	bh_fast_sense_zero = sense_zero;
	bh_fast_hist_n = 0;
	bh_fast_b = 0.0f;
	bh_fast_last_vmed = 0.0f;
	bh_fast_have_vmed = false;
	bh_fast_last_good_vs = 0.0f;
	bh_fast_clip_window_samples = 0;
	bh_fast_clip_window_events = 0;
	bh_fast_de_v1 = 0.0f;
	bh_fast_de_v2 = 0.0f;
	bh_fast_de_n = 0U;
	bh_fast_scope_a1_v = 0.0f;
	memset(bh_fast_vhist, 0, sizeof(bh_fast_vhist));
	memset(bh_fast_ihist, 0, sizeof(bh_fast_ihist));

	bh_fast_fs = mc_interface_get_sampling_frequency_now();
	if (bh_fast_fs < 1000.0f) bh_fast_fs = 30000.0f;
	bh_fast_update_hz = bh_fast_fs / (float)BH_FAST_DIAG_DIV;
	bh_fast_dt = 1.0f / bh_fast_update_hz;
	bh_fast_phase_step = freq / bh_fast_update_hz;
	bh_fast_oc_lim = fmaxf(bh_i_pk * 1.50f, bh_i_pk + 0.75f);

	bh_fast_vs_scale = 1.0f / BH_INA_GAIN;
	bh_fast_h_scale = BH_EXC_TURNS / bh_path_m;
	bh_fast_b_step_scale = 0.5f * bh_fast_dt /
			(BH_SENSE_TURNS * bh_area_m2);
	bh_fast_de_k1 = BH_FAST_A1_INV_A_S / (2.0f * bh_fast_dt);
	bh_fast_de_k2 = BH_FAST_A1_INV_B_S2 / (bh_fast_dt * bh_fast_dt);
	bh_fast_clip_window_limit = (int)(bh_fast_update_hz + 0.5f);
	if (bh_fast_clip_window_limit < 1) bh_fast_clip_window_limit = 1;
}

static bool bh_fast_live_plot(float freq) {
	float hardoff_zero = bh_measure_sense_zero();
	if (!bh_sense_zero_ok) {
		commands_printf("BH_FAST_ABORT A1zero %.6f", (double)hardoff_zero);
		return false;
	}
	if (!bh_fast_install_tight_current_limits()) return false;
	bh_fast_reset_state(freq, hardoff_zero);

	commands_printf("BH_FAST_START D=%.6f f=%.3f fs=%.0f/%.0f turns=%g/%g rawSVM",
			(double)bh_fast_duty_pk, (double)freq, (double)bh_fast_fs,
			(double)bh_fast_update_hz,
			(double)BH_EXC_TURNS, (double)BH_SENSE_TURNS);

	bh_apply_current(0.0f);
	if (mc_interface_get_state() != MC_STATE_RUNNING) {
		commands_printf("BH_FAST_ABORT FOC start");
		bh_release_locked();
		return false;
	}

	mcpwm_foc_set_current_off_delay(0.25f);
	float active_zero = 0.0f;
	float zero_i = 0.0f;
	if (!bh_fast_measure_active_zero(&active_zero, &zero_i)) {
		commands_printf("BH_FAST_ABORT A1 pwm zero");
		bh_fast_set_current_ref(0.0f);
		bh_release_locked();
		bh_foc_svm_override_active = false;
		return false;
	}
	bh_fast_sense_zero = active_zero;
	commands_printf("BH_FAST_A1_ZERO off=%.7f pwm=%.7f d=%+.3fmV I=%+.4f LUT=%d",
			(double)hardoff_zero, (double)active_zero,
			(double)((active_zero - hardoff_zero) * 1e3f),
			(double)zero_i, bh_sense_pwm_lut_valid ? 1 : 0);

	mc_interface_set_pwm_callback(bh_fast_pwm_callback);
	bh_fast_active = true;
	bh_fast_set_current_ref(0.0f);

	uint32_t stat_t0 = timer_time_now();
	while (!bh_stop_requested && bh_fast_active &&
			!bh_fast_clip_abort && !bh_fast_overcurrent_abort) {
		timeout_reset();
		mc_interface_lock();

		mc_fault_code fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			commands_printf("BH_FAST_ABORT fault=%d", (int)fault);
			break;
		}

		if (timer_seconds_elapsed_since(stat_t0) >= 1.0f) {
			stat_t0 = timer_time_now();
			commands_printf("BH_FAST_STAT D=%+.5f Iamp=%.4f Ic=%+.4f H=%+.1f B=%+.6f clips=%u",
					(double)bh_fast_last_duty_cmd,
					(double)bh_fast_last_amp,
					(double)bh_fast_last_center,
					(double)bh_fast_last_h, (double)bh_fast_last_b,
					(unsigned)bh_fast_hard_clips);
		}
		chThdSleepMilliseconds(1);
	}

	bh_fast_active = false;
	mc_interface_set_pwm_callback(0);
	bh_fast_set_current_ref(0.0f);
	bh_release_locked();
	bh_foc_svm_override_active = false;

	if (bh_fast_clip_abort) {
		commands_printf("BH_FAST_DONE A1 clips");
		return false;
	}
	if (bh_fast_overcurrent_abort) {
		commands_printf("BH_FAST_DONE overcurrent");
		return false;
	}
	commands_printf("BH_FAST_DONE %s",
			bh_fast_external_stop ? "TOOL_STOP" : "STOP");
	return true;
}

static void terminal_bh_live_fast(int argc, const char **argv) {
	if (bh_is_busy()) {
		commands_printf("A B-H job is already running or queued. Use bh_stop first.");
		return;
	}
	if (argc != 5) {
		commands_printf("Usage: bh_live_fast <Ipk_A> <freq_Hz> <path_mm> <area_mm2>");
		return;
	}
	float i_pk = strtof(argv[1], 0);
	float freq = strtof(argv[2], 0);
	float path_mm = strtof(argv[3], 0);
	float area_mm2 = strtof(argv[4], 0);
	if (!(freq >= BH_MIN_FREQ_HZ && freq <= BH_MAX_FREQ_HZ) ||
			!bh_parse_common_run(i_pk, path_mm, area_mm2, 1)) return;

	bh_freq = freq;
	bh_fast_live_mode = true;
	bh_live_mode = false;
	bh_live_suppress_metric = false;
	if (!bh_queue_job(BH_JOB_RUN)) {
		bh_fast_live_mode = false;
		return;
	}
	commands_printf("BH fast queued");
}

static void bh_fast_init_commands(void) {
	terminal_register_command_callback(
			"bh_live_fast", "Duty B-H tracer.",
			"<D> <f> <path> <area>", terminal_bh_live_fast);
}

#undef BH_FAST_A1_INV_B_S2
#undef BH_FAST_A1_INV_A_S
#undef BH_FAST_A1_INV_GAIN
#undef BH_FAST_A1_ZERO_SAMPLES