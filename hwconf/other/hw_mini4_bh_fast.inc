/* PWM-rate model-feedforward B-H live mode.
 *
 * This path deliberately does not replace the existing current-controlled
 * bh_live implementation. It is an experimental A/B path that:
 *   - generates the requested triangle at the FOC/PWM callback rate,
 *   - drives the fixed A-B winding from a line model V = R*I + L*dI/dt,
 *   - uses open-loop duty at a fixed q-axis angle so the voltage vector lies
 *     on the same physical A-B axis as the ordinary -30 degree d-axis tracer,
 *   - slowly trims model voltage from measured cycle peak current,
 *   - samples current and A1 at the PWM callback rate,
 *   - median-rejects isolated one-sample A1 impulses before B integration,
 *   - closes and centers each plotted B loop cycle-wise in the thread.
 *
 * The callback installed with mc_interface_set_pwm_callback() runs from the
 * motor-control interrupt after every PWM control cycle. Keep it short: no
 * printf, allocation, terminal packets or sleeps are permitted there.
 */

#define BH_FAST_MAX_CYCLE_POINTS          400
#define BH_FAST_MAX_PLOT_LOOPS_HZ         5.0f
#define BH_FAST_WARMUP_CYCLES             3
#define BH_FAST_TRIM_GAIN                 0.05f
#define BH_FAST_TRIM_MIN                  0.50f
#define BH_FAST_TRIM_MAX                  2.00f
#define BH_FAST_MAX_DUTY                  0.12f
#define BH_FAST_HARD_CLIPS_PER_SEC        10

/* Open-loop duty mode controls q-axis voltage. Positive q at -120 degrees has
 * its alpha/beta vector at -30 degrees, exactly matching the A-B physical axis
 * used by the ordinary d-axis current tracer.
 */
#define BH_FAST_DUTY_PHASE_DEG            (BH_FOC_PHASE_DEG - 90.0f)

typedef struct {
	volatile int count;
	volatile float h[BH_FAST_MAX_CYCLE_POINTS];
	volatile float b[BH_FAST_MAX_CYCLE_POINTS];
	volatile float phase[BH_FAST_MAX_CYCLE_POINTS];
	volatile float b_end;
	volatile float peak_a;
	volatile float trim;
	volatile unsigned hard_clips;
} bh_fast_cycle_buf_t;

static bool bh_fast_live_mode = false;
static volatile bool bh_fast_active = false;
static volatile bool bh_fast_external_stop = false;
static volatile bool bh_fast_clip_abort = false;
static volatile bool bh_fast_overcurrent_abort = false;
static volatile bool bh_fast_duty_limited = false;

static volatile float bh_fast_trim = 1.0f;
static volatile float bh_fast_last_peak = 0.0f;
static volatile float bh_fast_last_duty = 0.0f;
static volatile unsigned bh_fast_hard_clips = 0;

static float bh_fast_fs = 30000.0f;
static float bh_fast_dt = 1.0f / 30000.0f;
static float bh_fast_phase = 0.0f;
static float bh_fast_phase_step = 0.0f;
static float bh_fast_sense_zero = 1.65f;
static float bh_fast_overmod = 1.0f;
static int bh_fast_plot_div = 1;
static int bh_fast_capture_stride = 1;
static int bh_fast_cycle_n = 0;
static int bh_fast_sample_n = 0;
static bool bh_fast_capture_cycle = false;

static float bh_fast_b = 0.0f;
static float bh_fast_last_vmed = 0.0f;
static bool bh_fast_have_vmed = false;
static float bh_fast_vhist[3] = {0.0f, 0.0f, 0.0f};
static float bh_fast_ihist[3] = {0.0f, 0.0f, 0.0f};
static int bh_fast_hist_n = 0;
static float bh_fast_last_good_vs = 0.0f;
static float bh_fast_cycle_peak = 0.0f;
static int bh_fast_overcurrent_count = 0;
static int bh_fast_clip_window_samples = 0;
static int bh_fast_clip_window_events = 0;

static bh_fast_cycle_buf_t bh_fast_buf[2];
static int bh_fast_write_idx = 0;
static volatile int bh_fast_ready_idx = -1;
static int bh_fast_write_count = 0;

static inline float bh_fast_triangle(float p) {
	if (p < 0.25f) {
		return 4.0f * p;
	} else if (p < 0.75f) {
		return 2.0f - 4.0f * p;
	}
	return 4.0f * p - 4.0f;
}

static inline float bh_fast_triangle_slope(float p) {
	return (p < 0.25f || p >= 0.75f) ? 4.0f : -4.0f;
}

static inline float bh_fast_median3(float a, float b, float c) {
	if (a > b) { float t = a; a = b; b = t; }
	if (b > c) { float t = b; b = c; c = t; }
	if (a > b) { float t = a; a = b; b = t; }
	return b;
}

static inline float bh_fast_duty_for_phase(float p) {
	float i_ref = bh_i_pk * bh_fast_triangle(p);
	float didt = bh_i_pk * bh_freq * bh_fast_triangle_slope(p);
	float v_line = bh_r_line_ohm * i_ref + bh_l_line_h * didt;
	float vbus = GET_INPUT_VOLTAGE();
	if (vbus < 2.0f) {
		vbus = 2.0f;
	}
	float den = vbus * bh_fast_overmod;
	if (den < 1.0f) {
		den = vbus;
	}
	float duty = bh_fast_trim * v_line / den;
	if (duty > BH_FAST_MAX_DUTY) {
		duty = BH_FAST_MAX_DUTY;
		bh_fast_duty_limited = true;
	} else if (duty < -BH_FAST_MAX_DUTY) {
		duty = -BH_FAST_MAX_DUTY;
		bh_fast_duty_limited = true;
	}
	return duty;
}

static inline void bh_fast_publish_cycle(void) {
	if (!bh_fast_capture_cycle || bh_fast_write_count <= 2) {
		bh_fast_write_count = 0;
		return;
	}

	/* Do not overwrite a buffer while the tracer thread is sending it. Dropping
	 * an occasional display cycle is harmless; drive and integration continue.
	 */
	if (bh_fast_ready_idx < 0) {
		bh_fast_cycle_buf_t *b = &bh_fast_buf[bh_fast_write_idx];
		b->count = bh_fast_write_count;
		b->b_end = bh_fast_b;
		b->peak_a = bh_fast_cycle_peak;
		b->trim = bh_fast_trim;
		b->hard_clips = bh_fast_hard_clips;
		bh_fast_ready_idx = bh_fast_write_idx;
		bh_fast_write_idx ^= 1;
	}
	bh_fast_write_count = 0;
}

static void bh_fast_pwm_callback(void) {
	if (!bh_fast_active) {
		return;
	}

	/* An estop/release underneath us is an external request to stop. Never call
	 * the open-loop setter after state leaves RUNNING or we would restart FOC.
	 */
	if (mc_interface_get_state() != MC_STATE_RUNNING) {
		bh_fast_external_stop = true;
		bh_fast_active = false;
		return;
	}

	if (bh_stop_requested || mc_interface_get_fault() != FAULT_CODE_NONE) {
		mcpwm_foc_set_openloop_duty_phase(0.0f, BH_FAST_DUTY_PHASE_DEG);
		return;
	}

	float i_now = bh_coil_current();
	float ai = fabsf(i_now);
	if (ai > bh_fast_cycle_peak) {
		bh_fast_cycle_peak = ai;
	}

	/* Independent software sanity limit. VESC's configured current limit remains
	 * active as the primary protection, but a few consecutive samples well above
	 * the requested fixture current terminate this diagnostic too.
	 */
	float oc_lim = fmaxf(bh_i_pk * 1.35f, bh_i_pk + 0.50f);
	if (ai > oc_lim) {
		bh_fast_overcurrent_count++;
		if (bh_fast_overcurrent_count >= 4) {
			bh_fast_overcurrent_abort = true;
			mcpwm_foc_set_openloop_duty_phase(0.0f, BH_FAST_DUTY_PHASE_DEG);
			return;
		}
	} else {
		bh_fast_overcurrent_count = 0;
	}

	uint16_t raw = ADC_Value[ADC_IND_EXT];
	bool hard = bh_adc_sample_hard_rail(raw);
	bh_fast_clip_window_samples++;
	if (hard) {
		bh_fast_hard_clips++;
		bh_fast_clip_window_events++;
	}
	if (bh_fast_clip_window_events > BH_FAST_HARD_CLIPS_PER_SEC) {
		bh_fast_clip_abort = true;
		mcpwm_foc_set_openloop_duty_phase(0.0f, BH_FAST_DUTY_PHASE_DEG);
		return;
	}
	if (bh_fast_clip_window_samples >= (int)bh_fast_fs) {
		bh_fast_clip_window_samples = 0;
		bh_fast_clip_window_events = 0;
	}

	float vs = bh_fast_last_good_vs;
	if (!hard) {
		vs = (bh_sense_out_v() - bh_fast_sense_zero) / BH_INA_GAIN;
		bh_fast_last_good_vs = vs;
	}

	/* A causal 3-sample median removes a single PWM/common-mode impulse without
	 * imposing the long phase shift of a conventional low-pass. Pair it with the
	 * middle current sample so H and dB/dt remain aligned to one PWM sample.
	 */
	bh_fast_vhist[0] = bh_fast_vhist[1];
	bh_fast_vhist[1] = bh_fast_vhist[2];
	bh_fast_vhist[2] = vs;
	bh_fast_ihist[0] = bh_fast_ihist[1];
	bh_fast_ihist[1] = bh_fast_ihist[2];
	bh_fast_ihist[2] = i_now;
	if (bh_fast_hist_n < 3) {
		bh_fast_hist_n++;
	}

	if (bh_fast_hist_n >= 3) {
		float vmed = bh_fast_median3(bh_fast_vhist[0], bh_fast_vhist[1], bh_fast_vhist[2]);
		if (bh_fast_have_vmed) {
			bh_fast_b += 0.5f * (bh_fast_last_vmed + vmed) * bh_fast_dt /
					(BH_SENSE_TURNS * bh_area_m2);
		}
		bh_fast_last_vmed = vmed;
		bh_fast_have_vmed = true;

		if (bh_fast_capture_cycle &&
				(bh_fast_sample_n % bh_fast_plot_div) == 0 &&
				bh_fast_write_count < BH_FAST_MAX_CYCLE_POINTS) {
			bh_fast_cycle_buf_t *b = &bh_fast_buf[bh_fast_write_idx];
			int n = bh_fast_write_count++;
			b->h[n] = BH_EXC_TURNS * bh_fast_ihist[1] / bh_path_m;
			b->b[n] = bh_fast_b;
			b->phase[n] = bh_fast_phase;
		}
	}

	bh_fast_sample_n++;
	float next_phase = bh_fast_phase + bh_fast_phase_step;
	if (next_phase >= 1.0f) {
		/* Publish the just-completed cycle before zeroing its integral. The plot
		 * thread removes the residual linear volt-second drift and recenters B.
		 */
		bh_fast_publish_cycle();

		bh_fast_last_peak = bh_fast_cycle_peak;
		if (bh_fast_cycle_n >= BH_FAST_WARMUP_CYCLES && bh_i_pk > 0.01f &&
				bh_fast_cycle_peak > 0.02f) {
			float err = (bh_i_pk - bh_fast_cycle_peak) / bh_i_pk;
			utils_truncate_number(&err, -0.50f, 0.50f);
			bh_fast_trim += BH_FAST_TRIM_GAIN * err;
			utils_truncate_number((float*)&bh_fast_trim, BH_FAST_TRIM_MIN, BH_FAST_TRIM_MAX);
		}

		bh_fast_cycle_n++;
		bh_fast_cycle_peak = 0.0f;
		bh_fast_b = 0.0f;
		bh_fast_sample_n = 0;
		next_phase -= 1.0f;
		bh_fast_capture_cycle = bh_fast_cycle_n >= BH_FAST_WARMUP_CYCLES &&
				((bh_fast_cycle_n - BH_FAST_WARMUP_CYCLES) % bh_fast_capture_stride) == 0;
	}
	bh_fast_phase = next_phase;

	float duty = bh_fast_duty_for_phase(bh_fast_phase);
	bh_fast_last_duty = duty;
	mcpwm_foc_set_openloop_duty_phase(duty, BH_FAST_DUTY_PHASE_DEG);
}

static bool bh_fast_install_tight_current_limits(void) {
	mc_configuration *cfg = mempools_alloc_mcconf();
	if (!cfg) {
		commands_printf("BH_FAST_ABORT could not allocate temporary current-limit config");
		return false;
	}
	*cfg = *mc_interface_get_configuration();

	float axis_lim = bh_i_pk * BH_TWO_BY_SQRT3 * 1.50f;
	if (axis_lim < 1.0f) {
		axis_lim = 1.0f;
	}
	cfg->l_current_max = axis_lim;
	cfg->l_current_min = -axis_lim;
	cfg->l_abs_current_max = fmaxf(2.0f, axis_lim * 1.25f);
	mc_interface_set_configuration(cfg);
	mempools_free_mcconf(cfg);
	return true;
}

static void bh_fast_reset_state(float freq, float sense_zero) {
	bh_fast_active = false;
	bh_fast_external_stop = false;
	bh_fast_clip_abort = false;
	bh_fast_overcurrent_abort = false;
	bh_fast_duty_limited = false;
	bh_fast_trim = 1.0f;
	bh_fast_last_peak = 0.0f;
	bh_fast_last_duty = 0.0f;
	bh_fast_hard_clips = 0;
	bh_fast_phase = 0.0f;
	bh_fast_sense_zero = sense_zero;
	bh_fast_cycle_n = 0;
	bh_fast_sample_n = 0;
	bh_fast_b = 0.0f;
	bh_fast_last_vmed = 0.0f;
	bh_fast_have_vmed = false;
	bh_fast_hist_n = 0;
	bh_fast_last_good_vs = 0.0f;
	bh_fast_cycle_peak = 0.0f;
	bh_fast_overcurrent_count = 0;
	bh_fast_clip_window_samples = 0;
	bh_fast_clip_window_events = 0;
	bh_fast_ready_idx = -1;
	bh_fast_write_idx = 0;
	bh_fast_write_count = 0;
	memset((void*)bh_fast_vhist, 0, sizeof(bh_fast_vhist));
	memset((void*)bh_fast_ihist, 0, sizeof(bh_fast_ihist));

	bh_fast_fs = mc_interface_get_sampling_frequency_now();
	if (bh_fast_fs < 1000.0f) {
		bh_fast_fs = 30000.0f;
	}
	bh_fast_dt = 1.0f / bh_fast_fs;
	bh_fast_phase_step = freq / bh_fast_fs;

	float samples_per_cycle = bh_fast_fs / freq;
	bh_fast_plot_div = (int)ceilf(samples_per_cycle / (float)BH_FAST_MAX_CYCLE_POINTS);
	if (bh_fast_plot_div < 1) {
		bh_fast_plot_div = 1;
	}
	bh_fast_capture_stride = (int)ceilf(freq / BH_FAST_MAX_PLOT_LOOPS_HZ);
	if (bh_fast_capture_stride < 1) {
		bh_fast_capture_stride = 1;
	}

	const volatile mc_configuration *cfg = mc_interface_get_configuration();
	bh_fast_overmod = cfg->foc_overmod_factor;
	if (bh_fast_overmod < 0.1f) {
		bh_fast_overmod = 1.0f;
	}
}

static void bh_fast_plot_ready_cycle(void) {
	int idx = bh_fast_ready_idx;
	if (idx < 0 || idx > 1) {
		return;
	}

	bh_fast_cycle_buf_t *b = &bh_fast_buf[idx];
	int count = b->count;
	if (count > BH_FAST_MAX_CYCLE_POINTS) {
		count = BH_FAST_MAX_CYCLE_POINTS;
	}
	if (count < 2) {
		bh_fast_ready_idx = -1;
		return;
	}

	/* Search-coil integration only knows relative B. Remove the residual linear
	 * volt-second error so each complete cycle closes, then center its extrema
	 * around zero. This is done after acquisition so the ISR remains tiny.
	 */
	float bmin = 1e30f;
	float bmax = -1e30f;
	for (int i = 0; i < count; i++) {
		float bc = b->b[i] - b->phase[i] * b->b_end;
		if (bc < bmin) bmin = bc;
		if (bc > bmax) bmax = bc;
	}
	float center = 0.5f * (bmax + bmin);

	for (int i = 0; i < count; i++) {
		float bc = b->b[i] - b->phase[i] * b->b_end - center;
		commands_send_plot_points(b->h[i], bc);
		if ((i & 15) == 0) {
			timeout_reset();
			mc_interface_lock();
		}
	}

	bh_fast_ready_idx = -1;
}

static bool bh_fast_live_plot(float freq) {
	if (!bh_rl_valid || bh_r_line_ohm <= 0.0f || bh_l_line_h <= 0.0f) {
		commands_printf("BH_FAST_ABORT R/L model is not valid; run bh_measure_rl after flashing first.");
		return false;
	}

	float sense_zero = bh_measure_sense_zero();
	if (!bh_sense_zero_ok) {
		commands_printf("BH_FAST_ABORT A1 zero invalid V=%.6f", (double)sense_zero);
		return false;
	}

	if (!bh_fast_install_tight_current_limits()) {
		return false;
	}
	bh_fast_reset_state(freq, sense_zero);

	commands_printf("BH_FAST_START Ipk=%.6f A f=%.6f Hz Rline=%.6f mOhm Lline=%.6f uH fs=%.1f Hz turns=%g/%g zero=%.7f V sense=%+d",
			(double)bh_i_pk, (double)freq, (double)(bh_r_line_ohm * 1e3f),
			(double)(bh_l_line_h * 1e6f), (double)bh_fast_fs,
			(double)BH_EXC_TURNS, (double)BH_SENSE_TURNS,
			(double)sense_zero, bh_sense_polarity);
	commands_printf("BH_FAST: PWM-rate V=R*I+L*dI/dt duty drive, slow peak trim, 3-sample median A1 integration; Experiment Plot History controls displayed loops.");

	commands_init_plot("H (A/m)", "B relative (T)");
	commands_plot_add_graph("B-H fast");
	commands_plot_set_graph(0);

	/* Clear any stale d-axis request before switching to q-axis duty mode. With
	 * cc_min_current=0, this enters RUNNING even at zero current.
	 */
	bh_apply_current(0.0f);
	if (mc_interface_get_state() != MC_STATE_RUNNING) {
		commands_printf("BH_FAST_ABORT could not enter FOC RUNNING state");
		bh_release_locked();
		return false;
	}

	float duty0 = bh_fast_duty_for_phase(0.0f);
	mcpwm_foc_set_openloop_duty_phase(duty0, BH_FAST_DUTY_PHASE_DEG);
	bh_fast_last_duty = duty0;
	bh_fast_active = true;
	mc_interface_set_pwm_callback(bh_fast_pwm_callback);

	uint32_t stat_t0 = timer_time_now();
	while (!bh_stop_requested && bh_fast_active &&
			!bh_fast_clip_abort && !bh_fast_overcurrent_abort) {
		timeout_reset();
		mc_interface_lock();
		bh_fast_plot_ready_cycle();

		mc_fault_code fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			commands_printf("BH_FAST_ABORT fault=%d (%s)", (int)fault,
					mc_interface_fault_to_string(fault));
			break;
		}

		if (timer_seconds_elapsed_since(stat_t0) >= 1.0f) {
			stat_t0 = timer_time_now();
			commands_printf("BH_FAST_STAT Ipk_meas=%.5f A trim=%.5f duty=%.6f clips=%u%s",
					(double)bh_fast_last_peak, (double)bh_fast_trim,
					(double)bh_fast_last_duty, (unsigned)bh_fast_hard_clips,
					bh_fast_duty_limited ? " DUTY_LIMIT" : "");
		}
		chThdSleepMilliseconds(1);
	}

	bh_fast_active = false;
	mc_interface_set_pwm_callback(0);
	mcpwm_foc_set_openloop_duty_phase(0.0f, BH_FAST_DUTY_PHASE_DEG);
	bh_release_locked();

	if (bh_fast_clip_abort) {
		commands_printf("BH_FAST_DONE ABORT sustained A1 clipping (>10 hard-rail samples/s)");
		return false;
	}
	if (bh_fast_overcurrent_abort) {
		commands_printf("BH_FAST_DONE ABORT measured current exceeded sanity limit");
		return false;
	}
	if (bh_fast_external_stop) {
		commands_printf("BH_FAST_DONE VESC_TOOL_STOP");
	} else {
		commands_printf("BH_FAST_DONE STOP_REQUESTED");
	}
	return true;
}

static void terminal_bh_live_fast(int argc, const char **argv) {
	if (bh_is_busy()) {
		commands_printf("A B-H job is already running or queued. Use bh_stop first.");
		return;
	}
	if (argc != 5) {
		commands_printf("Usage: bh_live_fast <Ipk_A> <freq_Hz> <path_mm> <area_mm2>");
		commands_printf("Example: bh_live_fast 0.50 20 21.9 3.65");
		return;
	}

	float i_pk = strtof(argv[1], 0);
	float freq = strtof(argv[2], 0);
	float path_mm = strtof(argv[3], 0);
	float area_mm2 = strtof(argv[4], 0);
	if (!(freq >= BH_MIN_FREQ_HZ && freq <= BH_MAX_FREQ_HZ) ||
			!bh_parse_common_run(i_pk, path_mm, area_mm2, 1)) {
		commands_printf("Frequency limit: %.1f..%.1f Hz", (double)BH_MIN_FREQ_HZ,
				(double)BH_MAX_FREQ_HZ);
		return;
	}

	bh_freq = freq;
	bh_fast_live_mode = true;
	bh_live_mode = false;
	bh_live_suppress_metric = false;
	if (!bh_queue_job(BH_JOB_RUN)) {
		bh_fast_live_mode = false;
		return;
	}

	commands_printf("B-H fast live run queued: Ipk=%.4f A f=%.4f Hz path=%.3f mm area=%.4f mm^2",
			(double)i_pk, (double)freq, (double)path_mm, (double)area_mm2);
}

static void bh_fast_init_commands(void) {
	terminal_register_command_callback(
			"bh_live_fast",
			"PWM-rate model-feedforward B-H live plot with fast A1 integration.",
			"<Ipk_A> <freq_Hz> <path_mm> <area_mm2>",
			terminal_bh_live_fast);
}
