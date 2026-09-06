/* PWM-rate model-feedforward B-H live mode.
 *
 * This path deliberately does not replace the existing current-controlled
 * bh_live implementation. It is an experimental A/B path that:
 *   - generates the requested triangle at the FOC/PWM callback rate,
 *   - drives the phase-1/phase-2 fixture winding from
 *         Vab = Rline * Iref + Lline * dIref/dt
 *     plus a deliberately weak proportional current-error correction,
 *   - injects the resulting fixed-axis alpha/beta vector immediately before
 *     VESC's normal foc_svm() stage, bypassing current/duty regulation while
 *     retaining VESC's sector selection, PWM timing, deadtime and ADC sampling,
 *   - slowly trims the model voltage from measured cycle current amplitude,
 *   - samples phase current and A1 at the PWM callback rate,
 *   - median-rejects isolated one-sample A1 impulses before B integration,
 *   - closes and centers each plotted B loop cycle-wise in the thread.
 *
 * The callback installed with mc_interface_set_pwm_callback() runs after each
 * motor-control cycle. It computes the command that the next FOC cycle will
 * consume through the MINI4-only foc_svm wrapper in foc_math.h. No timer compare
 * registers are touched here; the FOC driver remains the sole owner of the FETs.
 */

#define BH_FAST_MAX_CYCLE_POINTS          400
#define BH_FAST_MAX_PLOT_LOOPS_HZ         5.0f
#define BH_FAST_WARMUP_CYCLES             3
#define BH_FAST_TRIM_GAIN                 0.05f
#define BH_FAST_TRIM_MIN                  0.50f
#define BH_FAST_TRIM_MAX                  2.00f
#define BH_FAST_CURRENT_P_OHM             0.020f
#define BH_FAST_MAX_LINE_RATIO            0.12f
#define BH_FAST_HARD_CLIPS_PER_SEC        10
#define BH_FAST_OVERCURRENT_SAMPLES       4

/* These have external linkage because motor/foc_math.h references them from
 * the separately compiled mcpwm_foc.c translation unit on this target only.
 */
volatile bool bh_foc_svm_override_active = false;
volatile float bh_foc_svm_override_alpha = 0.0f;
volatile float bh_foc_svm_override_beta = 0.0f;

typedef struct {
	volatile int count;
	volatile float h[BH_FAST_MAX_CYCLE_POINTS];
	volatile float b[BH_FAST_MAX_CYCLE_POINTS];
	volatile float phase[BH_FAST_MAX_CYCLE_POINTS];
	volatile float b_end;
	volatile float amp_a;
	volatile float center_a;
	volatile float trim;
	volatile unsigned hard_clips;
} bh_fast_cycle_buf_t;

/* bh_mini4_bh.h forward-declares these two so bh_live's wrapper can dispatch
 * this optional mode before this include is expanded.
 */
static bool bh_fast_live_mode = false;
static volatile bool bh_fast_active = false;
static volatile bool bh_fast_external_stop = false;
static volatile bool bh_fast_clip_abort = false;
static volatile bool bh_fast_overcurrent_abort = false;
static volatile bool bh_fast_mod_limited = false;

static volatile float bh_fast_trim = 1.0f;
static volatile float bh_fast_last_amp = 0.0f;
static volatile float bh_fast_last_center = 0.0f;
static volatile float bh_fast_last_mod = 0.0f;
static volatile float bh_fast_last_vcmd = 0.0f;
static volatile unsigned bh_fast_hard_clips = 0;

static float bh_fast_fs = 30000.0f;
static float bh_fast_dt = 1.0f / 30000.0f;
static float bh_fast_phase = 0.0f;
static float bh_fast_phase_step = 0.0f;
static float bh_fast_sense_zero = 1.65f;
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
static float bh_fast_phist[3] = {0.0f, 0.0f, 0.0f};
static int bh_fast_hist_n = 0;
static float bh_fast_last_good_vs = 0.0f;
static float bh_fast_cycle_i_max = -1.0e30f;
static float bh_fast_cycle_i_min = 1.0e30f;
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

/* Convert desired A-B line-voltage ratio to the alpha/beta modulation that
 * VESC's foc_svm() expects.
 *
 * For this fixed A-B winding the electrical vector is at -30 degrees and
 * Vab = sqrt(3) * Vd. Therefore
 *     v_alpha = +Vab/2
 *     v_beta  = -Vab/(2*sqrt(3))
 * and VESC normalizes alpha/beta by 2/3*Vbus, giving
 *     mod_alpha = +0.75 * Vab/Vbus
 *     mod_beta  = -sqrt(3)/4 * Vab/Vbus.
 *
 * The override is consumed inside foc_svm(), so VESC still calculates the
 * actual SVM sector and timer duties and keeps its ADC/current reconstruction
 * synchronized with the waveform that really reaches the bridge.
 */
static inline void bh_fast_set_line_ratio(float line_ratio) {
	if (line_ratio > BH_FAST_MAX_LINE_RATIO) {
		line_ratio = BH_FAST_MAX_LINE_RATIO;
		bh_fast_mod_limited = true;
	} else if (line_ratio < -BH_FAST_MAX_LINE_RATIO) {
		line_ratio = -BH_FAST_MAX_LINE_RATIO;
		bh_fast_mod_limited = true;
	}

	bh_foc_svm_override_alpha = 0.75f * line_ratio;
	bh_foc_svm_override_beta = -0.43301270189f * line_ratio;
	bh_foc_svm_override_active = true;
	bh_fast_last_mod = line_ratio;
}

static inline void bh_fast_disable_svm_override(void) {
	/* Drop active first so a concurrent FOC cycle cannot consume half-updated
	 * zero values. On MINI4 all 32-bit stores are naturally atomic.
	 */
	bh_foc_svm_override_active = false;
	bh_foc_svm_override_alpha = 0.0f;
	bh_foc_svm_override_beta = 0.0f;
	bh_fast_last_mod = 0.0f;
}

static inline void bh_fast_apply_model_voltage(float p, float i_now) {
	float i_ref = bh_i_pk * bh_fast_triangle(p);
	float didt_ref = bh_i_pk * bh_freq * bh_fast_triangle_slope(p);

	/* R/L feed-forward does nearly all the work. The small proportional term is
	 * intentionally much weaker than the winding resistance; it corrects model
	 * error without turning this back into an aggressive current controller.
	 */
	float v_model = bh_r_line_ohm * i_ref + bh_l_line_h * didt_ref;
	float v_cmd = bh_fast_trim * v_model +
			BH_FAST_CURRENT_P_OHM * (i_ref - i_now);

	float vbus = GET_INPUT_VOLTAGE();
	if (vbus < 2.0f) {
		vbus = 2.0f;
	}

	bh_fast_last_vcmd = v_cmd;
	bh_fast_set_line_ratio(v_cmd / vbus);
}

static inline void bh_fast_publish_cycle(void) {
	if (!bh_fast_capture_cycle || bh_fast_write_count <= 2) {
		bh_fast_write_count = 0;
		return;
	}

	if (bh_fast_ready_idx < 0) {
		bh_fast_cycle_buf_t *b = &bh_fast_buf[bh_fast_write_idx];
		b->count = bh_fast_write_count;
		b->b_end = bh_fast_b;
		b->amp_a = 0.5f * (bh_fast_cycle_i_max - bh_fast_cycle_i_min);
		b->center_a = 0.5f * (bh_fast_cycle_i_max + bh_fast_cycle_i_min);
		b->trim = bh_fast_trim;
		b->hard_clips = bh_fast_hard_clips;
		bh_fast_ready_idx = bh_fast_write_idx;
		bh_fast_write_idx ^= 1;
	}
	bh_fast_write_count = 0;
}

static inline void bh_fast_abort_drive(void) {
	/* Keep the SVM override active at a zero vector until the tracer thread can
	 * execute the normal hard-off path. Do not hand control back to the current
	 * PI while winding current is still present.
	 */
	bh_fast_set_line_ratio(0.0f);
	bh_fast_active = false;
}

static void bh_fast_pwm_callback(void) {
	if (!bh_fast_active) {
		return;
	}

	/* VESC Tool's estop releases the motor underneath us. Do not command a new
	 * vector after that state transition; hardware is already being shut down.
	 */
	if (mc_interface_get_state() != MC_STATE_RUNNING) {
		bh_fast_external_stop = true;
		bh_fast_active = false;
		return;
	}

	if (bh_stop_requested || mc_interface_get_fault() != FAULT_CODE_NONE) {
		bh_fast_abort_drive();
		return;
	}

	float i_now = bh_coil_current();
	if (i_now > bh_fast_cycle_i_max) bh_fast_cycle_i_max = i_now;
	if (i_now < bh_fast_cycle_i_min) bh_fast_cycle_i_min = i_now;

	/* Independent fast sanity limit. VESC's configured absolute-current fault
	 * remains intact, while this much tighter fixture-specific limit removes the
	 * commanded vector after four consecutive excessive current samples.
	 */
	float ai = fabsf(i_now);
	float oc_lim = fmaxf(bh_i_pk * 1.35f, bh_i_pk + 0.50f);
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

	/* A causal 3-sample median removes isolated in-range PWM/common-mode spikes
	 * without the long phase delay of a conventional low-pass. Pair B with the
	 * middle current and phase sample so H, B and cycle-drift correction remain
	 * time aligned.
	 */
	bh_fast_vhist[0] = bh_fast_vhist[1];
	bh_fast_vhist[1] = bh_fast_vhist[2];
	bh_fast_vhist[2] = vs;
	bh_fast_ihist[0] = bh_fast_ihist[1];
	bh_fast_ihist[1] = bh_fast_ihist[2];
	bh_fast_ihist[2] = i_now;
	bh_fast_phist[0] = bh_fast_phist[1];
	bh_fast_phist[1] = bh_fast_phist[2];
	bh_fast_phist[2] = bh_fast_phase;
	if (bh_fast_hist_n < 3) {
		bh_fast_hist_n++;
	}

	if (bh_fast_hist_n >= 3) {
		float vmed = bh_fast_median3(
				bh_fast_vhist[0], bh_fast_vhist[1], bh_fast_vhist[2]);
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
			b->phase[n] = bh_fast_phist[1];
		}
	}

	bh_fast_sample_n++;
	float next_phase = bh_fast_phase + bh_fast_phase_step;
	if (next_phase >= 1.0f) {
		/* Publish the completed cycle before zeroing its relative B integral. */
		bh_fast_publish_cycle();

		float amp = 0.5f * (bh_fast_cycle_i_max - bh_fast_cycle_i_min);
		float center = 0.5f * (bh_fast_cycle_i_max + bh_fast_cycle_i_min);
		bh_fast_last_amp = amp;
		bh_fast_last_center = center;

		if (bh_fast_cycle_n >= BH_FAST_WARMUP_CYCLES && bh_i_pk > 0.01f &&
				amp > 0.02f) {
			float err = (bh_i_pk - amp) / bh_i_pk;
			utils_truncate_number(&err, -0.50f, 0.50f);
			bh_fast_trim += BH_FAST_TRIM_GAIN * err;
			utils_truncate_number((float*)&bh_fast_trim,
					BH_FAST_TRIM_MIN, BH_FAST_TRIM_MAX);
		}

		bh_fast_cycle_n++;
		bh_fast_cycle_i_max = -1.0e30f;
		bh_fast_cycle_i_min = 1.0e30f;
		bh_fast_b = 0.0f;
		bh_fast_sample_n = 0;
		next_phase -= 1.0f;
		bh_fast_capture_cycle = bh_fast_cycle_n >= BH_FAST_WARMUP_CYCLES &&
				((bh_fast_cycle_n - BH_FAST_WARMUP_CYCLES) %
						bh_fast_capture_stride) == 0;
	}
	bh_fast_phase = next_phase;

	/* Command the vector for the next FOC/SVM cycle. */
	bh_fast_apply_model_voltage(bh_fast_phase, i_now);
}

static bool bh_fast_install_tight_current_limits(void) {
	mc_configuration *cfg = mempools_alloc_mcconf();
	if (!cfg) {
		commands_printf("BH_FAST_ABORT could not allocate temporary current-limit config");
		return false;
	}
	*cfg = *mc_interface_get_configuration();

	/* Ensure ordinary FOC current-limit bookkeeping will not reject the fixture
	 * current. Do not artificially lower l_abs_current_max here: the fast
	 * fixture-specific four-sample guard above is much tighter, while retaining
	 * the user's normal hardware absolute-current protection avoids false faults
	 * from normal PWM ripple/current-sense transients.
	 */
	float axis_req = bh_i_pk * BH_TWO_BY_SQRT3;
	float axis_lim = fmaxf(1.0f, axis_req * 1.60f);
	cfg->l_current_max = fmaxf(cfg->l_current_max, axis_lim);
	cfg->l_current_min = fminf(cfg->l_current_min, -axis_lim);

	mc_interface_set_configuration(cfg);
	mempools_free_mcconf(cfg);
	return true;
}

static void bh_fast_reset_state(float freq, float sense_zero) {
	bh_fast_disable_svm_override();
	bh_fast_active = false;
	bh_fast_external_stop = false;
	bh_fast_clip_abort = false;
	bh_fast_overcurrent_abort = false;
	bh_fast_mod_limited = false;
	bh_fast_trim = 1.0f;
	bh_fast_last_amp = 0.0f;
	bh_fast_last_center = 0.0f;
	bh_fast_last_mod = 0.0f;
	bh_fast_last_vcmd = 0.0f;
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
	bh_fast_cycle_i_max = -1.0e30f;
	bh_fast_cycle_i_min = 1.0e30f;
	bh_fast_overcurrent_count = 0;
	bh_fast_clip_window_samples = 0;
	bh_fast_clip_window_events = 0;
	bh_fast_ready_idx = -1;
	bh_fast_write_idx = 0;
	bh_fast_write_count = 0;
	memset((void*)bh_fast_vhist, 0, sizeof(bh_fast_vhist));
	memset((void*)bh_fast_ihist, 0, sizeof(bh_fast_ihist));
	memset((void*)bh_fast_phist, 0, sizeof(bh_fast_phist));

	bh_fast_fs = mc_interface_get_sampling_frequency_now();
	if (bh_fast_fs < 1000.0f) {
		bh_fast_fs = 30000.0f;
	}
	bh_fast_dt = 1.0f / bh_fast_fs;
	bh_fast_phase_step = freq / bh_fast_fs;

	float samples_per_cycle = bh_fast_fs / freq;
	bh_fast_plot_div = (int)ceilf(samples_per_cycle /
			(float)BH_FAST_MAX_CYCLE_POINTS);
	if (bh_fast_plot_div < 1) {
		bh_fast_plot_div = 1;
	}
	bh_fast_capture_stride = (int)ceilf(freq / BH_FAST_MAX_PLOT_LOOPS_HZ);
	if (bh_fast_capture_stride < 1) {
		bh_fast_capture_stride = 1;
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
	 * around zero. This happens after acquisition so the ISR stays deterministic.
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
		commands_printf("BH_FAST_ABORT R/L model invalid; run bh_measure_rl first.");
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

	commands_printf("BH_FAST_START Ipk=%.6f A f=%.6f Hz R=%.6f mOhm L=%.6f uH fs=%.1f Hz turns=%g/%g zero=%.7f V sense=%+d",
			(double)bh_i_pk, (double)freq, (double)(bh_r_line_ohm * 1e3f),
			(double)(bh_l_line_h * 1e6f), (double)bh_fast_fs,
			(double)BH_EXC_TURNS, (double)BH_SENSE_TURNS,
			(double)sense_zero, bh_sense_polarity);
	commands_printf("BH_FAST: raw fixed-axis modulation through normal FOC SVM; model feed-forward + weak P + slow amplitude trim.");

	commands_init_plot("H (A/m)", "B relative (T)");
	commands_plot_add_graph("B-H fast");
	commands_plot_set_graph(0);

	/* Enter RUNNING with a harmless zero-current fixed phase. The FOC controller
	 * keeps all normal timer/gate/current-sampling/fault machinery alive, while
	 * the SVM wrapper below replaces only its final alpha/beta command.
	 */
	bh_apply_current(0.0f);
	if (mc_interface_get_state() != MC_STATE_RUNNING) {
		commands_printf("BH_FAST_ABORT could not enter FOC RUNNING state");
		bh_release_locked();
		return false;
	}

	/* Install the high-rate generator and immediately provide its first vector.
	 * There is no direct TIM1 manipulation and no 1 ms active-zero dwell.
	 */
	mc_interface_set_pwm_callback(bh_fast_pwm_callback);
	bh_fast_active = true;
	bh_fast_apply_model_voltage(0.0f, bh_coil_current());

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
			commands_printf("BH_FAST_STAT Iamp=%.5f A Icenter=%+.5f A trim=%.5f Vcmd=%+.6f V Vab/Vbus=%+.6f clips=%u%s",
					(double)bh_fast_last_amp, (double)bh_fast_last_center,
					(double)bh_fast_trim, (double)bh_fast_last_vcmd,
					(double)bh_fast_last_mod, (unsigned)bh_fast_hard_clips,
					bh_fast_mod_limited ? " MOD_LIMIT" : "");
		}
		chThdSleepMilliseconds(1);
	}

	bh_fast_active = false;
	mc_interface_set_pwm_callback(0);

	/* Hold a zero SVM vector until PWM is physically disabled, then remove the
	 * override. This prevents the dormant current PI from taking control during
	 * the shutdown transition.
	 */
	bh_fast_set_line_ratio(0.0f);
	bh_release_locked();
	bh_fast_disable_svm_override();

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
			"PWM-rate model-voltage B-H live plot through normal FOC SVM.",
			"<Ipk_A> <freq_Hz> <path_mm> <area_mm2>",
			terminal_bh_live_fast);
}
