/* PWM-rate current-controlled B-H live mode.
 *
 * The original fast prototype synthesized Vab from an R/L model and injected
 * a raw fixed-axis SVM vector. That proved the high-rate waveform/acquisition
 * path, but it also showed exactly why voltage drive is undesirable for this
 * fixture: incremental inductance collapses in saturation, so a voltage ramp
 * naturally produces current spikes where the magnetic plant gets faster.
 *
 * This version keeps the useful parts of that prototype -- PWM-rate triangle
 * generation, PWM-rate A1/current acquisition, median impulse rejection,
 * cycle-wise B closure/centering and buffered Experiment Plot output -- while
 * moving the excitation back to VESC's normal FOC d-axis current controller.
 * The current reference itself is updated on every FOC callback, so the old
 * 4 kHz staircase is gone without bypassing VESC's current regulator.
 */

#define BH_FAST_MAX_CYCLE_POINTS          400
#define BH_FAST_MAX_PLOT_LOOPS_HZ         20.0f
#define BH_FAST_WARMUP_CYCLES             3
#define BH_FAST_HARD_CLIPS_PER_SEC        10
#define BH_FAST_OVERCURRENT_SAMPLES       4

/* foc_math.h still contains the MINI4 raw-SVM hook from the voltage-drive
 * prototype. Keep its linkage satisfied but leave it permanently disabled;
 * current-controlled fast mode never writes a raw modulation vector.
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
	volatile unsigned hard_clips;
} bh_fast_cycle_buf_t;

/* Forward-declared by hw_mini4_bh.h so bh_live's wrapper can dispatch here. */
static bool bh_fast_live_mode = false;
static volatile bool bh_fast_active = false;
static volatile bool bh_fast_external_stop = false;
static volatile bool bh_fast_clip_abort = false;
static volatile bool bh_fast_overcurrent_abort = false;

static volatile float bh_fast_last_amp = 0.0f;
static volatile float bh_fast_last_center = 0.0f;
static volatile float bh_fast_last_iref = 0.0f;
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

static inline float bh_fast_median3(float a, float b, float c) {
	if (a > b) { float t = a; a = b; b = t; }
	if (b > c) { float t = b; b = c; c = t; }
	if (a > b) { float t = a; a = b; b = t; }
	return b;
}

/* Update the ordinary FOC d-axis current target directly from the PWM callback.
 * The worker owns mc_interface's external-control lock, so use the FOC-level
 * setter rather than the public mc_interface wrapper. This is the same fixed
 * -30 degree geometry used everywhere else in the tracer:
 *
 *     Ia = +Iline, Ib = -Iline, Ic = 0
 *     Id = 2/sqrt(3) * Iline.
 *
 * VESC remains responsible for the current PI, SVM, deadtime, current-sample
 * placement, hardware current limits and gate timing.
 */
static inline void bh_fast_set_current_ref(float i_line) {
	float i_axis = i_line * BH_TWO_BY_SQRT3;
	mcpwm_foc_set_openloop_phase(i_axis, BH_FOC_PHASE_DEG);
	/* Crossing exactly through zero must not let FOC's idle-release housekeeping
	 * drop PWM between adjacent triangle samples. A real VESC Tool Stop changes
	 * MC_STATE_RUNNING before the next callback, which we detect at its top.
	 */
	mcpwm_foc_set_current_off_delay(0.10f);
	bh_fast_last_iref = i_line;
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
		b->hard_clips = bh_fast_hard_clips;
		bh_fast_ready_idx = bh_fast_write_idx;
		bh_fast_write_idx ^= 1;
	}
	bh_fast_write_count = 0;
}

static inline void bh_fast_abort_drive(void) {
	/* Return the current target to zero immediately. Hard-off is performed by
	 * the tracer thread a moment later through the normal safe cleanup path.
	 */
	bh_fast_set_current_ref(0.0f);
	bh_fast_active = false;
}

static void bh_fast_pwm_callback(void) {
	if (!bh_fast_active) {
		return;
	}

	/* VESC Tool Stop/release changes the FOC state underneath us. Never issue a
	 * new current command after that transition or we could re-enter RUNNING.
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

	/* Independent fixture sanity limit. The ordinary VESC absolute-current fault
	 * remains active as a separate hardware-level protection.
	 */
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

	/* A causal 3-sample median rejects isolated in-range PWM/common-mode spikes.
	 * Pair the integrated B sample with the middle current/phase sample so H and
	 * B remain time aligned to the median's one-sample group delay.
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
		bh_fast_publish_cycle();

		bh_fast_last_amp = 0.5f * (bh_fast_cycle_i_max - bh_fast_cycle_i_min);
		bh_fast_last_center = 0.5f * (bh_fast_cycle_i_max + bh_fast_cycle_i_min);

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

	/* Reference for the next FOC cycle. */
	bh_fast_set_current_ref(bh_i_pk * bh_fast_triangle(bh_fast_phase));
}

static bool bh_fast_install_tight_current_limits(void) {
	mc_configuration *cfg = mempools_alloc_mcconf();
	if (!cfg) {
		commands_printf("BH_FAST_ABORT could not allocate temporary current-limit config");
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
	bh_fast_hard_clips = 0;
	bh_fast_phase = 0.0f;
	bh_fast_sense_zero = sense_zero;
	bh_fast_cycle_n = 0;
	bh_fast_sample_n = 0;
	bh_fast_capture_cycle = false;
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

	/* Search-coil integration determines only relative B. Remove the residual
	 * linear volt-second closure error over the cycle and then center the loop's
	 * B extrema around zero. Acquisition itself remains untouched/raw.
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
	}

	bh_fast_ready_idx = -1;
}

static bool bh_fast_live_plot(float freq) {
	float sense_zero = bh_measure_sense_zero();
	if (!bh_sense_zero_ok) {
		commands_printf("BH_FAST_ABORT A1 zero invalid V=%.6f", (double)sense_zero);
		return false;
	}

	if (!bh_fast_install_tight_current_limits()) {
		return false;
	}
	bh_fast_reset_state(freq, sense_zero);

	commands_printf("BH_FAST_START Ipk=%.6f A f=%.6f Hz fs=%.1f Hz turns=%g/%g zero=%.7f V sense=%+d",
			(double)bh_i_pk, (double)freq, (double)bh_fast_fs,
			(double)BH_EXC_TURNS, (double)BH_SENSE_TURNS,
			(double)sense_zero, bh_sense_polarity);
	commands_printf("BH_FAST: PWM-rate triangle current reference through normal FOC current PI; full-rate H/B acquisition and median A1 impulse rejection.");

	commands_init_plot("H (A/m)", "B relative (T)");
	commands_plot_add_graph("B-H fast current");
	commands_plot_set_graph(0);

	/* Enter RUNNING gently before installing the PWM callback. hw_mini4_bh.h
	 * maps this startup-only zero command to a harmless 10 mA keeper current.
	 */
	bh_apply_current(0.0f);
	if (mc_interface_get_state() != MC_STATE_RUNNING) {
		commands_printf("BH_FAST_ABORT could not enter FOC RUNNING state");
		bh_release_locked();
		return false;
	}

	mc_interface_set_pwm_callback(bh_fast_pwm_callback);
	bh_fast_active = true;
	bh_fast_set_current_ref(0.0f);

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
			float track = bh_i_pk > 0.001f ? 100.0f * bh_fast_last_amp / bh_i_pk : 0.0f;
			commands_printf("BH_FAST_STAT Iamp=%.5f A track=%.2f%% Icenter=%+.5f A Iref=%+.5f A Id=%+.5f A Vd=%+.5f V duty=%+.5f clips=%u",
					(double)bh_fast_last_amp, (double)track,
					(double)bh_fast_last_center, (double)bh_fast_last_iref,
					(double)mcpwm_foc_get_id(), (double)mcpwm_foc_get_vd(),
					(double)mc_interface_get_duty_cycle_now(),
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
		commands_printf("Example: bh_live_fast 1.0 60 21.9 3.65");
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

	commands_printf("B-H fast current-controlled live run queued: Ipk=%.4f A f=%.4f Hz path=%.3f mm area=%.4f mm^2",
			(double)i_pk, (double)freq, (double)path_mm, (double)area_mm2);
}

static void bh_fast_init_commands(void) {
	terminal_register_command_callback(
			"bh_live_fast",
			"PWM-rate triangle-current B-H live plot through normal FOC current control.",
			"<Ipk_A> <freq_Hz> <path_mm> <area_mm2>",
			terminal_bh_live_fast);
}
