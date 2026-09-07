/* PWM-rate current-controlled B-H live mode.
 *
 * CPU diagnostic variant: keep only the pieces needed to generate the
 * triangle current reference, track measured current and preserve the normal
 * VESC fault/Stop safety path. Full INA282/H/B acquisition and plotting are
 * intentionally absent in this build so we can measure the cost of the
 * current-reference callback itself on MINI4.
 *
 * The FOC/current PI still runs at its normal full rate. Only the extra tracer
 * callback work is decimated.
 */

#define BH_FAST_OVERCURRENT_SAMPLES       4
#define BH_FAST_DIAG_DIV                  2U

/* foc_math.h still contains the MINI4 raw-SVM hook from the old voltage-drive
 * prototype. Keep its linkage satisfied but leave it permanently disabled.
 */
volatile bool bh_foc_svm_override_active = false;
volatile float bh_foc_svm_override_alpha = 0.0f;
volatile float bh_foc_svm_override_beta = 0.0f;

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
static float bh_fast_update_hz = 15000.0f;
static float bh_fast_phase = 0.0f;
static float bh_fast_phase_step = 0.0f;
static float bh_fast_oc_lim = 1.0f;
static unsigned bh_fast_diag_div_count = 0U;
static int bh_fast_cycle_n = 0;
static int bh_fast_sample_n = 0;
static float bh_fast_cycle_i_max = -1.0e30f;
static float bh_fast_cycle_i_min = 1.0e30f;
static int bh_fast_overcurrent_count = 0;

static inline float bh_fast_triangle(float p) {
	if (p < 0.25f) {
		return 4.0f * p;
	} else if (p < 0.75f) {
		return 2.0f - 4.0f * p;
	}
	return 4.0f * p - 4.0f;
}

/* Update the ordinary FOC d-axis current target. Fixed A-B geometry:
 *
 *     Ia = +Iline, Ib = -Iline, Ic = 0
 *     Id = 2/sqrt(3) * Iline.
 *
 * VESC remains responsible for the full-rate current PI, SVM, deadtime,
 * current-sample placement, hardware current limits and gate timing.
 */
static inline void bh_fast_set_current_ref(float i_line) {
	float i_axis = i_line * BH_TWO_BY_SQRT3;
	mcpwm_foc_set_openloop_phase(i_axis, BH_FOC_PHASE_DEG);
	bh_fast_last_iref = i_line;
}

static inline void bh_fast_abort_drive(void) {
	bh_fast_set_current_ref(0.0f);
	bh_fast_active = false;
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
	(void)sense_zero;
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
	bh_fast_cycle_n = 0;
	bh_fast_sample_n = 0;
	bh_fast_cycle_i_max = -1.0e30f;
	bh_fast_cycle_i_min = 1.0e30f;
	bh_fast_overcurrent_count = 0;
	bh_fast_diag_div_count = 0U;

	bh_fast_fs = mc_interface_get_sampling_frequency_now();
	if (bh_fast_fs < 1000.0f) {
		bh_fast_fs = 30000.0f;
	}
	bh_fast_update_hz = bh_fast_fs / (float)BH_FAST_DIAG_DIV;
	bh_fast_phase_step = freq / bh_fast_update_hz;
	bh_fast_oc_lim = fmaxf(bh_i_pk * 1.50f, bh_i_pk + 0.75f);
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

	commands_printf("BH_FAST_START Ipk=%.6f A f=%.6f Hz foc_fs=%.1f Hz tracer_fs=%.1f Hz div=%u turns=%g/%g zero=%.7f V sense=%+d",
			(double)bh_i_pk, (double)freq, (double)bh_fast_fs,
			(double)bh_fast_update_hz, (unsigned)BH_FAST_DIAG_DIV,
			(double)BH_EXC_TURNS, (double)BH_SENSE_TURNS,
			(double)sense_zero, bh_sense_polarity);
	commands_printf("BH_FAST CPU diagnostic: tracer reference/current checks at 1/%u FOC cadence; H/B acquisition and plot transport disabled.",
			(unsigned)BH_FAST_DIAG_DIV);

	/* Enter RUNNING gently before installing the callback. hw_mini4_bh.h maps
	 * this startup-only zero command to a harmless 10 mA keeper current.
	 */
	bh_apply_current(0.0f);
	if (mc_interface_get_state() != MC_STATE_RUNNING) {
		commands_printf("BH_FAST_ABORT could not enter FOC RUNNING state");
		bh_release_locked();
		return false;
	}

	/* The old callback refreshed this on every FOC invocation. It is simply a
	 * hold-time setting, so install it once before the diagnostic starts.
	 */
	mcpwm_foc_set_current_off_delay(0.10f);
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
