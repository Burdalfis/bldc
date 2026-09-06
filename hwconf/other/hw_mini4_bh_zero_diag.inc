/* Static and active zero-state diagnostics for the MINI4 B-H fixture.
 *
 * The excitation winding is connected between phase outputs P1 and P2. This
 * diagnostic compares several genuinely static bridge states with the state
 * produced by the normal FOC current controller at a commanded 0 A.
 *
 * The active FOC-zero case is important because the tracer sets
 * cc_min_current=0. In VESC FOC that means an exact zero-current command still
 * enters MC_STATE_RUNNING and enables normal PWM modulation. Thus "0 A" is not
 * the same electrical state as all FETs off.
 */

typedef enum {
	BH_ZERO_FLOAT = 0,
	BH_ZERO_P1_GND,
	BH_ZERO_P2_GND,
	BH_ZERO_BOTH_GND,
	BH_ZERO_FOC_ZERO
} bh_zero_mode_t;

static const char *bh_zero_mode_name(bh_zero_mode_t mode) {
	switch (mode) {
	case BH_ZERO_FLOAT: return "float";
	case BH_ZERO_P1_GND: return "p1_gnd";
	case BH_ZERO_P2_GND: return "p2_gnd";
	case BH_ZERO_BOTH_GND: return "both_gnd";
	case BH_ZERO_FOC_ZERO: return "foc_zero";
	default: return "unknown";
	}
}

static void bh_zero_force_mode(bh_zero_mode_t mode) {
	/* Always start from the real FOC-off state. */
	timeout_reset();
	mc_interface_lock();
	mcpwm_foc_stop_pwm(false);

	if (mode == BH_ZERO_FLOAT) {
		return;
	}

	if (mode == BH_ZERO_FOC_ZERO) {
		/* bh_apply_current() uses the same locked-command path as the actual
		 * tracer. With the temporary tracer config below cc_min_current is 0,
		 * therefore this exact-zero command intentionally enters RUNNING and
		 * starts the ordinary FOC PWM zero-vector modulation.
		 */
		bh_apply_current(0.0f);
		return;
	}

	utils_sys_lock_cnt();

	/* Force all main outputs inactive and start with all complementary outputs
	 * disabled. This is the same polarity convention used by stop_pwm_hw().
	 */
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	if (mode == BH_ZERO_P1_GND || mode == BH_ZERO_BOTH_GND) {
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
	}
	if (mode == BH_ZERO_P2_GND || mode == BH_ZERO_BOTH_GND) {
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
	}

	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
	utils_sys_unlock_cnt();
}

typedef struct {
	uint16_t raw_min;
	uint16_t raw_max;
	float mean_v;
	float rms_v;
	float mean_icoil;
	int samples;
	int running_samples;
} bh_zero_stats_t;

static void bh_zero_collect_stats(bh_zero_mode_t mode, int hold_ms, bh_zero_stats_t *st) {
	memset(st, 0, sizeof(*st));
	st->raw_min = 4095;
	st->raw_max = 0;

	double sum_v = 0.0;
	double sum_v2 = 0.0;
	double sum_i = 0.0;
	uint32_t t0 = timer_time_now();

	while (timer_seconds_elapsed_since(t0) < (float)hold_ms * 1e-3f) {
		/* Do not let VESC's ordinary motor timeout unlock this diagnostic and
		 * overwrite a static bridge state with timeout_brake_current. For the
		 * active FOC-zero mode, reissuing the current command already resets the
		 * timeout. For the truly static modes we only reset timeout + reassert the
		 * lock; do NOT call bh_keep_hard_off_alive(), because that would destroy
		 * the intentionally forced low-side state we are measuring.
		 */
		if (mode == BH_ZERO_FOC_ZERO) {
			bh_apply_current(0.0f);
		} else {
			timeout_reset();
			mc_interface_lock();
		}

		uint16_t raw = ADC_Value[ADC_IND_EXT];
		float v = bh_sense_out_v();
		float i = bh_coil_current();
		if (raw < st->raw_min) st->raw_min = raw;
		if (raw > st->raw_max) st->raw_max = raw;
		sum_v += v;
		sum_v2 += (double)v * v;
		sum_i += i;
		if (mc_interface_get_state() == MC_STATE_RUNNING) {
			st->running_samples++;
		}
		st->samples++;
		chThdSleepMicroseconds(200);
	}

	if (st->samples > 0) {
		st->mean_v = (float)(sum_v / (double)st->samples);
		double mean_v2 = sum_v2 / (double)st->samples;
		double var = mean_v2 - (double)st->mean_v * st->mean_v;
		if (var < 0.0) var = 0.0;
		st->rms_v = (float)sqrt(var);
		st->mean_icoil = (float)(sum_i / (double)st->samples);
	}
}

static void bh_zero_print_stats(bh_zero_mode_t mode, const bh_zero_stats_t *st) {
	float vmin = (float)st->raw_min * (3.3f / 4095.0f);
	float vmax = (float)st->raw_max * (3.3f / 4095.0f);
	float running_pct = st->samples > 0 ?
			100.0f * (float)st->running_samples / (float)st->samples : 0.0f;
	commands_printf("BH_ZERO_RESULT mode=%s samples=%d running=%.1f%% A1mean=%.7f V A1rms=%.7f V A1min=%u(%.6f V) A1max=%u(%.6f V) A1pp=%.6f V Icoil_mean=%.6f A",
			bh_zero_mode_name(mode), st->samples, (double)running_pct,
			(double)st->mean_v, (double)st->rms_v,
			(unsigned)st->raw_min, (double)vmin,
			(unsigned)st->raw_max, (double)vmax,
			(double)(vmax - vmin), (double)st->mean_icoil);
}

static bool bh_zero_parse_mode(const char *s, bh_zero_mode_t *mode) {
	if (!strcmp(s, "float")) {
		*mode = BH_ZERO_FLOAT;
	} else if (!strcmp(s, "p1") || !strcmp(s, "p1_gnd")) {
		*mode = BH_ZERO_P1_GND;
	} else if (!strcmp(s, "p2") || !strcmp(s, "p2_gnd")) {
		*mode = BH_ZERO_P2_GND;
	} else if (!strcmp(s, "both") || !strcmp(s, "both_gnd")) {
		*mode = BH_ZERO_BOTH_GND;
	} else if (!strcmp(s, "foc") || !strcmp(s, "foc0") || !strcmp(s, "foc_zero")) {
		*mode = BH_ZERO_FOC_ZERO;
	} else {
		return false;
	}
	return true;
}

static void bh_zero_run_one(bh_zero_mode_t mode, int hold_ms) {
	bh_zero_force_mode(mode);
	/* Give MOSFET/gate-driver transients and the INA output a moment to settle.
	 * Keep the normal motor timeout alive during this dwell too, otherwise a
	 * short configured timeout can undo the state before statistics even start.
	 */
	for (int i = 0; i < 10; i++) {
		if (mode == BH_ZERO_FOC_ZERO) {
			bh_apply_current(0.0f);
		} else {
			timeout_reset();
			mc_interface_lock();
		}
		chThdSleepMilliseconds(1);
	}
	bh_zero_stats_t st;
	bh_zero_collect_stats(mode, hold_ms, &st);
	bh_zero_print_stats(mode, &st);
}

/* Apply the same core FOC settings used by a real tracer job while the motor is
 * already locked. This removes ordinary app/CAN commands from the experiment
 * and guarantees cc_min_current=0 for the active-zero state.
 */
static bool bh_zero_prepare_cfg(mc_configuration **old_out, mc_configuration **test_out) {
	mc_configuration *old_cfg = mempools_alloc_mcconf();
	mc_configuration *test_cfg = mempools_alloc_mcconf();
	if (!old_cfg || !test_cfg) {
		if (old_cfg) mempools_free_mcconf(old_cfg);
		if (test_cfg) mempools_free_mcconf(test_cfg);
		commands_printf("BH_ZERO_ABORT could not allocate motor configuration");
		return false;
	}

	*old_cfg = *mc_interface_get_configuration();
	*test_cfg = *old_cfg;
	bh_common_cfg(test_cfg, 1.0f);

	timeout_reset();
	mc_interface_lock();
	mcpwm_foc_stop_pwm(false);
	mc_interface_set_configuration(test_cfg);
	bh_release_locked();
	bh_hold_hard_off_ms(20);

	*old_out = old_cfg;
	*test_out = test_cfg;
	return true;
}

static void bh_zero_restore_cfg(mc_configuration *old_cfg, mc_configuration *test_cfg) {
	bh_release_locked();
	bh_hold_hard_off_ms(5);
	mc_interface_set_configuration(old_cfg);
	bh_release_locked();
	mc_interface_unlock();
	mempools_free_mcconf(test_cfg);
	mempools_free_mcconf(old_cfg);
}

static void terminal_bh_zero_test(int argc, const char **argv) {
	if (bh_is_busy()) {
		commands_printf("A B-H job is already running or queued. Use bh_stop first.");
		return;
	}

	int hold_ms = 250;
	if (argc == 2) {
		hold_ms = atoi(argv[1]);
	} else if (argc != 1) {
		commands_printf("Usage: bh_zero_test [hold_ms]");
		commands_printf("Runs float, P1-low, P2-low, both-low and active FOC-zero states; default 250 ms each.");
		return;
	}
	if (hold_ms < 20 || hold_ms > 2000) {
		commands_printf("BH zero-test hold must be 20..2000 ms per state.");
		return;
	}

	commands_printf("BH_ZERO_TEST_START hold=%d ms/state", hold_ms);
	commands_printf("Modes: float=all FETs off; p1_gnd=P1 low-side on; p2_gnd=P2 low-side on; both_gnd=P1+P2 low-sides on; foc_zero=0 A command with normal 30 kHz FOC PWM active.");

	/* Reuse bh_running so the existing Sampled Data phase-C overlay continues to
	 * show A1 while this diagnostic is active.
	 */
	bh_running = true;
	bh_stop_requested = false;

	mc_configuration *old_cfg = 0;
	mc_configuration *test_cfg = 0;
	if (!bh_zero_prepare_cfg(&old_cfg, &test_cfg)) {
		bh_running = false;
		return;
	}

	bh_zero_run_one(BH_ZERO_FLOAT, hold_ms);
	bh_zero_run_one(BH_ZERO_P1_GND, hold_ms);
	bh_zero_run_one(BH_ZERO_P2_GND, hold_ms);
	bh_zero_run_one(BH_ZERO_BOTH_GND, hold_ms);
	bh_zero_run_one(BH_ZERO_FOC_ZERO, hold_ms);

	bh_zero_force_mode(BH_ZERO_FLOAT);
	bh_zero_restore_cfg(old_cfg, test_cfg);
	bh_running = false;
	commands_printf("BH_ZERO_TEST_DONE bridge released");
}

static void terminal_bh_zero_hold(int argc, const char **argv) {
	if (bh_is_busy()) {
		commands_printf("A B-H job is already running or queued. Use bh_stop first.");
		return;
	}
	if (argc != 3) {
		commands_printf("Usage: bh_zero_hold <float|p1|p2|both|foc> <hold_ms>");
		return;
	}
	bh_zero_mode_t mode;
	if (!bh_zero_parse_mode(argv[1], &mode)) {
		commands_printf("Unknown zero mode. Use float, p1, p2, both or foc.");
		return;
	}
	int hold_ms = atoi(argv[2]);
	if (hold_ms < 20 || hold_ms > 5000) {
		commands_printf("BH zero-hold time must be 20..5000 ms.");
		return;
	}

	bh_running = true;
	bh_stop_requested = false;

	mc_configuration *old_cfg = 0;
	mc_configuration *test_cfg = 0;
	if (!bh_zero_prepare_cfg(&old_cfg, &test_cfg)) {
		bh_running = false;
		return;
	}

	commands_printf("BH_ZERO_HOLD mode=%s hold=%d ms", bh_zero_mode_name(mode), hold_ms);
	bh_zero_run_one(mode, hold_ms);
	bh_zero_force_mode(BH_ZERO_FLOAT);
	bh_zero_restore_cfg(old_cfg, test_cfg);
	bh_running = false;
	commands_printf("BH_ZERO_HOLD_DONE bridge released");
}

static void bh_zero_diag_init_commands(void) {
	terminal_register_command_callback(
			"bh_zero_test",
			"Compare INA/A1 noise with static excitation states and active 0-A FOC PWM.",
			"[hold_ms]",
			terminal_bh_zero_test);
	terminal_register_command_callback(
			"bh_zero_hold",
			"Hold one excitation zero state and report INA/A1 noise plus RUNNING-state fraction.",
			"<float|p1|p2|both|foc> <hold_ms>",
			terminal_bh_zero_hold);
}
