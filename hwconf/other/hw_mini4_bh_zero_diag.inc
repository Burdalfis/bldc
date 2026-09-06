/* Static excitation-winding zero-state diagnostics for the MINI4 B-H fixture.
 *
 * The excitation winding is connected between phase outputs P1 and P2. When
 * the bridge is released both ends of that winding float. This diagnostic can
 * instead turn on one or both low-side MOSFETs while keeping every high-side
 * MOSFET off, then quantify the INA282/A1 noise in each state.
 *
 * This deliberately mirrors VESC FOC's stop_pwm_hw/full_brake_hw timer setup:
 * forced inactive main output means high side OFF; enabling the complementary
 * output turns the corresponding low-side MOSFET ON. Phase C remains off.
 */

typedef enum {
	BH_ZERO_FLOAT = 0,
	BH_ZERO_P1_GND,
	BH_ZERO_P2_GND,
	BH_ZERO_BOTH_GND
} bh_zero_mode_t;

static const char *bh_zero_mode_name(bh_zero_mode_t mode) {
	switch (mode) {
	case BH_ZERO_FLOAT: return "float";
	case BH_ZERO_P1_GND: return "p1_gnd";
	case BH_ZERO_P2_GND: return "p2_gnd";
	case BH_ZERO_BOTH_GND: return "both_gnd";
	default: return "unknown";
	}
}

static void bh_zero_force_mode(bh_zero_mode_t mode) {
	/* Put FOC's internal PWM state into the normal disabled state first. This
	 * also guarantees all three high/low outputs are off before we selectively
	 * enable any low side below. The internal mode remains disabled, so the
	 * next ordinary FOC command will restart PWM normally.
	 */
	mcpwm_foc_stop_pwm(false);

	if (mode == BH_ZERO_FLOAT) {
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
} bh_zero_stats_t;

static void bh_zero_collect_stats(int hold_ms, bh_zero_stats_t *st) {
	memset(st, 0, sizeof(*st));
	st->raw_min = 4095;
	st->raw_max = 0;

	double sum_v = 0.0;
	double sum_v2 = 0.0;
	double sum_i = 0.0;
	uint32_t t0 = timer_time_now();

	while (timer_seconds_elapsed_since(t0) < (float)hold_ms * 1e-3f) {
		uint16_t raw = ADC_Value[ADC_IND_EXT];
		float v = bh_sense_out_v();
		float i = bh_coil_current();
		if (raw < st->raw_min) st->raw_min = raw;
		if (raw > st->raw_max) st->raw_max = raw;
		sum_v += v;
		sum_v2 += (double)v * v;
		sum_i += i;
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
	commands_printf("BH_ZERO_RESULT mode=%s samples=%d A1mean=%.7f V A1rms=%.7f V A1min=%u(%.6f V) A1max=%u(%.6f V) A1pp=%.6f V Icoil_mean=%.6f A",
			bh_zero_mode_name(mode), st->samples,
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
	} else {
		return false;
	}
	return true;
}

static void bh_zero_run_one(bh_zero_mode_t mode, int hold_ms) {
	bh_zero_force_mode(mode);
	/* Give MOSFET/gate-driver transients and the INA output a moment to settle
	 * before collecting statistics. Any long-lived instability remains visible.
	 */
	chThdSleepMilliseconds(10);
	bh_zero_stats_t st;
	bh_zero_collect_stats(hold_ms, &st);
	bh_zero_print_stats(mode, &st);
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
		commands_printf("Runs float, P1-low, P2-low and both-low states; default 250 ms each.");
		return;
	}
	if (hold_ms < 20 || hold_ms > 2000) {
		commands_printf("BH zero-test hold must be 20..2000 ms per state.");
		return;
	}

	commands_printf("BH_ZERO_TEST_START hold=%d ms/state", hold_ms);
	commands_printf("Modes: float=all FETs off; p1_gnd=P1 low-side on; p2_gnd=P2 low-side on; both_gnd=P1+P2 low-sides on.");

	/* Reuse bh_running so the existing Sampled Data phase-C overlay continues to
	 * show A1 while this diagnostic is active. No excitation current is commanded.
	 */
	bh_running = true;
	bh_stop_requested = false;
	mc_interface_lock();

	bh_zero_run_one(BH_ZERO_FLOAT, hold_ms);
	bh_zero_run_one(BH_ZERO_P1_GND, hold_ms);
	bh_zero_run_one(BH_ZERO_P2_GND, hold_ms);
	bh_zero_run_one(BH_ZERO_BOTH_GND, hold_ms);

	bh_zero_force_mode(BH_ZERO_FLOAT);
	mc_interface_unlock();
	bh_running = false;
	commands_printf("BH_ZERO_TEST_DONE bridge released");
}

static void terminal_bh_zero_hold(int argc, const char **argv) {
	if (bh_is_busy()) {
		commands_printf("A B-H job is already running or queued. Use bh_stop first.");
		return;
	}
	if (argc != 3) {
		commands_printf("Usage: bh_zero_hold <float|p1|p2|both> <hold_ms>");
		return;
	}
	bh_zero_mode_t mode;
	if (!bh_zero_parse_mode(argv[1], &mode)) {
		commands_printf("Unknown zero mode. Use float, p1, p2 or both.");
		return;
	}
	int hold_ms = atoi(argv[2]);
	if (hold_ms < 20 || hold_ms > 5000) {
		commands_printf("BH zero-hold time must be 20..5000 ms.");
		return;
	}

	bh_running = true;
	bh_stop_requested = false;
	mc_interface_lock();
	commands_printf("BH_ZERO_HOLD mode=%s hold=%d ms", bh_zero_mode_name(mode), hold_ms);
	bh_zero_run_one(mode, hold_ms);
	bh_zero_force_mode(BH_ZERO_FLOAT);
	mc_interface_unlock();
	bh_running = false;
	commands_printf("BH_ZERO_HOLD_DONE bridge released");
}

static void bh_zero_diag_init_commands(void) {
	terminal_register_command_callback(
			"bh_zero_test",
			"Compare INA/A1 noise with excitation winding floating, one end grounded, or both ends grounded by low-side FETs.",
			"[hold_ms]",
			terminal_bh_zero_test);
	terminal_register_command_callback(
			"bh_zero_hold",
			"Hold one static excitation-winding zero state and report INA/A1 noise statistics.",
			"<float|p1|p2|both> <hold_ms>",
			terminal_bh_zero_hold);
}
