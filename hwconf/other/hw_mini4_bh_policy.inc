/* MINI4 B-H tracer policy overrides.
 *
 * Keep the user's programmed VESC current-controller gains. The fixture is a
 * fixed-axis load and the operator can tune Kp/Ki directly in VESC Tool. The
 * old tracer auto-tuning helpers remain compiled for reference, but calls made
 * by the measurement and run paths below are redirected to no-ops.
 *
 * Search-winding polarity is explicit and RAM-only. +1 means increasing
 * positive excitation flux should produce positive corrected Vsense in the
 * tracer convention; -1 flips the sense winding in software.
 */
static int bh_sense_polarity = 1;

static bool bh_l_sweep_mode = false;
static bool bh_l_sweep_suppress_metric = false;
static float bh_l_i_min = 0.05f;
static float bh_l_i_max = 3.2f;
static float bh_l_i_ppo = 1.0f;

static void bh_keep_programmed_measurement_gains(mc_configuration *cfg, float r_line_hint) {
	(void)cfg;
	(void)r_line_hint;
}

static void bh_keep_programmed_run_gains(mc_configuration *cfg) {
	(void)cfg;
}

/* The generic FOC "release motor" path does not mean "state OFF right now".
 * It asks the FOC controller to release, which can leave MC_STATE_RUNNING set
 * while the release state is being processed. That is fine for a normal ESC,
 * but it is bad for this fixture for two reasons:
 *   1. Sampled Data's motor-start trigger keys directly off MC_STATE_RUNNING.
 *   2. We need the excitation winding genuinely undriven while measuring A1
 *      zero, not merely commanded to zero current.
 *
 * There is a second trap: VESC's ordinary motor-timeout thread calls
 * mc_interface_unlock() and then commands timeout_brake_current. With the usual
 * 0 A timeout brake current and cc_min_current=0 that can restart active FOC
 * zero-vector PWM in the middle of an otherwise static A1-zero measurement.
 * Keep feeding the motor timeout while the tracer intentionally holds the
 * bridge off, and reclaim the motor lock before checking/forcing MC_STATE_OFF.
 */
static void bh_release_hard_off(void) {
	timeout_reset();
	mc_interface_lock();
	mcpwm_foc_stop_pwm(false);
}

static inline void bh_keep_hard_off_alive(void) {
	timeout_reset();
	mc_interface_lock();
	if (mc_interface_get_state() != MC_STATE_OFF) {
		mcpwm_foc_stop_pwm(false);
	}
}

static void bh_hold_hard_off_ms(int hold_ms) {
	for (int i = 0; i < hold_ms && !bh_stop_requested; i++) {
		bh_keep_hard_off_alive();
		chThdSleepMilliseconds(1);
	}
}

static float bh_programmed_current_bw_hz(void) {
	const volatile mc_configuration *cfg = mc_interface_get_configuration();
	float sum = 0.0f;
	int n = 0;
	if (bh_rl_valid && bh_l_line_h > 1e-9f && cfg->foc_current_kp > 0.0f) {
		float l_d = 0.5f * bh_l_line_h;
		sum += cfg->foc_current_kp / l_d / (2.0f * M_PI);
		n++;
	}
	if (bh_rl_valid && bh_r_line_ohm > 1e-6f && cfg->foc_current_ki > 0.0f) {
		float r_d = 0.5f * bh_r_line_ohm;
		sum += cfg->foc_current_ki / r_d / (2.0f * M_PI);
		n++;
	}
	return n > 0 ? sum / (float)n : 0.0f;
}

/* All downstream search-voltage calculations already divide by BH_INA_GAIN.
 * Making its sign depend on bh_sense_polarity applies the explicit polarity to
 * B integration, B-H plots, harmonics and impedance without altering the raw
 * 1.65 V-centered ADC reading or zero measurement.
 */
#undef BH_INA_GAIN
#define BH_INA_GAIN (50.0f / (float)bh_sense_polarity)

/* Legacy status text prints BH_CURRENT_BW_HZ. Once gains are user-controlled,
 * derive that number from programmed Kp/Ki and the measured fixed-axis plant
 * instead of claiming the old hard-coded 250 Hz design target.
 */
#undef BH_CURRENT_BW_HZ
#define BH_CURRENT_BW_HZ (bh_programmed_current_bw_hz())

#define bh_set_measurement_gains bh_keep_programmed_measurement_gains
#define bh_set_run_gains bh_keep_programmed_run_gains
#define bh_release_locked bh_release_hard_off
