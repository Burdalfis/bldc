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

/* All downstream search-voltage calculations already divide by BH_INA_GAIN.
 * Making its sign depend on bh_sense_polarity applies the explicit polarity to
 * B integration, B-H plots, harmonics and impedance without altering the raw
 * 1.65 V-centered ADC reading or zero measurement.
 */
#undef BH_INA_GAIN
#define BH_INA_GAIN (50.0f / (float)bh_sense_polarity)

#define bh_set_measurement_gains bh_keep_programmed_measurement_gains
#define bh_set_run_gains bh_keep_programmed_run_gains
