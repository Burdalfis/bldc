/* Sampled-data scope overlay for the dedicated MINI4 B-H fixture.
 *
 * mc_interface invokes the PWM callback after the FOC/current calculations but
 * before it copies the debug-sampling channels. Phase C is physically unused
 * by this fixture, so while a tracer job is running replace the sampled phase-C
 * current channel with INA282 OUT relative to its nominal 1.65 V centre.
 *
 * VESC Tool will still label this trace as amperes. For the B-H target only,
 * interpret 1.000 displayed A on phase C as +1.000 V from 1.65 V at INA OUT.
 * Raw sampled-data mode is intentionally not remapped; use normal/non-raw mode.
 */
static void bh_sample_scope_pwm_cb(void) {
	if (bh_running) {
		ADC_curr_norm_value[2] = bh_sense_out_v() - 1.65f;
	}
}

static void terminal_bh_ctrl_print(void) {
	const volatile mc_configuration *cfg = mc_interface_get_configuration();
	float bw_kp = 0.0f;
	float bw_ki = 0.0f;
	if (cfg->foc_motor_l > 1e-12f) {
		bw_kp = cfg->foc_current_kp / (2.0f * M_PI * cfg->foc_motor_l);
	}
	if (cfg->foc_motor_r > 1e-12f) {
		bw_ki = cfg->foc_current_ki / (2.0f * M_PI * cfg->foc_motor_r);
	}
	commands_printf("BH_CTRL_PROGRAMMED Kp=%.7f Ki=%.7f R=%.6f mOhm L=%.6f uH BWkp=%.2f Hz BWki=%.2f Hz",
			(double)cfg->foc_current_kp, (double)cfg->foc_current_ki,
			(double)(cfg->foc_motor_r * 1e3f), (double)(cfg->foc_motor_l * 1e6f),
			(double)bw_kp, (double)bw_ki);
}

static bool bh_ctrl_apply(float kp, float ki, bool save) {
	if (bh_is_busy()) {
		commands_printf("Cannot change current-controller gains while a B-H job is active.");
		return false;
	}
	if (!(kp > 0.0f && kp <= 5.0f) || !(ki > 0.0f && ki <= 5000.0f)) {
		commands_printf("BH controller limits: 0<Kp<=5, 0<Ki<=5000.");
		return false;
	}

	mc_configuration *cfg = mempools_alloc_mcconf();
	if (!cfg) {
		commands_printf("BH_CTRL_ABORT could not allocate motor configuration.");
		return false;
	}
	*cfg = *mc_interface_get_configuration();
	cfg->foc_current_kp = kp;
	cfg->foc_current_ki = ki;
	mc_interface_set_configuration(cfg);

	bool stored = true;
	if (save) {
		stored = conf_general_store_mc_configuration(cfg, false);
	}
	mempools_free_mcconf(cfg);

	if (save) {
		commands_printf("BH controller Kp=%.7f Ki=%.7f applied; EEPROM save %s.",
				(double)kp, (double)ki, stored ? "OK" : "FAILED");
	} else {
		commands_printf("BH controller Kp=%.7f Ki=%.7f applied in RAM only.",
				(double)kp, (double)ki);
	}
	terminal_bh_ctrl_print();
	return stored;
}

static void terminal_bh_ctrl(int argc, const char **argv) {
	if (argc == 1) {
		terminal_bh_ctrl_print();
		return;
	}
	if (argc != 3) {
		commands_printf("Usage: bh_ctrl [Kp Ki]");
		commands_printf("Example: bh_ctrl 0.06 188");
		return;
	}
	bh_ctrl_apply(strtof(argv[1], 0), strtof(argv[2], 0), false);
}

static void terminal_bh_ctrl_save(int argc, const char **argv) {
	if (argc != 3) {
		commands_printf("Usage: bh_ctrl_save <Kp> <Ki>");
		commands_printf("Example: bh_ctrl_save 0.06 188");
		return;
	}
	bh_ctrl_apply(strtof(argv[1], 0), strtof(argv[2], 0), true);
}

static void terminal_bh_sense_pol(int argc, const char **argv) {
	if (bh_is_busy()) {
		commands_printf("Cannot change sense polarity while a B-H job is active.");
		return;
	}
	if (argc == 1) {
		commands_printf("BH sense polarity=%+d (RAM only). Use bh_sense_pol 1 or bh_sense_pol -1.",
				bh_sense_polarity);
		return;
	}
	if (argc != 2) {
		commands_printf("Usage: bh_sense_pol <1|-1>");
		return;
	}
	int pol = atoi(argv[1]);
	if (pol != 1 && pol != -1) {
		commands_printf("Sense polarity must be +1 or -1.");
		return;
	}
	bh_sense_polarity = pol;
	commands_printf("BH sense polarity set to %+d (RAM only).", bh_sense_polarity);
}

static int bh_log_sweep_points(float lo, float hi, float ppo) {
	if (!(lo > 0.0f && hi >= lo && ppo > 0.0f)) {
		return 0;
	}
	float oct = logf(hi / lo) / logf(2.0f);
	return (int)floorf(oct * ppo + 1e-5f) + 1;
}

static bool bh_queue_l_sweep(float imin, float imax, float ippo,
		float fmin, float fmax, float fppo) {
	if (bh_is_busy()) {
		commands_printf("A B-H job is already running or queued. Use bh_stop first.");
		return false;
	}
	if (!(imin > 0.0f && imax >= imin && imax <= BH_MAX_CURRENT_A) ||
			!(ippo >= 1.0f && ippo <= 24.0f) ||
			!(fmin >= BH_RL_MIN_FREQ_HZ && fmax >= fmin && fmax <= BH_RL_MAX_FREQ_HZ) ||
			!(fppo >= 1.0f && fppo <= 24.0f)) {
		commands_printf("L-sweep limits: 0<Imin<=Imax<=%.1f A, %.1f<=fmin<=fmax<=%.1f Hz, 1..24 points/octave.",
				(double)BH_MAX_CURRENT_A, (double)BH_RL_MIN_FREQ_HZ,
				(double)BH_RL_MAX_FREQ_HZ);
		return false;
	}

	int ipoints = bh_log_sweep_points(imin, imax, ippo);
	int fpoints = bh_log_sweep_points(fmin, fmax, fppo);
	if (ipoints < 1 || fpoints < 1 || ipoints * fpoints > BH_MAX_SWEEP_POINTS) {
		commands_printf("L-sweep matrix too large: %d current x %d frequency = %d points; max=%d.",
				ipoints, fpoints, ipoints * fpoints, BH_MAX_SWEEP_POINTS);
		return false;
	}

	bh_l_i_min = imin;
	bh_l_i_max = imax;
	bh_l_i_ppo = ippo;
	bh_i_pk = imax; // lets bh_common_cfg raise current limits for the whole matrix
	bh_fmin = fmin;
	bh_fmax = fmax;
	bh_sweep_step = fppo; // existing logarithmic worker is the frequency axis
	bh_cycles = 1;
	bh_l_sweep_mode = true;
	bh_l_sweep_suppress_metric = false;
	if (!bh_queue_job(BH_JOB_SWEEP_LOG)) {
		bh_l_sweep_mode = false;
		return false;
	}
	commands_printf("BH inductance matrix queued: %d points (%d I x %d f), I %.6f..%.6f A @ %.2f pts/oct, f %.6f..%.6f Hz @ %.2f pts/oct, sense=%+d",
			ipoints * fpoints, ipoints, fpoints,
			(double)imin, (double)imax, (double)ippo,
			(double)fmin, (double)fmax, (double)fppo,
			bh_sense_polarity);
	return true;
}

static void terminal_bh_l_sweep(int argc, const char **argv) {
	if (argc != 7) {
		commands_printf("Usage: bh_l_sweep <Imin_A> <Imax_A> <I_points_per_octave> <fmin_Hz> <fmax_Hz> <f_points_per_octave>");
		commands_printf("Example: bh_l_sweep 0.05 3.2 1 5 200 1");
		return;
	}
	bh_queue_l_sweep(strtof(argv[1], 0), strtof(argv[2], 0), strtof(argv[3], 0),
			strtof(argv[4], 0), strtof(argv[5], 0), strtof(argv[6], 0));
}

static void terminal_bh_l_point(int argc, const char **argv) {
	if (argc != 3) {
		commands_printf("Usage: bh_l_point <Ipk_A> <freq_Hz>");
		commands_printf("Example: bh_l_point 0.25 50");
		return;
	}
	float i = strtof(argv[1], 0);
	float f = strtof(argv[2], 0);
	bh_queue_l_sweep(i, i, 1.0f, f, f, 1.0f);
}

static void bh_init_commands(void) {
	// Dedicated target: claim the single global PWM callback for the sampled-data
	// scope overlay. The callback is inert unless a B-H job is actually running.
	mc_interface_set_pwm_callback(bh_sample_scope_pwm_cb);
	bh_init_commands_legacy();
	terminal_register_command_callback(
			"bh_ctrl",
			"Get or set the programmed FOC current-controller Kp/Ki without VESC Tool motor-config access.",
			"[Kp Ki]",
			terminal_bh_ctrl);
	terminal_register_command_callback(
			"bh_ctrl_save",
			"Set FOC current-controller Kp/Ki and persist the motor configuration to EEPROM.",
			"<Kp> <Ki>",
			terminal_bh_ctrl_save);
	terminal_register_command_callback(
			"bh_sense_pol",
			"Get or set search-winding polarity used by B-H integration and diagnostics.",
			"[1|-1]",
			terminal_bh_sense_pol);
	terminal_register_command_callback(
			"bh_l_point",
			"Measure one search-coil-derived inductance/phase point using programmed FOC Kp/Ki.",
			"<Ipk_A> <freq_Hz>",
			terminal_bh_l_point);
	terminal_register_command_callback(
			"bh_l_sweep",
			"Sweep excitation current and frequency; report tracking, phase, Rmag, Xmag and signed L.",
			"<Imin_A> <Imax_A> <I_points_per_octave> <fmin_Hz> <fmax_Hz> <f_points_per_octave>",
			terminal_bh_l_sweep);
}
