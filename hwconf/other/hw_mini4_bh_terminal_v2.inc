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
	commands_printf("BH inductance matrix queued: I %.6f..%.6f A @ %.2f pts/oct, f %.6f..%.6f Hz @ %.2f pts/oct, sense=%+d",
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
	bh_init_commands_legacy();
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
