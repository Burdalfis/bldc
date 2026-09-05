static inline bool bh_is_busy(void) {
	return bh_running || bh_job != BH_JOB_NONE;
}

static void terminal_bh_adc(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	commands_printf("BH A1 raw=%u out=%.6f V, Ia=%.5f A Ib=%.5f A Icoil=%.5f A Vab=%.6f V",
			(unsigned)ADC_Value[ADC_IND_EXT], (double)bh_sense_out_v(),
			(double)ADC_curr_norm_value[0], (double)ADC_curr_norm_value[1],
			(double)bh_coil_current(), (double)bh_line_voltage());
}

static void terminal_bh_status(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	commands_printf("BH status: running=%d queued=%d A1=%u %.6f V",
			bh_running ? 1 : 0, (int)bh_job, (unsigned)ADC_Value[ADC_IND_EXT],
			(double)bh_sense_out_v());
	if (bh_rl_valid) {
		commands_printf("BH fixture Rline=%.6f mOhm Lline=%.6f uH; FOC Rd=%.6f mOhm Ld=%.6f uH; current BW=%.1f Hz",
				(double)(bh_r_line_ohm * 1e3f), (double)(bh_l_line_h * 1e6f),
				(double)(bh_r_line_ohm * 500.0f), (double)(bh_l_line_h * 500000.0f),
				(double)BH_CURRENT_BW_HZ);
	} else {
		commands_printf("BH fixture R/L not measured. Run bh_measure_rl first.");
	}
}

static void terminal_bh_stop(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	if (bh_running) {
		bh_stop_requested = true;
		commands_printf("B-H job stop requested.");
	} else if (bh_job != BH_JOB_NONE) {
		bh_job = BH_JOB_NONE;
		bh_stop_requested = false;
		commands_printf("Queued B-H job cancelled.");
	} else {
		commands_printf("No B-H job is running.");
	}
}

static void terminal_bh_measure_rl(int argc, const char **argv) {
	if (bh_is_busy()) {
		commands_printf("A B-H job is already running or queued. Use bh_stop first.");
		return;
	}
	float current = BH_RL_DEFAULT_CURRENT_A;
	float freq = BH_RL_DEFAULT_FREQ_HZ;
	if (argc != 1 && argc != 3) {
		commands_printf("Usage: bh_measure_rl [I_A freq_Hz]");
		commands_printf("Default: bh_measure_rl %.3f %.1f", (double)current, (double)freq);
		return;
	}
	if (argc == 3) {
		current = strtof(argv[1], 0);
		freq = strtof(argv[2], 0);
	}
	if (!(current >= BH_RL_MIN_CURRENT_A && current <= BH_RL_MAX_CURRENT_A) ||
			!(freq >= BH_RL_MIN_FREQ_HZ && freq <= BH_RL_MAX_FREQ_HZ)) {
		commands_printf("Limits: %.2f<=I<=%.2f A, %.1f<=f<=%.1f Hz",
				(double)BH_RL_MIN_CURRENT_A, (double)BH_RL_MAX_CURRENT_A,
				(double)BH_RL_MIN_FREQ_HZ, (double)BH_RL_MAX_FREQ_HZ);
		return;
	}
	bh_rl_current = current;
	bh_rl_freq = freq;
	if (bh_queue_job(BH_JOB_MEASURE_RL)) {
		commands_printf("B-H R/L measurement queued.");
	}
}

static void terminal_bh_set_rl(int argc, const char **argv) {
	if (bh_is_busy()) {
		commands_printf("Cannot change B-H R/L while a job is running or queued.");
		return;
	}
	if (argc != 3) {
		commands_printf("Usage: bh_set_rl <R_mOhm> <L_uH>");
		return;
	}
	float r = strtof(argv[1], 0) * 1e-3f;
	float l = strtof(argv[2], 0) * 1e-6f;
	if (!(r > 0.0001f && r < 5.0f) || !(l > 0.1e-6f && l < 0.100f)) {
		commands_printf("Implausible R/L.");
		return;
	}
	bh_r_line_ohm = r;
	bh_l_line_h = l;
	bh_rl_valid = true;
	commands_printf("BH R/L override: Rline=%.6f mOhm Lline=%.6f uH",
			(double)(r * 1e3f), (double)(l * 1e6f));
}

static bool bh_parse_common_run(float i_pk, float path_mm, float area_mm2, int cycles) {
	if (!bh_rl_valid) {
		commands_printf("Measure fixture R/L first with bh_measure_rl (or use bh_set_rl).");
		return false;
	}
	if (!(i_pk > 0.0f && i_pk <= BH_MAX_CURRENT_A) ||
			!(path_mm > 0.0f) || !(area_mm2 > 0.0f) ||
			cycles < 1 || cycles > BH_MAX_CYCLES) {
		commands_printf("Invalid B-H arguments. 0<Ipk<=%.1f A, geometry>0, 1<=cycles<=%d.",
				(double)BH_MAX_CURRENT_A, BH_MAX_CYCLES);
		return false;
	}
	bh_i_pk = i_pk;
	bh_path_m = path_mm * 1e-3f;
	bh_area_m2 = area_mm2 * 1e-6f;
	bh_cycles = cycles;
	return true;
}

static void terminal_bh_run(int argc, const char **argv) {
	if (bh_is_busy()) {
		commands_printf("A B-H job is already running or queued. Use bh_stop first.");
		return;
	}
	if (argc != 6) {
		commands_printf("Usage: bh_run <Ipk_A> <freq_Hz> <path_mm> <area_mm2> <cycles>");
		commands_printf("Example: bh_run 0.25 10 21.9 3.65 3");
		return;
	}
	float i_pk = strtof(argv[1], 0);
	float freq = strtof(argv[2], 0);
	float path_mm = strtof(argv[3], 0);
	float area_mm2 = strtof(argv[4], 0);
	int cycles = atoi(argv[5]);
	if (!(freq >= BH_MIN_FREQ_HZ && freq <= BH_MAX_FREQ_HZ) ||
			!bh_parse_common_run(i_pk, path_mm, area_mm2, cycles)) {
		commands_printf("Frequency limit: %.1f..%.1f Hz", (double)BH_MIN_FREQ_HZ, (double)BH_MAX_FREQ_HZ);
		return;
	}
	bh_freq = freq;
	if (bh_queue_job(BH_JOB_RUN)) {
		commands_printf("B-H triangle run queued: Ipk=%.4f A f=%.4f Hz path=%.3f mm area=%.4f mm^2 cycles=%d",
				(double)i_pk, (double)freq, (double)path_mm, (double)area_mm2, cycles);
	}
}

static void terminal_bh_sweep(int argc, const char **argv) {
	if (bh_is_busy()) {
		commands_printf("A B-H job is already running or queued. Use bh_stop first.");
		return;
	}
	if (argc != 8) {
		commands_printf("Usage: bh_sweep <Ipk_A> <fmin_Hz> <fmax_Hz> <points_per_octave> <path_mm> <area_mm2> <cycles>");
		commands_printf("Example: bh_sweep 0.25 0.5 100 2 21.9 3.65 2");
		return;
	}
	float i_pk = strtof(argv[1], 0);
	float fmin = strtof(argv[2], 0);
	float fmax = strtof(argv[3], 0);
	float ppo = strtof(argv[4], 0);
	float path_mm = strtof(argv[5], 0);
	float area_mm2 = strtof(argv[6], 0);
	int cycles = atoi(argv[7]);
	if (!(fmin >= BH_MIN_FREQ_HZ && fmax <= BH_MAX_FREQ_HZ && fmax >= fmin) ||
			!(ppo >= 1.0f && ppo <= 24.0f) ||
			!bh_parse_common_run(i_pk, path_mm, area_mm2, cycles)) {
		commands_printf("Sweep limits: %.1f<=fmin<=fmax<=%.1f Hz, 1<=points/octave<=24.",
				(double)BH_MIN_FREQ_HZ, (double)BH_MAX_FREQ_HZ);
		return;
	}
	bh_fmin = fmin;
	bh_fmax = fmax;
	bh_sweep_step = ppo;
	if (bh_queue_job(BH_JOB_SWEEP_LOG)) {
		commands_printf("B-H logarithmic sine sweep queued: %.4f..%.4f Hz, %.2f points/octave",
				(double)fmin, (double)fmax, (double)ppo);
	}
}

static void terminal_bh_sweep_linear(int argc, const char **argv) {
	if (bh_is_busy()) {
		commands_printf("A B-H job is already running or queued. Use bh_stop first.");
		return;
	}
	if (argc != 8) {
		commands_printf("Usage: bh_sweep_linear <Ipk_A> <fmin_Hz> <fmax_Hz> <step_Hz> <path_mm> <area_mm2> <cycles>");
		commands_printf("Example: bh_sweep_linear 0.25 25 50 0.25 21.9 3.65 2");
		return;
	}
	float i_pk = strtof(argv[1], 0);
	float fmin = strtof(argv[2], 0);
	float fmax = strtof(argv[3], 0);
	float step = strtof(argv[4], 0);
	float path_mm = strtof(argv[5], 0);
	float area_mm2 = strtof(argv[6], 0);
	int cycles = atoi(argv[7]);
	int points = step > 0.0f ? (int)floorf((fmax - fmin) / step) + 1 : BH_MAX_SWEEP_POINTS + 1;
	if (!(fmin >= BH_MIN_FREQ_HZ && fmax <= BH_MAX_FREQ_HZ && fmax >= fmin) ||
			!(step > 0.0f) || points < 1 || points > BH_MAX_SWEEP_POINTS ||
			!bh_parse_common_run(i_pk, path_mm, area_mm2, cycles)) {
		commands_printf("Linear sweep invalid or >%d points. Frequency limit %.1f..%.1f Hz.",
				BH_MAX_SWEEP_POINTS, (double)BH_MIN_FREQ_HZ, (double)BH_MAX_FREQ_HZ);
		return;
	}
	bh_fmin = fmin;
	bh_fmax = fmax;
	bh_sweep_step = step;
	if (bh_queue_job(BH_JOB_SWEEP_LINEAR)) {
		commands_printf("B-H linear sine sweep queued: %.4f..%.4f Hz step %.4f Hz (%d points)",
				(double)fmin, (double)fmax, (double)step, points);
	}
}

static THD_FUNCTION(bh_thread, arg) {
	(void)arg;
	chRegSetThreadName("MINI4 B-H tracer");

	for (;;) {
		if (bh_job == BH_JOB_NONE) {
			chEvtWaitAny(ALL_EVENTS);
			continue;
		}

		bh_job_t job = bh_job;
		bh_running = true;
		bh_stop_requested = false;

		// The stock VESC IWDG is deliberately very tight: roughly 12..33 ms
		// over LSI tolerance. A diagnostic acquisition plus high-rate VESC Tool
		// RT Data can occasionally delay the normal watchdog housekeeping thread
		// enough to cause a nuisance reset. During a B-H job only, use VESC's
		// existing slow watchdog setting (roughly 22..60 s), then restore the
		// normal watchdog before returning to ordinary ESC operation.
		utils_sys_lock_cnt();
		timeout_configure_IWDT_slowest();
		utils_sys_unlock_cnt();
		commands_printf("BH_WDT slow mode active for tracer job");

		mc_configuration *cfg_old = mempools_alloc_mcconf();
		mc_configuration *cfg = mempools_alloc_mcconf();
		if (!cfg_old || !cfg) {
			commands_printf("BH_ABORT could not allocate motor configuration");
			if (cfg) mempools_free_mcconf(cfg);
			if (cfg_old) mempools_free_mcconf(cfg_old);
			utils_sys_lock_cnt();
			timeout_configure_IWDT();
			utils_sys_unlock_cnt();
			bh_job = BH_JOB_NONE;
			bh_running = false;
			continue;
		}
		*cfg_old = *mc_interface_get_configuration();
		*cfg = *cfg_old;

		bool ok = true;
		if (job == BH_JOB_MEASURE_RL) {
			bh_common_cfg(cfg, bh_rl_current);
			bh_set_measurement_gains(cfg, 0.020f);
			mc_interface_set_configuration(cfg);
			chThdSleepMilliseconds(100);
			mc_interface_lock();
			ok = bh_measure_rl_locked(cfg, bh_rl_current, bh_rl_freq);
		} else {
			bh_common_cfg(cfg, bh_i_pk);
			bh_set_run_gains(cfg);
			mc_interface_set_configuration(cfg);
			chThdSleepMilliseconds(100);
			commands_printf("BH_CTRL Rline=%.6f mOhm Lline=%.6f uH Kp=%.7g Ki=%.7g BW=%.1f Hz",
					(double)(bh_r_line_ohm * 1e3f), (double)(bh_l_line_h * 1e6f),
					(double)cfg->foc_current_kp, (double)cfg->foc_current_ki,
					(double)BH_CURRENT_BW_HZ);
			mc_interface_lock();

			if (job == BH_JOB_RUN) {
				bh_metrics_t m;
				ok = bh_run_one(bh_freq, false, true, &m);
				if (ok) {
					bh_print_metrics(bh_freq, &m, false);
				}
			} else {
				float f = bh_fmin;
				int step_n = 0;
				while (!bh_stop_requested && f <= bh_fmax * 1.000001f &&
						step_n < BH_MAX_SWEEP_POINTS) {
					commands_printf("BH_SWEEP_STEP n=%d f=%.6f Hz", step_n, (double)f);
					bh_metrics_t m;
					ok = bh_run_one(f, true, false, &m);
					if (!ok) {
						break;
					}
					bh_print_metrics(f, &m, true);
					step_n++;
					if (job == BH_JOB_SWEEP_LOG) {
						f *= powf(2.0f, 1.0f / bh_sweep_step);
					} else {
						f = bh_fmin + (float)step_n * bh_sweep_step;
					}
				}
				commands_printf("BH_SWEEP_DONE points=%d%s", step_n,
						bh_stop_requested ? " STOPPED" : (ok ? "" : " ABORTED"));
			}
		}

		// Always leave outputs released and restore the user's original motor config.
		bh_release_locked();
		mc_interface_unlock();
		chThdSleepMilliseconds(20);
		mc_interface_set_configuration(cfg_old);
		mempools_free_mcconf(cfg);
		mempools_free_mcconf(cfg_old);

		utils_sys_lock_cnt();
		timeout_configure_IWDT();
		utils_sys_unlock_cnt();
		commands_printf("BH_WDT normal mode restored");

		if (job == BH_JOB_MEASURE_RL && !ok) {
			commands_printf("BH_RL_DONE ABORTED");
		} else if (job == BH_JOB_RUN) {
			commands_printf("BH_DONE%s", ok ? "" : " ABORTED");
		}
		bh_job = BH_JOB_NONE;
		bh_running = false;
		bh_stop_requested = false;
	}
}

static void bh_init_commands(void) {
	terminal_register_command_callback(
			"bh_run",
			"Run one triangular-current MINI4 B-H capture. Excitation P1-P2; INA282 OUT on A1.",
			"<Ipk_A> <freq_Hz> <path_mm> <area_mm2> <cycles>",
			terminal_bh_run);
	terminal_register_command_callback(
			"bh_measure_rl",
			"Measure the two-terminal excitation winding R and small-signal L with the VESC.",
			"[I_A freq_Hz]",
			terminal_bh_measure_rl);
	terminal_register_command_callback(
			"bh_set_rl",
			"Manually override measured two-terminal fixture R/L.",
			"<R_mOhm> <L_uH>",
			terminal_bh_set_rl);
	terminal_register_command_callback(
			"bh_sweep",
			"Logarithmic sine-current frequency sweep with B-H, impedance and harmonic metrics.",
			"<Ipk_A> <fmin_Hz> <fmax_Hz> <points_per_octave> <path_mm> <area_mm2> <cycles>",
			terminal_bh_sweep);
	terminal_register_command_callback(
			"bh_sweep_linear",
			"Dense linear sine-current frequency sweep for suspicious frequency regions.",
			"<Ipk_A> <fmin_Hz> <fmax_Hz> <step_Hz> <path_mm> <area_mm2> <cycles>",
			terminal_bh_sweep_linear);
	terminal_register_command_callback(
			"bh_stop",
			"Stop or cancel the active MINI4 B-H job.",
			0,
			terminal_bh_stop);
	terminal_register_command_callback(
			"bh_adc",
			"Print A1/search-coil, phase-current and reconstructed A-B voltage readings.",
			0,
			terminal_bh_adc);
	terminal_register_command_callback(
			"bh_status",
			"Print MINI4 B-H tracer state and measured fixture R/L.",
			0,
			terminal_bh_status);
}
