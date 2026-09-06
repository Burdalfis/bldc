/* Continuous single-point B-H plot mode.
 *
 * bh_live reuses the ordinary BH_JOB_RUN worker so it inherits the tracer's
 * motor lock, temporary FOC configuration, watchdog policy and hard-off
 * cleanup. The only special behavior is that bh_run_one() stays in this
 * routine until a stop is requested.
 *
 * VESC Tool's global red Stop button sends a motor-estop after commanding
 * zero current. The motor-estop forcibly releases FOC and makes ordinary motor
 * commands be ignored for the configured estop interval. Because the tracer
 * intentionally owns mc_interface's control lock, watch MC_STATE_RUNNING here:
 * an unexpected transition out of RUNNING after excitation has started is
 * treated as an external Stop request. The terminal bh_stop command remains a
 * direct cancellation path through bh_stop_requested.
 */
static bool bh_live_mode = false;
static bool bh_live_suppress_metric = false;

#define BH_LIVE_MAX_PLOT_POINTS_PER_CYCLE 400
#define BH_LIVE_MAX_REFRESH_HZ             5.0f

static bool bh_live_plot(float freq) {
	float sense_zero = bh_measure_sense_zero();
	if (bh_adc_clipped()) {
		commands_printf("BH_LIVE_ABORT A1 zero near rail raw=%u V=%.6f",
				(unsigned)ADC_Value[ADC_IND_EXT], (double)sense_zero);
		return false;
	}

	// Use the same triangle-wave preconditioning and volt-second offset estimate
	// as the finite B-H capture. The live display intentionally stays otherwise
	// raw: B is only the integrated search-coil voltage, with no R/C de-embedding
	// or post-fit loop correction applied.
	float sense_mean = 0.0f;
	if (!bh_precondition_and_offset(freq, false, sense_zero, &sense_mean)) {
		return false;
	}

	const float dt_target = 1.0f / BH_SAMPLE_HZ;
	const float cycle_time = 1.0f / freq;
	int samples_per_cycle = (int)ceilf(BH_SAMPLE_HZ / freq);
	if (samples_per_cycle < 1) {
		samples_per_cycle = 1;
	}
	int plot_div = (samples_per_cycle + BH_LIVE_MAX_PLOT_POINTS_PER_CYCLE - 1) /
			BH_LIVE_MAX_PLOT_POINTS_PER_CYCLE;
	if (plot_div < 1) {
		plot_div = 1;
	}
	const float plot_period = 1.0f / BH_LIVE_MAX_REFRESH_HZ;

	commands_printf("BH_LIVE_START Ipk=%.6f A f=%.6f Hz path=%.6f mm area=%.6f mm^2 zero=%.7f V sense_mean=%.9f V sense=%+d plot_div=%d",
			(double)bh_i_pk, (double)freq, (double)(bh_path_m * 1e3f),
			(double)(bh_area_m2 * 1e6f), (double)sense_zero,
			(double)sense_mean, bh_sense_polarity, plot_div);
	commands_printf("BH_LIVE: Experiment Plot redraws one raw triangle-current B-H loop at a time; use VESC Tool Stop or bh_stop to finish.");

	bool first_plot = true;
	bool seen_running = mc_interface_get_state() == MC_STATE_RUNNING;
	bool external_stop = false;
	uint32_t last_plot_start = timer_time_now();

	while (!bh_stop_requested) {
		bool plot_this_cycle = first_plot ||
				timer_seconds_elapsed_since(last_plot_start) >= plot_period;
		if (plot_this_cycle) {
			commands_init_plot("H (A/m)", "B relative (T)");
			commands_plot_add_graph("B-H live");
			commands_plot_set_graph(0);
			last_plot_start = timer_time_now();
			first_plot = false;
		}

		uint32_t t0 = timer_time_now();
		uint32_t t_prev = t0;
		float b_live = 0.0f;
		float last_vs = 0.0f;
		bool first_sample = true;
		int sample_n = 0;

		while (!bh_stop_requested) {
			uint32_t loop_start = timer_time_now();
			float t = timer_seconds_elapsed_since(t0);
			if (t >= cycle_time) {
				break;
			}

			float i_cmd = bh_i_pk * bh_triangle(t * freq);
			bh_apply_current(i_cmd);

			uint32_t now = timer_time_now();
			float dt = timer_calc_diff(t_prev, now);
			t_prev = now;
			float i_now = bh_coil_current();
			uint16_t raw = ADC_Value[ADC_IND_EXT];
			float vs = (bh_sense_out_v() - sense_zero) / BH_INA_GAIN - sense_mean;
			float h = BH_EXC_TURNS * i_now / bh_path_m;

			if (!first_sample && dt > 0.0f && dt < 0.02f) {
				b_live += 0.5f * (vs + last_vs) * dt /
						(BH_SENSE_TURNS * bh_area_m2);
			}
			first_sample = false;
			last_vs = vs;

			if (plot_this_cycle && (sample_n % plot_div) == 0) {
				commands_send_plot_points(h, b_live);
			}

			if (raw < BH_A1_RAW_MIN || raw > BH_A1_RAW_MAX) {
				commands_printf("BH_LIVE_ABORT A1 clipping raw=%u at f=%.6f Hz",
						(unsigned)raw, (double)freq);
				bh_release_locked();
				return false;
			}

			mc_fault_code fault = mc_interface_get_fault();
			if (fault != FAULT_CODE_NONE) {
				commands_printf("BH_LIVE_ABORT fault=%d (%s) at f=%.6f Hz", (int)fault,
						mc_interface_fault_to_string(fault), (double)freq);
				bh_release_locked();
				return false;
			}

			/* A VESC Tool motor-estop releases the motor underneath us and blocks
			 * subsequent motor commands. Once this live run has reached RUNNING,
			 * that state transition is an unambiguous request to stop the fixture.
			 */
			if (mc_interface_get_state() == MC_STATE_RUNNING) {
				seen_running = true;
			} else if (seen_running) {
				external_stop = true;
				bh_stop_requested = true;
				break;
			}

			sample_n++;
			float elapsed = timer_seconds_elapsed_since(loop_start);
			if (elapsed < dt_target) {
				timer_sleep(dt_target - elapsed);
			}
		}
	}

	bh_release_locked();
	commands_printf("BH_LIVE_DONE %s",
			external_stop ? "VESC_TOOL_STOP" : "STOP_REQUESTED");
	return true;
}

/* capture_v2 is compiled under these base names by hw_mini4_bh.h. Keep all of
 * its L-sweep behavior intact and intercept only an explicitly queued live run.
 */
static bool bh_run_one(float freq, bool sine_wave, bool print_raw, bh_metrics_t *m) {
	if (!bh_live_mode) {
		return bh_run_one_v2_base(freq, sine_wave, print_raw, m);
	}

	(void)sine_wave;
	(void)print_raw;
	memset(m, 0, sizeof(*m));
	bool ok = bh_live_plot(freq);
	m->valid = ok;
	bh_live_mode = false;
	bh_live_suppress_metric = true;
	return ok;
}

static void bh_print_metrics(float freq, const bh_metrics_t *m, bool sweep) {
	if (bh_live_suppress_metric) {
		bh_live_suppress_metric = false;
		return;
	}
	bh_print_metrics_v2_base(freq, m, sweep);
}

static void terminal_bh_live(int argc, const char **argv) {
	if (bh_is_busy()) {
		commands_printf("A B-H job is already running or queued. Use bh_stop first.");
		return;
	}
	if (argc != 5) {
		commands_printf("Usage: bh_live <Ipk_A> <freq_Hz> <path_mm> <area_mm2>");
		commands_printf("Example: bh_live 0.25 10 21.9 3.65");
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
	bh_live_mode = true;
	bh_live_suppress_metric = false;
	if (!bh_queue_job(BH_JOB_RUN)) {
		bh_live_mode = false;
		return;
	}

	commands_printf("B-H live triangle run queued: Ipk=%.4f A f=%.4f Hz path=%.3f mm area=%.4f mm^2",
			(double)i_pk, (double)freq, (double)path_mm, (double)area_mm2);
}

static void bh_live_init_commands(void) {
	terminal_register_command_callback(
			"bh_live",
			"Continuously redraw one raw B-H loop in Experiment Plot until VESC Tool Stop or bh_stop.",
			"<Ipk_A> <freq_Hz> <path_mm> <area_mm2>",
			terminal_bh_live);
}
