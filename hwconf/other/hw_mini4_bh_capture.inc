static bool bh_capture_frequency(float freq, int cycles, bool sine_wave,
		bool print_raw, bh_metrics_t *m) {
	memset(m, 0, sizeof(*m));
	float sense_zero = bh_measure_sense_zero();
	if (!bh_sense_zero_ok) {
		commands_printf("BH_ABORT A1 zero invalid V=%.6f", (double)sense_zero);
		return false;
	}

	float sense_mean = 0.0f;
	if (!bh_precondition_and_offset(freq, sine_wave, sense_zero, &sense_mean)) {
		return false;
	}
	m->sense_mean_v = sense_mean;

	commands_init_plot("H (A/m)", "B (T)");
	commands_plot_add_graph("B-H loop");
	commands_plot_set_graph(0);
	commands_printf("BH_FREQ_START f=%.6f Hz wave=%s zero=%.7f V sense_mean=%.9f V",
			(double)freq, sine_wave ? "sine" : "triangle",
			(double)sense_zero, (double)sense_mean);
	if (print_raw) {
		commands_printf("BH_RAW columns: f_Hz,t_s,Icmd_A,Icoil_A,A1_raw,A1_V,Vsense_V,H_Apm,Brel_T");
	}

	const float dt_target = 1.0f / BH_SAMPLE_HZ;
	const int plot_div = (int)(BH_SAMPLE_HZ / BH_PLOT_HZ);
	const float total_time = (float)cycles / freq;
	const float metric_start = (float)(cycles - 1) / freq;
	uint32_t t0 = timer_time_now();
	uint32_t t_prev = t0;
	float b_live = 0.0f;
	float b_metric = 0.0f;
	float last_vs = 0.0f;
	float last_vs_metric = 0.0f;
	float last_h_metric = 0.0f;
	float last_b_metric = 0.0f;
	bool first = true;
	bool metric_started = false;
	bool metric_first = true;
	bool have_br_mid = false;
	float br_mid = 0.0f;
	float b_min = 1e30f;
	float b_max = -1e30f;
	float h_pk = 0.0f;
	double loop_area = 0.0;
	double vhs[BH_MAX_HARMONIC + 1] = {0};
	double vhc[BH_MAX_HARMONIC + 1] = {0};
	double i_s = 0.0, i_c = 0.0;
	int sample_n = 0;
	bh_adc_clip_reset();

	int max_h = (int)(BH_SAMPLE_HZ / (4.0f * freq));
	if (max_h > BH_MAX_HARMONIC) {
		max_h = BH_MAX_HARMONIC;
	}
	if (max_h < 1) {
		max_h = 1;
	}
	m->max_harmonic = max_h;

	while (!bh_stop_requested) {
		uint32_t loop_start = timer_time_now();
		float t = timer_seconds_elapsed_since(t0);
		if (t >= total_time) {
			break;
		}
		float phase01 = t * freq;
		float i_cmd = bh_i_pk * bh_wave(phase01, sine_wave);
		bh_apply_current(i_cmd);

		uint32_t now = timer_time_now();
		float dt = timer_calc_diff(t_prev, now);
		t_prev = now;
		float i_now = bh_coil_current();
		uint16_t raw = ADC_Value[ADC_IND_EXT];
		bool hard_sample = bh_adc_sample_hard_rail(raw);
		float a1_v = bh_sense_out_v();
		float vs = (a1_v - sense_zero) / BH_INA_GAIN - sense_mean;
		float h = BH_EXC_TURNS * i_now / bh_path_m;

		if (bh_adc_clipped()) {
			commands_printf("BH_ABORT sustained A1 clipping raw=%u at f=%.6f Hz (>10 hits/s)",
					(unsigned)raw, (double)freq);
			return false;
		}

		/* A rare hard-rail conversion is treated as an ADC glitch: leave B where
		 * it was and exclude that sample from metrics/harmonics. Persistent rail
		 * contact still aborts through the rolling rate guard above.
		 */
		if (!hard_sample) {
			if (!first && dt > 0.0f && dt < 0.02f) {
				b_live += 0.5f * (vs + last_vs) * dt /
						(BH_SENSE_TURNS * bh_area_m2);
			}
			first = false;
			last_vs = vs;

			if (t >= metric_start) {
				if (!metric_started) {
					metric_started = true;
					metric_first = true;
					b_metric = 0.0f;
					last_vs_metric = vs;
					last_h_metric = h;
					last_b_metric = 0.0f;
				}
				if (!metric_first && dt > 0.0f && dt < 0.02f) {
					b_metric += 0.5f * (vs + last_vs_metric) * dt /
							(BH_SENSE_TURNS * bh_area_m2);
					loop_area += 0.5 * ((double)h + last_h_metric) *
							((double)b_metric - last_b_metric);
					if (!have_br_mid && ((last_h_metric > 0.0f && h <= 0.0f) ||
							(last_h_metric < 0.0f && h >= 0.0f))) {
						float denom_h = h - last_h_metric;
						if (fabsf(denom_h) > 1e-12f) {
							float a = -last_h_metric / denom_h;
							br_mid = last_b_metric + a * (b_metric - last_b_metric);
							have_br_mid = true;
						}
					}
				}
				metric_first = false;
				last_vs_metric = vs;
				last_h_metric = h;
				last_b_metric = b_metric;
				b_min = fminf(b_min, b_metric);
				b_max = fmaxf(b_max, b_metric);
				h_pk = fmaxf(h_pk, fabsf(h));

				float ang = 2.0f * M_PI * (phase01 - floorf(phase01));
				for (int n = 1; n <= max_h; n++) {
					vhs[n] += (double)vs * sinf((float)n * ang) * dt;
					vhc[n] += (double)vs * cosf((float)n * ang) * dt;
				}
				i_s += (double)i_now * sinf(ang) * dt;
				i_c += (double)i_now * cosf(ang) * dt;
			}
		}

		if ((sample_n % plot_div) == 0) {
			commands_send_plot_points(h, b_live);
			if (print_raw) {
				commands_printf("BH_RAW %.6f,%.7f,%.7f,%.7f,%u,%.7f,%.9f,%.7f,%.9f",
						(double)freq, (double)t, (double)i_cmd, (double)i_now,
						(unsigned)raw, (double)a1_v, (double)vs,
						(double)h, (double)b_live);
			}
		}

		mc_fault_code fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			commands_printf("BH_ABORT fault=%d (%s) at f=%.6f Hz", (int)fault,
					mc_interface_fault_to_string(fault), (double)freq);
			return false;
		}

		sample_n++;
		float elapsed = timer_seconds_elapsed_since(loop_start);
		if (elapsed < dt_target) {
			timer_sleep(dt_target - elapsed);
		}
	}

	if (bh_stop_requested || !metric_started || !(b_max > b_min)) {
		return false;
	}

	float b_center = 0.5f * (b_max + b_min);
	m->b_pk = 0.5f * (b_max - b_min);
	m->h_pk = h_pk;
	m->loop_j_m3 = (float)fabs(loop_area);
	float br_start = fabsf(-b_center);
	m->b_r = have_br_mid ? 0.5f * (br_start + fabsf(br_mid - b_center)) : br_start;

	// Search-coil harmonic ratios for the final captured cycle.
	double v1 = sqrt(vhs[1] * vhs[1] + vhc[1] * vhc[1]);
	double thd_sq = 0.0;
	for (int n = 1; n <= max_h; n++) {
		double vn = sqrt(vhs[n] * vhs[n] + vhc[n] * vhc[n]);
		m->harm[n] = v1 > 1e-15 ? (float)(vn / v1) : 0.0f;
		if (n >= 2 && v1 > 1e-15) {
			thd_sq += (vn / v1) * (vn / v1);
		}
	}
	m->thd = (float)sqrt(thd_sq);

	// Fundamental fixture impedance referred through the search winding. The
	// search coil measures Ns*dPhi/dt directly, so (Np/Ns)*Vs is the core-linked
	// primary back-EMF. This avoids relying on the FOC controller's reconstructed
	// millivolt-level terminal voltage, which is strongly affected by dead-time
	// compensation near zero current. Sense lead polarity is chosen such that
	// the inductive reactance is positive; the same sign is then applied to the
	// in-phase magnetic-loss component. Add the measured DC line resistance to
	// obtain total effective Rac. Primary leakage inductance is not included.
	double zden = i_s * i_s + i_c * i_c;
	if (zden > 1e-18) {
		float r_sec = (float)((vhs[1] * i_s + vhc[1] * i_c) / zden);
		float x_sec = (float)((vhc[1] * i_s - vhs[1] * i_c) / zden);
		float sense_sign = x_sec >= 0.0f ? 1.0f : -1.0f;
		float turns_ratio = BH_EXC_TURNS / BH_SENSE_TURNS;
		m->r_ac = bh_r_line_ohm + turns_ratio * sense_sign * r_sec;
		m->l_ac = turns_ratio * fabsf(x_sec) / (2.0f * M_PI * freq);
	}

	// One extra identically phased cycle finds Hc after the one-cycle B offset
	// has been determined. B is reset to zero at the same H=0 periodic state.
	uint32_t a0 = timer_time_now();
	uint32_t aprev = a0;
	float ba = 0.0f, last_ba = 0.0f, last_vsa = 0.0f, last_ha = 0.0f;
	bool afirst = true;
	double hc_sum = 0.0;
	int hc_n = 0;
	bh_adc_clip_reset();
	while (!bh_stop_requested) {
		uint32_t loop_start = timer_time_now();
		float t = timer_seconds_elapsed_since(a0);
		if (t >= 1.0f / freq) {
			break;
		}
		float i_cmd = bh_i_pk * bh_wave(t * freq, sine_wave);
		bh_apply_current(i_cmd);
		uint32_t now = timer_time_now();
		float dt = timer_calc_diff(aprev, now);
		aprev = now;
		uint16_t raw = ADC_Value[ADC_IND_EXT];
		bool hard_sample = bh_adc_sample_hard_rail(raw);
		float vs = (bh_sense_out_v() - sense_zero) / BH_INA_GAIN - sense_mean;
		float h = BH_EXC_TURNS * bh_coil_current() / bh_path_m;
		if (bh_adc_clipped()) {
			commands_printf("BH_ABORT sustained A1 clipping during Hc cycle raw=%u (>10 hits/s)",
					(unsigned)raw);
			return false;
		}
		if (!hard_sample) {
			if (!afirst && dt > 0.0f && dt < 0.02f) {
				ba += 0.5f * (vs + last_vsa) * dt /
						(BH_SENSE_TURNS * bh_area_m2);
				float y0 = last_ba - b_center;
				float y1 = ba - b_center;
				if ((y0 > 0.0f && y1 <= 0.0f) || (y0 < 0.0f && y1 >= 0.0f)) {
					float dy = y1 - y0;
					if (fabsf(dy) > 1e-12f) {
						float a = -y0 / dy;
						float hc = last_ha + a * (h - last_ha);
						hc_sum += fabsf(hc);
						hc_n++;
					}
				}
			}
			afirst = false;
			last_vsa = vs;
			last_ba = ba;
			last_ha = h;
		}
		if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			return false;
		}
		float elapsed = timer_seconds_elapsed_since(loop_start);
		if (elapsed < dt_target) {
			timer_sleep(dt_target - elapsed);
		}
	}
	m->h_c = hc_n > 0 ? (float)(hc_sum / (double)hc_n) : 0.0f;
	m->valid = !bh_stop_requested;
	return m->valid;
}

static void bh_print_metrics(float freq, const bh_metrics_t *m, bool sweep) {
	commands_printf("BH_RESULT f=%.6f Hz Hpk=%.6f A/m Bpk=%.9f T Br=%.9f T Hc=%.6f A/m loop=%.9f J/m^3 Rac=%.6f mOhm Lac=%.6f uH THD=%.4f%% Pcore=%.6f W/m^3",
			(double)freq, (double)m->h_pk, (double)m->b_pk, (double)m->b_r,
			(double)m->h_c, (double)m->loop_j_m3, (double)(m->r_ac * 1e3f),
			(double)(m->l_ac * 1e6f), (double)(m->thd * 100.0f),
			(double)(m->loop_j_m3 * freq));
	if (sweep) {
		char line[256];
		int off = snprintf(line, sizeof(line), "BH_HARM f=%.6f", (double)freq);
		for (int n = 2; n <= m->max_harmonic && off > 0 && off < (int)sizeof(line); n++) {
			off += snprintf(line + off, sizeof(line) - (size_t)off,
					" H%d=%.3f%%", n, (double)(m->harm[n] * 100.0f));
		}
		commands_printf("%s", line);
	}
}

static bool bh_run_one(float freq, bool sine_wave, bool print_raw, bh_metrics_t *m) {
	bool ok = bh_capture_frequency(freq, bh_cycles, sine_wave, print_raw, m);
	bh_apply_current(0.0f);
	bh_release_locked();
	bh_hold_hard_off_ms(20);
	return ok;
}

static bool bh_queue_job(bh_job_t job) {
	if (bh_running || bh_job != BH_JOB_NONE) {
		commands_printf("A B-H job is already running or queued. Use bh_stop first.");
		return false;
	}
	bh_stop_requested = false;
	bh_job = job;
	if (!bh_tp) {
		// Measurement is diagnostic/background work. Keep it below the normal
		// watchdog, USB and telemetry housekeeping priority so RT Data cannot be
		// starved by the tracer even when acquisition is busy.
		bh_tp = chThdCreateStatic(bh_thread_wa, sizeof(bh_thread_wa),
				NORMALPRIO - 1, bh_thread, 0);
	} else {
		chEvtSignal(bh_tp, (eventmask_t)1);
	}
	return true;
}
