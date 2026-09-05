typedef enum {
	BH_LPOINT_OK = 0,
	BH_LPOINT_HEADROOM,
	BH_LPOINT_ABORT
} bh_lpoint_status_t;

typedef struct {
	float i_amp;
	float tracking_pct;
	float i_phase_cmd_deg;
	float vs_amp;
	float vs_phase_cmd_deg;
	float vi_phase_deg;
	float r_mag;
	float x_mag;
	float l_line;
	float vs_mean;
	uint16_t a1_min;
	uint16_t a1_max;
} bh_lpoint_t;

static float bh_measure_sense_zero_quick(void) {
	bh_release_locked();
	chThdSleepMilliseconds(20);
	double sum = 0.0;
	const int samples = 250;
	for (int i = 0; i < samples; i++) {
		sum += bh_sense_out_v();
		chThdSleepMicroseconds(200);
	}
	return (float)(sum / (double)samples);
}

static bh_lpoint_status_t bh_measure_l_point(float current, float freq,
		bh_lpoint_t *out) {
	memset(out, 0, sizeof(*out));
	out->a1_min = 4095;
	out->a1_max = 0;

	float sense_zero = bh_measure_sense_zero_quick();
	if (bh_adc_clipped()) {
		commands_printf("BH_LPOINT_ABORT A1 zero near rail raw=%u V=%.6f",
				(unsigned)ADC_Value[ADC_IND_EXT], (double)sense_zero);
		return BH_LPOINT_ABORT;
	}

	const float dt_target = 1.0f / BH_SAMPLE_HZ;
	const int settle_cycles = 3;
	const int measure_cycles = 8;
	const float total_time = (float)(settle_cycles + measure_cycles) / freq;
	const float measure_start = (float)settle_cycles / freq;
	uint32_t t0 = timer_time_now();
	uint32_t t_prev = t0;
	double i_s = 0.0, i_c = 0.0;
	double v_s = 0.0, v_c = 0.0;
	double i_dc = 0.0, v_dc = 0.0;
	double s_sum = 0.0, c_sum = 0.0, wsum = 0.0;
	bool headroom = false;

	while (!bh_stop_requested) {
		uint32_t loop_start = timer_time_now();
		float t = timer_seconds_elapsed_since(t0);
		if (t >= total_time) {
			break;
		}
		float ph = 2.0f * M_PI * freq * t;
		bh_apply_current(current * sinf(ph));

		uint32_t now = timer_time_now();
		float dt = timer_calc_diff(t_prev, now);
		t_prev = now;
		float i_now = bh_coil_current();
		uint16_t raw = ADC_Value[ADC_IND_EXT];
		float vs = (bh_sense_out_v() - sense_zero) / BH_INA_GAIN;
		if (raw < out->a1_min) out->a1_min = raw;
		if (raw > out->a1_max) out->a1_max = raw;

		// Preserve about 150 ADC counts of headroom on either rail. Reaching this
		// threshold is not an electrical fault; it just means higher current at
		// this frequency would no longer produce trustworthy search-coil data.
		if (raw < 150U || raw > 3945U) {
			headroom = true;
			break;
		}

		if (t >= measure_start && dt > 0.0f && dt < 0.02f) {
			float s = sinf(ph);
			float c = cosf(ph);
			i_s += (double)i_now * s * dt;
			i_c += (double)i_now * c * dt;
			v_s += (double)vs * s * dt;
			v_c += (double)vs * c * dt;
			i_dc += (double)i_now * dt;
			v_dc += (double)vs * dt;
			s_sum += (double)s * dt;
			c_sum += (double)c * dt;
			wsum += dt;
		}

		mc_fault_code fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			commands_printf("BH_LPOINT_ABORT fault=%d (%s)", (int)fault,
					mc_interface_fault_to_string(fault));
			bh_release_locked();
			return BH_LPOINT_ABORT;
		}

		float elapsed = timer_seconds_elapsed_since(loop_start);
		if (elapsed < dt_target) {
			timer_sleep(dt_target - elapsed);
		}
	}

	bh_apply_current(0.0f);
	bh_release_locked();
	if (bh_stop_requested) {
		return BH_LPOINT_ABORT;
	}
	if (headroom) {
		return BH_LPOINT_HEADROOM;
	}
	if (wsum < 0.01) {
		commands_printf("BH_LPOINT_ABORT insufficient samples f=%.6f I=%.6f",
				(double)freq, (double)current);
		return BH_LPOINT_ABORT;
	}

	// Remove DC components because the RTOS sample times are not perfectly
	// uniformly distributed over the integer number of commanded cycles.
	double i_mean = i_dc / wsum;
	double v_mean = v_dc / wsum;
	i_s -= i_mean * s_sum;
	i_c -= i_mean * c_sum;
	v_s -= v_mean * s_sum;
	v_c -= v_mean * c_sum;

	double zden = i_s * i_s + i_c * i_c;
	if (zden < 1e-18) {
		commands_printf("BH_LPOINT_ABORT measured AC current too small");
		return BH_LPOINT_ABORT;
	}

	float turns_ratio = BH_EXC_TURNS / BH_SENSE_TURNS;
	float r_sec = (float)((v_s * i_s + v_c * i_c) / zden);
	float x_sec = (float)((v_c * i_s - v_s * i_c) / zden);
	out->i_amp = (float)(2.0 * sqrt(zden) / wsum);
	out->tracking_pct = current > 0.0f ? 100.0f * out->i_amp / current : 0.0f;
	out->i_phase_cmd_deg = RAD2DEG_f(atan2f((float)i_c, (float)i_s));
	out->vs_amp = (float)(2.0 * sqrt(v_s * v_s + v_c * v_c) / wsum);
	out->vs_phase_cmd_deg = RAD2DEG_f(atan2f((float)v_c, (float)v_s));
	out->r_mag = turns_ratio * r_sec;
	out->x_mag = turns_ratio * x_sec;
	out->l_line = out->x_mag / (2.0f * M_PI * freq);
	out->vi_phase_deg = RAD2DEG_f(atan2f(out->x_mag, out->r_mag));
	out->vs_mean = (float)v_mean;
	return BH_LPOINT_OK;
}

/* The existing logarithmic frequency-sweep worker is reused as the outer loop.
 * In L-sweep mode each frequency point performs a logarithmic current sweep
 * instead of a B-H capture. This keeps cancellation, watchdog handling, motor
 * locking and configuration restoration in the already-tested worker.
 */
static bool bh_run_one(float freq, bool sine_wave, bool print_raw, bh_metrics_t *m) {
	if (!bh_l_sweep_mode) {
		return bh_run_one_legacy(freq, sine_wave, print_raw, m);
	}

	(void)sine_wave;
	(void)print_raw;
	memset(m, 0, sizeof(*m));
	commands_printf("BH_LFREQ_START f=%.6f Hz sense=%+d", (double)freq,
			bh_sense_polarity);

	float current = bh_l_i_min;
	int point = 0;
	bool any_ok = false;
	while (!bh_stop_requested && current <= bh_l_i_max * 1.000001f &&
			point < BH_MAX_SWEEP_POINTS) {
		bh_lpoint_t p;
		bh_lpoint_status_t st = bh_measure_l_point(current, freq, &p);
		if (st == BH_LPOINT_ABORT) {
			bh_l_sweep_mode = false;
			bh_l_sweep_suppress_metric = false;
			return false;
		}
		if (st == BH_LPOINT_HEADROOM) {
			commands_printf("BH_LPOINT_HEADROOM f=%.6f Hz Icmd=%.6f A A1min=%u A1max=%u; skipping higher currents at this frequency",
					(double)freq, (double)current,
					(unsigned)p.a1_min, (unsigned)p.a1_max);
			break;
		}

		float rac = bh_rl_valid ? bh_r_line_ohm + p.r_mag : p.r_mag;
		commands_printf("BH_LPOINT n=%d f=%.6f Hz Icmd=%.6f A Iac=%.6f A track=%.2f%% Iphase_cmd=%.3f deg Vsense=%.8f V Vsphase_cmd=%.3f deg VIphase=%.3f deg Rmag=%.6f mOhm Xmag=%.6f mOhm L=%.6f uH Rac=%.6f mOhm A1min=%u A1max=%u A1mean=%.9f V",
				point, (double)freq, (double)current, (double)p.i_amp,
				(double)p.tracking_pct, (double)p.i_phase_cmd_deg,
				(double)p.vs_amp, (double)p.vs_phase_cmd_deg,
				(double)p.vi_phase_deg, (double)(p.r_mag * 1e3f),
				(double)(p.x_mag * 1e3f), (double)(p.l_line * 1e6f),
				(double)(rac * 1e3f), (unsigned)p.a1_min,
				(unsigned)p.a1_max, (double)p.vs_mean);
		any_ok = true;
		point++;
		current = bh_l_i_min * powf(2.0f, (float)point / bh_l_i_ppo);
		chThdSleepMilliseconds(20);
	}

	commands_printf("BH_LFREQ_DONE f=%.6f Hz points=%d", (double)freq, point);
	m->valid = any_ok;
	bh_l_sweep_suppress_metric = true;
	if (freq >= bh_fmax * 0.999999f) {
		bh_l_sweep_mode = false;
	}
	return any_ok;
}

static void bh_print_metrics(float freq, const bh_metrics_t *m, bool sweep) {
	if (bh_l_sweep_suppress_metric) {
		bh_l_sweep_suppress_metric = false;
		return;
	}
	bh_print_metrics_legacy(freq, m, sweep);
}
