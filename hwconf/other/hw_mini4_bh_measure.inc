typedef struct {
	float l_line;
	float r_mag;
	float i_amp;
	float vs_amp;
	float vs_mean;
} bh_ac_probe_t;

static bool bh_sense_zero_ok = true;

static float bh_measure_sense_zero_samples(int settle_ms, int samples, int sample_us) {
	bh_sense_zero_ok = true;
	bh_release_locked();
	bh_hold_hard_off_ms(settle_ms);
	bh_adc_clip_reset();

	double sum = 0.0;
	int valid = 0;
	for (int i = 0; i < samples; i++) {
		if (bh_stop_requested) {
			bh_sense_zero_ok = false;
			break;
		}

		/* Keep the ordinary VESC motor timeout from unlocking the tracer and
		 * replacing our true bridge-off state with an active 0-A FOC command.
		 */
		bh_keep_hard_off_alive();
		uint16_t raw = ADC_Value[ADC_IND_EXT];
		bool hard_sample = bh_adc_sample_hard_rail(raw);
		if (bh_adc_clipped()) {
			bh_sense_zero_ok = false;
			commands_printf("BH_ZERO_ABORT sustained A1 clipping raw=%u (>10 hits/s)",
					(unsigned)raw);
			break;
		}

		// Ignore isolated rail glitches so one bad conversion cannot bias zero.
		if (!hard_sample) {
			sum += bh_sense_out_v();
			valid++;
		}
		chThdSleepMicroseconds(sample_us);
	}

	if (valid <= 0) {
		bh_sense_zero_ok = false;
		return bh_sense_out_v();
	}
	return (float)(sum / (double)valid);
}

static float bh_measure_sense_zero(void) {
	return bh_measure_sense_zero_samples(50, 1000, 250);
}

static bool bh_measure_ac_search_once(float ac_current, float freq,
		float sense_zero, bh_ac_probe_t *out) {
	memset(out, 0, sizeof(*out));
	const float dt_target = 1.0f / BH_RL_SAMPLE_HZ;
	const int settle_cycles = 2;
	const int measure_cycles = 8;
	const float total_time = (float)(settle_cycles + measure_cycles) / freq;
	const float measure_start = (float)settle_cycles / freq;
	uint32_t t0 = timer_time_now();
	uint32_t t_prev = t0;
	double i_s = 0.0, i_c = 0.0;
	double vs_s = 0.0, vs_c = 0.0;
	double i_dc = 0.0, vs_dc = 0.0;
	double s_sum = 0.0, c_sum = 0.0, wsum = 0.0;
	bh_adc_clip_reset();

	while (!bh_stop_requested) {
		uint32_t loop_start = timer_time_now();
		float t = timer_seconds_elapsed_since(t0);
		if (t >= total_time) {
			break;
		}
		float ph = 2.0f * M_PI * freq * t;
		float i_cmd = ac_current * sinf(ph);
		bh_apply_current(i_cmd);

		uint32_t sample_now = timer_time_now();
		float dt = timer_calc_diff(t_prev, sample_now);
		t_prev = sample_now;
		float i_now = bh_coil_current();
		uint16_t raw = ADC_Value[ADC_IND_EXT];
		bool hard_sample = bh_adc_sample_hard_rail(raw);
		float vs_now = (bh_sense_out_v() - sense_zero) / BH_INA_GAIN;
		if (bh_adc_clipped()) {
			commands_printf("BH_RL_ABORT sustained A1 clipping raw=%u (>10 hits/s)",
					(unsigned)raw);
			bh_release_locked();
			return false;
		}
		if (!hard_sample && t >= measure_start && dt > 0.0f && dt < 0.01f) {
			float s = sinf(ph);
			float c = cosf(ph);
			i_s += (double)i_now * s * dt;
			i_c += (double)i_now * c * dt;
			vs_s += (double)vs_now * s * dt;
			vs_c += (double)vs_now * c * dt;
			i_dc += (double)i_now * dt;
			vs_dc += (double)vs_now * dt;
			s_sum += (double)s * dt;
			c_sum += (double)c * dt;
			wsum += dt;
		}

		mc_fault_code fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			commands_printf("BH_RL_ABORT fault=%d (%s)", (int)fault,
					mc_interface_fault_to_string(fault));
			bh_release_locked();
			return false;
		}

		float elapsed = timer_seconds_elapsed_since(loop_start);
		if (elapsed < dt_target) {
			timer_sleep(dt_target - elapsed);
		}
	}
	bh_apply_current(0.0f);
	bh_release_locked();

	if (bh_stop_requested || wsum < 0.01) {
		return false;
	}

	// Remove measured DC components from the lock-in coefficients. RTOS timing
	// jitter means the sampled sine/cosine basis does not sum to exactly zero.
	double i_mean = i_dc / wsum;
	double vs_mean = vs_dc / wsum;
	i_s -= i_mean * s_sum;
	i_c -= i_mean * c_sum;
	vs_s -= vs_mean * s_sum;
	vs_c -= vs_mean * c_sum;

	double zden = i_s * i_s + i_c * i_c;
	if (zden < 1e-18) {
		commands_printf("BH_RL_ABORT AC current is too small");
		return false;
	}

	// The open-circuit search winding measures Ns*dPhi/dt directly. Refer that
	// voltage to the 20-turn excitation winding using Np/Ns. Sense lead polarity
	// is arbitrary, so choose the sign that makes inductive reactance positive;
	// use that same sign for the in-phase magnetic-loss component.
	float r_sec = (float)((vs_s * i_s + vs_c * i_c) / zden);
	float x_sec = (float)((vs_c * i_s - vs_s * i_c) / zden);
	float sense_sign = x_sec >= 0.0f ? 1.0f : -1.0f;
	float turns_ratio = BH_EXC_TURNS / BH_SENSE_TURNS;
	out->r_mag = turns_ratio * sense_sign * r_sec;
	out->l_line = turns_ratio * fabsf(x_sec) / (2.0f * M_PI * freq);
	out->i_amp = (float)(2.0 * sqrt(zden) / wsum);
	out->vs_amp = (float)(2.0 * sqrt(vs_s * vs_s + vs_c * vs_c) / wsum);
	out->vs_mean = (float)vs_mean;
	return true;
}

static bool bh_measure_rl_locked(mc_configuration *cfg, float current, float freq) {
	const float ac_current = fminf(current, BH_RL_AC_CURRENT_A);
	commands_printf("BH_RL_START Ir=%.4f A Il=%.4f A f=%.3f Hz",
			(double)current, (double)ac_current, (double)freq);

	const float target[4] = {-1.0f, -0.5f, 0.5f, 1.0f};
	double si = 0.0, sv = 0.0, sii = 0.0, siv = 0.0;
	for (int n = 0; n < 4; n++) {
		float ia = 0.0f, va = 0.0f;
		if (!bh_measure_dc_point(target[n] * current, &ia, &va)) {
			bh_release_locked();
			return false;
		}
		commands_printf("BH_R_POINT I=%.7f A Vab=%.7f V", (double)ia, (double)va);
		si += ia;
		sv += va;
		sii += (double)ia * ia;
		siv += (double)ia * va;
	}

	double den = 4.0 * sii - si * si;
	if (fabs(den) < 1e-12) {
		commands_printf("BH_RL_ABORT resistance fit is singular");
		bh_release_locked();
		return false;
	}
	float r_line = (float)((4.0 * siv - si * sv) / den);
	float v_offset = (float)((sv - (double)r_line * si) / 4.0);
	if (!(r_line > 0.0001f && r_line < 5.0f)) {
		commands_printf("BH_RL_ABORT implausible Rline=%.8f ohm", (double)r_line);
		bh_release_locked();
		return false;
	}
	commands_printf("BH_RL_STAGE R_DONE Rline=%.6f mOhm Voff=%.6f V",
			(double)(r_line * 1e3f), (double)v_offset);

	bh_release_locked();
	bh_hold_hard_off_ms(20);
	float sense_zero = bh_measure_sense_zero();
	if (!bh_sense_zero_ok) {
		commands_printf("BH_RL_ABORT A1 zero invalid V=%.6f", (double)sense_zero);
		return false;
	}

	// First use intentionally gentle gains. Even if the current only reaches a
	// fraction of the requested amplitude, the search winding still gives a good
	// preliminary L because the calculation uses measured current, not command.
	commands_printf("BH_RL_STAGE AC_PROBE A1zero=%.7f V", (double)sense_zero);
	bh_set_measurement_gains(cfg, r_line);
	mc_interface_set_configuration(cfg);
	bh_hold_hard_off_ms(50);
	bh_ac_probe_t probe;
	if (!bh_measure_ac_search_once(ac_current, freq, sense_zero, &probe)) {
		return false;
	}
	if (!(probe.l_line > 0.1e-6f && probe.l_line < 0.100f) || probe.i_amp < 0.010f) {
		commands_printf("BH_RL_ABORT bad AC probe L=%.6f uH I=%.6f A Vsense=%.7f V",
				(double)(probe.l_line * 1e6f), (double)probe.i_amp, (double)probe.vs_amp);
		return false;
	}
	commands_printf("BH_RL_PROBE Lline=%.6f uH Iac=%.6f A Vsense=%.7f V Rmag=%.6f mOhm",
			(double)(probe.l_line * 1e6f), (double)probe.i_amp,
			(double)probe.vs_amp, (double)(probe.r_mag * 1e3f));

	// Now tune the d-axis PI from the measured fixture itself. The physical A-B
	// plant transforms to Rd=Rline/2 and Ld=Lline/2. Use at least 250 Hz loop
	// bandwidth and about five times the requested test frequency, capped at
	// 1 kHz. Kp is capped at 1 V/A so even an unexpectedly large inductance does
	// not generate an absurd controller gain.
	float r_d = 0.5f * r_line;
	float l_d = 0.5f * probe.l_line;
	float bw = fmaxf(250.0f, fminf(1000.0f, 5.0f * freq));
	float wc = 2.0f * M_PI * bw;
	float kp = l_d * wc;
	float ki = r_d * wc;
	kp = fminf(fmaxf(kp, 0.00001f), 1.0f);
	ki = fminf(fmaxf(ki, 0.1f), 300.0f);
	cfg->foc_motor_r = r_d;
	cfg->foc_motor_l = l_d;
	cfg->foc_current_kp = kp;
	cfg->foc_current_ki = ki;
	mc_interface_set_configuration(cfg);
	bh_hold_hard_off_ms(50);
	commands_printf("BH_RL_STAGE AC_TUNED Kp=%.7f Ki=%.7f BW=%.1f Hz",
			(double)kp, (double)ki, (double)bw);

	bh_ac_probe_t final;
	if (!bh_measure_ac_search_once(ac_current, freq, sense_zero, &final)) {
		return false;
	}
	if (!(final.l_line > 0.1e-6f && final.l_line < 0.100f)) {
		commands_printf("BH_RL_ABORT implausible Lline=%.6f uH (Iac=%.5f A Vsense=%.7f V)",
				(double)(final.l_line * 1e6f), (double)final.i_amp, (double)final.vs_amp);
		return false;
	}
	if (final.i_amp < fmaxf(0.010f, ac_current * 0.25f)) {
		commands_printf("BH_RL_ABORT tuned AC current too small: %.6f A", (double)final.i_amp);
		return false;
	}

	float r_ac = r_line + final.r_mag;
	bh_r_line_ohm = r_line;
	bh_l_line_h = final.l_line;
	bh_rl_valid = true;
	commands_printf("BH_RL_DONE Rline=%.6f mOhm Lline=%.6f uH Rac=%.6f mOhm Rmag=%.6f mOhm Iac=%.6f A Vsense=%.7f V A1mean=%.9f V",
			(double)(r_line * 1e3f), (double)(final.l_line * 1e6f),
			(double)(r_ac * 1e3f), (double)(final.r_mag * 1e3f),
			(double)final.i_amp, (double)final.vs_amp, (double)final.vs_mean);
	commands_printf("BH_RL_FOC effective Rd=%.6f mOhm Ld=%.6f uH",
			(double)(r_line * 500.0f), (double)(final.l_line * 500000.0f));
	return true;
}

static bool bh_precondition_and_offset(float freq, bool sine_wave, float sense_zero,
		float *sense_mean) {
	const float dt_target = 1.0f / BH_SAMPLE_HZ;
	const int total_cycles = BH_SETTLE_CYCLES + BH_OFFSET_CYCLES;
	const float total_time = (float)total_cycles / freq;
	const float mean_start = (float)BH_SETTLE_CYCLES / freq;
	uint32_t t0 = timer_time_now();
	uint32_t t_prev = t0;
	double vs_dt = 0.0;
	double dt_sum = 0.0;
	bh_adc_clip_reset();

	while (!bh_stop_requested) {
		uint32_t loop_start = timer_time_now();
		float t = timer_seconds_elapsed_since(t0);
		if (t >= total_time) {
			break;
		}
		float i_cmd = bh_i_pk * bh_wave(t * freq, sine_wave);
		bh_apply_current(i_cmd);

		uint32_t now = timer_time_now();
		float dt = timer_calc_diff(t_prev, now);
		t_prev = now;
		uint16_t raw = ADC_Value[ADC_IND_EXT];
		bool hard_sample = bh_adc_sample_hard_rail(raw);
		if (bh_adc_clipped()) {
			commands_printf("BH_ABORT sustained A1 clipping raw=%u (>10 hits/s)",
					(unsigned)raw);
			return false;
		}
		if (!hard_sample && t >= mean_start && dt > 0.0f && dt < 0.02f) {
			float vs = (bh_sense_out_v() - sense_zero) / BH_INA_GAIN;
			vs_dt += (double)vs * dt;
			dt_sum += dt;
		}

		mc_fault_code fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			commands_printf("BH_ABORT fault=%d (%s)", (int)fault,
					mc_interface_fault_to_string(fault));
			return false;
		}

		float elapsed = timer_seconds_elapsed_since(loop_start);
		if (elapsed < dt_target) {
			timer_sleep(dt_target - elapsed);
		}
	}

	if (bh_stop_requested || dt_sum <= 0.0) {
		return false;
	}
	*sense_mean = (float)(vs_dt / dt_sum);
	return true;
}
