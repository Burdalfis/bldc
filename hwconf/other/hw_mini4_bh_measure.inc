static float bh_measure_sense_zero(void) {
	bh_release_locked();
	chThdSleepMilliseconds(50);
	double sum = 0.0;
	const int samples = 1000;
	for (int i = 0; i < samples; i++) {
		sum += bh_sense_out_v();
		chThdSleepMicroseconds(250);
	}
	return (float)(sum / (double)samples);
}

static bool bh_measure_rl_locked(mc_configuration *cfg, float current, float freq) {
	const float ac_current = fminf(current, BH_RL_AC_CURRENT_A);
	commands_printf("BH_RL_START Ir=%.4f A Il=%.4f A f=%.3f Hz",
			(double)current, (double)ac_current, (double)freq);

	// Four-point bipolar DC fit. The slope suppresses voltage offset and is much
	// less sensitive to inverter asymmetry than a single V/I reading. The high
	// current pulses are deliberately short (see bh_measure_dc_point) so the
	// 26 AWG excitation winding does not appreciably self-heat during the fit.
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

	// The phase-voltage value reported by the FOC controller is reconstructed
	// from modulation and dead-time compensation. At the few tens of millivolts
	// used for this small-signal test those corrections can dominate and even
	// produce an apparent negative AC resistance. The fixture already has a
	// search winding, so use it as a transformer instead: Vs = Ns*dPhi/dt and
	// the core-linked primary back-EMF is (Np/Ns)*Vs. This directly measures the
	// magnetic impedance while avoiding inverter voltage-model errors.
	bh_release_locked();
	chThdSleepMilliseconds(20);
	float sense_zero = bh_measure_sense_zero();
	if (bh_adc_clipped()) {
		commands_printf("BH_RL_ABORT A1 zero near rail raw=%u V=%.6f",
				(unsigned)ADC_Value[ADC_IND_EXT], (double)sense_zero);
		return false;
	}

	commands_printf("BH_RL_STAGE AC_CONFIG A1zero=%.7f V", (double)sense_zero);
	bh_set_measurement_gains(cfg, r_line);
	mc_interface_set_configuration(cfg);
	chThdSleepMilliseconds(50);
	commands_printf("BH_RL_STAGE AC_START I=%.4f A f=%.3f Hz",
			(double)ac_current, (double)freq);

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
		float vs_now = (bh_sense_out_v() - sense_zero) / BH_INA_GAIN;
		if (t >= measure_start && dt > 0.0f && dt < 0.01f) {
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

		if (bh_adc_clipped()) {
			commands_printf("BH_RL_ABORT A1 clipping raw=%u", (unsigned)ADC_Value[ADC_IND_EXT]);
			bh_release_locked();
			return false;
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

	// Remove measured DC components from the lock-in coefficients. This matters
	// because RTOS scheduling jitter means the sine/cosine basis does not sum to
	// exactly zero even though we capture an integer number of nominal cycles.
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

	// Search-winding impedance referred to one ampere of excitation current.
	// Depending on how the sense winding leads are connected, its polarity can
	// be either sign. Choose the polarity that makes inductive reactance positive
	// and apply the same sign to its in-phase (core-loss) component.
	float r_sec = (float)((vs_s * i_s + vs_c * i_c) / zden);
	float x_sec = (float)((vs_c * i_s - vs_s * i_c) / zden);
	float sense_sign = x_sec >= 0.0f ? 1.0f : -1.0f;
	float turns_ratio = BH_EXC_TURNS / BH_SENSE_TURNS;
	float r_mag = turns_ratio * sense_sign * r_sec;
	float x_mag = turns_ratio * fabsf(x_sec);
	float l_line = x_mag / (2.0f * M_PI * freq);
	float r_ac = r_line + r_mag;
	float i_amp = (float)(2.0 * sqrt(zden) / wsum);
	float vs_amp = (float)(2.0 * sqrt(vs_s * vs_s + vs_c * vs_c) / wsum);

	if (!(l_line > 0.1e-6f && l_line < 0.100f)) {
		commands_printf("BH_RL_ABORT implausible Lline=%.6f uH (Rmag=%.6f mOhm Iac=%.5f A Vsense=%.7f V)",
				(double)(l_line * 1e6f), (double)(r_mag * 1e3f),
				(double)i_amp, (double)vs_amp);
		return false;
	}
	if (i_amp < fmaxf(0.010f, ac_current * 0.05f)) {
		commands_printf("BH_RL_ABORT AC current amplitude too small: %.6f A", (double)i_amp);
		return false;
	}

	bh_r_line_ohm = r_line;
	bh_l_line_h = l_line;
	bh_rl_valid = true;
	commands_printf("BH_RL_DONE Rline=%.6f mOhm Lline=%.6f uH Rac=%.6f mOhm Rmag=%.6f mOhm Iac=%.6f A Vsense=%.7f V A1mean=%.9f V",
			(double)(r_line * 1e3f), (double)(l_line * 1e6f),
			(double)(r_ac * 1e3f), (double)(r_mag * 1e3f),
			(double)i_amp, (double)vs_amp, (double)vs_mean);
	commands_printf("BH_RL_FOC effective Rd=%.6f mOhm Ld=%.6f uH",
			(double)(r_line * 500.0f), (double)(l_line * 500000.0f));
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
		if (t >= mean_start && dt > 0.0f && dt < 0.02f) {
			float vs = (bh_sense_out_v() - sense_zero) / BH_INA_GAIN;
			vs_dt += (double)vs * dt;
			dt_sum += dt;
		}

		if (bh_adc_clipped()) {
			commands_printf("BH_ABORT A1 clipping raw=%u", (unsigned)ADC_Value[ADC_IND_EXT]);
			return false;
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
