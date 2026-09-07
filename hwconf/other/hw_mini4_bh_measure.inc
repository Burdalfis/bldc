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
	// voltage to the excitation winding using Np/Ns. Sense lead polarity is
	// arbitrary, so choose the sign that makes inductive reactance positive;
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

/* Shared preconditioning/volt-second offset helper used by the ordinary
 * finite and live B-H capture paths. The obsolete legacy R/L worker that used
 * to sit above this function has been removed; hw_mini4_bh_measure_v2.inc is
 * now the only R/L implementation.
 */
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
