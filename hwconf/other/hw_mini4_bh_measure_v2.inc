/* Replacement for the legacy R/L worker. It deliberately does not retune the
 * current controller: cfg already contains the user's programmed VESC Kp/Ki.
 * The legacy low-level AC lock-in helper is reused because it compares Vsense
 * against measured current, so current lag relative to the command cancels.
 *
 * Run the AC/search-coil measurement before the DC resistance pulses. Besides
 * avoiding any needless preconditioning of the core, this makes VESC Tool's
 * Sampled Data "Trigger at Motor Start" land on the waveform we actually want
 * to inspect rather than on the first resistance pulse hundreds of ms earlier.
 */
static bool bh_measure_rl_locked(mc_configuration *cfg, float current, float freq) {
	const float ac_current = fminf(current, BH_RL_AC_CURRENT_A);
	commands_printf("BH_RL_START Ir=%.4f A Il=%.4f A f=%.3f Hz Kp=%.7f Ki=%.7f sense=%+d",
			(double)current, (double)ac_current, (double)freq,
			(double)cfg->foc_current_kp, (double)cfg->foc_current_ki,
			bh_sense_polarity);

	// Keep the bridge released while finding the INA282 centre. The first motor
	// RUNNING transition after this is therefore the beginning of the AC sine.
	bh_release_locked();
	chThdSleepMilliseconds(20);
	float sense_zero = bh_measure_sense_zero();
	if (bh_adc_clipped()) {
		commands_printf("BH_RL_ABORT A1 zero near rail raw=%u V=%.6f",
				(unsigned)ADC_Value[ADC_IND_EXT], (double)sense_zero);
		return false;
	}

	commands_printf("BH_RL_STAGE AC_PROGRAMMED A1zero=%.7f V Kp=%.7f Ki=%.7f",
			(double)sense_zero, (double)cfg->foc_current_kp,
			(double)cfg->foc_current_ki);

	bh_ac_probe_t final;
	if (!bh_measure_ac_search_once(ac_current, freq, sense_zero, &final)) {
		return false;
	}
	if (!(final.l_line > 0.1e-6f && final.l_line < 0.100f)) {
		commands_printf("BH_RL_ABORT implausible Lline=%.6f uH (Iac=%.5f A Vsense=%.7f V)",
				(double)(final.l_line * 1e6f), (double)final.i_amp,
				(double)final.vs_amp);
		return false;
	}
	if (final.i_amp < fmaxf(0.010f, ac_current * 0.25f)) {
		commands_printf("BH_RL_ABORT AC current too small: %.6f A", (double)final.i_amp);
		return false;
	}

	// Measure DC resistance after the AC capture. Sampled Data will normally have
	// completed its post-trigger buffer by now, so these pulses no longer consume
	// the useful scope window.
	commands_printf("BH_RL_STAGE R_START");
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

	float tracking = 100.0f * final.i_amp / ac_current;
	float r_ac = r_line + final.r_mag;
	bh_r_line_ohm = r_line;
	bh_l_line_h = final.l_line;
	bh_rl_valid = true;
	commands_printf("BH_RL_DONE Rline=%.6f mOhm Lline=%.6f uH Rac=%.6f mOhm Rmag_norm=%.6f mOhm Icmd=%.6f A Iac=%.6f A track=%.2f%% Vsense=%.7f V A1mean=%.9f V",
			(double)(r_line * 1e3f), (double)(final.l_line * 1e6f),
			(double)(r_ac * 1e3f), (double)(final.r_mag * 1e3f),
			(double)ac_current, (double)final.i_amp, (double)tracking,
			(double)final.vs_amp, (double)final.vs_mean);
	commands_printf("BH_RL_FOC effective Rd=%.6f mOhm Ld=%.6f uH; programmed Kp=%.7f Ki=%.7f",
			(double)(r_line * 500.0f), (double)(final.l_line * 500000.0f),
			(double)cfg->foc_current_kp, (double)cfg->foc_current_ki);
	return true;
}
