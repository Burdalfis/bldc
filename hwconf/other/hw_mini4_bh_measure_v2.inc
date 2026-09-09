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

#define BH_A1_PWM_LUT_POINTS 11
#define BH_A1_PWM_LUT_SEGMENTS (BH_A1_PWM_LUT_POINTS - 1)
#define BH_A1_DC_SAMPLES 40

/* Nonlinear active-PWM feedthrough calibration. At settled DC excitation the
 * true search-coil voltage is zero. The zero-current active-PWM A1 reading is
 * therefore the baseline, and the LUT contains only the current-dependent
 * residual around that baseline. By construction E_I(0 A) = 0 V.
 *
 * Measured coil current is the independent variable: unlike Vd it does not
 * acquire an L*dI/dt term during live AC drive. Segment slopes are precomputed
 * so the hot fast-acquisition path does not perform a floating-point division.
 */
static float bh_sense_pwm_i_lut[BH_A1_PWM_LUT_POINTS];
static float bh_sense_pwm_err_lut[BH_A1_PWM_LUT_POINTS];
static float bh_sense_pwm_slope_lut[BH_A1_PWM_LUT_SEGMENTS];
static bool bh_sense_pwm_lut_valid = false;

static inline float bh_sense_pwm_lut_error_v(float i_line) {
	if (!bh_sense_pwm_lut_valid) {
		return 0.0f;
	}
	if (i_line <= bh_sense_pwm_i_lut[0]) {
		return bh_sense_pwm_err_lut[0];
	}
	if (i_line >= bh_sense_pwm_i_lut[BH_A1_PWM_LUT_POINTS - 1]) {
		return bh_sense_pwm_err_lut[BH_A1_PWM_LUT_POINTS - 1];
	}

	int seg = 0;
	while (seg < BH_A1_PWM_LUT_SEGMENTS - 1 &&
			i_line > bh_sense_pwm_i_lut[seg + 1]) {
		seg++;
	}
	return bh_sense_pwm_err_lut[seg] +
			bh_sense_pwm_slope_lut[seg] *
			(i_line - bh_sense_pwm_i_lut[seg]);
}

/* Shorter DC sample point for the 11-point resistance/feedthrough sweep. Every
 * point starts and ends at zero current so high-current dwell stays brief even
 * though the calibration now has many more operating points. The zero target is
 * intentional: bh_apply_current(0) keeps FOC/PWM active, giving us the active-PWM
 * zero-current baseline separately from the hard-off A1 zero.
 */
static bool bh_measure_dc_cal_point(float target, float *i_avg, float *vab_avg,
		float *vd_avg, float *duty_avg, float *a1_avg) {
	if (!bh_ramp_current(0.0f, target, 0.015f)) {
		return false;
	}
	if (!bh_wait_and_check(0.012f)) {
		return false;
	}

	double isum = 0.0;
	double vabsum = 0.0;
	double vdsum = 0.0;
	double dutysum = 0.0;
	double a1sum = 0.0;
	for (int i = 0; i < BH_A1_DC_SAMPLES; i++) {
		if (bh_stop_requested) {
			return false;
		}
		bh_apply_current(target);
		float vd = mcpwm_foc_get_vd();
		isum += bh_coil_current();
		vdsum += vd;
		vabsum += BH_SQRT3 * vd;
		dutysum += mc_interface_get_duty_cycle_now();
		a1sum += bh_sense_out_v();

		mc_fault_code fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			commands_printf("BH_ABORT fault=%d (%s)", (int)fault,
					mc_interface_fault_to_string(fault));
			return false;
		}
		chThdSleepMicroseconds(500);
	}

	const double inv_n = 1.0 / (double)BH_A1_DC_SAMPLES;
	*i_avg = (float)(isum * inv_n);
	*vab_avg = (float)(vabsum * inv_n);
	*vd_avg = (float)(vdsum * inv_n);
	*duty_avg = (float)(dutysum * inv_n);
	*a1_avg = (float)(a1sum * inv_n);
	return bh_ramp_current(target, 0.0f, 0.008f);
}

static bool bh_measure_rl_locked(mc_configuration *cfg, float current, float freq) {
	const float ac_current = fminf(current, BH_RL_AC_CURRENT_A);
	commands_printf("BH_RL_START Ir=%.4f A Il=%.4f A f=%.3f Hz Kp=%.7f Ki=%.7f sense=%+d",
			(double)current, (double)ac_current, (double)freq,
			(double)cfg->foc_current_kp, (double)cfg->foc_current_ki,
			bh_sense_polarity);

	// Do not carry an old feedthrough LUT through a failed/incomplete R test.
	bh_sense_pwm_lut_valid = false;
	memset(bh_sense_pwm_i_lut, 0, sizeof(bh_sense_pwm_i_lut));
	memset(bh_sense_pwm_err_lut, 0, sizeof(bh_sense_pwm_err_lut));
	memset(bh_sense_pwm_slope_lut, 0, sizeof(bh_sense_pwm_slope_lut));

	// The shared zero sampler keeps the bridge genuinely OFF and feeds the
	// normal VESC motor timeout so it cannot unlock us and restart 0-A FOC PWM.
	float sense_zero = bh_measure_sense_zero();
	if (!bh_sense_zero_ok) {
		commands_printf("BH_RL_ABORT A1 zero invalid V=%.6f", (double)sense_zero);
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

	/* Re-measure hard-off A1 immediately before the DC sweep. Keep this value
	 * for diagnostics only; the nonlinear LUT below is referenced to the active
	 * zero-current PWM point, not to this hard-off value.
	 */
	float dc_sense_zero = bh_measure_sense_zero();
	if (!bh_sense_zero_ok) {
		commands_printf("BH_RL_ABORT DC-stage A1 zero invalid V=%.6f",
				(double)dc_sense_zero);
		return false;
	}

	commands_printf("BH_RL_STAGE R_A1_LUT_START points=%d A1off=%.7f V axis=Icoil",
			BH_A1_PWM_LUT_POINTS, (double)dc_sense_zero);

	float lut_i[BH_A1_PWM_LUT_POINTS];
	float lut_a1[BH_A1_PWM_LUT_POINTS];
	float lut_err[BH_A1_PWM_LUT_POINTS];
	double si = 0.0, sv = 0.0, sii = 0.0, siv = 0.0;

	for (int n = 0; n < BH_A1_PWM_LUT_POINTS; n++) {
		float frac = -1.0f + 2.0f * (float)n /
				(float)(BH_A1_PWM_LUT_POINTS - 1);
		float ia = 0.0f;
		float vab = 0.0f;
		float vd = 0.0f;
		float duty = 0.0f;
		float a1 = 0.0f;
		if (!bh_measure_dc_cal_point(frac * current, &ia, &vab, &vd, &duty, &a1)) {
			bh_release_locked();
			return false;
		}

		lut_i[n] = ia;
		lut_a1[n] = a1;
		commands_printf("BH_R_POINT idx=%d frac=%+.3f I=%+.7f A Vd=%+.7f V Vab=%+.7f V duty=%+.6f A1=%.7f V HardErr=%+.6f mV",
				n, (double)frac, (double)ia, (double)vd, (double)vab,
				(double)duty, (double)a1,
				(double)((a1 - dc_sense_zero) * 1e3f));

		si += ia;
		sv += vab;
		sii += (double)ia * ia;
		siv += (double)ia * vab;
	}

	const int zero_idx = BH_A1_PWM_LUT_POINTS / 2;
	float pwm_zero = lut_a1[zero_idx];
	float zero_i_meas = lut_i[zero_idx];
	if (!isfinite(pwm_zero) || !isfinite(zero_i_meas) ||
			fabsf(zero_i_meas) > fmaxf(0.10f, current * 0.05f)) {
		commands_printf("BH_RL_ABORT active-PWM zero invalid A1=%.7f V I=%+.6f A",
				(double)pwm_zero, (double)zero_i_meas);
		bh_release_locked();
		return false;
	}

	/* Define the current-dependent correction around active-PWM zero. Force the
	 * center knot to exactly (0 A, 0 V); the measured near-zero current is still
	 * retained above for the resistance fit and diagnostic print.
	 */
	for (int n = 0; n < BH_A1_PWM_LUT_POINTS; n++) {
		lut_err[n] = lut_a1[n] - pwm_zero;
	}
	lut_i[zero_idx] = 0.0f;
	lut_err[zero_idx] = 0.0f;

	commands_printf("BH_A1_PWM_ZERO hardoff=%.7f V active=%.7f V shift=%+.6f mV I=%+.6f A",
			(double)dc_sense_zero, (double)pwm_zero,
			(double)((pwm_zero - dc_sense_zero) * 1e3f),
			(double)zero_i_meas);

	const double npts = (double)BH_A1_PWM_LUT_POINTS;
	double den = npts * sii - si * si;
	if (fabs(den) < 1e-12) {
		commands_printf("BH_RL_ABORT resistance fit is singular");
		bh_release_locked();
		return false;
	}
	float r_line = (float)((npts * siv - si * sv) / den);
	float v_offset = (float)((sv - (double)r_line * si) / npts);
	if (!(r_line > 0.0001f && r_line < 5.0f)) {
		commands_printf("BH_RL_ABORT implausible Rline=%.8f ohm", (double)r_line);
		bh_release_locked();
		return false;
	}
	commands_printf("BH_RL_STAGE R_DONE Rline=%.6f mOhm Voff=%.6f V",
			(double)(r_line * 1e3f), (double)v_offset);

	/* Sort by measured current rather than assuming command order is perfectly
	 * monotonic. This keeps interpolation robust to current-control asymmetry.
	 */
	for (int i = 1; i < BH_A1_PWM_LUT_POINTS; i++) {
		float i_key = lut_i[i];
		float err_key = lut_err[i];
		int j = i - 1;
		while (j >= 0 && lut_i[j] > i_key) {
			lut_i[j + 1] = lut_i[j];
			lut_err[j + 1] = lut_err[j];
			j--;
		}
		lut_i[j + 1] = i_key;
		lut_err[j + 1] = err_key;
	}

	bool lut_ok = true;
	for (int i = 0; i < BH_A1_PWM_LUT_POINTS; i++) {
		if (!isfinite(lut_i[i]) || !isfinite(lut_err[i]) ||
				fabsf(lut_err[i]) > 1.0f) {
			lut_ok = false;
		}
		if (i > 0 && lut_i[i] - lut_i[i - 1] < 1.0e-4f) {
			lut_ok = false;
		}
	}

	if (lut_ok) {
		for (int i = 0; i < BH_A1_PWM_LUT_POINTS; i++) {
			bh_sense_pwm_i_lut[i] = lut_i[i];
			bh_sense_pwm_err_lut[i] = lut_err[i];
		}
		for (int i = 0; i < BH_A1_PWM_LUT_SEGMENTS; i++) {
			float di = bh_sense_pwm_i_lut[i + 1] -
					bh_sense_pwm_i_lut[i];
			bh_sense_pwm_slope_lut[i] =
					(bh_sense_pwm_err_lut[i + 1] -
					 bh_sense_pwm_err_lut[i]) / di;
		}
		bh_sense_pwm_lut_valid = true;
		commands_printf("BH_A1_PWM_LUT valid points=%d I=[%+.7f,%+.7f] A A1pwm0=%.7f V",
				BH_A1_PWM_LUT_POINTS,
				(double)bh_sense_pwm_i_lut[0],
				(double)bh_sense_pwm_i_lut[BH_A1_PWM_LUT_POINTS - 1],
				(double)pwm_zero);
		for (int i = 0; i < BH_A1_PWM_LUT_POINTS; i++) {
			commands_printf("BH_A1_LUT idx=%d I=%+.7f A Err=%+.6f mV",
					i, (double)bh_sense_pwm_i_lut[i],
					(double)(bh_sense_pwm_err_lut[i] * 1e3f));
		}
	} else {
		commands_printf("BH_A1_PWM_LUT rejected: non-finite, excessive error, or non-distinct current points; live compensation disabled");
	}

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
