/* B-H worker sequencing fix.
 *
 * The tracer configuration sets cc_min_current=0 so an ordinary 0-A control
 * command can legitimately enter MC_STATE_RUNNING and enable FOC PWM. The
 * legacy worker installed that temporary configuration, waited 100 ms, and
 * only then locked external control. That left a window where app/CAN/USB
 * traffic could start active zero-vector PWM before the tracer waveform began,
 * which both polluted INA282/A1 and caused Sampled Data "motor start" to
 * trigger on setup instead of on the actual excitation waveform.
 *
 * This replacement worker locks control and forces PWM off BEFORE installing
 * the temporary tracer configuration. It keeps the bridge off during the
 * settling/zero-measurement preamble and restores the user's original motor
 * configuration while still locked. Only after the original configuration is
 * back in place is external control unlocked again.
 */
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

		utils_sys_lock_cnt();
		timeout_configure_IWDT_slowest();
		utils_sys_unlock_cnt();
		commands_printf("BH_WDT slow mode active for tracer job");

		/* The ordinary VESC command timeout is intended to stop a motor when its
		 * command source disappears. A tracer job is itself the command source,
		 * and long plot/sample transmissions can legitimately keep this worker
		 * away from timeout_reset() for longer than the user's configured motor
		 * timeout. Save the setting, disable only the age timeout for the duration
		 * of the job, and retain the configured kill-switch mode. The IWDG remains
		 * active independently, and VESC Tool Stop/fault paths still work.
		 */
		systime_t timeout_old_msec = timeout_get_timeout_msec();
		float timeout_old_brake = timeout_get_brake_current();
		KILL_SW_MODE timeout_old_ksw = timeout_get_kill_sw_mode();
		timeout_configure(0, timeout_old_brake, timeout_old_ksw);
		timeout_reset();

		mc_configuration *cfg_old = mempools_alloc_mcconf();
		mc_configuration *cfg = mempools_alloc_mcconf();
		if (!cfg_old || !cfg) {
			commands_printf("BH_ABORT could not allocate motor configuration");
			if (cfg) mempools_free_mcconf(cfg);
			if (cfg_old) mempools_free_mcconf(cfg_old);
			timeout_configure(timeout_old_msec, timeout_old_brake, timeout_old_ksw);
			timeout_reset();
			utils_sys_lock_cnt();
			timeout_configure_IWDT();
			utils_sys_unlock_cnt();
			bh_job = BH_JOB_NONE;
			bh_running = false;
			continue;
		}

		*cfg_old = *mc_interface_get_configuration();
		*cfg = *cfg_old;

		/* Critical ordering: block all ordinary control commands before the
		 * cc_min_current=0 tracer config can become active. Keep feeding timeout
		 * calls in the existing helpers as harmless bookkeeping even though the
		 * age timeout itself is disabled for the job.
		 */
		timeout_reset();
		bh_release_locked();

		bool ok = true;
		if (job == BH_JOB_MEASURE_RL) {
			bh_common_cfg(cfg, bh_rl_current);
			bh_set_measurement_gains(cfg, 0.020f);
			mc_interface_set_configuration(cfg);
			bh_release_locked();
			bh_hold_hard_off_ms(100);
			commands_printf("BH_SETUP locked/off; tracer config settled");
			ok = bh_measure_rl_locked(cfg, bh_rl_current, bh_rl_freq);
		} else {
			bh_common_cfg(cfg, bh_i_pk);
			bh_set_run_gains(cfg);
			mc_interface_set_configuration(cfg);
			bh_release_locked();
			bh_hold_hard_off_ms(100);
			commands_printf("BH_SETUP locked/off; tracer config settled");
			commands_printf("BH_CTRL Rline=%.6f mOhm Lline=%.6f uH Kp=%.7g Ki=%.7g BW=%.1f Hz",
					(double)(bh_r_line_ohm * 1e3f), (double)(bh_l_line_h * 1e6f),
					(double)cfg->foc_current_kp, (double)cfg->foc_current_ki,
					(double)BH_CURRENT_BW_HZ);

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

		/* Restore while control is STILL locked. Never expose the temporary
		 * cc_min_current=0 tracer config to ordinary app/CAN/USB commands.
		 */
		bh_release_locked();
		bh_hold_hard_off_ms(5);
		mc_interface_set_configuration(cfg_old);
		bh_release_locked();
		mc_interface_unlock();

		mempools_free_mcconf(cfg);
		mempools_free_mcconf(cfg_old);

		/* Restore the user's normal command timeout only after the original motor
		 * configuration is back and the bridge is hard-off. Reset its age so the
		 * freshly restored timeout cannot fire immediately on worker exit.
		 */
		timeout_configure(timeout_old_msec, timeout_old_brake, timeout_old_ksw);
		timeout_reset();

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
