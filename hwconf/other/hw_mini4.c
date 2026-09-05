/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se
	B-H tracer additions 2026

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
*/

#include "hw.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_math.h"
#include "drv8301.h"
#include "mc_interface.h"
#include "terminal.h"
#include "commands.h"
#include "timeout.h"
#include "mempools.h"
#include "timer.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// Variables
static volatile bool i2c_running = false;

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

// -----------------------------------------------------------------------------
// B-H tracer
// -----------------------------------------------------------------------------
// Fixture wiring:
//   Excitation winding: 20 turns, phase output 1 to phase output 2
//   Search winding:     10 turns -> INA282 -> COMM A1 / ADC_EXT1
//   INA282 gain:        50 V/V, nominal output center 1.65 V
//
// The fixed FOC vector is -30 degrees. For an A-B two-terminal current I:
//   Ia = +I, Ib = -I, Ic = 0
// and the equivalent d-axis current is 2/sqrt(3) * I.

#define BH_EXC_TURNS             20.0f
#define BH_SENSE_TURNS           10.0f
#define BH_INA_GAIN              50.0f
#define BH_FOC_PHASE_DEG         (-30.0f)
#define BH_SAMPLE_HZ             2000.0f
#define BH_PLOT_HZ               100.0f
#define BH_MAX_CURRENT_A         10.0f
#define BH_MIN_FREQ_HZ           0.2f
#define BH_MAX_FREQ_HZ           50.0f
#define BH_MAX_CYCLES            100

static THD_WORKING_AREA(bh_thread_wa, 768);
static thread_t *bh_tp = 0;
static volatile bool bh_running = false;
static volatile bool bh_stop_requested = false;

static float bh_i_pk = 0.0f;
static float bh_freq = 0.0f;
static float bh_path_m = 0.0f;
static float bh_area_m2 = 0.0f;
static float bh_r_ohm = 0.0f;
static float bh_l_h = 0.0f;
static int bh_cycles = 0;

static void terminal_bh_run(int argc, const char **argv);
static void terminal_bh_stop(int argc, const char **argv);
static void terminal_bh_adc(int argc, const char **argv);
static THD_FUNCTION(bh_thread, arg);

static inline float bh_triangle(float phase01) {
	// Triangle wave in [-1, +1], starts at zero heading positive.
	phase01 -= floorf(phase01);
	if (phase01 < 0.25f) {
		return phase01 * 4.0f;
	} else if (phase01 < 0.75f) {
		return 2.0f - phase01 * 4.0f;
	} else {
		return phase01 * 4.0f - 4.0f;
	}
}

static inline float bh_coil_current(void) {
	// ADC_curr_norm_value is already offset-corrected and scaled to amperes by FOC.
	// Averaging the two terminal currents rejects common-mode current-sense error.
	return 0.5f * (ADC_curr_norm_value[0] - ADC_curr_norm_value[1]);
}

static inline float bh_sense_out_v(void) {
	return ADC_VOLTS(ADC_IND_EXT);
}

static void terminal_bh_adc(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	commands_printf("BH A1 raw=%u out=%.6f V, Ia=%.5f A Ib=%.5f A Icoil=%.5f A",
			(unsigned)ADC_Value[ADC_IND_EXT],
			(double)bh_sense_out_v(),
			(double)ADC_curr_norm_value[0],
			(double)ADC_curr_norm_value[1],
			(double)bh_coil_current());
}

static void terminal_bh_stop(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	if (bh_running) {
		bh_stop_requested = true;
		commands_printf("B-H test stop requested.");
	} else {
		commands_printf("B-H test is not running.");
	}
}

static void terminal_bh_run(int argc, const char **argv) {
	if (argc != 8) {
		commands_printf("Usage: bh_run <Ipk_A> <freq_Hz> <path_mm> <area_mm2> <R_mOhm> <L_uH> <cycles>");
		commands_printf("Example: bh_run 1.0 10 50 4.0 100 500 5");
		return;
	}

	if (bh_running) {
		commands_printf("A B-H test is already running. Use bh_stop first.");
		return;
	}

	float i_pk = strtof(argv[1], 0);
	float freq = strtof(argv[2], 0);
	float path_mm = strtof(argv[3], 0);
	float area_mm2 = strtof(argv[4], 0);
	float r_mohm = strtof(argv[5], 0);
	float l_uh = strtof(argv[6], 0);
	int cycles = atoi(argv[7]);

	if (!(i_pk > 0.0f && i_pk <= BH_MAX_CURRENT_A) ||
			!(freq >= BH_MIN_FREQ_HZ && freq <= BH_MAX_FREQ_HZ) ||
			!(path_mm > 0.0f) || !(area_mm2 > 0.0f) ||
			!(r_mohm > 0.0f) || !(l_uh > 0.0f) ||
			cycles < 1 || cycles > BH_MAX_CYCLES) {
		commands_printf("Invalid B-H test arguments.");
		commands_printf("Limits: 0<Ipk<=%.1f A, %.1f<=f<=%.1f Hz, 1<=cycles<=%d; geometry/R/L must be >0.",
				(double)BH_MAX_CURRENT_A, (double)BH_MIN_FREQ_HZ,
				(double)BH_MAX_FREQ_HZ, BH_MAX_CYCLES);
		return;
	}

	bh_i_pk = i_pk;
	bh_freq = freq;
	bh_path_m = path_mm * 1e-3f;
	bh_area_m2 = area_mm2 * 1e-6f;
	bh_r_ohm = r_mohm * 1e-3f;
	bh_l_h = l_uh * 1e-6f;
	bh_cycles = cycles;
	bh_stop_requested = false;

	if (!bh_tp) {
		bh_tp = chThdCreateStatic(bh_thread_wa, sizeof(bh_thread_wa), NORMALPRIO + 1, bh_thread, 0);
	} else {
		chEvtSignal(bh_tp, (eventmask_t)1);
	}

	commands_printf("B-H test queued: Ipk=%.4f A, f=%.3f Hz, path=%.3f mm, area=%.4f mm^2, R=%.3f mOhm, L=%.3f uH, cycles=%d",
			(double)i_pk, (double)freq, (double)path_mm, (double)area_mm2,
			(double)r_mohm, (double)l_uh, cycles);
}

static THD_FUNCTION(bh_thread, arg) {
	(void)arg;
	chRegSetThreadName("MINI4 B-H tracer");

	for (;;) {
		if (bh_tp && !bh_running && bh_i_pk <= 0.0f) {
			chEvtWaitAny(ALL_EVENTS);
		}

		if (bh_i_pk <= 0.0f || bh_running) {
			chThdSleepMilliseconds(10);
			continue;
		}

		bh_running = true;
		bh_stop_requested = false;

		mc_configuration *cfg_old = mempools_alloc_mcconf();
		mc_configuration *cfg = mempools_alloc_mcconf();
		*cfg_old = *mc_interface_get_configuration();
		*cfg = *cfg_old;

		// Use FOC as a precision bidirectional current regulator for the A-B winding.
		cfg->motor_type = MOTOR_TYPE_FOC;
		cfg->foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
		cfg->foc_cc_decoupling = FOC_CC_DECOUPLING_DISABLED;
		cfg->foc_mtpa_mode = MTPA_MODE_OFF;
		cfg->foc_fw_current_max = 0.0f;
		cfg->foc_motor_r = bh_r_ohm;
		cfg->foc_motor_l = bh_l_h;
		cfg->foc_motor_ld_lq_diff = 0.0f;
		cfg->foc_motor_flux_linkage = 1e-6f;
		cfg->foc_f_zv = 30000.0f;
		cfg->foc_control_sample_mode = FOC_CONTROL_SAMPLE_MODE_V0_V7;

		// VESC current-loop convention: Kp=L*bw, Ki=R*bw.
		// 1 kHz is intentionally conservative for this measurement fixture.
		const float current_bw = 2.0f * M_PI * 1000.0f;
		cfg->foc_current_kp = bh_l_h * current_bw;
		cfg->foc_current_ki = bh_r_ohm * current_bw;

		float id_lim = bh_i_pk * (2.0f / SQRT3);
		cfg->l_current_max = fmaxf(cfg->l_current_max, id_lim * 1.25f);
		cfg->l_current_min = fminf(cfg->l_current_min, -id_lim * 1.25f);
		cfg->l_abs_current_max = fmaxf(cfg->l_abs_current_max, id_lim * 1.5f);

		mc_interface_set_configuration(cfg);
		chThdSleepMilliseconds(300);

		// Calibrate INA/ADC center with the excitation off. Using measured zero rather
		// than assuming 1.650 V removes INA offset and 3.3 V rail mismatch.
		mc_interface_release_motor();
		chThdSleepMilliseconds(100);
		double zero_sum = 0.0;
		const int zero_samples = 1000;
		for (int i = 0; i < zero_samples; i++) {
			zero_sum += bh_sense_out_v();
			chThdSleepMicroseconds(500);
		}
		const float sense_zero_v = (float)(zero_sum / (double)zero_samples);

		commands_printf("BH_START zero=%.7f V, Kp=%.7g, Ki=%.7g",
				(double)sense_zero_v, (double)cfg->foc_current_kp, (double)cfg->foc_current_ki);
		commands_printf("BH_RAW columns: t_s,Icmd_A,Icoil_A,A1_raw,A1_V,Vsense_V,H_Apm,B_T");

		commands_init_plot("H (A/m)", "B (T)");
		commands_plot_add_graph("B-H loop");
		commands_plot_set_graph(0);

		const float dt_target = 1.0f / BH_SAMPLE_HZ;
		const int plot_div = (int)(BH_SAMPLE_HZ / BH_PLOT_HZ);
		const float total_time = (float)bh_cycles / bh_freq;
		uint32_t t_start = timer_time_now();
		uint32_t t_prev = t_start;
		int sample_n = 0;
		float b_t = 0.0f;
		float last_vsense = 0.0f;
		bool first = true;
		bool faulted = false;

		while (!bh_stop_requested) {
			float t = timer_seconds_elapsed_since(t_start);
			if (t >= total_time) {
				break;
			}

			float tri = bh_triangle(t * bh_freq);
			float i_cmd = bh_i_pk * tri;
			float id_cmd = i_cmd * (2.0f / SQRT3);

			timeout_reset();
			mc_interface_lock_override_once();
			mc_interface_set_openloop_phase(id_cmd, BH_FOC_PHASE_DEG);

			uint32_t now = timer_time_now();
			float dt = timer_calc_diff(t_prev, now);
			t_prev = now;

			float i_coil = bh_coil_current();
			uint16_t a1_raw = ADC_Value[ADC_IND_EXT];
			float a1_v = bh_sense_out_v();
			float v_sense = (a1_v - sense_zero_v) / BH_INA_GAIN;

			if (!first && dt > 0.0f && dt < 0.01f) {
				b_t += 0.5f * (v_sense + last_vsense) * dt /
						(BH_SENSE_TURNS * bh_area_m2);
			}
			first = false;
			last_vsense = v_sense;

			float h_apm = BH_EXC_TURNS * i_coil / bh_path_m;

			// Send raw data at 100 Hz to keep terminal bandwidth reasonable. Acquisition
			// and integration still run at 2 kHz.
			if ((sample_n % plot_div) == 0) {
				commands_send_plot_points(h_apm, b_t);
				commands_printf("BH_RAW %.7f,%.7f,%.7f,%u,%.7f,%.9f,%.7f,%.9f",
						(double)t, (double)i_cmd, (double)i_coil, (unsigned)a1_raw,
						(double)a1_v, (double)v_sense, (double)h_apm, (double)b_t);
			}

			mc_fault_code fault = mc_interface_get_fault();
			if (fault != FAULT_CODE_NONE) {
				commands_printf("BH_ABORT fault=%d", (int)fault);
				faulted = true;
				break;
			}

			// INA282 or ADC clipping guard. Stop well before the STM32 input rails.
			if (a1_raw < 40 || a1_raw > 4055) {
				commands_printf("BH_ABORT A1 clipping raw=%u", (unsigned)a1_raw);
				faulted = true;
				break;
			}

			sample_n++;
			float elapsed = timer_seconds_elapsed_since(now);
			if (elapsed < dt_target) {
				timer_sleep(dt_target - elapsed);
			}
		}

		mc_interface_release_motor();
		chThdSleepMilliseconds(50);
		mc_interface_set_configuration(cfg_old);
		mempools_free_mcconf(cfg);
		mempools_free_mcconf(cfg_old);

		commands_printf("BH_DONE samples=%d%s", sample_n, faulted ? " ABORTED" : "");
		bh_i_pk = 0.0f;
		bh_running = false;
		bh_stop_requested = false;
	}
}

void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// LEDs
	palSetPadMode(GPIOB, 0, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOB, 1, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	// ENABLE_GATE
	palSetPadMode(GPIOB, 5, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	ENABLE_GATE();

	// PWM outputs
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// Fault pin
	palSetPadMode(GPIOB, 7, PAL_MODE_INPUT_PULLUP);

	// ADC pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);

	drv8301_init();

	terminal_register_command_callback(
			"bh_run",
			"Run MINI4 B-H tracer. Excitation P1-P2, INA282 OUT on A1.",
			"<Ipk_A> <freq_Hz> <path_mm> <area_mm2> <R_mOhm> <L_uH> <cycles>",
			terminal_bh_run);
	terminal_register_command_callback(
			"bh_stop",
			"Stop the active MINI4 B-H tracer test.",
			0,
			terminal_bh_stop);
	terminal_register_command_callback(
			"bh_adc",
			"Print current A1/search-coil and phase-current readings.",
			0,
			terminal_bh_adc);
}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_15Cycles);

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 5, ADC_SampleTime_15Cycles);

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 5, ADC_SampleTime_15Cycles);

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_15Cycles);
}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);
	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) | PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 | PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) | PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 | PAL_STM32_PUDR_PULLUP);
		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}
	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);
	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);
		i2cStop(&HW_I2C_DEV);
		i2c_running = false;
	}
	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_MID1 | PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_MID1 | PAL_STM32_PUDR_PULLUP);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		for (int i = 0; i < 16; i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) | PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 | PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) | PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 | PAL_STM32_PUDR_PULLUP);
		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2cReleaseBus(&HW_I2C_DEV);
	}
}
