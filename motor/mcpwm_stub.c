#include "mcpwm.h"

volatile float mcpwm_detect_currents[6];
volatile float mcpwm_detect_voltages[6];
volatile float mcpwm_detect_currents_diff[6];
volatile int mcpwm_vzero;

void mcpwm_init(volatile mc_configuration *configuration) {(void)configuration;}
void mcpwm_deinit(void) {}
bool mcpwm_init_done(void) {return true;}
void mcpwm_set_configuration(volatile mc_configuration *configuration) {(void)configuration;}
void mcpwm_init_hall_table(int8_t *table) {(void)table;}
void mcpwm_set_duty(float dutyCycle) {(void)dutyCycle;}
void mcpwm_set_duty_noramp(float dutyCycle) {(void)dutyCycle;}
void mcpwm_set_pid_speed(float rpm) {(void)rpm;}
void mcpwm_set_pid_pos(float pos) {(void)pos;}
void mcpwm_set_current(float current) {(void)current;}
void mcpwm_release_motor(void) {}
void mcpwm_set_brake_current(float current) {(void)current;}
int mcpwm_set_tachometer_value(int steps) {return steps;}
void mcpwm_brake_now(void) {}
int mcpwm_get_comm_step(void) {return 0;}
float mcpwm_get_duty_cycle_set(void) {return 0.0f;}
float mcpwm_get_duty_cycle_now(void) {return 0.0f;}
float mcpwm_get_switching_frequency_now(void) {return 0.0f;}
float mcpwm_get_rpm(void) {return 0.0f;}
mc_state mcpwm_get_state(void) {return MC_STATE_OFF;}
float mcpwm_get_kv(void) {return 0.0f;}
float mcpwm_get_kv_filtered(void) {return 0.0f;}
int mcpwm_get_tachometer_value(bool reset) {(void)reset; return 0;}
int mcpwm_get_tachometer_abs_value(bool reset) {(void)reset; return 0;}
void mcpwm_stop_pwm(void) {}
float mcpwm_get_tot_current(void) {return 0.0f;}
float mcpwm_get_tot_current_filtered(void) {return 0.0f;}
float mcpwm_get_tot_current_directional(void) {return 0.0f;}
float mcpwm_get_tot_current_directional_filtered(void) {return 0.0f;}
float mcpwm_get_tot_current_in(void) {return 0.0f;}
float mcpwm_get_tot_current_in_filtered(void) {return 0.0f;}
void mcpwm_set_detect(void) {}
float mcpwm_get_detect_pos(void) {return 0.0f;}
void mcpwm_reset_hall_detect_table(void) {}
int mcpwm_get_hall_detect_result(int8_t *table) {(void)table; return 0;}
int mcpwm_read_hall_phase(void) {return 0;}
float mcpwm_read_reset_avg_cycle_integrator(void) {return 0.0f;}
void mcpwm_set_comm_mode(mc_comm_mode mode) {(void)mode;}
mc_comm_mode mcpwm_get_comm_mode(void) {return COMM_MODE_INTEGRATE;}
float mcpwm_get_last_adc_isr_duration(void) {return 0.0f;}
float mcpwm_get_last_inj_adc_isr_duration(void) {return 0.0f;}
mc_rpm_dep_struct mcpwm_get_rpm_dep(void) {mc_rpm_dep_struct r = {0}; return r;}
bool mcpwm_is_dccal_done(void) {return true;}
void mcpwm_switch_comm_mode(mc_comm_mode next) {(void)next;}
void drv8323s_dccal_on(void) {}
void drv8323s_dccal_off(void) {}
void mcpwm_adc_inj_int_handler(void) {}
void mcpwm_adc_int_handler(void *p, uint32_t flags) {(void)p; (void)flags;}
