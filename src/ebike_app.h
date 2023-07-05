/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EBIKE_APP_H_
#define _EBIKE_APP_H_

#include <stdint.h>
#include "main.h"

// startup boost mode
#define CADENCE						0
#define SPEED						1

// cadence sensor
extern uint16_t ui16_cadence_ticks_count_min_speed_adj;
extern uint8_t ui8_pedal_cadence_fast_stop;
extern uint8_t ui8_lights_state;
extern uint8_t ui8_field_weakening_enabled;
extern uint8_t ui8_assist_level_flag;

// Torque sensor coaster brake engaged threshold value
extern uint16_t ui16_adc_coaster_brake_threshold;

// ADC motor phase current max
extern volatile uint8_t ui8_adc_motor_phase_current_max;

// ADC battery current target
//extern volatile uint8_t ui8_adc_battery_current_target;

// Motor
extern uint8_t ui8_motor_inductance_x1048576; 
extern volatile uint8_t ui8_motor_enabled;

// Throttle
extern uint8_t ui8_adc_throttle_assist;

void ebike_app_controller(void);

#endif /* _EBIKE_APP_H_ */
