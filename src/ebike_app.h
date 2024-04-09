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

// Torque sensor coaster brake engaged threshold value
extern uint16_t ui16_adc_coaster_brake_threshold;
extern uint8_t ui8_coaster_brake_enabled;

// ADC motor phase current max
extern volatile uint8_t ui8_adc_motor_phase_current_max;

// Motor ERPS
extern uint16_t ui16_motor_speed_erps;


void ebike_app_controller(void);

#endif /* _EBIKE_APP_H_ */
