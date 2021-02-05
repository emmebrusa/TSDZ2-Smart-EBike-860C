/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#ifndef COMMON_COMMON_H_
#define COMMON_COMMON_H_

// riding modes
#define OFF_MODE                                  0
#define POWER_ASSIST_MODE                         1
#define TORQUE_ASSIST_MODE                        2
#define CADENCE_ASSIST_MODE                       3
#define eMTB_ASSIST_MODE                          4
#define HYBRID_ASSIST_MODE						  5
#define CRUISE_MODE                               6
#define WALK_ASSIST_MODE                          7
//#define WALK_ASSIST_MODE                          5
//#define CRUISE_MODE                               6

// walk assist
#define WALK_ASSIST_THRESHOLD_SPEED_X10           70  // 70 -> 7.0 kph, this is the maximum speed limit from which walk assist can be activated

// cruise
#define CRUISE_THRESHOLD_SPEED_X10                90  // 90 -> 9.0 kph, this is the minimum speed limit from which cruise can be activated

// optional ADC function
#define NOT_IN_USE                                0
#define TEMPERATURE_CONTROL                       1
#define THROTTLE_CONTROL                          2

int16_t map_ui16(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);
uint8_t map_ui8(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_max, uint8_t out_min);
uint8_t ui8_max(uint8_t value_a, uint8_t value_b);
uint8_t ui8_min(uint8_t value_a, uint8_t value_b);
uint16_t filter(uint16_t ui16_new_value, uint16_t ui16_old_value, uint8_t ui8_alpha);
void crc16(uint8_t ui8_data, uint16_t *ui16_crc);

#endif /* COMMON_COMMON_H_ */
