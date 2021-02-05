/*
 * TongSheng TSDZ2 motor controller firmware
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#include "ebike_app.h"
#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "stm8s_gpio.h"
#include "main.h"
#include "interrupts.h"
#include "adc.h"
#include "motor.h"
#include "pwm.h"
#include "uart.h"
#include "brake.h"
#include "lights.h"
#include "common.h"

// from v.1.1.0
// Error state (changed)
#define NO_ERROR                                0			// "None"
#define ERROR_NOT_INIT                          1			// "Motor not init"
#define ERROR_TORQUE_SENSOR                     (1 << 1)	// "Torque Fault"
#define ERROR_CADENCE_SENSOR		    		(1 << 2)	// "Cadence fault"
#define ERROR_MOTOR_BLOCKED     				(1 << 3)	// "Motor Blocked"
//#define ERROR_THROTTLE						 (1 << 4)	// "Throttle Fault"
//#define ERROR_          						 (1 << 5)	// "Free"
#define ERROR_FATAL                             (1 << 6)	// "Comms"
#define ERROR_SPEED_SENSOR	                    (1 << 7)	// "Speed fault"

// Motor init state
#define MOTOR_INIT_STATE_RESET                  0
#define MOTOR_INIT_STATE_NO_INIT                1
#define MOTOR_INIT_STATE_INIT_START_DELAY       2
#define MOTOR_INIT_STATE_INIT_WAIT_DELAY        3
#define MOTOR_INIT_OK                           4

// Motor init status
#define MOTOR_INIT_STATUS_RESET                 0
#define MOTOR_INIT_STATUS_GOT_CONFIG            1
#define MOTOR_INIT_STATUS_INIT_OK               2

// Communications package frame type
#define COMM_FRAME_TYPE_ALIVE                         0
#define COMM_FRAME_TYPE_STATUS                        1
#define COMM_FRAME_TYPE_PERIODIC                      2
#define COMM_FRAME_TYPE_CONFIGURATIONS                3
#define COMM_FRAME_TYPE_FIRMWARE_VERSION              4

// variables for various system functions
volatile uint8_t ui8_m_system_state = ERROR_NOT_INIT; // start with system error because configurations are empty at startup
volatile uint8_t ui8_m_motor_init_state = MOTOR_INIT_STATE_RESET;
volatile uint8_t ui8_m_motor_init_status = MOTOR_INIT_STATUS_RESET;
static uint8_t m_ui8_got_configurations_timer = 0;

// Initial configuration values
volatile struct_configuration_variables m_configuration_variables = {
        .ui16_battery_low_voltage_cut_off_x10 = 300, // 36 V battery, 30.0V (3.0 * 10)
        .ui16_wheel_perimeter = 2050,                // 26'' wheel: 2050 mm perimeter
        .ui8_wheel_speed_max = 25,                   // 25 Km/h
        .ui8_motor_inductance_x1048576 = 80,         // 36V motor 76 uH
        .ui8_pedal_torque_per_10_bit_ADC_step_x100 = 67,
        .ui8_target_battery_max_power_div25 = 20,    // 500W (500/25 = 20)
        .ui8_optional_ADC_function = 0               // 0 = no function
        };

// system
static uint8_t ui8_assist_level = 0;
static uint8_t ui8_riding_mode = OFF_MODE;
static uint8_t ui8_riding_mode_parameter = 0;
static uint8_t ui8_walk_assist_parameter = 0;
static uint8_t ui8_motor_enabled = 1;
static uint8_t ui8_assist_without_pedal_rotation_threshold = 0;
static uint8_t ui8_assist_without_pedal_rotation_enabled = 0;
static uint8_t ui8_lights_configuration = 0;
static uint8_t ui8_lights_state = 0;
static uint8_t ui8_hybrid_assist_enabled = 0;
static uint8_t ui8_assist_whit_error_enabled = 0;

// power control
static uint8_t ui8_battery_current_max = DEFAULT_VALUE_BATTERY_CURRENT_MAX;
static uint8_t ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
static uint8_t ui8_duty_cycle_ramp_up_inverse_step_default = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
static uint8_t ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
static uint16_t ui16_battery_voltage_filtered_x1000 = 0;
static uint8_t ui8_battery_current_filtered_x10 = 0;
static uint8_t ui8_adc_battery_current_max = ADC_10_BIT_BATTERY_CURRENT_MAX;
static uint8_t ui8_adc_battery_current_target = 0;
static uint8_t ui8_duty_cycle_target = 0;
static uint8_t ui8_adc_battery_current_min = 0;

// brakes
static uint8_t ui8_brakes_engaged = 0;

// cadence sensor
uint16_t ui16_cadence_sensor_ticks_counter_min_speed_adjusted = CADENCE_SENSOR_CALC_COUNTER_MIN;
static uint8_t ui8_pedal_cadence_RPM = 0;

// torque sensor
static uint16_t ui16_adc_pedal_torque_offset = ADC_TORQUE_SENSOR_OFFSET_DEFAULT;
static uint16_t ui16_adc_pedal_torque_offset_fix = ADC_TORQUE_SENSOR_OFFSET_DEFAULT;
static uint16_t ui16_adc_pedal_torque_offset_init = ADC_TORQUE_SENSOR_OFFSET_DEFAULT;
static uint16_t ui16_adc_pedal_torque_offset_min = ADC_TORQUE_SENSOR_OFFSET_DEFAULT;
static uint16_t ui16_adc_pedal_torque_offset_max = ADC_TORQUE_SENSOR_OFFSET_DEFAULT;
static uint16_t ui16_adc_pedal_torque_offset_temp = ADC_TORQUE_SENSOR_OFFSET_DEFAULT;
static uint8_t ui8_adc_pedal_torque_offset_error = 0;
static uint16_t ui16_adc_pedal_torque_range = 0;
uint8_t ui8_adc_coaster_brake_threshold = 0;
static uint8_t ui8_coaster_brake_enabled = 0;
static uint8_t ui8_coaster_brake_torque_threshold = 0;
static uint8_t ui8_coaster_brake_torque_threshold_temp = 0;
static uint16_t ui16_adc_pedal_torque = 0;
static uint16_t ui16_adc_pedal_torque_delta = 0;
static uint16_t ui16_adc_pedal_torque_delta_no_boost = 0;
static uint16_t ui16_pedal_torque_x100 = 0;
static uint8_t ui8_torque_sensor_calibration_enabled = 0;
static uint8_t ui8_hybrid_torque_parameter = 0;

// wheel speed sensor
static uint16_t ui16_wheel_speed_x10 = 0;

// throttle control
volatile uint8_t ui8_throttle_adc = 0;
volatile uint8_t ui8_throttle_virtual;

// motor temperature control
static uint16_t ui16_adc_motor_temperature_filtered = 0;
static uint16_t ui16_motor_temperature_filtered_x10 = 0;
static uint8_t ui8_motor_temperature_max_value_to_limit = 0;
static uint8_t ui8_motor_temperature_min_value_to_limit = 0;

// eMTB assist
#define eMTB_POWER_FUNCTION_ARRAY_SIZE      241

static const uint8_t ui8_eMTB_power_function_160[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5,
        5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11, 11, 12,
        12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 17, 17, 17, 17, 18, 18, 18, 18,
        19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 22, 23, 23, 23, 24, 24, 24, 24, 25, 25, 25, 26, 26, 26, 27,
        27, 27, 27, 28, 28, 28, 29, 29, 29, 30, 30, 30, 31, 31, 31, 32, 32, 32, 33, 33, 33, 34, 34, 34, 35, 35, 35, 36,
        36, 36, 37, 37, 37, 38, 38, 38, 39, 39, 40, 40, 40, 41, 41, 41, 42, 42, 42, 43, 43, 44, 44, 44, 45, 45, 45, 46,
        46, 47, 47, 47, 48, 48, 48, 49, 49, 50, 50, 50, 51, 51, 52, 52, 52, 53, 53, 54, 54, 54, 55, 55, 56, 56, 56, 57,
        57, 58, 58, 58, 59, 59, 60, 60, 61, 61, 61, 62, 62, 63, 63, 63, 64, 64 };
static const uint8_t ui8_eMTB_power_function_165[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6,
        6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 14, 14,
        14, 14, 15, 15, 15, 16, 16, 16, 16, 17, 17, 17, 18, 18, 18, 19, 19, 19, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23,
        23, 23, 24, 24, 24, 25, 25, 25, 26, 26, 27, 27, 27, 28, 28, 28, 29, 29, 30, 30, 30, 31, 31, 32, 32, 32, 33, 33,
        34, 34, 34, 35, 35, 36, 36, 36, 37, 37, 38, 38, 39, 39, 39, 40, 40, 41, 41, 42, 42, 42, 43, 43, 44, 44, 45, 45,
        46, 46, 47, 47, 47, 48, 48, 49, 49, 50, 50, 51, 51, 52, 52, 53, 53, 54, 54, 55, 55, 56, 56, 57, 57, 58, 58, 59,
        59, 60, 60, 61, 61, 62, 62, 63, 63, 64, 64, 65, 65, 66, 66, 67, 67, 68, 68, 69, 69, 70, 71, 71, 72, 72, 73, 73,
        74, 74, 75, 75, 76, 77, 77, 78, 78, 79, 79, 80, 81, 81, 82, 82, 83, 83, 84, 85 };
static const uint8_t ui8_eMTB_power_function_170[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7,
        7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 16, 16, 16,
        17, 17, 18, 18, 18, 19, 19, 19, 20, 20, 21, 21, 21, 22, 22, 23, 23, 23, 24, 24, 25, 25, 26, 26, 26, 27, 27, 28,
        28, 29, 29, 30, 30, 30, 31, 31, 32, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37, 38, 38, 39, 39, 40, 40, 41, 41,
        42, 42, 43, 43, 44, 45, 45, 46, 46, 47, 47, 48, 48, 49, 49, 50, 51, 51, 52, 52, 53, 53, 54, 55, 55, 56, 56, 57,
        58, 58, 59, 59, 60, 61, 61, 62, 63, 63, 64, 64, 65, 66, 66, 67, 68, 68, 69, 70, 70, 71, 71, 72, 73, 73, 74, 75,
        75, 76, 77, 77, 78, 79, 80, 80, 81, 82, 82, 83, 84, 84, 85, 86, 87, 87, 88, 89, 89, 90, 91, 92, 92, 93, 94, 94,
        95, 96, 97, 97, 98, 99, 100, 100, 101, 102, 103, 103, 104, 105, 106, 107, 107, 108, 109, 110, 110, 111 };
static const uint8_t ui8_eMTB_power_function_175[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
        1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 9,
        9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 16, 16, 17, 17, 17, 18, 18, 19, 19, 20,
        20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29, 30, 31, 31, 32, 32, 33, 33, 34,
        34, 35, 36, 36, 37, 37, 38, 39, 39, 40, 40, 41, 42, 42, 43, 44, 44, 45, 45, 46, 47, 47, 48, 49, 49, 50, 51, 51,
        52, 53, 53, 54, 55, 56, 56, 57, 58, 58, 59, 60, 61, 61, 62, 63, 64, 64, 65, 66, 67, 67, 68, 69, 70, 70, 71, 72,
        73, 74, 74, 75, 76, 77, 78, 78, 79, 80, 81, 82, 83, 83, 84, 85, 86, 87, 88, 88, 89, 90, 91, 92, 93, 94, 95, 95,
        96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117,
        118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139,
        140, 141, 142, 143, 144, 145, 146 };
static const uint8_t ui8_eMTB_power_function_180[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
        1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10,
        11, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 23, 23, 24,
        24, 25, 25, 26, 27, 27, 28, 28, 29, 30, 30, 31, 32, 32, 33, 34, 34, 35, 36, 36, 37, 38, 38, 39, 40, 41, 41, 42,
        43, 43, 44, 45, 46, 46, 47, 48, 49, 50, 50, 51, 52, 53, 54, 54, 55, 56, 57, 58, 59, 59, 60, 61, 62, 63, 64, 65,
        66, 67, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92,
        93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 105, 106, 107, 108, 109, 110, 111, 112, 114, 115, 116, 117, 118,
        119, 120, 122, 123, 124, 125, 126, 128, 129, 130, 131, 132, 134, 135, 136, 137, 139, 140, 141, 142, 144, 145,
        146, 147, 149, 150, 151, 153, 154, 155, 157, 158, 159, 161, 162, 163, 165, 166, 167, 169, 170, 171, 173, 174,
        176, 177, 178, 180, 181, 182, 184, 185, 187, 188, 190, 191, 192 };
static const uint8_t ui8_eMTB_power_function_185[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
        1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 9, 10, 10, 11, 11, 11, 12,
        12, 13, 13, 14, 14, 15, 15, 16, 17, 17, 18, 18, 19, 19, 20, 21, 21, 22, 23, 23, 24, 25, 25, 26, 27, 27, 28, 29,
        29, 30, 31, 32, 32, 33, 34, 35, 36, 36, 37, 38, 39, 40, 40, 41, 42, 43, 44, 45, 46, 46, 47, 48, 49, 50, 51, 52,
        53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 74, 75, 76, 77, 78, 79, 80, 81,
        83, 84, 85, 86, 87, 89, 90, 91, 92, 93, 95, 96, 97, 98, 100, 101, 102, 104, 105, 106, 107, 109, 110, 111, 113,
        114, 115, 117, 118, 120, 121, 122, 124, 125, 127, 128, 129, 131, 132, 134, 135, 137, 138, 140, 141, 143, 144,
        146, 147, 149, 150, 152, 153, 155, 156, 158, 160, 161, 163, 164, 166, 168, 169, 171, 172, 174, 176, 177, 179,
        181, 182, 184, 186, 187, 189, 191, 193, 194, 196, 198, 199, 201, 203, 205, 207, 208, 210, 212, 214, 216, 217,
        219, 221, 223, 225, 227, 228, 230, 232, 234, 236, 238, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_190[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
        1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14,
        14, 15, 16, 16, 17, 18, 18, 19, 20, 20, 21, 22, 22, 23, 24, 25, 25, 26, 27, 28, 29, 29, 30, 31, 32, 33, 34, 35,
        36, 37, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 51, 52, 53, 54, 55, 56, 57, 58, 60, 61, 62, 63, 64,
        66, 67, 68, 69, 70, 72, 73, 74, 76, 77, 78, 80, 81, 82, 84, 85, 86, 88, 89, 91, 92, 94, 95, 96, 98, 99, 101,
        102, 104, 105, 107, 108, 110, 112, 113, 115, 116, 118, 120, 121, 123, 124, 126, 128, 130, 131, 133, 135, 136,
        138, 140, 142, 143, 145, 147, 149, 150, 152, 154, 156, 158, 160, 162, 163, 165, 167, 169, 171, 173, 175, 177,
        179, 181, 183, 185, 187, 189, 191, 193, 195, 197, 199, 201, 203, 205, 207, 209, 211, 214, 216, 218, 220, 222,
        224, 227, 229, 231, 233, 235, 238, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240 };
static const uint8_t ui8_eMTB_power_function_195[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
        1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 5, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 13, 13, 14, 15, 15, 16,
        17, 17, 18, 19, 20, 21, 21, 22, 23, 24, 25, 26, 27, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 39, 40, 41, 42,
        43, 44, 45, 47, 48, 49, 50, 51, 53, 54, 55, 57, 58, 59, 61, 62, 63, 65, 66, 68, 69, 70, 72, 73, 75, 76, 78, 79,
        81, 83, 84, 86, 87, 89, 91, 92, 94, 96, 97, 99, 101, 103, 104, 106, 108, 110, 112, 113, 115, 117, 119, 121, 123,
        125, 127, 129, 131, 132, 134, 136, 139, 141, 143, 145, 147, 149, 151, 153, 155, 157, 160, 162, 164, 166, 168,
        171, 173, 175, 177, 180, 182, 184, 187, 189, 191, 194, 196, 199, 201, 203, 206, 208, 211, 213, 216, 218, 221,
        224, 226, 229, 231, 234, 237, 239, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_200[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
        1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 13, 14, 14, 15, 16, 17, 18, 18, 19,
        20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 34, 35, 36, 37, 38, 40, 41, 42, 44, 45, 46, 48, 49, 50, 52,
        53, 55, 56, 58, 59, 61, 62, 64, 66, 67, 69, 71, 72, 74, 76, 77, 79, 81, 83, 85, 86, 88, 90, 92, 94, 96, 98, 100,
        102, 104, 106, 108, 110, 112, 114, 117, 119, 121, 123, 125, 128, 130, 132, 135, 137, 139, 142, 144, 146, 149,
        151, 154, 156, 159, 161, 164, 166, 169, 172, 174, 177, 180, 182, 185, 188, 190, 193, 196, 199, 202, 204, 207,
        210, 213, 216, 219, 222, 225, 228, 231, 234, 237, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_205[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
        2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 9, 9, 10, 11, 11, 12, 13, 14, 15, 16, 16, 17, 18, 19, 20, 21, 22,
        23, 24, 26, 27, 28, 29, 30, 32, 33, 34, 36, 37, 38, 40, 41, 43, 44, 46, 47, 49, 50, 52, 54, 55, 57, 59, 61, 62,
        64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 95, 97, 99, 101, 104, 106, 108, 111, 113, 116, 118,
        121, 123, 126, 128, 131, 134, 136, 139, 142, 145, 147, 150, 153, 156, 159, 162, 165, 168, 171, 174, 177, 180,
        183, 186, 189, 192, 196, 199, 202, 205, 209, 212, 216, 219, 222, 226, 229, 233, 236, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_210[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2,
        2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 11, 12, 13, 14, 14, 15, 16, 17, 19, 20, 21, 22, 23, 24, 26, 27,
        28, 30, 31, 32, 34, 35, 37, 39, 40, 42, 43, 45, 47, 49, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 71, 73, 75, 77,
        80, 82, 84, 87, 89, 92, 94, 97, 99, 102, 104, 107, 110, 113, 115, 118, 121, 124, 127, 130, 133, 136, 139, 142,
        145, 149, 152, 155, 158, 162, 165, 169, 172, 176, 179, 183, 186, 190, 194, 197, 201, 205, 209, 213, 216, 220,
        224, 228, 232, 237, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_215[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2,
        2, 2, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 21, 22, 24, 25, 26, 28, 29, 31,
        33, 34, 36, 38, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 60, 62, 64, 67, 69, 71, 74, 76, 79, 82, 84, 87, 90, 93,
        96, 98, 101, 104, 107, 111, 114, 117, 120, 123, 127, 130, 134, 137, 141, 144, 148, 152, 155, 159, 163, 167, 171,
        175, 179, 183, 187, 191, 195, 200, 204, 208, 213, 217, 222, 226, 231, 235, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_220[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2,
        2, 3, 3, 4, 4, 5, 6, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 18, 19, 20, 22, 23, 25, 27, 28, 30, 32, 33, 35, 37,
        39, 41, 43, 46, 48, 50, 52, 55, 57, 60, 62, 65, 67, 70, 73, 76, 79, 82, 85, 88, 91, 94, 97, 101, 104, 108, 111,
        115, 118, 122, 126, 130, 133, 137, 141, 145, 150, 154, 158, 162, 167, 171, 176, 180, 185, 190, 194, 199, 204,
        209, 214, 219, 224, 230, 235, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_225[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2,
        3, 3, 4, 4, 5, 6, 7, 8, 8, 9, 10, 12, 13, 14, 15, 17, 18, 20, 21, 23, 24, 26, 28, 30, 32, 34, 36, 38, 40, 43,
        45, 47, 50, 52, 55, 58, 61, 64, 66, 70, 73, 76, 79, 82, 86, 89, 93, 96, 100, 104, 108, 112, 116, 120, 124, 128,
        133, 137, 142, 146, 151, 156, 161, 166, 171, 176, 181, 186, 191, 197, 202, 208, 214, 219, 225, 231, 237, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_230[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2,
        3, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15, 16, 18, 20, 21, 23, 25, 27, 29, 31, 33, 36, 38, 40, 43, 46, 48, 51,
        54, 57, 60, 63, 67, 70, 74, 77, 81, 85, 88, 92, 96, 101, 105, 109, 114, 118, 123, 128, 133, 138, 143, 148, 153,
        158, 164, 170, 175, 181, 187, 193, 199, 205, 212, 218, 225, 231, 238, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_235[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3,
        3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 16, 18, 19, 21, 23, 25, 27, 30, 32, 34, 37, 40, 43, 45, 48, 52, 55, 58, 62,
        65, 69, 73, 77, 81, 85, 89, 94, 98, 103, 108, 113, 118, 123, 128, 134, 139, 145, 151, 157, 163, 169, 176, 182,
        189, 196, 202, 210, 217, 224, 232, 239, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_240[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 3, 3,
        4, 5, 6, 7, 8, 9, 10, 12, 13, 15, 17, 19, 21, 23, 25, 27, 30, 32, 35, 38, 41, 44, 47, 51, 54, 58, 62, 66, 70,
        74, 79, 83, 88, 93, 98, 103, 108, 114, 120, 125, 131, 137, 144, 150, 157, 164, 171, 178, 185, 193, 200, 208,
        216, 224, 233, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240 };
static const uint8_t ui8_eMTB_power_function_245[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4,
        4, 5, 6, 8, 9, 10, 12, 14, 15, 17, 19, 22, 24, 27, 29, 32, 35, 38, 42, 45, 49, 53, 57, 61, 65, 70, 74, 79, 84,
        89, 95, 100, 106, 112, 119, 125, 132, 138, 145, 153, 160, 168, 176, 184, 192, 200, 209, 218, 227, 237, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240 };
static const uint8_t ui8_eMTB_power_function_250[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4,
        5, 6, 7, 9, 10, 12, 14, 16, 18, 20, 23, 25, 28, 31, 34, 38, 41, 45, 49, 54, 58, 63, 67, 72, 78, 83, 89, 95, 101,
        108, 114, 121, 128, 136, 144, 151, 160, 168, 177, 186, 195, 204, 214, 224, 235, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240 };
static const uint8_t ui8_eMTB_power_function_255[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 1, 1, 1, 2, 3, 4, 5,
        6, 7, 8, 10, 12, 14, 16, 18, 21, 24, 26, 30, 33, 37, 41, 45, 49, 54, 58, 64, 69, 75, 80, 87, 93, 100, 107, 114,
        122, 130, 138, 146, 155, 164, 174, 184, 194, 204, 215, 226, 238, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
        240, 240, 240 };

// cruise
static int16_t i16_cruise_pid_kp = 0;
static int16_t i16_cruise_pid_ki = 0;
static uint8_t ui8_cruise_PID_initialize = 1;
static uint16_t ui16_wheel_speed_target_received_x10 = 0;

// startup boost
static uint8_t ui8_startup_boost_enabled = 0;
static uint16_t ui16_startup_boost_factor_array[120];
static uint8_t ui8_startup_boost_cadence_step = 0;

// UART
#define UART_NUMBER_DATA_BYTES_TO_RECEIVE   88
#define UART_NUMBER_DATA_BYTES_TO_SEND      29

volatile uint8_t ui8_received_package_flag = 0;
volatile uint8_t ui8_rx_buffer[UART_NUMBER_DATA_BYTES_TO_RECEIVE];
//volatile uint8_t ui8_rx_counter = 0;
volatile uint8_t ui8_rx_cnt = 0;
volatile uint8_t ui8_rx_len = 0;
volatile uint8_t ui8_tx_buffer[UART_NUMBER_DATA_BYTES_TO_SEND];
// initialize the ui8_tx_counter like at the end of send operation to enable the first send.
//volatile uint8_t ui8_tx_counter = UART_NUMBER_DATA_BYTES_TO_SEND + 1;
static volatile uint8_t ui8_m_tx_buffer_index;
volatile uint8_t ui8_i;
volatile uint8_t ui8_byte_received;
volatile uint8_t ui8_state_machine = 0;
static uint16_t ui16_crc_rx;
static uint16_t ui16_crc_tx;
//volatile uint8_t ui8_message_ID = 0;
static uint8_t ui8_comm_error_counter = 0;

static void communications_controller(void);
static void communications_process_packages(uint8_t ui8_frame_type);
//static void uart_receive_package(void);
//static void uart_send_package(void);

// system functions
static void get_battery_voltage_filtered(void);
static void get_battery_current_filtered(void);
static void get_pedal_torque(void);
static void calc_wheel_speed(void);
static void calc_cadence(void);
static void calc_pedal_torque_offset(void);

static void ebike_control_lights(void);
static void ebike_control_motor(void);
static void check_system(void);
static void check_brakes(void);

static void apply_power_assist();
static void apply_torque_assist();
static void apply_cadence_assist();
static void apply_emtb_assist();
static void apply_hybrid_assist();
static void apply_cruise();
static void apply_walk_assist();
static void apply_throttle();
static void apply_temperature_limiting();
static void apply_speed_limit();

void ebike_app_controller(void)
{
    calc_wheel_speed();	// calculate the wheel speed
    calc_cadence();		// calculate the cadence and set limits from wheel speed

    get_battery_voltage_filtered(); // get filtered voltage from FOC calculations
    get_battery_current_filtered(); // get filtered current from FOC calculations
    get_pedal_torque();				// get pedal torque

    check_system();					// check if there are any errors for motor control
    check_brakes();					// check if brakes are enabled for motor control

	// send/receive data every 2 cycles (25ms * 2)
	// control external lights every 4 cycles (25ms * 4)
	static uint8_t ui8_counter;
	
	switch (ui8_counter++ & 0x03) {
		case 0: 
			communications_controller();
			break;
		case 1:
			ebike_control_lights();
			break;
		case 2:
			communications_controller(); 
			break;
	}
	
    ebike_control_motor(); // use received data and sensor input to control motor

    /*------------------------------------------------------------------------

     NOTE: regarding function call order

     Do not change order of functions if not absolutely sure it will
     not cause any undesirable consequences.

     ------------------------------------------------------------------------*/
}


static void ebike_control_motor(void)
{
    // reset control variables (safety)
    ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
    ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
    ui8_adc_battery_current_target = 0;
    ui8_duty_cycle_target = 0;

    // reset initialization of Cruise PID controller
    if (ui8_riding_mode != CRUISE_MODE) {
		ui8_cruise_PID_initialize = 1;
	}

    // select riding mode
    switch (ui8_riding_mode) {
		case POWER_ASSIST_MODE: apply_power_assist(); break;
		case TORQUE_ASSIST_MODE: apply_torque_assist(); break;
		case CADENCE_ASSIST_MODE: apply_cadence_assist(); break;
		case eMTB_ASSIST_MODE: apply_emtb_assist(); break;
		case HYBRID_ASSIST_MODE: apply_hybrid_assist(); break;
		case CRUISE_MODE: apply_cruise(); break;
		case WALK_ASSIST_MODE: apply_walk_assist(); break;
    }

    // select optional ADC function
    switch (m_configuration_variables.ui8_optional_ADC_function) {
		case THROTTLE_CONTROL:
			apply_throttle();
			break;
		case TEMPERATURE_CONTROL:
			apply_temperature_limiting();
			if (ui8_throttle_virtual) {apply_throttle();}
			break;
		default:
			if (ui8_throttle_virtual) {apply_throttle();}
			break;
    }

    // speed limit
    apply_speed_limit();


	// check if motor init delay has to be done (from v.1.1.0)
	switch (ui8_m_motor_init_state)
	{
    case MOTOR_INIT_STATE_INIT_START_DELAY:
      m_ui8_got_configurations_timer = 40;
      ui8_m_motor_init_state = MOTOR_INIT_STATE_INIT_WAIT_DELAY;
      // no break to execute next code

    case MOTOR_INIT_STATE_INIT_WAIT_DELAY:
      if (m_ui8_got_configurations_timer > 0) {
        m_ui8_got_configurations_timer--;
      }
      else
      {
        ui8_m_motor_init_state = MOTOR_INIT_OK;
        ui8_m_motor_init_status = MOTOR_INIT_STATUS_INIT_OK;
        ui8_m_system_state &= ~ERROR_NOT_INIT;
	  }
	break;
	}

    // check if to enable the motor
    if ((!ui8_motor_enabled) &&
		(ui16_motor_speed_erps == 0) && // only enable motor if stopped, else something bad can happen due to high currents/regen or similar
        (ui8_adc_battery_current_target) &&
		(!ui8_brakes_engaged)) {
			ui8_motor_enabled = 1;
			ui8_g_duty_cycle = 0;
			ui8_fw_angle = 0;
			motor_enable_pwm();
    }

    // check if to disable the motor
    if ((ui8_motor_enabled) &&
		(ui16_motor_speed_erps == 0) &&
		(!ui8_adc_battery_current_target) &&
		(!ui8_g_duty_cycle)) {
			ui8_motor_enabled = 0;
			motor_disable_pwm();
    }

    // reset control parameters if... (safety)
    if((ui8_brakes_engaged) ||
	  (ui8_m_system_state & 8) || // motor blocked
	  (!ui8_motor_enabled) ||
	  (!ui8_assist_level) ||
	  ((ui8_m_system_state)&&(!ui8_assist_whit_error_enabled))) {
        ui8_controller_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
        ui8_controller_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
        ui8_controller_adc_battery_current_target = 0;
        ui8_controller_duty_cycle_target = 0;
    } else {
        // limit max current if higher than configured hardware limit (safety)
        if (ui8_adc_battery_current_max > ADC_10_BIT_BATTERY_CURRENT_MAX) {
            ui8_adc_battery_current_max = ADC_10_BIT_BATTERY_CURRENT_MAX;
        }

        // limit target current if higher than max value (safety)
        if (ui8_adc_battery_current_target > ui8_adc_battery_current_max) {
            ui8_adc_battery_current_target = ui8_adc_battery_current_max;
        }

        // limit target duty cycle if higher than max value
        if (ui8_duty_cycle_target > PWM_DUTY_CYCLE_MAX) {
            ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
        }

        // limit target duty cycle ramp up inverse step if lower than min value (safety)
        if (ui8_duty_cycle_ramp_up_inverse_step < PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN) {
            ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
        }

        // limit target duty cycle ramp down inverse step if lower than min value (safety)
        if (ui8_duty_cycle_ramp_down_inverse_step < PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN) {
            ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
        }

        // set duty cycle ramp up in controller
        ui8_controller_duty_cycle_ramp_up_inverse_step = ui8_duty_cycle_ramp_up_inverse_step;

        // set duty cycle ramp down in controller
        ui8_controller_duty_cycle_ramp_down_inverse_step = ui8_duty_cycle_ramp_down_inverse_step;

        // set target battery current in controller
        ui8_controller_adc_battery_current_target = ui8_adc_battery_current_target;

        // set target duty cycle in controller
        ui8_controller_duty_cycle_target = ui8_duty_cycle_target;
    }
}


static void apply_power_assist()
{
    uint8_t ui8_power_assist_multiplier_x50 = ui8_riding_mode_parameter;
	
	// check for assist without pedal rotation when there is no pedal rotation
	if(ui8_assist_without_pedal_rotation_enabled) {
		if((ui8_pedal_cadence_RPM < 4) &&
		   (ui16_adc_pedal_torque_delta_no_boost > (110 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 4;
		}
	}
	
	// calculate torque on pedals + torque startup boost
    uint32_t ui32_pedal_torque_x100 = (uint32_t)(ui16_adc_pedal_torque_delta * m_configuration_variables.ui8_pedal_torque_per_10_bit_ADC_step_x100);
	
    // calculate power assist by multiplying human power with the power assist multiplier
    uint32_t ui32_power_assist_x100 = (((uint32_t)(ui8_pedal_cadence_RPM * ui8_power_assist_multiplier_x50))
            * ui32_pedal_torque_x100) / 480U; // see note below

    /*------------------------------------------------------------------------

     NOTE: regarding the human power calculation

     (1) Formula: pedal power = torque * rotations per second * 2 * pi
     (2) Formula: pedal power = torque * rotations per minute * 2 * pi / 60
     (3) Formula: pedal power = torque * rotations per minute * 0.1047
     (4) Formula: pedal power = torque * 100 * rotations per minute * 0.001047
     (5) Formula: pedal power = torque * 100 * rotations per minute / 955
     (6) Formula: pedal power * 100  =  torque * 100 * rotations per minute * (100 / 955)
     (7) Formula: assist power * 100  =  torque * 100 * rotations per minute * (100 / 955) * (ui8_power_assist_multiplier_x50 / 50)
     (8) Formula: assist power * 100  =  torque * 100 * rotations per minute * (2 / 955) * ui8_power_assist_multiplier_x50
     (9) Formula: assist power * 100  =  torque * 100 * rotations per minute * ui8_power_assist_multiplier_x50 / 480

     ------------------------------------------------------------------------*/

    // calculate target current
    uint16_t ui16_battery_current_target_x100 = (ui32_power_assist_x100 * 1000) / ui16_battery_voltage_filtered_x1000;

    // set battery current target in ADC steps
    uint16_t ui16_adc_battery_current_target = ui16_battery_current_target_x100 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;

    // set motor acceleration
    if (ui16_wheel_speed_x10 >= 200) {
        ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
        ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
    } else {
        ui8_duty_cycle_ramp_up_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                (uint8_t)40, // 40 -> 4 kph
                (uint8_t)200, // 200 -> 20 kph
                (uint8_t)ui8_duty_cycle_ramp_up_inverse_step_default,
                (uint8_t)PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);

        ui8_duty_cycle_ramp_down_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                (uint8_t)40, // 40 -> 4 kph
                (uint8_t)200, // 200 -> 20 kph
                (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,
                (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
    }

    // set battery current target
    if (ui16_adc_battery_current_target > ui8_adc_battery_current_max) {
        ui8_adc_battery_current_target = ui8_adc_battery_current_max;
    } else {
        ui8_adc_battery_current_target = ui16_adc_battery_current_target;
    }
	
	if((ui8_adc_battery_current_min)&&
	   (ui8_pedal_cadence_RPM)&&(!ui8_adc_battery_current_target)) {
			ui8_adc_battery_current_target = ui8_adc_battery_current_min;
	}
	
    // set duty cycle target
    if (ui8_adc_battery_current_target) {
        ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
    } else {
        ui8_duty_cycle_target = 0;
    }
}


static void apply_torque_assist()
{
	// check for assist without pedal rotation when there is no pedal rotation
	if(ui8_assist_without_pedal_rotation_enabled) {
		if((!ui8_pedal_cadence_RPM)&&
			(ui16_adc_pedal_torque_delta > (110 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 1;
		}
	}
	
	if(ui8_assist_whit_error_enabled)
		ui8_pedal_cadence_RPM = 1;
	
    // calculate torque assistance
    if (ui16_adc_pedal_torque_delta && ui8_pedal_cadence_RPM) {
        // get the torque assist factor
        uint8_t ui8_torque_assist_factor = ui8_riding_mode_parameter;

        // calculate torque assist target current
        uint16_t ui16_adc_battery_current_target_torque_assist = ((uint16_t) ui16_adc_pedal_torque_delta
                * ui8_torque_assist_factor) / TORQUE_ASSIST_FACTOR_DENOMINATOR;

        // set motor acceleration
        if (ui16_wheel_speed_x10 >= 200) {
            ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
            ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
        } else {
            ui8_duty_cycle_ramp_up_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                    (uint8_t)40, // 40 -> 4 kph
                    (uint8_t)200, // 200 -> 20 kph
                    (uint8_t)ui8_duty_cycle_ramp_up_inverse_step_default,
                    (uint8_t)PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);

            ui8_duty_cycle_ramp_down_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                    (uint8_t)40, // 40 -> 4 kph
                    (uint8_t)200, // 200 -> 20 kph
                    (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,
                    (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
        }
        // set battery current target
        if (ui16_adc_battery_current_target_torque_assist > ui8_adc_battery_current_max) {
            ui8_adc_battery_current_target = ui8_adc_battery_current_max;
        } else {
            ui8_adc_battery_current_target = ui16_adc_battery_current_target_torque_assist;
        }

		if((ui8_adc_battery_current_min)&&
	       (ui8_pedal_cadence_RPM)&&(!ui8_adc_battery_current_target)) {
				ui8_adc_battery_current_target = ui8_adc_battery_current_min;
		}
		
        // set duty cycle target
        if (ui8_adc_battery_current_target) {
            ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
        } else {
            ui8_duty_cycle_target = 0;
        }
    }
}


static void apply_cadence_assist()
{
#define CADENCE_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_OFFSET   50

    if (ui8_pedal_cadence_RPM) {
        // get the cadence assist duty cycle target
        //uint8_t ui8_cadence_assist_duty_cycle_target = ui8_riding_mode_parameter;
		uint8_t ui8_cadence_assist_duty_cycle_target = ui8_riding_mode_parameter + ui8_pedal_cadence_RPM;
		
        // limit cadence assist duty cycle target
        if (ui8_cadence_assist_duty_cycle_target > PWM_DUTY_CYCLE_MAX) {
            ui8_cadence_assist_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
        }

        // set motor acceleration
        if (ui16_wheel_speed_x10 >= 200) {
            ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
            ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
        } else {
            ui8_duty_cycle_ramp_up_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                    (uint8_t)40, // 40 -> 4 kph
                    (uint8_t)200, // 200 -> 20 kph
                    (uint8_t)ui8_duty_cycle_ramp_up_inverse_step_default + CADENCE_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_OFFSET,
                    (uint8_t)PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);

            ui8_duty_cycle_ramp_down_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                    (uint8_t)40, // 40 -> 4 kph
                    (uint8_t)200, // 200 -> 20 kph
                    (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,
                    (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
        }
        // set battery current target
        ui8_adc_battery_current_target = ui8_adc_battery_current_max;

        // set duty cycle target
        ui8_duty_cycle_target = ui8_cadence_assist_duty_cycle_target;
    }
}


static void apply_emtb_assist()
{
#define eMTB_ASSIST_ADC_TORQUE_OFFSET    10

	// check for assist without pedal rotation when there is no pedal rotation
	if(ui8_assist_without_pedal_rotation_enabled) {
		if((!ui8_pedal_cadence_RPM)&&
			(ui16_adc_pedal_torque_delta > (110 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 1;
		}
	}

	if(ui8_assist_whit_error_enabled)
		ui8_pedal_cadence_RPM = 1;
	
	if ((ui16_adc_pedal_torque_delta > 0) && 
		(ui16_adc_pedal_torque_delta < (eMTB_POWER_FUNCTION_ARRAY_SIZE - eMTB_ASSIST_ADC_TORQUE_OFFSET)) &&
		(ui8_pedal_cadence_RPM))
	{
		// initialize eMTB assist target current
		uint8_t ui8_adc_battery_current_target_eMTB_assist = 0;
		
		// get the eMTB assist sensitivity
		uint8_t ui8_eMTB_assist_sensitivity = ui8_riding_mode_parameter;
		
		switch (ui8_eMTB_assist_sensitivity)
		{
			case 0: ui8_adc_battery_current_target_eMTB_assist = 0;
			case 1: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_160[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 2: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_165[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 3: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_170[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 4: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_175[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 5: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_180[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 6: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_185[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 7: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_190[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 8: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_195[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 9: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_200[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 10: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_205[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 11: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_210[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 12: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_215[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 13: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_220[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 14: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_225[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 15: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_230[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 16: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_235[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 17: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_240[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 18: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_245[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 19: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_250[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
			case 20: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_255[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
		}

        // set motor acceleration
        if (ui16_wheel_speed_x10 >= 200) {
            ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
            ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
        } else {
            ui8_duty_cycle_ramp_up_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                    (uint8_t)40, // 40 -> 4 kph
                    (uint8_t)200, // 200 -> 20 kph
                    (uint8_t)ui8_duty_cycle_ramp_up_inverse_step_default,
                    (uint8_t)PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);

            ui8_duty_cycle_ramp_down_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                    (uint8_t)40, // 40 -> 4 kph
                    (uint8_t)200, // 200 -> 20 kph
                    (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,
                    (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
        }
        // set battery current target
        if (ui8_adc_battery_current_target_eMTB_assist > ui8_adc_battery_current_max) {
            ui8_adc_battery_current_target = ui8_adc_battery_current_max;
        } else {
            ui8_adc_battery_current_target = ui8_adc_battery_current_target_eMTB_assist;
        }

		if((ui8_adc_battery_current_min)&&
	       (ui8_pedal_cadence_RPM)&&(!ui8_adc_battery_current_target)) {
				ui8_adc_battery_current_target = ui8_adc_battery_current_min;
		}
		
        // set duty cycle target
        if (ui8_adc_battery_current_target) {
            ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
        } else {
            ui8_duty_cycle_target = 0;
        }
    }
}


static void apply_walk_assist()
{
#define WALK_ASSIST_DUTY_CYCLE_MAX                      80
#define WALK_ASSIST_ADC_BATTERY_CURRENT_MAX             80

    if (ui16_wheel_speed_x10 < WALK_ASSIST_THRESHOLD_SPEED_X10) {
        // get the walk assist duty cycle target
        uint8_t ui8_walk_assist_duty_cycle_target = ui8_walk_assist_parameter;

        // check so that walk assist level factor is not too large (too powerful), if it is -> limit the value
        if (ui8_walk_assist_duty_cycle_target > WALK_ASSIST_DUTY_CYCLE_MAX) {
            ui8_walk_assist_duty_cycle_target = WALK_ASSIST_DUTY_CYCLE_MAX;
        }

        // set motor acceleration
        ui8_duty_cycle_ramp_up_inverse_step = WALK_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP;
        ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;

        // set battery current target
        ui8_adc_battery_current_target = ui8_min(
        WALK_ASSIST_ADC_BATTERY_CURRENT_MAX, ui8_adc_battery_current_max);

        // set duty cycle target
        ui8_duty_cycle_target = ui8_walk_assist_duty_cycle_target;
    }
}


static void apply_cruise()
{
//#define CRUISE_PID_KP                             14    // 48 volt motor: 12, 36 volt motor: 14
//#define CRUISE_PID_KI                             0.7   // 48 volt motor: 1, 36 volt motor: 0.7
#define CRUISE_PID_INTEGRAL_LIMIT                 1000
#define CRUISE_PID_KD                             0

    if (ui16_wheel_speed_x10 > CRUISE_THRESHOLD_SPEED_X10) {
        static int16_t i16_error;
        static int16_t i16_last_error;
        static int16_t i16_integral;
        static int16_t i16_derivative;
        static int16_t i16_control_output;
        static uint16_t ui16_wheel_speed_target_x10;

        // initialize cruise PID controller
        if (ui8_cruise_PID_initialize) {
            ui8_cruise_PID_initialize = 0;

            // reset PID variables
            i16_error = 0;
            i16_last_error = 0;
            i16_integral = 320; // initialize integral to a value so the motor does not start from zero
            i16_derivative = 0;
            i16_control_output = 0;

            // check what target wheel speed to use (received or current)
            //uint16_t ui16_wheel_speed_target_received_x10 = ui8_riding_mode_parameter * (uint8_t)10;

            if (ui16_wheel_speed_target_received_x10 > 0) {
                // set received target wheel speed to target wheel speed
                ui16_wheel_speed_target_x10 = ui16_wheel_speed_target_received_x10;
            } else {
                // set current wheel speed to maintain
                ui16_wheel_speed_target_x10 = ui16_wheel_speed_x10;
            }
        }

        // calculate error
        i16_error = (ui16_wheel_speed_target_x10 - ui16_wheel_speed_x10);

        // calculate integral
        i16_integral = i16_integral + i16_error;

        // limit integral
        if (i16_integral > CRUISE_PID_INTEGRAL_LIMIT) {
            i16_integral = CRUISE_PID_INTEGRAL_LIMIT;
        } else if (i16_integral < 0) {
            i16_integral = 0;
        }

        // calculate derivative
        i16_derivative = i16_error - i16_last_error;

        // save error to last error
        i16_last_error = i16_error;

        // calculate control output ( output =  P I D )
        i16_control_output = (i16_cruise_pid_kp * i16_error) + (i16_cruise_pid_ki * i16_integral) + (CRUISE_PID_KD * i16_derivative);

        // limit control output to just positive values
        if (i16_control_output < 0) {
            i16_control_output = 0;
        }

        // limit control output to the maximum value
        if (i16_control_output > 1000) {
            i16_control_output = 1000;
        }

        // set motor acceleration
        ui8_duty_cycle_ramp_up_inverse_step = CRUISE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP;
        ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;

        // set battery current target
        ui8_adc_battery_current_target = ui8_adc_battery_current_max;

        // set duty cycle target  |  map the control output to an appropriate target PWM value
        ui8_duty_cycle_target = map_ui8((uint8_t) (i16_control_output >> 2),
                (uint8_t)0,                   // minimum control output from PID
                (uint8_t)250,                 // maximum control output from PID
                (uint8_t)0,                   // minimum duty cycle
                (uint8_t)PWM_DUTY_CYCLE_MAX); // maximum duty cycle
    }
}


static void apply_hybrid_assist()
{		
	uint16_t ui16_adc_battery_current_target_power_assist;
	uint16_t ui16_adc_battery_current_target_torque_assist;
	uint16_t ui16_adc_battery_current_target;
	
	// check for assist without pedal rotation when there is no pedal rotation
	if(ui8_assist_without_pedal_rotation_enabled) {
		if((!ui8_pedal_cadence_RPM)&&
			(ui16_adc_pedal_torque_delta > (110 - ui8_assist_without_pedal_rotation_threshold))) {
				ui8_pedal_cadence_RPM = 1;
		}
	}
	
	// calculate torque assistance
	if (ui16_adc_pedal_torque_delta && ui8_pedal_cadence_RPM)
	{	
		// get the torque assist factor
		uint16_t ui16_torque_assist_factor = ui8_hybrid_torque_parameter;
		
		// calculate torque assist target current
		ui16_adc_battery_current_target_torque_assist = ((uint16_t) ui16_adc_pedal_torque_delta * ui16_torque_assist_factor) / TORQUE_ASSIST_FACTOR_DENOMINATOR;
	}
	else
	{
		ui16_adc_battery_current_target_torque_assist = 0;
	}
		
	// calculate power assistance
	// get the power assist multiplier
	uint8_t ui8_power_assist_multiplier_x50 = ui8_riding_mode_parameter;

	// calculate power assist by multiplying human power with the power assist multiplier
    uint32_t ui32_power_assist_x100 = (((uint32_t)(ui8_pedal_cadence_RPM * ui8_power_assist_multiplier_x50))
            * ui16_pedal_torque_x100) / 480U; // see note below

	// calculate power assist target current x100
	uint16_t ui16_battery_current_target_x100 = (ui32_power_assist_x100 * 1000) / ui16_battery_voltage_filtered_x1000;
		
	// calculate power assist target current
	ui16_adc_battery_current_target_power_assist = ui16_battery_current_target_x100 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;
	
	// set battery current target in ADC steps
	if(ui16_adc_battery_current_target_power_assist > ui16_adc_battery_current_target_torque_assist)
		ui16_adc_battery_current_target = ui16_adc_battery_current_target_power_assist;
	else
		ui16_adc_battery_current_target = ui16_adc_battery_current_target_torque_assist;
	
	// set motor acceleration
	if (ui16_wheel_speed_x10 >= 200) {
        ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
        ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
    } else {
        ui8_duty_cycle_ramp_up_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                (uint8_t)40, // 40 -> 4 kph
                (uint8_t)200, // 200 -> 20 kph
                (uint8_t)ui8_duty_cycle_ramp_up_inverse_step_default,
                (uint8_t)PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);

																							  
        ui8_duty_cycle_ramp_down_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                (uint8_t)40, // 40 -> 4 kph
                (uint8_t)200, // 200 -> 20 kph
                (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,
                (uint8_t)PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
    }
	
	// set battery current target
	if (ui16_adc_battery_current_target > ui8_adc_battery_current_max) { ui8_adc_battery_current_target = ui8_adc_battery_current_max; }
	else { ui8_adc_battery_current_target = ui16_adc_battery_current_target; }
	
	if((ui8_adc_battery_current_min)&&
	   (ui8_pedal_cadence_RPM)&&(!ui8_adc_battery_current_target)) {
			ui8_adc_battery_current_target = ui8_adc_battery_current_min;
	}
	
	// set duty cycle target
	if (ui8_adc_battery_current_target) { ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX; }
	else { ui8_duty_cycle_target = 0; }
}


static void apply_throttle()
{
	uint8_t ui8_throttle = ui8_throttle_virtual;
	
    // map adc value from 0 to 255
    ui8_throttle_adc = map_ui8((uint8_t) UI8_ADC_THROTTLE,
            (uint8_t) ADC_THROTTLE_MIN_VALUE,
            (uint8_t) ADC_THROTTLE_MAX_VALUE,
            (uint8_t) 0,
            (uint8_t) 255);
			
	// apply throttle if it is enabled and the motor temperature limit function is not enabled instead
	if(m_configuration_variables.ui8_optional_ADC_function == THROTTLE_CONTROL)
	{
      // use the value that is max
      ui8_throttle = ui8_max(ui8_throttle, ui8_throttle_adc);
    }
	
    // map ADC throttle value from 0 to max battery current
    uint8_t ui8_adc_battery_current_target_throttle = map_ui8((uint8_t) ui8_throttle,
            (uint8_t) 0,
            (uint8_t) 255,
            (uint8_t) 0,
            (uint8_t) ui8_adc_battery_current_max);
			
    if (ui8_adc_battery_current_target_throttle > ui8_adc_battery_current_target) {
        // set motor acceleration
        if (ui16_wheel_speed_x10 >= 255) {
            ui8_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN;
            ui8_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
        } else {
            ui8_duty_cycle_ramp_up_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                    (uint8_t) 40,
                    (uint8_t) 255,
                    (uint8_t) THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT,
                    (uint8_t) THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);

            ui8_duty_cycle_ramp_down_inverse_step = map_ui8((uint8_t) ui16_wheel_speed_x10,
                    (uint8_t) 40,
                    (uint8_t) 255,
                    (uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,
                    (uint8_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
        }
        // set battery current target
        ui8_adc_battery_current_target = ui8_adc_battery_current_target_throttle;

        // set duty cycle target
        ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
    }
}


static void apply_temperature_limiting()
{
    // get ADC measurement
    uint16_t ui16_temp = UI16_ADC_10_BIT_THROTTLE;

    // filter ADC measurement to motor temperature variable
    ui16_adc_motor_temperature_filtered = filter(ui16_temp, ui16_adc_motor_temperature_filtered, 8);

    // convert ADC value
    ui16_motor_temperature_filtered_x10 = (uint16_t)(((uint32_t) ui16_adc_motor_temperature_filtered * 10000) / 2048);

    // min temperature value can not be equal or higher than max temperature value
    if (ui8_motor_temperature_min_value_to_limit >= ui8_motor_temperature_max_value_to_limit) {
        ui8_adc_battery_current_target = 0;
    } else {
        // adjust target current if motor over temperature limit
        ui8_adc_battery_current_target = map_ui16((uint16_t) ui16_motor_temperature_filtered_x10,
				(uint16_t) ((uint8_t)ui8_motor_temperature_min_value_to_limit * (uint8_t)10U),
				(uint16_t) ((uint8_t)ui8_motor_temperature_max_value_to_limit * (uint8_t)10U),
				ui8_adc_battery_current_target,
				0);
	}
}


static void apply_speed_limit()
{
    if (m_configuration_variables.ui8_wheel_speed_max > 0) {
        // set battery current target
        ui8_adc_battery_current_target = map_ui16((uint16_t) ui16_wheel_speed_x10,
                (uint16_t) (((uint8_t)(m_configuration_variables.ui8_wheel_speed_max) * (uint8_t)10U) - (uint8_t)20U),
                (uint16_t) (((uint8_t)(m_configuration_variables.ui8_wheel_speed_max) * (uint8_t)10U) + (uint8_t)20U),
                ui8_adc_battery_current_target,
                0);
    }
}


static void calc_wheel_speed(void)
{
    // calc wheel speed (km/h x10)
    if (ui16_wheel_speed_sensor_ticks) {
        uint16_t ui16_tmp = ui16_wheel_speed_sensor_ticks;
        // rps = PWM_CYCLES_SECOND / ui16_wheel_speed_sensor_ticks (rev/sec)
        // km/h*10 = rps * ui16_wheel_perimeter * ((3600 / (1000 * 1000)) * 10)
        // !!!warning if PWM_CYCLES_SECOND is not a multiple of 1000
        ui16_wheel_speed_x10 = (uint16_t)(((uint32_t) m_configuration_variables.ui16_wheel_perimeter * ((PWM_CYCLES_SECOND/1000)*36U)) / ui16_tmp);
    } else {
        ui16_wheel_speed_x10 = 0;
    }
}


static void calc_cadence(void)
{
    // get the cadence sensor ticks
    uint16_t ui16_cadence_sensor_ticks_temp = ui16_cadence_sensor_ticks;

    // adjust cadence sensor ticks counter min depending on wheel speed
    ui16_cadence_sensor_ticks_counter_min_speed_adjusted = map_ui16(ui16_wheel_speed_x10,
            40,
            400,
            CADENCE_SENSOR_CALC_COUNTER_MIN,
            CADENCE_SENSOR_TICKS_COUNTER_MIN_AT_SPEED);

    // calculate cadence in RPM and avoid zero division
    // !!!warning if PWM_CYCLES_SECOND > 21845
    if (ui16_cadence_sensor_ticks_temp)
        ui8_pedal_cadence_RPM = (uint8_t)((PWM_CYCLES_SECOND * 3U) / ui16_cadence_sensor_ticks_temp);
    else
        ui8_pedal_cadence_RPM = 0;
	
    /*-------------------------------------------------------------------------------------------------

     NOTE: regarding the cadence calculation

     Cadence is calculated by counting how many ticks there are between two LOW to HIGH transitions.

     Formula for calculating the cadence in RPM:

     (1) Cadence in RPM = (60 * PWM_CYCLES_SECOND) / CADENCE_SENSOR_NUMBER_MAGNETS) / ticks

     (2) Cadence in RPM = (PWM_CYCLES_SECOND * 3) / ticks

     -------------------------------------------------------------------------------------------------*/
}


static void calc_pedal_torque_offset(void)
{
	if((ui16_adc_pedal_torque_offset_fix > ui16_adc_pedal_torque_offset_min)&& 
	  (ui16_adc_pedal_torque_offset_fix < ui16_adc_pedal_torque_offset_max))
		  ui8_adc_pedal_torque_offset_error = 0;
	else
		  ui8_adc_pedal_torque_offset_error = 1;

	if((ui8_torque_sensor_calibration_enabled)&&(!ui8_adc_pedal_torque_offset_error))
		ui16_adc_pedal_torque_offset = ui16_adc_pedal_torque_offset_fix + ADC_TORQUE_SENSOR_CALIBRATION_OFFSET;
	else
		ui16_adc_pedal_torque_offset = ui16_adc_pedal_torque_offset_init + ADC_TORQUE_SENSOR_CALIBRATION_OFFSET;

	if((ui8_coaster_brake_enabled)&&(ui16_adc_pedal_torque_offset > ui8_coaster_brake_torque_threshold)) {
		if ((ui16_adc_pedal_torque_offset % 4) < 3)
			ui8_adc_coaster_brake_threshold = (ui16_adc_pedal_torque_offset >> 2) - (ui8_coaster_brake_torque_threshold >> 2);
		else
			ui8_adc_coaster_brake_threshold = (ui16_adc_pedal_torque_offset >> 2) - ((ui8_coaster_brake_torque_threshold >> 2) - 1);
	}
	else {
		ui8_adc_coaster_brake_threshold = 0;
	}
}


static void get_battery_voltage_filtered(void)
{
    ui16_battery_voltage_filtered_x1000 = ui16_adc_battery_voltage_filtered * BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000;
}


static void get_battery_current_filtered(void)
{
	ui8_battery_current_filtered_x10 = (uint8_t)(((uint16_t) ui8_adc_battery_current_filtered * BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100) / 10);
}

#define TOFFSET_CYCLES 60
static uint8_t toffset_cycle_counter = 0;

static void get_pedal_torque(void)
{
    if(toffset_cycle_counter < TOFFSET_CYCLES) {
        uint16_t ui16_tmp = UI16_ADC_10_BIT_TORQUE_SENSOR;
        ui16_adc_pedal_torque_offset_init = filter(ui16_tmp, ui16_adc_pedal_torque_offset_init, 2);
        toffset_cycle_counter++;
		
        if((toffset_cycle_counter == TOFFSET_CYCLES)&&(ui8_m_motor_init_state == MOTOR_INIT_OK)) {
		//if(toffset_cycle_counter == TOFFSET_CYCLES) {
			ui16_adc_pedal_torque_offset_min = ui16_adc_pedal_torque_offset_init - ADC_TORQUE_SENSOR_OFFSET_THRESHOLD;
			ui16_adc_pedal_torque_offset_max = ui16_adc_pedal_torque_offset_init + ADC_TORQUE_SENSOR_OFFSET_THRESHOLD;
			calc_pedal_torque_offset();
		}
		
        ui16_adc_pedal_torque = ui16_adc_pedal_torque_offset;
    }
	else {
        // get adc pedal torque
        ui16_adc_pedal_torque = UI16_ADC_10_BIT_TORQUE_SENSOR;
    }

    // calculate the delta value of adc pedal torque and the adc pedal torque range from calibration
    if (ui16_adc_pedal_torque > ui16_adc_pedal_torque_offset) {
        ui16_adc_pedal_torque_delta = ui16_adc_pedal_torque - ui16_adc_pedal_torque_offset;
    
		// adc pedal torque delta remapping
		if((ui8_torque_sensor_calibration_enabled) &&
		   (ui16_adc_pedal_torque_range < ADC_TORQUE_SENSOR_RANGE_MIN)) {
			ui16_adc_pedal_torque_delta = (ui16_adc_pedal_torque_delta * ADC_TORQUE_SENSOR_RANGE_MIN) / (ADC_TORQUE_SENSOR_RANGE_MIN - ((ADC_TORQUE_SENSOR_RANGE_MIN - ui16_adc_pedal_torque_range) / 3));
		}
	}
	else {
        ui16_adc_pedal_torque_delta = 0;
    }
	
	// for startup assist without pedaling in power assist mode
	ui16_adc_pedal_torque_delta_no_boost = ui16_adc_pedal_torque_delta;
		
    // calculate torque on pedals
    ui16_pedal_torque_x100 = ui16_adc_pedal_torque_delta * m_configuration_variables.ui8_pedal_torque_per_10_bit_ADC_step_x100;
	
	// pedal torque delta + startup boost in power assist mode
	if((ui8_startup_boost_enabled)&&(ui8_riding_mode == POWER_ASSIST_MODE)) {
		if(ui8_pedal_cadence_RPM > 120) {ui8_pedal_cadence_RPM = 120;}
		
		// calculate startup boost torque & new pedal torque delta
		uint32_t ui32_temp = ((uint32_t)(ui16_adc_pedal_torque_delta * ui16_startup_boost_factor_array[ui8_pedal_cadence_RPM])) / 100;
		ui16_adc_pedal_torque_delta += (uint16_t) ui32_temp;
	}
	
	/*------------------------------------------------------------------------

    NOTE: regarding the human power calculation
    
    (1) Formula: power = torque * rotations per second * 2 * pi
    (2) Formula: power = torque * rotations per minute * 2 * pi / 60
    (3) Formula: power = torque * rotations per minute * 0.1047
    (4) Formula: power = torque * 100 * rotations per minute * 0.001047
    (5) Formula: power = torque * 100 * rotations per minute / 955
    (6) Formula: power * 10  =  torque * 100 * rotations per minute / 96
    
	------------------------------------------------------------------------*/
}


struct_configuration_variables* get_configuration_variables(void)
{
    return &m_configuration_variables;
}


static void check_brakes()
{
    if (ui8_brake_state)
        ui8_brakes_engaged = 1;
    else
        ui8_brakes_engaged = 0;
}


static void check_system()
{

#define CHECK_SPEED_SENSOR_COUNTER_THRESHOLD          250 // 250 * 25ms = 6.25 seconds
#define MOTOR_ERPS_SPEED_THRESHOLD	                  180
	static uint8_t ui8_check_speed_sensor_counter;
	static uint8_t ui8_error_speed_sensor_counter;
	
	// check speed sensor
	if(ui16_motor_speed_erps > MOTOR_ERPS_SPEED_THRESHOLD) { ui8_check_speed_sensor_counter++; }
	if(ui16_wheel_speed_x10) { ui8_check_speed_sensor_counter = 0; }
	  
	if(ui8_check_speed_sensor_counter > CHECK_SPEED_SENSOR_COUNTER_THRESHOLD) {
		// set speed sensor error code
		ui8_m_system_state |= ERROR_SPEED_SENSOR;
	}
	else if (ui8_m_system_state & ERROR_SPEED_SENSOR) {
		// increment speed sensor error reset counter
        ui8_error_speed_sensor_counter++;

        // check if the counter has counted to the set threshold for reset
        if (ui8_error_speed_sensor_counter > CHECK_SPEED_SENSOR_COUNTER_THRESHOLD) {
            // reset speed sensor error code
            if (ui8_m_system_state & ERROR_SPEED_SENSOR) {										   
				ui8_m_system_state &= ~ERROR_SPEED_SENSOR;
				ui8_error_speed_sensor_counter = 0;
            }
		}
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define CHECK_CADENCE_SENSOR_COUNTER_THRESHOLD          1000 // 1000 * 25ms = 10 seconds
#define CADENCE_SENSOR_RESET_COUNTER_THRESHOLD         	100  // 100 * 25ms = 2,5 seconds
#define ADC_TORQUE_SENSOR_DELTA_THRESHOLD				80	// to check						
	static uint16_t ui16_check_cadence_sensor_counter;
	static uint8_t ui8_error_cadence_sensor_counter;
	
	// check cadence sensor
	if((ui16_adc_pedal_torque_delta_no_boost > ADC_TORQUE_SENSOR_DELTA_THRESHOLD)&&
	  ((ui8_pedal_cadence_RPM > 130)||(!ui8_pedal_cadence_RPM))) {
		ui16_check_cadence_sensor_counter++;
	}
	else {
		ui16_check_cadence_sensor_counter = 0;
	}
	
	if(ui16_check_cadence_sensor_counter > CHECK_CADENCE_SENSOR_COUNTER_THRESHOLD) {
		// set cadence sensor error code
		ui8_m_system_state |= ERROR_CADENCE_SENSOR;
	}
	else if (ui8_m_system_state & ERROR_CADENCE_SENSOR) {
		// increment cadence sensor error reset counter
        ui8_error_cadence_sensor_counter++;

        // check if the counter has counted to the set threshold for reset
        if (ui8_error_cadence_sensor_counter > CADENCE_SENSOR_RESET_COUNTER_THRESHOLD) {
            // reset cadence sensor error code
            if (ui8_m_system_state & ERROR_CADENCE_SENSOR) {
				ui8_m_system_state &= ~ERROR_CADENCE_SENSOR;
				ui8_error_cadence_sensor_counter = 0;
            }
		}
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // check torque sensor
    if (((ui16_adc_pedal_torque_offset > 250) ||
		(ui16_adc_pedal_torque_offset < 10) ||
		(ui16_adc_pedal_torque > 500) ||
		((ui8_adc_pedal_torque_offset_error)&&(ui8_torque_sensor_calibration_enabled)))
        && ((ui8_riding_mode == POWER_ASSIST_MODE) ||
			(ui8_riding_mode == TORQUE_ASSIST_MODE)||
			(ui8_riding_mode == HYBRID_ASSIST_MODE)||
			(ui8_riding_mode == eMTB_ASSIST_MODE))) {
        // set error code
        ui8_m_system_state |= ERROR_TORQUE_SENSOR;
    } else if (ui8_m_system_state & ERROR_TORQUE_SENSOR) {
        // reset error code
        ui8_m_system_state &= ~ERROR_TORQUE_SENSOR;
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MOTOR_BLOCKED_COUNTER_THRESHOLD               	8  // 8 * 25ms => 0.2 second
#define MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X10   	30 // 30 = 3.0 amps
#define MOTOR_BLOCKED_ERPS_THRESHOLD                  	20 // 20 ERPS
#define MOTOR_BLOCKED_RESET_COUNTER_THRESHOLD         	250  // 250 * 25ms = 6.25 seconds

    static uint8_t ui8_motor_blocked_counter;
    static uint8_t ui8_motor_blocked_reset_counter;

    // if the motor blocked error is enabled start resetting it
    if (ui8_m_system_state & ERROR_MOTOR_BLOCKED) {
        // increment motor blocked reset counter with 100 milliseconds
        ui8_motor_blocked_reset_counter++;

        // check if the counter has counted to the set threshold for reset
        if (ui8_motor_blocked_reset_counter > MOTOR_BLOCKED_RESET_COUNTER_THRESHOLD) {
            // reset motor blocked error code
            if (ui8_m_system_state & ERROR_MOTOR_BLOCKED) {
                ui8_m_system_state &= ~ERROR_MOTOR_BLOCKED;
            }

            // reset the counter that clears the motor blocked error
            ui8_motor_blocked_reset_counter = 0;
        }
    } else {
        // if battery current is over the current threshold and the motor ERPS is below threshold start setting motor blocked error code
        if ((ui8_battery_current_filtered_x10 > MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X10)
                && (ui16_motor_speed_erps < MOTOR_BLOCKED_ERPS_THRESHOLD)) {
            // increment motor blocked counter with 100 milliseconds
            ++ui8_motor_blocked_counter;

            // check if motor is blocked for more than some safe threshold
            if (ui8_motor_blocked_counter > MOTOR_BLOCKED_COUNTER_THRESHOLD) {
                // set error code
                ui8_m_system_state |= ERROR_MOTOR_BLOCKED;

                // reset motor blocked counter as the error code is set
                ui8_motor_blocked_counter = 0;
            }
        } else {
            // current is below the threshold and/or motor ERPS is above the threshold so reset the counter
            ui8_motor_blocked_counter = 0;
        }
    }
}


void ebike_control_lights(void)
{
#define DEFAULT_FLASH_ON_COUNTER_MAX      3
#define DEFAULT_FLASH_OFF_COUNTER_MAX     2
#define BRAKING_FLASH_ON_COUNTER_MAX      1
#define BRAKING_FLASH_OFF_COUNTER_MAX     1

    static uint8_t ui8_default_flash_state;
    static uint8_t ui8_default_flash_state_counter; // increments every function call -> 100 ms
    static uint8_t ui8_braking_flash_state;
    static uint8_t ui8_braking_flash_state_counter; // increments every function call -> 100 ms

    /****************************************************************************/

    // increment flash counters
    ++ui8_default_flash_state_counter;
    ++ui8_braking_flash_state_counter;

    /****************************************************************************/

    // set default flash state
    if ((ui8_default_flash_state) && (ui8_default_flash_state_counter > DEFAULT_FLASH_ON_COUNTER_MAX)) {
        // reset flash state counter
        ui8_default_flash_state_counter = 0;

        // toggle flash state
        ui8_default_flash_state = 0;
    } else if ((!ui8_default_flash_state) && (ui8_default_flash_state_counter > DEFAULT_FLASH_OFF_COUNTER_MAX)) {
        // reset flash state counter
        ui8_default_flash_state_counter = 0;

        // toggle flash state
        ui8_default_flash_state = 1;
    }

    /****************************************************************************/

    // set braking flash state
    if ((ui8_braking_flash_state) && (ui8_braking_flash_state_counter > BRAKING_FLASH_ON_COUNTER_MAX)) {
        // reset flash state counter
        ui8_braking_flash_state_counter = 0;

        // toggle flash state
        ui8_braking_flash_state = 0;
    } else if ((!ui8_braking_flash_state) && (ui8_braking_flash_state_counter > BRAKING_FLASH_OFF_COUNTER_MAX)) {
        // reset flash state counter
        ui8_braking_flash_state_counter = 0;

        // toggle flash state
        ui8_braking_flash_state = 1;
    }

    /****************************************************************************/

    // select lights configuration
    switch (ui8_lights_configuration) {
    case 0:

        // set lights
        lights_set_state(ui8_lights_state);

        break;

    case 1:

        // check lights state
        if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        } else {
            // set lights
            lights_set_state(ui8_lights_state);
        }

        break;

    case 2:

        // check light and brake state
        if (ui8_lights_state && ui8_brakes_engaged) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        } else {
            // set lights
            lights_set_state(ui8_lights_state);
        }

        break;

    case 3:

        // check light and brake state
        if (ui8_lights_state && ui8_brakes_engaged) {
            // set lights
            lights_set_state(ui8_brakes_engaged);
        } else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        } else {
            // set lights
            lights_set_state(ui8_lights_state);
        }

        break;

    case 4:

        // check light and brake state
        if (ui8_lights_state && ui8_brakes_engaged) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        } else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        } else {
            // set lights
            lights_set_state(ui8_lights_state);
        }

        break;

    case 5:

        // check brake state
        if (ui8_brakes_engaged) {
            // set lights
            lights_set_state(ui8_brakes_engaged);
        } else {
            // set lights
            lights_set_state(ui8_lights_state);
        }

        break;

    case 6:

        // check brake state
        if (ui8_brakes_engaged) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        } else {
            // set lights
            lights_set_state(ui8_lights_state);
        }

        break;

    case 7:

        // check brake state
        if (ui8_brakes_engaged) {
            // set lights
            lights_set_state(ui8_brakes_engaged);
        } else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        } else {
            // set lights
            lights_set_state(ui8_lights_state);
        }

        break;

    case 8:

        // check brake state
        if (ui8_brakes_engaged) {
            // set lights
            lights_set_state(ui8_braking_flash_state);
        } else if (ui8_lights_state) {
            // set lights
            lights_set_state(ui8_default_flash_state);
        } else {
            // set lights
            lights_set_state(ui8_lights_state);
        }

        break;

    default:

        // set lights
        lights_set_state(ui8_lights_state);

        break;
    }

    /*------------------------------------------------------------------------------------------------------------------

     NOTE: regarding the various light modes

     (0) lights ON when enabled
     (1) lights FLASHING when enabled

     (2) lights ON when enabled and BRAKE-FLASHING when braking
     (3) lights FLASHING when enabled and ON when braking
     (4) lights FLASHING when enabled and BRAKE-FLASHING when braking

     (5) lights ON when enabled, but ON when braking regardless if lights are enabled
     (6) lights ON when enabled, but BRAKE-FLASHING when braking regardless if lights are enabled

     (7) lights FLASHING when enabled, but ON when braking regardless if lights are enabled
     (8) lights FLASHING when enabled, but BRAKE-FLASHING when braking regardless if lights are enabled

     ------------------------------------------------------------------------------------------------------------------*/
}

// from v.1.1.0 ********************************************************************************************************
 
// This is the interrupt that happens when UART2 receives data. We need it to be the fastest possible and so
// we do: receive every byte and assembly as a package, finally, signal that we have a package to process (on main slow loop)
// and disable the interrupt. The interrupt should be enable again on main loop, after the package being processed
void UART2_RX_IRQHandler(void) __interrupt(UART2_RX_IRQHANDLER)
{
	if (UART2_GetFlagStatus(UART2_FLAG_RXNE) == SET)
	{
		UART2->SR &= (uint8_t)~(UART2_FLAG_RXNE); // this may be redundant

		if (ui8_received_package_flag == 0) // only when package were previously processed
		{
			ui8_byte_received = UART2_ReceiveData8();
		 
			switch (ui8_state_machine)
			{
				case 0:
					if (ui8_byte_received == 0x59) // see if we get start package byte
					{
						ui8_rx_buffer[0] = ui8_byte_received;
						ui8_state_machine = 1;
					}
					else {
						ui8_state_machine = 0;
					}
					break;
 
				case 1:
					ui8_rx_buffer[1] = ui8_byte_received;
					ui8_rx_len = ui8_byte_received;
					ui8_state_machine = 2;
					break;

				case 2:
					ui8_rx_buffer[ui8_rx_cnt + 2] = ui8_byte_received;
					++ui8_rx_cnt;

					if (ui8_rx_cnt >= ui8_rx_len)
					{
						ui8_rx_cnt = 0;
						ui8_state_machine = 0;
						ui8_received_package_flag = 1; // signal that we have a full package to be processed
					}
					break;

				default:
					break;
			}
		}
	}
	else // if there was any error, restart our state machine
	{
		ui8_rx_cnt = 0;
		ui8_state_machine = 0;
	}
}


void UART2_TX_IRQHandler(void) __interrupt(UART2_TX_IRQHANDLER)
{
	if (UART2_GetFlagStatus(UART2_FLAG_TXE) == SET)
	{
		if (ui8_m_tx_buffer_index < UART_NUMBER_DATA_BYTES_TO_SEND)  // bytes to send
		{
			// clearing the TXE bit is always performed by a write to the data register
			UART2_SendData8(ui8_tx_buffer[ui8_m_tx_buffer_index]);
			++ui8_m_tx_buffer_index;
			if (ui8_m_tx_buffer_index == UART_NUMBER_DATA_BYTES_TO_SEND)
			{
				// buffer empty
				// disable TIEN (TXE)
				UART2_ITConfig(UART2_IT_TXE, DISABLE);
			}
		}
	}
	else
	{
		// TXE interrupt should never occur if there is nothing to send in the buffer
		// send a zero to clear TXE and disable the interrupt
		UART2_SendData8(0);
		UART2_ITConfig(UART2_IT_TXE, DISABLE);
		}
}

static void communications_controller(void)
{
	uint8_t ui8_frame_type_to_send = 0;
	uint8_t ui8_len;

	if (ui8_received_package_flag)
	{
		// just to make easy next calculations
		ui16_crc_rx = 0xffff;
		ui8_len = ui8_rx_buffer[1];
		for (ui8_i = 0; ui8_i < ui8_len; ui8_i++)
		{
			crc16(ui8_rx_buffer[ui8_i], &ui16_crc_rx);
		}

		// if CRC is correct read the package
		if (((((uint16_t) ui8_rx_buffer[ui8_len + 1]) << 8) +
           ((uint16_t) ui8_rx_buffer[ui8_len])) == ui16_crc_rx)
		{
			ui8_comm_error_counter = 0;

			if (ui8_m_motor_init_state == MOTOR_INIT_STATE_RESET)
				ui8_m_motor_init_state = MOTOR_INIT_STATE_NO_INIT;

			ui8_frame_type_to_send = ui8_rx_buffer[2];
			communications_process_packages(ui8_frame_type_to_send);
		}
		else
		{
			ui8_received_package_flag = 0;
			ui8_comm_error_counter++;
		}
	}
	else
	{
		ui8_comm_error_counter++;
	}

	// check for communications fail or display master fail
	// can't fail more then 1000ms ??? 20 * 50ms
	if (ui8_comm_error_counter > 20)
	{
		motor_disable_pwm();
		ui8_motor_enabled = 0;
		ui8_m_system_state |= ERROR_FATAL;
	}

	if (ui8_m_motor_init_state == MOTOR_INIT_STATE_RESET)
		communications_process_packages(COMM_FRAME_TYPE_ALIVE); 
}


static void communications_process_packages(uint8_t ui8_frame_type)
{
	uint8_t ui8_temp;
	uint16_t ui16_temp;
	//uint32_t ui32_temp;
	uint8_t ui8_len = 3; // 3 bytes: 1 type of frame + 2 CRC bytes
	//uint8_t j;
	//uint8_t i;
	uint16_t ui16_adc_battery_current = ui8_adc_battery_current_filtered;

	// start up byte
	ui8_tx_buffer[0] = 0x43;
	ui8_tx_buffer[2] = ui8_frame_type;

	// prepare payload
	switch (ui8_frame_type)
	{
	  // periodic data
	  case COMM_FRAME_TYPE_PERIODIC:
		// display will send periodic command after motor init ok, now reset so the state machine will be ready for next time
		ui8_m_motor_init_status = MOTOR_INIT_STATUS_RESET;
	  
		//m_config_vars.ui16_assist_level_factor_x1000 = (((uint16_t) ui8_rx_buffer[4]) << 8) + ((uint16_t) ui8_rx_buffer[3]);
		// riding mode parameter
		ui8_riding_mode_parameter = ui8_rx_buffer[3];
		
		// hybrid torque parameter
		ui8_hybrid_torque_parameter = ui8_rx_buffer[4];
		
		// lights state
		ui8_lights_state = (ui8_rx_buffer[5] & (1 << 0)) ? 1: 0;

		// walk assist / cruise function
		uint8_t ui8_walk_assist = (ui8_rx_buffer[5] & (1 << 1)) ? 1: 0;
		
		// assist level
		ui8_assist_level = (ui8_rx_buffer[5] & (1 << 2)) ? 1: 0;
		
		// battery max power target
		m_configuration_variables.ui8_target_battery_max_power_div25 = ui8_rx_buffer[6];

		// calculate max battery current in ADC steps from the received battery current limit
		uint8_t ui8_adc_battery_current_max_temp_1 = (uint16_t)(ui8_battery_current_max * (uint8_t)100) / (uint16_t)BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;

		// calculate max battery current in ADC steps from the received power limit
		uint32_t ui32_battery_current_max_x100 = ((uint32_t) m_configuration_variables.ui8_target_battery_max_power_div25 * 2500000) / ui16_battery_voltage_filtered_x1000;
		uint8_t ui8_adc_battery_current_max_temp_2 = ui32_battery_current_max_x100 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;

		// set max battery current
		ui8_adc_battery_current_max = ui8_min(ui8_adc_battery_current_max_temp_1, ui8_adc_battery_current_max_temp_2);

		// walk assist parameter
		ui8_walk_assist_parameter = ui8_rx_buffer[7];

		// riding mode
		if((ui8_walk_assist)&&(ui16_wheel_speed_x10 < WALK_ASSIST_THRESHOLD_SPEED_X10))
			// enable walk assist depending on speed
			ui8_riding_mode = WALK_ASSIST_MODE;
		else if((ui8_walk_assist)&&(ui16_wheel_speed_x10 > CRUISE_THRESHOLD_SPEED_X10))
			// enable cruise function depending on speed
			ui8_riding_mode = CRUISE_MODE; 
		else
			ui8_riding_mode = ui8_rx_buffer[8];
		
		// wheel max speed
		m_configuration_variables.ui8_wheel_speed_max = ui8_rx_buffer[9];

		// motor temperature limit function or throttle
		if(ui8_rx_buffer[10] == 1)
			m_configuration_variables.ui8_optional_ADC_function = TEMPERATURE_CONTROL;
		else if(ui8_rx_buffer[10] == 2)
			m_configuration_variables.ui8_optional_ADC_function = THROTTLE_CONTROL;
		else
			m_configuration_variables.ui8_optional_ADC_function = NOT_IN_USE;

		// virtual throttle
		ui8_throttle_virtual = ui8_rx_buffer[11];

		// now send data back
		// ADC 10 bits battery voltage
		ui16_temp = ui16_adc_battery_voltage_filtered;
		ui8_tx_buffer[3] = (ui16_temp & 0xff);
		ui8_tx_buffer[4] = ((uint8_t) (ui16_temp >> 4)) & 0x30;

		// battery current
		// ADC 10 bits each step current is 0.156
		// 0.156 * 5 = 0.78
		// send battery_current_x5
		//ui8_tx_buffer[5] = (uint8_t) ((ui16_g_adc_battery_current_filtered * 78) / 100);
		ui8_tx_buffer[5] = ui8_battery_current_filtered_x10 / 2;

		// wheel speed
		ui16_temp = ui16_wheel_speed_x10;
		ui8_tx_buffer[6] = (uint8_t) (ui16_temp & 0xff);
		ui8_tx_buffer[7] = ((uint8_t) (ui16_temp >> 8)) & 0x07;

		// last 2 bits of adc_motor_current
		ui8_tx_buffer[7] |= ((uint8_t) ((ui16_adc_battery_current & 0x300) >> 5));

		// brake state
		ui8_tx_buffer[8] = ui8_brakes_engaged;
		// add the hall sensors state, that should be 3 bits only, value from 0 to 7
		ui8_tx_buffer[8] |= (ui8_hall_sensors_state << 1);
		// add pas pedal position
		//ui8_tx_buffer[8] |= (ui8_pas_pedal_position_right << 4);

		// throttle value from ADC
		ui8_tx_buffer[9] = UI8_ADC_THROTTLE;
		// adjusted throttle value or temperature limit depending on user setup
		if(m_configuration_variables.ui8_optional_ADC_function == TEMPERATURE_CONTROL)
		{
			// temperature value
			ui8_tx_buffer[10] = ui16_adc_motor_temperature_filtered;
		}
		else
		{
			// throttle value with offset removed and mapped to 255
			ui8_tx_buffer[10] = ui8_throttle_adc;
		}
		
		// ADC torque_sensor
		ui8_tx_buffer[11] = (uint8_t) (UI16_ADC_10_BIT_TORQUE_SENSOR & 0xff);
		// ADC torque_sensor (higher bits), this bits are shared with wheel speed bits
		ui8_tx_buffer[7] |= (uint8_t) ((UI16_ADC_10_BIT_TORQUE_SENSOR & 0x300) >> 2); //xx00 0000

		// pedal torque delta
		ui8_tx_buffer[12] = (uint8_t) (ui16_adc_pedal_torque_delta_no_boost & 0xff);
		ui8_tx_buffer[13] = (uint8_t) (ui16_adc_pedal_torque_delta_no_boost >> 8);
		
		// PAS cadence
		ui8_tx_buffer[14] = ui8_pedal_cadence_RPM;
	  
		// PWM duty_cycle
		// convert duty-cycle to 0 - 100 %
/*  	// add field_weakening_angle
		ui16_temp = (uint16_t) ui8_g_duty_cycle;
		ui16_temp = (ui16_temp * 100) / PWM_DUTY_CYCLE_MAX;
		if (ui8_g_field_weakening_enable_state)
		{
			ui16_temp += (((uint16_t) ui8_g_field_weakening_angle) * 14) / 10;
		}
		ui8_tx_buffer[15] = (uint8_t) ui16_temp;
*/
		// PWM duty_cycle
		ui8_tx_buffer[15] = ui8_g_duty_cycle;
	  
		// motor speed in ERPS
		ui8_tx_buffer[16] = (uint8_t) (ui16_motor_speed_erps & 0xff);
		ui8_tx_buffer[17] = (uint8_t) (ui16_motor_speed_erps >> 8);

		// FOC angle
		ui8_tx_buffer[18] = ui8_g_foc_angle;

		// system state
		ui8_tx_buffer[19] = ui8_m_system_state;

		// motor current
		// ADC 10 bits each step current is 0.156
		// 0.156 * 5 = 0.78
		// send battery_current_x5
		//ui8_tx_buffer[20] = (uint8_t) ((ui16_g_adc_motor_current_filtered * 78) / 100);
		ui8_tx_buffer[20] = ui8_battery_current_filtered_x10 / 2;

		// wheel_speed_sensor_tick_counter
		ui8_tx_buffer[21] = (uint8_t) (ui32_wheel_speed_sensor_ticks_total & 0xff);
		ui8_tx_buffer[22] = (uint8_t) ((ui32_wheel_speed_sensor_ticks_total >> 8) & 0xff);
		ui8_tx_buffer[23] = (uint8_t) ((ui32_wheel_speed_sensor_ticks_total >> 16) & 0xff);

		// pedal torque delta boost
		ui8_tx_buffer[24] = (uint8_t) (ui16_adc_pedal_torque_delta & 0xff);
		ui8_tx_buffer[25] = (uint8_t) (ui16_adc_pedal_torque_delta >> 8);

		// first 8 bits of adc_motor_current
		//ui8_tx_buffer[26] = (uint8_t) (ui16_adc_battery_current & 0xff);
		ui8_tx_buffer[26] = ui8_adc_battery_current_filtered;
	  
		ui8_len += 24;
		break;

	  // set configurations
	  case COMM_FRAME_TYPE_CONFIGURATIONS:
		// disable the motor to avoid a quick of the motor while configurations are changed
		// disable the motor, lets hope this is safe to do here, in this way
		// the motor shold be enabled again on the ebike_control_motor()
		motor_disable_pwm();
		ui8_motor_enabled = 0;
		ui8_m_system_state |= ERROR_NOT_INIT;
		ui8_m_motor_init_state = MOTOR_INIT_STATE_INIT_START_DELAY;
		ui8_m_motor_init_status = MOTOR_INIT_STATUS_GOT_CONFIG;

		// battery low voltage cut-off x10
		m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 = (((uint16_t) ui8_rx_buffer[4]) << 8) + ((uint16_t) ui8_rx_buffer[3]);

		// set low voltage cutoff (8 bit)
		ui8_adc_battery_voltage_cut_off = (m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 * 25U) / BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000;

		// wheel perimeter
		m_configuration_variables.ui16_wheel_perimeter = (((uint16_t) ui8_rx_buffer[6]) << 8) + ((uint16_t) ui8_rx_buffer[5]);

		// battery max current
		//ebike_app_set_battery_max_current(ui8_rx_buffer[7]);
		ui8_battery_current_max = ui8_rx_buffer[7];

		//m_config_vars.ui8_startup_motor_power_boost_feature_enabled = ui8_rx_buffer[8] & 1;
		ui8_startup_boost_enabled = ui8_rx_buffer[8] & 1;

		//m_config_vars.ui8_startup_motor_power_boost_always = (ui8_rx_buffer[8] & 2) >> 1;
		//m_config_vars.ui8_startup_motor_power_boost_limit_to_max_power = (ui8_rx_buffer[8] & 4) >> 2;

		ui8_torque_sensor_calibration_enabled = (ui8_rx_buffer[8] & 8) >> 3;
		ui8_assist_whit_error_enabled = (ui8_rx_buffer[8] & 16) >> 4;
		ui8_assist_without_pedal_rotation_enabled = (ui8_rx_buffer[8] & 32) >> 5;
      
		// motor type
		ui8_temp = (ui8_rx_buffer[8] >> 6) & 1;

		//m_configuration_variables.ui8_motor_inductance_x1048576
		// motor inductance & cruise pid parameter
		if(ui8_temp == 0)
		{
			// 48 V motor
			m_configuration_variables.ui8_motor_inductance_x1048576 = 142;
			i16_cruise_pid_kp = 12;
			i16_cruise_pid_ki = 1;
		}
		else
		{
			// 36 V motor
			m_configuration_variables.ui8_motor_inductance_x1048576 = 80;
			i16_cruise_pid_kp = 14;
			i16_cruise_pid_ki = 0.7;
		}
	  
		// motor max current
		//ebike_app_set_motor_max_current(ui8_rx_buffer[9]);
		//ui8_motor_current_max = ui8_rx_buffer[9];
	  
		// startup motor power boost time
		//m_config_vars.ui8_startup_motor_power_boost_time = ui8_rx_buffer[10];
		// startup motor power boost fade time
		//m_config_vars.ui8_startup_motor_power_boost_fade_time = ui8_rx_buffer[11];
		// startup boost
		ui16_startup_boost_factor_array[0] = (uint16_t) ui8_rx_buffer[10] << 1;
		ui8_startup_boost_cadence_step = ui8_rx_buffer[11];

		for (ui8_i = 1; ui8_i < 120; ui8_i++)
		{
			ui16_temp = (ui16_startup_boost_factor_array[ui8_i - 1] * (uint16_t)ui8_startup_boost_cadence_step) >> 8;
			ui16_startup_boost_factor_array[ui8_i] = ui16_startup_boost_factor_array[ui8_i - 1] - ui16_temp;	
		}

		// motor over temperature min value limit
		ui8_motor_temperature_min_value_to_limit = ui8_rx_buffer[12];
		// motor over temperature max value limit
		ui8_motor_temperature_max_value_to_limit = ui8_rx_buffer[13];
	  
		// ramp up, amps per second
		//m_config_vars.ui8_ramp_up_amps_per_second_x10 = ui8_rx_buffer[14];
		// motor acceleration adjustment
		uint8_t ui8_motor_acceleration_adjustment = ui8_rx_buffer[14];
	  
		// set duty cycle ramp up inverse step
		ui8_duty_cycle_ramp_up_inverse_step_default = map_ui8((uint8_t)ui8_motor_acceleration_adjustment,
				(uint8_t) 0,
                (uint8_t) 100,
                (uint8_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT,
                (uint8_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);

		// received target speed for cruise
		ui16_wheel_speed_target_received_x10 = (uint16_t) (ui8_rx_buffer[15] * 10);


		
		// Torque ADC offset set (Right ADC1, weight=0)
		ui16_adc_pedal_torque_offset_fix = (((uint16_t) ui8_rx_buffer[51]) << 8) + ((uint16_t) ui8_rx_buffer[50]);
		if (toffset_cycle_counter == TOFFSET_CYCLES) {
			calc_pedal_torque_offset();
		}
		
		// pedal_torque_range (Right ADC8 - Right ADC1, weight=max)
		ui16_adc_pedal_torque_range = (((uint16_t) ui8_rx_buffer[79]) << 8) + ((uint16_t) ui8_rx_buffer[78]);
	  
		// battery current min ADC
		// warning ui8_rx_buffer[79] overlap
		//m_config_vars.ui8_battery_current_min_adc = ui8_rx_buffer[79];

		ui8_temp = ui8_rx_buffer[80];
		//ui8_g_pedal_cadence_fast_stop = ui8_temp & 1;
		//ui8_g_field_weakening_enable = (ui8_temp & 2) >> 1;
		ui8_coaster_brake_enabled = (ui8_temp & 4) >> 2;
		//m_config_vars.ui8_motor_current_control_mode = (ui8_temp & 8) >> 3;
		// hybrid assist mode enabled (power + torque)
		ui8_hybrid_assist_enabled = (ui8_temp & 8) >> 3;
		// (ui8_temp & 16) >> 4; available 
		ui8_adc_battery_current_min = (ui8_temp & 224 ) >> 5;
		
		// coast brake threshold
		ui8_coaster_brake_torque_threshold = ui8_rx_buffer[81];
		if(toffset_cycle_counter == TOFFSET_CYCLES) {
			calc_pedal_torque_offset();
		}
			
		//ui8_m_adc_lights_current_offset = (uint16_t) ui8_rx_buffer[82];
		// lights configuration
		ui8_lights_configuration = ui8_rx_buffer[82];
	  
		// torque sensor filter value
		//m_config_vars.ui8_torque_sensor_filter = ui8_rx_buffer[83];
		// torque sensor adc step (default 67)
		m_configuration_variables.ui8_pedal_torque_per_10_bit_ADC_step_x100 = ui8_rx_buffer[83];
	  
		// torque sensor ADC threshold
		if(ui8_assist_without_pedal_rotation_enabled)
		{
			ui8_assist_without_pedal_rotation_threshold = ui8_rx_buffer[84];
			if(ui8_assist_without_pedal_rotation_threshold > 100)
				{ ui8_assist_without_pedal_rotation_threshold = 100; }
		}
		else
		{
			ui8_assist_without_pedal_rotation_threshold = 0;
		}
	  
		break;

      // firmware version
      case COMM_FRAME_TYPE_FIRMWARE_VERSION:
		ui8_tx_buffer[3] = ui8_m_system_state;
		ui8_tx_buffer[4] = 0;
		ui8_tx_buffer[5] = 20;
		ui8_tx_buffer[6] = 1;
		ui8_len += 4;
		break;

      case COMM_FRAME_TYPE_ALIVE:
		// nothing to add
		break;

      case COMM_FRAME_TYPE_STATUS:
		ui8_tx_buffer[3] = ui8_m_motor_init_status;
		ui8_len += 1;
		break;

      default:
		break;
	}

	ui8_tx_buffer[1] = ui8_len;

	// prepare crc of the package
	ui16_crc_tx = 0xffff;
	for (ui8_i = 0; ui8_i < ui8_len; ui8_i++)
	{
		crc16(ui8_tx_buffer[ui8_i], &ui16_crc_tx);
	}
	ui8_tx_buffer[ui8_len] = (uint8_t) (ui16_crc_tx & 0xff);
	ui8_tx_buffer[ui8_len + 1] = (uint8_t) (ui16_crc_tx >> 8) & 0xff;

	ui8_m_tx_buffer_index = 0;
	// start transmition
	UART2_ITConfig(UART2_IT_TXE, ENABLE);

	// get ready to get next package
	ui8_received_package_flag = 0;
}
