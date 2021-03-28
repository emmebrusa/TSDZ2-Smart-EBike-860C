/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MAIN_H_
#define _MAIN_H_

//#define DEBUG_UART
//#define PWM_TIME_DEBUG
//#define MAIN_TIME_DEBUG

//#define FW_VERSION 7

// PWM related values
// motor
#define PWM_CYCLES_SECOND                                       19047U // 52us (PWM period)
#define PWM_CYCLES_COUNTER_MAX                                  3800U  // 5 erps minimum speed -> 1/5 = 200 ms; 200 ms / 50 us = 4000 (3125 at 15.625KHz)
#define DOUBLE_PWM_CYCLES_SECOND                                38094 // 25us (2 irq x PWM period)
// ramp up/down PWM cycles count
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT             40    // 160 -> 160 * 64 us for every duty cycle increment at 15.625KHz
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN                 24     // 20 -> 20 * 64 us for every duty cycle increment at 15.625KHz
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT           49     // 40 -> 40 * 64 us for every duty cycle decrement at 15.625KHz
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN               10     // 8 -> 8 * 64 us for every duty cycle decrement at 15.625KHz
#define MOTOR_OVER_SPEED_ERPS                                   650    // motor max speed | 30 points for the sinewave at max speed (less than PWM_CYCLES_SECOND/30)
#define CRUISE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP                  98    // 80 at 15.625KHz
#define WALK_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP             244    // 200 at 15.625KHz
#define THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT        98    // 80 at 15.625KHz
#define THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN            49     // 40 at 15.625KHz
// cadence
#define CADENCE_SENSOR_CALC_COUNTER_MIN                         4266  // 3500 at 15.625KHz
#define CADENCE_SENSOR_TICKS_COUNTER_MIN_AT_SPEED               341  // 280 at 15.625KHz
#define CADENCE_TICKS_STARTUP                                   7618  // ui16_cadence_sensor_ticks value for startup. About 7-8 RPM (6250 at 15.625KHz)
#define CADENCE_SENSOR_STANDARD_MODE_SCHMITT_TRIGGER_THRESHOLD  426   // software based Schmitt trigger to stop motor jitter when at resolution limits (350 at 15.625KHz)
// Wheel speed sensor
#define WHEEL_SPEED_SENSOR_TICKS_COUNTER_MAX                    165   // (135 at 15,625KHz) something like 200 m/h with a 6'' wheel
#define WHEEL_SPEED_SENSOR_TICKS_COUNTER_MIN                    39976 // could be a bigger number but will make for a slow detection of stopped wheel speed


#define PWM_DUTY_CYCLE_MAX                                        254
#define MIDDLE_SVM_TABLE                                          106
#define MIDDLE_PWM_COUNTER                                        105

/*---------------------------------------------------------
 NOTE: regarding duty cycle (PWM) ramping

 Do not change these values if not sure of the effects!

 A lower value of the duty cycle inverse step will mean
 a faster acceleration. Be careful not to choose too
 low values for acceleration.
 ---------------------------------------------------------*/

#define MOTOR_ROTOR_OFFSET_ANGLE                                  10
#define MOTOR_ROTOR_ANGLE_90                                      (63  + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_150                                     (106 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_210                                     (148 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_270                                     (191 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_330                                     (233 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_30                                      (20  + MOTOR_ROTOR_OFFSET_ANGLE)

/*---------------------------------------------------------
 NOTE: regarding motor rotor offset

 The motor rotor offset should be as close to 0 as
 possible. You can try to tune with the wheel in the air,
 full throttle and look at the batttery current. Adjust
 for the lowest battery current possible.
 ---------------------------------------------------------*/

#define MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES           10

// Torque sensor values
#define ADC_TORQUE_SENSOR_CALIBRATION_OFFSET    6
#define ADC_TORQUE_SENSOR_OFFSET_DEFAULT		150
// adc torque offset gap value for error
#define ADC_TORQUE_SENSOR_OFFSET_THRESHOLD		35
// adc torque delta range value for remapping
#define ADC_TORQUE_SENSOR_RANGE_MIN	  			160
// scale the torque assist target current
#define TORQUE_ASSIST_FACTOR_DENOMINATOR		110

/*---------------------------------------------------------
 NOTE: regarding motor start interpolation

 This value is the ERPS speed after which a transition
 happens from sinewave and no interpolation to
 interpolation 60 degrees. Must be found experimentally
 but a value of 25 may be good.
 ---------------------------------------------------------*/

#define ADC_10_BIT_BATTERY_CURRENT_MAX                            106 // 17 amps
#define ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX                        177 // 28 amps

/*---------------------------------------------------------
 NOTE: regarding ADC battery current max

 This is the maximum current in ADC steps that the motor
 will be able to draw from the battery. A higher value
 will give higher torque figures but the limit of the
 controller is 16 A and it should not be exceeded.
 ---------------------------------------------------------*/

// throttle ADC values
#define ADC_THROTTLE_MIN_VALUE                                    47
#define ADC_THROTTLE_MAX_VALUE                                    176

/*---------------------------------------------------------
 NOTE: regarding throttle ADC values

 Max voltage value for throttle, in ADC 8 bits step,
 each ADC 8 bits step = (5 V / 256) = 0.0195

 ---------------------------------------------------------*/

// cadence sensor
#define CADENCE_SENSOR_NUMBER_MAGNETS                           20U

/*-------------------------------------------------------------------------------
 NOTE: regarding the cadence sensor

 CADENCE_SENSOR_NUMBER_MAGNETS = 20, this is the number of magnets used for
 the cadence sensor. Was validated on August 2018 by Casainho and jbalat

 Cadence is calculated by counting how much time passes between two
 transitions. Depending on if all transitions are measured or simply
 transitions of the same kind it is important to adjust the calculation of
 pedal cadence.
 -------------------------------------------------------------------------------*/


// default values
#define DEFAULT_VALUE_BATTERY_CURRENT_MAX                         10  // 10 amps

/*---------------------------------------------------------

 NOTE: regarding the torque sensor output values

 Torque (force) value needs to be found experimentaly.

 One torque sensor ADC 10 bit step is equal to 0.38 kg

 Force (Nm) = 1 Kg * 9.81 * 0.17 (0.17 = arm cranks size)
 ---------------------------------------------------------*/

// ADC battery voltage measurement
#define BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X512                  44
#define BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000                 87  // conversion value verified with a cheap power meter

/*---------------------------------------------------------
 NOTE: regarding ADC battery voltage measurement

 0.344 per ADC 8 bit step:

 17.9 V -->  ADC 8 bits value  = 52;
 40 V   -->  ADC 8 bits value  = 116;

 This signal is atenuated by the opamp 358.
 ---------------------------------------------------------*/

// ADC battery current measurement
#define BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X512                  80
#define BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100                  16  // 0.16A x 10 bit ADC step

#endif // _MAIN_H_
