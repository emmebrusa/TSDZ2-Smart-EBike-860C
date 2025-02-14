/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2021.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MAIN_H_
#define _MAIN_H_

//#define TIME_DEBUG
//#define HALL_DEBUG

//#define FW_VERSION 15 // mspider65

/*---------------------------------------------------------
 NOTE: regarding motor rotor offset

 The motor rotor offset should be as close to 0 as
 possible. You can try to tune with the wheel in the air,
 full throttle and look at the batttery current. Adjust
 for the lowest battery current possible.
 ---------------------------------------------------------*/
#define MOTOR_ROTOR_OFFSET_ANGLE  (uint8_t)4
#define PHASE_ROTOR_ANGLE_30  (uint8_t)((uint8_t)21  + MOTOR_ROTOR_OFFSET_ANGLE - (uint8_t)64)
#define PHASE_ROTOR_ANGLE_90  (uint8_t)((uint8_t)64  + MOTOR_ROTOR_OFFSET_ANGLE - (uint8_t)64)
#define PHASE_ROTOR_ANGLE_150 (uint8_t)((uint8_t)107 + MOTOR_ROTOR_OFFSET_ANGLE - (uint8_t)64)
#define PHASE_ROTOR_ANGLE_210 (uint8_t)((uint8_t)149 + MOTOR_ROTOR_OFFSET_ANGLE - (uint8_t)64)
#define PHASE_ROTOR_ANGLE_270 (uint8_t)((uint8_t)192 + MOTOR_ROTOR_OFFSET_ANGLE - (uint8_t)64)
#define PHASE_ROTOR_ANGLE_330 (uint8_t)((uint8_t)235 + MOTOR_ROTOR_OFFSET_ANGLE - (uint8_t)64)

#define HALL_COUNTER_FREQ                                       250000U // 250KHz or 4us

// ----------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------
// PWM related values

//#define PWM_FREQ												18 // 18 Khz
#define PWM_FREQ												19 // 19 Khz

#if PWM_FREQ == 19
#define PWM_COUNTER_MAX                                         420 // 16MHz / 840 = 19,047 KHz                                        107
#define MIDDLE_SVM_TABLE										107 // svm table 19 Khz
#define MIDDLE_PWM_COUNTER										107
#else
#define PWM_COUNTER_MAX                                         444 // 16MHz / 888 = 18,018 KHz                                    110
#define MIDDLE_SVM_TABLE                                        110 // svm table 18 Khz
#define MIDDLE_PWM_COUNTER                                      110
#endif

#define PWM_CYCLES_SECOND                                       (16000000/(PWM_COUNTER_MAX*2)) // 55.5us (PWM period) 18 Khz

/*---------------------------------------------------------
 NOTE: regarding duty cycle (PWM) ramping

 Do not change these values if not sure of the effects!

 A lower value of the duty cycle inverse step will mean
 a faster acceleration. Be careful not to choose too
 low values for acceleration.
 ---------------------------------------------------------*/

// ramp up/down PWM cycles count
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT             (uint8_t)(PWM_CYCLES_SECOND/98)
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN                 (uint8_t)(PWM_CYCLES_SECOND/781)
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT           (uint8_t)(PWM_CYCLES_SECOND/260)
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN               (uint8_t)(PWM_CYCLES_SECOND/1953)
#define CRUISE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP                  (uint8_t)(PWM_CYCLES_SECOND/78)
#define WALK_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP             (uint8_t)(PWM_CYCLES_SECOND/78)
#define THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT        (uint8_t)(PWM_CYCLES_SECOND/78)
#define THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN            (uint8_t)(PWM_CYCLES_SECOND/390)

#define MOTOR_OVER_SPEED_ERPS                                   ((PWM_CYCLES_SECOND/29) < 650 ?  (PWM_CYCLES_SECOND/29) : 650) // motor max speed | 29 points for the sinewave at max speed (less than PWM_CYCLES_SECOND/29)
#define MOTOR_SPEED_FIELD_WEAKENING_MIN							490 // 90 rpm
#define ERPS_SPEED_OF_MOTOR_REENABLING							320 // 60 rpm

// cadence
#define CADENCE_SENSOR_CALC_COUNTER_MIN                         (uint16_t)((uint32_t)PWM_CYCLES_SECOND*100U/446U)	// 3500 at 15.625KHz
#define CADENCE_SENSOR_TICKS_COUNTER_MIN_AT_SPEED               (uint16_t)((uint32_t)PWM_CYCLES_SECOND*10U/558U)	// 280 at 15.625KHz
#define CADENCE_TICKS_STARTUP                                   (uint16_t)((uint32_t)PWM_CYCLES_SECOND*10U/25U)		// ui16_cadence_sensor_ticks value for startup. About 7-8 RPM (6250 at 15.625KHz)
//#define CADENCE_SENSOR_STANDARD_MODE_SCHMITT_TRIGGER_THRESHOLD	(uint16_t)((uint32_t)PWM_CYCLES_SECOND*10U/446U)	// software based Schmitt trigger to stop motor jitter when at resolution limits (350 at 15.625KHz)
#define CADENCE_SENSOR_STANDARD_MODE_SCHMITT_TRIGGER_THRESHOLD	(uint16_t)((uint32_t)PWM_CYCLES_SECOND*10U/892U)	// Quick stop value as version 4.4

// Wheel speed sensor
#define WHEEL_SPEED_SENSOR_TICKS_COUNTER_MAX                    (uint16_t)((uint32_t)PWM_CYCLES_SECOND*10U/1157U)   // (135 at 15,625KHz) something like 200 m/h with a 6'' wheel
#define WHEEL_SPEED_SENSOR_TICKS_COUNTER_MIN                    (uint16_t)((uint32_t)PWM_CYCLES_SECOND*1000U/477U) // 32767@15625KHz could be a bigger number but will make for a slow detection of stopped wheel speed
#define WHEEL_SPEED_SENSOR_SIMULATION							0

// duty cycle
#define PWM_DUTY_CYCLE_MAX										255
#define PWM_DUTY_CYCLE_STARTUP									30    // Initial PWM Duty Cycle at motor startup

// ----------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------

/* Hall Sensors NOTE! - results after Hall sensor calibration experiment
Dai test sulla calibrazione dei sensori Hall risulta che Trise - Tfall = 21 e cioè 84 us
(1 Hall counter step = 4us).
Quindi gli stati 6,3,5 (fronte di salita) vengono rilevati con un ritardo di 84us maggiore
rispetto agli stati 2,1,4.
Quindi per gli stati 6,3,5 va sommato 21 (21x4us=84us) al contatore Hall usato per l'interpolazione,
visto che è partito con 84us di ritardo rispetto agli altri stati.
In questo modo il contatore Hall viene allineato allo stesso modo per tutti gli stati, ma sarà
comunque in ritardo di Tfall per tutti gli stati. Questo ritardo viene gestito con un ulteriore
offset da sommare al contatore per tutti gli stati.
Dai test effettuati risulta che Tfall vale circa 66us (16,5 step) a cui va sommato il ritardo fra							   
la lettura del contatore Hall e la scrittura dei registri PWM che è sempre uguale a mezzo
ciclo PWM (1/(19047*2) = 26,25us o 6,5 step).
Quindi l'offset per gli stati 2,1,4 vale 23 (16,5+6,5) mentre per gli stati 6,3,5
vale 44 (16,5+6,5+21).
I test effettuati hanno inoltre calcolato che il riferimento angolare corretto non è 10 ma 4 step
***************************************
Test effettuato il 21/1/2021
MOTOR_ROTOR_OFFSET_ANGLE:  10 -> 4
HALL_COUNTER_OFFSET_DOWN:  8  -> 23
HALL_COUNTER_OFFSET_UP:    29 -> 44
****************************************
*/

#define HALL_COUNTER_OFFSET_DOWN                (HALL_COUNTER_FREQ/PWM_CYCLES_SECOND/2 + 17)
#define HALL_COUNTER_OFFSET_UP                  (HALL_COUNTER_OFFSET_DOWN + 21)
#define FW_HALL_COUNTER_OFFSET_MAX              5 // 5*4=20us max time offset

#define MOTOR_ROTOR_INTERPOLATION_MIN_ERPS      10

// set on the display with motor type
#define FOC_ANGLE_MULTIPLIER_36V				30 // 36 volt motor
#define FOC_ANGLE_MULTIPLIER_48V				39 // 48 volt motor

// adc torque offset gap value for error
#define ADC_TORQUE_SENSOR_OFFSET_THRESHOLD		30

// Torque sensor values
#define ADC_TORQUE_SENSOR_OFFSET_DEFAULT		150
// adc torque range parameters for remapping
#define ADC_TORQUE_SENSOR_RANGE_TARGET	  		160
#define ADC_TORQUE_SENSOR_RANGE_TARGET_MIN 		133
#define ADC_TORQUE_SENSOR_ANGLE_COEFF			11
#define ADC_TORQUE_SENSOR_ANGLE_COEFF_X10		(uint8_t)(ADC_TORQUE_SENSOR_ANGLE_COEFF * 10)
// scale the torque assist target current
#define TORQUE_ASSIST_FACTOR_DENOMINATOR		120

// smooth start ramp
#define SMOOTH_START_RAMP_DEFAULT				165 // 35% (255=0% long ramp)
#define SMOOTH_START_RAMP_MIN					30

// adc current
//#define ADC_10_BIT_BATTERY_EXTRACURRENT						38  //  6 amps
#define ADC_10_BIT_BATTERY_EXTRACURRENT						50  //  8 amps
#define ADC_10_BIT_BATTERY_CURRENT_MAX						112	// 18 amps // 1 = 0.16 Amp
//#define ADC_10_BIT_BATTERY_CURRENT_MAX						124	// 20 amps // 1 = 0.16 Amp
//#define ADC_10_BIT_BATTERY_CURRENT_MAX						136	// 22 amps // 1 = 0.16 Amp
#define ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX					187	// 30 amps // 1 = 0.16 Amp

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
#define BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000                 87  // conversion value verified with a cheap power meter

// ADC battery voltage to be subtracted from the cut-off
#define DIFFERENCE_CUT_OFF_SHUTDOWN_10_BIT						  100 // 9 Volts

/*---------------------------------------------------------
 NOTE: regarding ADC battery voltage measurement

 0.344 per ADC 8 bit step:

 17.9 V -->  ADC 8 bits value  = 52;
 40 V   -->  ADC 8 bits value  = 116;

 This signal is atenuated by the opamp 358.
 ---------------------------------------------------------*/

// ADC battery current measurement
#define BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100                  16  // 0.16A x 10 bit ADC step

// walk assist
#define WALK_ASSIST_WHEEL_SPEED_MIN_DETECT_X10	42
#define WALK_ASSIST_ERPS_THRESHOLD				20
#define WALK_ASSIST_ADJ_DELAY_MIN				4
#define WALK_ASSIST_ADJ_DELAY_STARTUP			10
#define WALK_ASSIST_DUTY_CYCLE_MIN              40
#define WALK_ASSIST_DUTY_CYCLE_STARTUP			50
#define WALK_ASSIST_DUTY_CYCLE_MAX              130
#define WALK_ASSIST_ADC_BATTERY_CURRENT_MAX     40

#endif // _MAIN_H_
