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

#define FW_VERSION 12

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

// PWM related values
// motor
#define PWM_CYCLES_SECOND                                       19047U // 52us (PWM period)
#define HALL_COUNTER_FREQ                                       250000U // 250KHz or 4us
#define HALL_COUNTER_INTERP_MAX                                 4166 // (HALL_COUNTER_FREQ/MOTOR_ROTOR_INTERPOLATION_MIN_ERPS/6)
// ramp up/down PWM cycles count
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_CADENCE_OFFSET      50     // PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP offset for cadence assist mode
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT             195    // (should be less than 255-50->205) 160 -> 160 * 64 us for every duty cycle increment at 15.625KHz
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN                 24     // 20 -> 20 * 64 us for every duty cycle increment at 15.625KHz
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_STARTUP             60     // Initial RAMP UP at motor startup
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

/*---------------------------------------------------------
 NOTE: regarding duty cycle (PWM) ramping

 Do not change these values if not sure of the effects!

 A lower value of the duty cycle inverse step will mean
 a faster acceleration. Be careful not to choose too
 low values for acceleration.
 ---------------------------------------------------------*/

#define PWM_DUTY_CYCLE_MAX                                        254
#define MIDDLE_SVM_TABLE                                          106
#define MIDDLE_PWM_COUNTER                                        105
#define PWM_DUTY_CYCLE_STARTUP                                    30    // Initial PWM Duty Cycle at motor startup
	
//#define MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES           10

/* Hall Sensors NOTE! - results after Hall sensor calibration experiment
Dai calcoli risulta che Trise - Tfall = 20 e cioè 80 us (1 Hall counter step e' 4us).
Quindi Trise è molto più lungo di Tfall. Quindi le transizioni del sensore Hall vengono rilevate
con un ritardo diverso in funzione della transizione del segnale. Quindi gli stati 6,3,5 (fronte di
salita) vengono rilevati con un ritardo di 80us (o 20 step) maggiore rispetto agli stati 2,1,4.
Quindi negli stati 6,3,5 va sommato 20 (20x4us=80us) al contatore Hall usato per l'interpolazione,
visto che è partito con 80us di ritardo rispetto agli altri stati. In questo modo
il contatore Hall sarà allineato allo stesso modo per tutti gli stati, ma sarà comunque in ritardo
di Tfall per tutti gli stati. Questo ritardo fisso viene gestito con un offset fisso da sommare
al contatore. Questo spiega la necessità dell'offset fisso che si è reso necessario fino ad ora.
Visto che il valore attuale dell'offset fisso è 18, si deve sommare 28 (20+8) agli stati 6,3,5
e 8 agli stati 2,1,4 poichè (28+8)/2=18. Di questo 8 (8x4=32us), 6,5 (6,56*4=26,25us) servono a
compensare il ritardo fisso fra la lettura del contatore Hall e l'applicazione delle fasi che
avviene con un ritardo fisso pari a mezzo ciclo PWM (1/19047 = 52,5us, 52,5/2=26,25us).
Purtroppo non c'è modo di calcolare i valori assoluti di Trise e Tfall utilizzando i contatori
Hall ma solo la differenza fra i due valori.
In realtà sarebbe possibile misurando con precisionea la corrente a diverse velocità prestabilite
del motore. Mantenedo fissa la velocità si fa variare il duty cycle, e l'offset del contatore hall
e si trova il valore di offset per cui la corrente assorbita è minima per una data velocita.
Facendo alcune rilevazioni e poi una regressione lineare, sarebbe poi possibile ottenere quali sono
gli offset del contatore Hall e del'angolo della phase ottimali da applicare.
Il problema è che la misura di corrente fornita dal controller è troppo grossolana e si dovrebbe
usare uno strumento di precisione.
***************************************
Test effettuato il 21/1/2012
MOTOR_ROTOR_OFFSET_ANGLE:  10 - 6 -> 4
HALL_COUNTER_OFFSET_DOWN:  8 + 15 -> 23
HALL_COUNTER_OFFSET_UP:   28 + 15 -> 43
****************************************
*/

#define HALL_COUNTER_OFFSET_UP                  44
#define HALL_COUNTER_OFFSET_DOWN                23
#define FW_HALL_COUNTER_OFFSET_MAX              6


#define MOTOR_ROTOR_INTERPOLATION_MIN_ERPS      10

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
#define ADC_10_BIT_BATTERY_CURRENT_MAX                            112	// 18 amps
#define ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX                        187	// 30 amps
//#define ADC_10_BIT_BATTERY_CURRENT_MAX                            106	// 17 amps
//#define ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX                        177	// 28 amps

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
