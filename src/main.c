/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "interrupts.h"
#include "stm8s.h"
#include "pins.h"
#include "uart.h"
#include "pwm.h"
#include "motor.h"
#include "wheel_speed_sensor.h"
#include "brake.h"
#include "pas.h"
#include "adc.h"
#include "timers.h"
#include "ebike_app.h"
#include "torque_sensor.h"
#include "lights.h"

/////////////////////////////////////////////////////////////////////////////////////////////
//// Functions prototypes

// main -- start of firmware and main loop
int main(void);

// With SDCC, interrupt service routine function prototypes must be placed in the file that contains main ()
// in order for an vector for the interrupt to be placed in the the interrupt vector space.  It's acceptable
// to place the function prototype in a header file as long as the header file is included in the file that
// contains main ().  SDCC will not generate any warnings or errors if this is not done, but the vector will
// not be in place so the ISR will not be executed when the interrupt occurs.

// Calling a function from interrupt not always works, SDCC manual says to avoid it. Maybe the best is to put
// all the code inside the interrupt

// Local VS global variables
// Sometimes I got the following error when compiling the firmware: motor.asm:750: Error: <r> relocation error
// when I have this code inside a function: "static uint8_t ui8_example_counter = 0;"
// and the solution was define the variable as global instead
// Another error example:
// *** buffer overflow detected ***: sdcc terminated
// Caught signal 6: SIGABRT

// PWM cycle interrupt (called every 64us)
void TIM1_CAP_COM_IRQHandler(void) __interrupt(TIM1_CAP_COM_IRQHANDLER);

// UART Receive interrupt
void UART2_RX_IRQHandler(void) __interrupt(UART2_RX_IRQHANDLER);
void UART2_TX_IRQHandler(void) __interrupt(UART2_TX_IRQHANDLER);

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

#ifdef MAIN_TIME_DEBUG
volatile uint8_t ui8_main_time;
uint8_t ui8_max_motor_time = 0;
uint8_t ui8_max_ebike_time = 0;
#endif

int main(void) {
    uint16_t ui16_TIM3_counter = 0;
    uint16_t ui16_ebike_app_controller_counter = 0;
    uint16_t ui16_motor_controller_counter = 0;
    uint16_t ui16_debug_uart_counter = 0;


    // set clock at the max 16 MHz
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

    brake_init();
    while (GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) == 0)
        ; // hold here while brake is pressed -- this is a protection for development
    lights_init();
    uart2_init();
    timer2_init();  // 50 KHz and 2us pulse. (Not used ??)
    timer3_init();  // 1KHz or 1ms period used for main loop timing
    adc_init();
    torque_sensor_init();
    pas_init();
    wheel_speed_sensor_init();
    hall_sensor_init();
    pwm_init_bipolar_4q();  // init TIM1 at 20KHz (50us)
    enableInterrupts();

    while (1) {
        // because of continue, the first if block code will have higher priority over the other
        ui16_TIM3_counter = TIM3_GetCounter();
        if ((ui16_TIM3_counter - ui16_motor_controller_counter) > 4) {
            // run every 4ms. Max measured motor_controller() duration is 0,15ms

            #ifdef MAIN_TIME_DEBUG
            // incremented every 50us by PWM interrupt function
            ui8_main_time = 0;
            #endif

            ui16_motor_controller_counter = ui16_TIM3_counter;
            motor_controller();

            #ifdef MAIN_TIME_DEBUG
            if (ui8_main_time > ui8_max_motor_time)
                ui8_max_motor_time = ui8_main_time;
            #endif

            continue;
        }

        ui16_TIM3_counter = TIM3_GetCounter();
        if ((ui16_TIM3_counter - ui16_ebike_app_controller_counter) > 25) {

            #ifdef MAIN_TIME_DEBUG
            // incremented every 50us by PWM interrupt function
            ui8_main_time = 0;
            #endif

            // run every 25ms. Max measured ebike_app_controller() duration is 3,1 ms.
            ui16_ebike_app_controller_counter = ui16_TIM3_counter;
            ebike_app_controller();

            #ifdef MAIN_TIME_DEBUG
            if (ui8_main_time > ui8_max_ebike_time)
                ui8_max_ebike_time = ui8_main_time;
            #endif
        }
    }
}
