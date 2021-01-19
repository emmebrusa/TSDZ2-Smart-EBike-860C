/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _INTERRUPTS_H_
#define _INTERRUPTS_H_

#define EXTI_PORTA_IRQHANDLER 3             // Not used
#define EXTI_PORTB_IRQHANDLER 4             // Not used
#define EXTI_PORTC_IRQHANDLER 5             // Initilized for brake signal but not used
#define EXTI_PORTD_IRQHANDLER 6             // Not used
#define EXTI_PORTE_IRQHANDLER 7             // Not used
#define TIM1_CAP_COM_IRQHANDLER 	12      // Motor control loop (64us)
#define TIM2_UPD_OVF_TRG_BRK_IRQHANDLER 13  // Not used
#define UART2_RX_IRQHANDLER 21              // UART Data received
#define UART2_TX_IRQHANDLER 20              // UART Data sent
#define ADC1_IRQHANDLER 22                  // ADC1 End Of Conversion (not used)

#endif
