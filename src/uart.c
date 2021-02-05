/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>

#include "stm8s.h"
#include "stm8s_uart2.h"
#include "interrupts.h"


void uart2_init(void)
{
    UART2_DeInit();

    UART2_Init((uint32_t) 19200,
            UART2_WORDLENGTH_8D,
            UART2_STOPBITS_1,
            UART2_PARITY_NO,
            UART2_SYNCMODE_CLOCK_DISABLE,
            UART2_MODE_TXRX_ENABLE);

    UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);

    // Set UART2 TX IRQ priority to level 1 :0=lowest - 3=highest(default value)
    ITC_SetSoftwarePriority(UART2_TX_IRQHANDLER, ITC_PRIORITYLEVEL_1);
    ITC_SetSoftwarePriority(UART2_RX_IRQHANDLER, ITC_PRIORITYLEVEL_2);
}
