/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "pins.h"
#include "stm8s_adc1.h"
#include "adc.h"
#include "ebike_app.h"
#include "motor.h"
#include "common.h"


void adc_init(void) {
    //init GPIO for the used ADC pins
    GPIO_Init(GPIOB, (GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4), GPIO_MODE_IN_FL_NO_IT);

    ADC1_DeInit();
    ADC1_ConversionConfig(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_7, ADC1_ALIGN_LEFT);
    ADC1_PrescalerConfig(ADC1_PRESSEL_FCPU_D3);
    ADC1_ExternalTriggerConfig(ADC1_EXTTRIG_TIM, DISABLE);
    ADC1_SchmittTriggerConfig(ADC1_SCHMITTTRIG_CHANNEL4, DISABLE);
    ADC1_SchmittTriggerConfig(ADC1_SCHMITTTRIG_CHANNEL5, DISABLE);
    ADC1_SchmittTriggerConfig(ADC1_SCHMITTTRIG_CHANNEL6, DISABLE);
    ADC1_SchmittTriggerConfig(ADC1_SCHMITTTRIG_CHANNEL7, DISABLE);
    ADC1_ScanModeCmd(ENABLE);
    ADC1_Cmd(ENABLE);

    // delay 3 sec (3000 ms) - perform some conversions
    uint16_t ui16_counter;
    for (uint8_t ui8 = 0; ui8 < 30; ui8++) {
        // start ADC1 conversion
        ADC1_ClearFlag(ADC1_FLAG_EOC);
        ADC1_StartConversion();
        ui16_counter = TIM3_GetCounter();
        while ((TIM3_GetCounter() - ui16_counter) < 100)
            ;
    }
}
