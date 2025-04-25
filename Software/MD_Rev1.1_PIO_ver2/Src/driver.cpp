#include "driver.hpp"

#include <math.h>
#include <stdio.h>

#include "adc.h"
#include "tim.h"

void Driver::init() {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);  // HA
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);  // LA
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);  // HB
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);  // LB
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);  // HC
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);  // LC
}

void Driver::drive(uint16_t ha, uint16_t hb, uint16_t hc, uint16_t la, uint16_t lb, uint16_t lc) {
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, ha);  // HA
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, la);  // LA
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, hb);  // HB
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, lb);  // LB
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, hc);  // HC
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, lc);  // LC
}

void Driver::driveSquareWave(uint8_t step, uint16_t duty) {
    if (step == 0) {                                        // A -> B
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, duty);  // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, duty);  // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);     // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);     // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);     // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 1000);  // LC
    } else if (step == 1) {                                 // A -> C
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, duty);  // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, duty);  // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);     // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 1000);  // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);     // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);     // LC
    } else if (step == 2) {                                 // B -> C
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);     // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 1000);  // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, duty);  // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, duty);  // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);     // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);     // LC
    } else if (step == 3) {                                 // B -> A
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);     // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);     // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, duty);  // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, duty);  // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);     // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 1000);  // LC
    } else if (step == 4) {                                 // C -> A
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);     // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);     // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);     // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 1000);  // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, duty);  // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, duty);  // LC
    } else if (step == 5) {                                 // C -> B
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);     // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 1000);  // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);     // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);     // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, duty);  // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, duty);  // LC
    }
}

void Driver::driveSinWave(float phase, uint16_t duty) {
    phase = phase / 180.0f * M_PI;
    uint16_t a = (uint16_t)(duty / 2.0f) * (1.0f + sinf(phase));
    uint16_t b = (uint16_t)(duty / 2.0f) * (1.0f + sinf(phase - 2.0f * M_PI / 3.0f));
    uint16_t c = (uint16_t)(duty / 2.0f) * (1.0f + sinf(phase + 2.0f * M_PI / 3.0f));
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, a);  // HA
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, a);  // LA
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, b);  // HB
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, b);  // LB
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, c);  // HC
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, c);  // LC
}