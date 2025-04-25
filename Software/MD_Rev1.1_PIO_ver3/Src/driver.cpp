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
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, ha);  // HA
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, la);  // LA
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, hb);  // HB
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, lb);  // LB
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, hc);  // HC
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, lc);  // LC
}

void Driver::driveSquareWave(uint8_t step, uint16_t duty) {
    switch (step) {
        case 0:  // A -> B
            drive(duty, 0, 0, duty, 0, 1000);
            break;
        case 1:  // A -> C
            drive(duty, 0, 0, duty, 1000, 0);
            break;
        case 2:  // B -> C
            drive(0, duty, 0, 1000, duty, 0);
            break;
        case 3:  // B -> A
            drive(0, duty, 0, 0, duty, 1000);
            break;
        case 4:  // C -> A
            drive(0, 0, duty, 0, 1000, duty);
            break;
        case 5:  // C -> B
            drive(0, 0, duty, 1000, 0, duty);
            break;
        default:
            drive(0, 0, 0, 0, 0, 0);
            break;
    }
}

void Driver::driveSinWave(float phase, uint16_t duty) {
    phase = phase / 180.0f * M_PI;
    uint16_t a = (uint16_t)(duty / 2.0f) * (1.0f + sinf(phase));
    uint16_t b = (uint16_t)(duty / 2.0f) * (1.0f + sinf(phase - 2.0f * M_PI / 3.0f));
    uint16_t c = (uint16_t)(duty / 2.0f) * (1.0f + sinf(phase + 2.0f * M_PI / 3.0f));

    drive(a, b, c, a, b, c);
}