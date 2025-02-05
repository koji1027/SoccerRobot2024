#include "current_sensor.h"

#include <stdio.h>

#include "adc.h"

void CurrentSensor::init() {
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adcVal, 3);
    calibrate();
}

float CurrentSensor::getCurrent(uint8_t phase) {
    return current[phase];
}

void CurrentSensor::update() {
    float _current[3] = {0.0f};
    for (int i = 0; i < 3; i++) {
        _current[i] = ((float)adcVal[i] - currentOffset[i]) / GAIN[i];
        float a = 1 / (2.0f * 3.14159265359f * 300 * 0.0002f + 1);
        current[i] = a * _current[i] + (1 - a) * current[i];
    }
}

void CurrentSensor::calibrate() {
    HAL_Delay(100);
    uint64_t adcSum[3] = {0};
    for (int i = 0; i < 100; i++) {
        for (int j = 0; j < 3; j++) {
            adcSum[j] += adcVal[j];
        }
        HAL_Delay(10);
    }
    for (int i = 0; i < 3; i++) {
        currentOffset[i] = (float)adcSum[i] / 100.0f;
    }
}