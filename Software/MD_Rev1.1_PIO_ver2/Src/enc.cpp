#include "enc.hpp"

#include "adc.h"

void Encoder::init() {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&encRawVal, 1);
}

void Encoder::setEncOffsetVal(uint16_t offsetVal) {
    encOffsetVal = offsetVal;
}

void Encoder::updateEncVal() {
    encVal = encMax + encRawVal - encOffsetVal;
    encVal %= encMax;
}

uint16_t Encoder::getEncRawVal() {
    return encRawVal;
}

uint16_t Encoder::getEncVal() {
    this->updateEncVal();
    return encVal;
}