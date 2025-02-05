#include "enc.hpp"

#include "adc.h"
#include "i2c.h"

void Encoder::init() {
}

void Encoder::setEncOffsetVal(uint16_t offsetVal) {
    encOffsetVal = offsetVal;
}

void Encoder::updateEncVal() {
    uint8_t ret[2] = {0};
    HAL_I2C_Mem_Read(&hi2c2, (ENCODER_ADDR << 1) | 1, ENCODER_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT, ret, sizeof(ret), 1);

    encRawVal = (ret[0] << 8) | ret[1];
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
