#include "enc.hpp"

#include <cmath>

#include "i2c.h"

void Encoder::init() {
}

void Encoder::readRaw() {
    uint8_t data[2];
    HAL_I2C_Mem_Read(&hi2c2, ENC_ADDR << 1, ENC_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT, data, 2, 1);
    rawVal = (data[0] << 8) | data[1];
}

void Encoder::updateVal() {
    readRaw();
    val = ((rawVal - offset) % ENC_RES + ENC_RES) % ENC_RES;
    elecAngle = fmodf(fmodf((float)val, (float)ELEC_ANGLE) + (float)ELEC_ANGLE, (float)ELEC_ANGLE) / (float)ELEC_ANGLE * TWO_PI;
}

void Encoder::setOffset(uint16_t offset) {
    this->offset = offset;
}

uint16_t Encoder::getVal(uint8_t type = 0) {
    if (type == 0) {
        return rawVal;
    } else if (type == 1) {
        return val;
    } else if (type == 2) {
        return offset;
    } else {
        return 0;
    }
}