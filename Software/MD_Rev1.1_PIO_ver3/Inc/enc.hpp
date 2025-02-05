#pragma once

#include "main.h"

#define ENCODER_ADDR 0x36
#define ENCODER_REG_RAW_ANGLE 0x0C

class Encoder {
   public:
    Encoder(const uint16_t encMax) : encMax(encMax) {}
    ~Encoder() {}

    void init();
    void setEncOffsetVal(uint16_t offsetVal);
    void updateEncVal();
    // uint16_t getAngleI2C();

    uint16_t getEncRawVal();
    uint16_t getEncVal();

    const uint16_t encMax;

   private:
    uint16_t encRawVal = 0;
    uint16_t encVal = 0;
    uint16_t encOffsetVal = 0;
};