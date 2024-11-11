#pragma once

#include "main.h"

class Encoder {
   public:
    Encoder(uint16_t& encVal, const uint16_t encMax) : encRawVal(encVal), encMax(encMax) {}
    ~Encoder() {}

    void init();
    void setEncOffsetVal(uint16_t offsetVal);
    void updateEncVal();

    uint16_t getEncRawVal();
    uint16_t getEncVal();

    const uint16_t encMax;

   private:
    uint16_t& encRawVal;
    uint16_t encVal = 0;
    uint16_t encOffsetVal = 0;
};