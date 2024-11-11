#pragma once

#include "driver.hpp"
#include "enc.hpp"
#include "main.h"

class Motor {
   public:
    Motor(Encoder& enc, Driver& driver, const uint16_t maxSpeed, const uint8_t rotorPolesNum)
        : enc(enc), driver(driver), maxSpeed(maxSpeed), rotorPolesNum(rotorPolesNum) {}
    ~Motor() {}

    void init();
    void calibrate();
    float getElectricalAngle();
    void calElectricalAngle();
    void setAdvancedAngle(float angleTurn0, float angleTurn1);
    void calPhase(bool turn);
    float getPhase();
    void controlDriver(uint8_t mode, uint16_t duty);

   private:
    const uint16_t maxSpeed;
    const uint8_t rotorPolesNum;

    Encoder& enc;
    Driver& driver;

    float electricalAngle = 0.0f;
    float phase = 0.0f;
    float advancedAngle[2] = {-150.0f, 150.0f};  // 0: turn 0, 1: turn 1
    // -180 135
};