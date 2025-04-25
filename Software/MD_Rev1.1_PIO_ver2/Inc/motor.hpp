#pragma once

#include <vector>

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
    void monitorEnc();
    void setThresholds(const std::vector<int>& newThresholds);
    void setTimeInterval(float timeInterval);
    float getRpm();

   private:
    const uint16_t maxSpeed;
    const uint8_t rotorPolesNum;
    uint16_t ENCODER_MAX = 4096;
    float TIME_INTERVAL_US = 200.0f;
    float OUTLIER_THRESHOLD = 0.2f;
    uint32_t RESET_TIME_US = 250000;

    uint16_t prevEncVal = 0;
    uint32_t timeSinceLastUpdate = 0;

    std::vector<int> thresholds = {100, 250, 500, 750, 1000, 1500, 2000};
    std::vector<float> provisionalRpms;

    float calRpm(uint16_t currentVal, uint16_t prevVal, uint32_t deltaTime);
    float calFinalRpm(const std::vector<float>& rpms);

    Encoder&
        enc;
    Driver& driver;

    float electricalAngle = 0.0f;
    float phase = 0.0f;
    float advancedAngle[2] = {150.0f, -150.0f};  // 0: turn 0, 1: turn 1
    float rpm = 0.0f;
};