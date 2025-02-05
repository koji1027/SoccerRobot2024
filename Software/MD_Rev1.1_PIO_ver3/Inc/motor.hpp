#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <vector>

#include "current_sensor.h"
#include "driver.hpp"
#include "enc.hpp"
#include "main.h"

#define SPEED_P_GAIN 0.0025f
#define SPEED_I_GAIN 0.0f  // 0.00005f
#define SPEED_D_GAIN 0.4f

class Motor {
   public:
    Motor(Encoder& enc, Driver& driver, const uint16_t maxSpeed, const uint8_t rotorPolesNum, uint16_t TIME_INTERVAL_US)
        : enc(enc), driver(driver), maxSpeed(maxSpeed), rotorPolesNum(rotorPolesNum), TIME_INTERVAL_US(TIME_INTERVAL_US) {}
    ~Motor() {}

    void init(bool calibrateFlag);
    void calibrate();
    float getElectricalAngle();
    void calElectricalAngle();
    void setAdvancedAngle(float angleTurn0, float angleTurn1);
    void calPhase(bool turn);
    float getPhase();
    void controlDriver(uint8_t mode);
    void setThresholds(const std::vector<int>& newThresholds);
    void setTimeInterval(float timeInterval);
    void calRpm();
    float getRpm();
    void calDuty(float targetRpm, bool turn);
    void getCorrectCurrent(float IA, float IB, float IC);

    int32_t encValList[100] = {0};

    float IA = 0.0f;
    float IB = 0.0f;
    float IC = 0.0f;

    uint16_t dutyA = 0;
    uint16_t dutyB = 0;
    uint16_t dutyC = 0;

    uint16_t duty = 0;

   private:
    const uint16_t maxSpeed;
    const uint8_t rotorPolesNum;
    bool initialized = false;
    uint16_t ENCODER_MAX = 4096;
    uint16_t TIME_INTERVAL_US;
    float OUTLIER_THRESHOLD = 0.2f;
    uint32_t RESET_TIME_US = 250000;

    uint16_t prevEncVal = 0;
    uint32_t timeSinceLastUpdate = 0;

    std::vector<int> thresholds = {100, 250, 500, 750, 1000, 1500, 2000};
    std::vector<float> provisionalRpms;

    Encoder& enc;
    Driver& driver;

    float electricalAngle = 0.0f;
    float phase = 0.0f;
    float advancedAngle[2] = {-150.0f, 150.0f};  // 0: turn 0, 1: turn 1
    float rpm = 0.0f;
};

#endif  // MOTOR_HPP