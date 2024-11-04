#pragma once

#include <stdint.h>

class Motor {
   public:
    Motor();
    void begin();
    void driveSquareWave(uint8_t state, uint16_t power);
    void driveSinWave(float phase, uint16_t power);
    void setAdvancedAngle(float angle0, float angle1);  // 0:正転, 1:逆転
    void setEncOffset(uint16_t offset);
    void calibrateEnc();
    void updateEncValue(uint16_t encValue);
    void brake();
    void calRpm(uint16_t calRpmCnt);
    void correctRpm();
    float calPhase(Motor *motor);
    uint16_t calPower(Motor *motor);

    bool motorDriveFlag;
    uint16_t power;
    uint16_t encOffset;
    uint16_t encRawValue;
    uint16_t encValue;
    float advancedAngle[2];  // 0:正転, 1:逆転
    float rpm[3];            // 0: short_rpm, 1: middle_rpm, 2: long_rpm
    float correctedRpm;
    uint16_t periodSlow[3];  // 0: short_period, 1: middle_period, 2: long_period
    uint16_t periodFast[3];  // 0: short_period, 1: middle_period, 2: long_period
    float targetRpm;
    bool speedFlag;  // 0: slow, 1: fast
};