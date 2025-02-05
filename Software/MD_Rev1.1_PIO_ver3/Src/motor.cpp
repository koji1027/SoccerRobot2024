#include "motor.hpp"

#include <stdio.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>

void Motor::init(bool calibrateFlag) {
    enc.init();
    driver.init();

    enc.setEncOffsetVal(3201);

    if (calibrateFlag) {
        this->calibrate();
    }
    initialized = true;
}

void Motor::calibrate() {
    uint32_t encValueSumTurn1 = 0;
    uint16_t encOffsetTurn1 = 0;
    for (int k = 0; k < 3; k++) {
        for (int i = 0; i < rotorPolesNum; i++) {
            for (int j = 0; j < 6; j++) {
                driver.driveSquareWave(j, 300);
                HAL_Delay(10);
            }
        }
        encValueSumTurn1 += enc.getEncRawVal();
    }
    encOffsetTurn1 = (uint16_t)((float)encValueSumTurn1 / 3.0f);

    HAL_Delay(200);

    uint32_t encValueSumTurn0 = 0;
    uint16_t encOffsetTurn0 = 0;
    for (int k = 0; k < 3; k++) {
        for (int i = 0; i < 7; i++) {
            for (int j = 10; j > 4; j--) {
                driver.driveSquareWave(j % 6, 300);
                HAL_Delay(10);
            }
        }
        encValueSumTurn0 += enc.getEncRawVal();
    }
    encOffsetTurn0 = (uint16_t)((float)encValueSumTurn0 / 3.0f);

    uint16_t encOffset = (float)(encOffsetTurn0 + encOffsetTurn1) / 2.0f;
    enc.setEncOffsetVal(encOffset);
    printf("encOffset: %d\n", encOffset);

    HAL_Delay(200);
}

float Motor::getElectricalAngle() {
    this->calElectricalAngle();
    return this->electricalAngle;
}

void Motor::calElectricalAngle() {
    uint16_t encVal = enc.getEncVal();
    uint16_t singleTurn = enc.encMax / rotorPolesNum;
    this->electricalAngle = (float)(encVal % singleTurn) / (float)singleTurn * 360.0f;
}

void Motor::setAdvancedAngle(float angleTurn0, float angleTurn1) {
    advancedAngle[0] = angleTurn0;
    advancedAngle[1] = angleTurn1;
}

void Motor::calPhase(bool turn) {
    this->calElectricalAngle();
    if (turn) {
        this->phase = this->electricalAngle + advancedAngle[1];
    } else {
        this->phase = this->electricalAngle + advancedAngle[0];
    }
    if (this->phase < 0) {
        this->phase += 360.0f;
    } else if (this->phase >= 360.0f) {
        this->phase -= 360.0f;
    }
}

float Motor::getPhase() {
    return this->phase;
}

void Motor::controlDriver(uint8_t mode) {
    if (this->initialized == false) {
        return;
    }
    switch (mode) {
        case 0:
            driver.driveSquareWave(this->phase, duty);
            break;
        case 1:
            driver.driveSinWave(this->phase, duty);
            dutyA = (uint16_t)(duty / 2.0f) * (1.0f + sinf(phase));
            dutyB = (uint16_t)(duty / 2.0f) * (1.0f + sinf(phase - 2.0f * M_PI / 3.0f));
            dutyC = (uint16_t)(duty / 2.0f) * (1.0f + sinf(phase + 2.0f * M_PI / 3.0f));
            break;
        default:
            break;
    }
}

void Motor::setThresholds(const std::vector<int>& newThresholds) {
    thresholds = newThresholds;
}

float Motor::getRpm() {
    return rpm;
}

void Motor::calRpm() {
    static uint16_t prevEncVal = 0;
    uint16_t encVal = enc.getEncVal();
    int32_t encDiff = -encVal + prevEncVal;
    if (encDiff < -enc.encMax / 2) {
        encDiff += enc.encMax;
    } else if (encDiff > enc.encMax / 2) {
        encDiff -= enc.encMax;
    }

    prevEncVal = encVal;

    for (int i = 0; i < 99; i++) {
        encValList[i] = encValList[i + 1];
    }
    encValList[99] = encDiff;

    float _rpm = 0;

    for (int i = 0; i < 100; i++) {
        _rpm += (float)encValList[99 - i];
    }

    _rpm = _rpm / 100.0f / (float)TIME_INTERVAL_US * 1000000.0f * 60.0f / enc.encMax / rotorPolesNum;

    if (abs(_rpm) < 10000) {
        rpm = _rpm;
    }
}

void Motor::calDuty(float targetRpm, bool turn) {
    static uint16_t _duty = 0;
    float error = 0;
    if (turn) {
        error = -targetRpm + rpm;
    } else {
        error = targetRpm - rpm;
    }
    static float prevError = 0;
    float dError = error - prevError;
    static float integral = 0;
    if (abs(error) < targetRpm / 20.0f) {
        integral = 0;
    }
    integral += error;
    prevError = error;
    _duty += error * SPEED_P_GAIN + dError * SPEED_D_GAIN + integral * SPEED_I_GAIN;
    if (_duty > 900) {
        _duty = 900;
    } else if (duty < 0) {
        _duty = 0;
    }
    if (duty == 0) {
        duty = targetRpm / 400.0f / 8.0f * 1000.0f;
    } else {
        duty = _duty;
    }
}

void Motor::getCorrectCurrent(float _IA, float _IB, float _IC) {
    uint8_t minDutyIndex = 0;
    if (dutyA < dutyB) {
        if (dutyA < dutyC) {
            minDutyIndex = 0;
        } else {
            minDutyIndex = 2;
        }
    } else {
        if (dutyB < dutyC) {
            minDutyIndex = 1;
        } else {
            minDutyIndex = 2;
        }
    }
    if (minDutyIndex == 0) {
        IA = -_IB - _IC;
        IB = _IB;
        IC = _IC;
    } else if (minDutyIndex == 1) {
        IA = _IA;
        IB = -_IA - _IC;
        IC = _IC;
    } else {
        IA = _IA;
        IB = _IB;
        IC = -_IA - _IB;
    }
}