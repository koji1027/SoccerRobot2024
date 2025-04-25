#include "motor.hpp"

#include <stdio.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>

void Motor::init() {
    enc.init();
    driver.init();

    enc.setEncOffsetVal(2269);

    // this->calibrate();
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

void Motor::controlDriver(uint8_t mode, uint16_t duty) {
    switch (mode) {
        case 0:
            driver.driveSquareWave(this->phase, duty);
            break;
        case 1:
            driver.driveSinWave(this->phase, duty);
            break;
        default:
            break;
    }
}

float Motor::calRpm(uint16_t currentVal, uint16_t prevVal, uint32_t deltaTime) {
    int16_t delta = static_cast<int16_t>(currentVal) - static_cast<int16_t>(prevVal);

    if (delta > ENCODER_MAX / 2) {
        delta -= ENCODER_MAX;
    } else if (delta < -ENCODER_MAX / 2) {
        delta += ENCODER_MAX;
    }

    float revolutions = static_cast<float>(delta) / ENCODER_MAX;
    float rpm = (revolutions / (deltaTime / 1e6)) * 60.0f;
    return rpm;
}

float Motor::calFinalRpm(const std::vector<float>& rpms) {
    if (rpms.empty()) {
        return this->rpm;
    }

    std::vector<float> sortedRpms = rpms;
    std::sort(sortedRpms.begin(), sortedRpms.end());
    float median = (sortedRpms.size() % 2 == 0)
                       ? (sortedRpms[sortedRpms.size() / 2 - 1] + sortedRpms[sortedRpms.size() / 2]) / 2.0f
                       : sortedRpms[sortedRpms.size() / 2];

    std::vector<float> validRpms;
    for (float tempRpm : rpms) {
        if (std::abs(tempRpm - median) / median < OUTLIER_THRESHOLD) {
            validRpms.push_back(tempRpm);
        }
    }

    if (validRpms.empty()) {
        return this->rpm;
    }

    return std::accumulate(validRpms.begin(), validRpms.end(), 0.0f) / validRpms.size();
}

void Motor::monitorEnc() {
    uint16_t currentEncVal = enc.getEncRawVal();
    timeSinceLastUpdate += TIME_INTERVAL_US;

    bool thresholdExceeded = false;

    provisionalRpms.clear();
    for (int threshold : thresholds) {
        if (std::abs(static_cast<int16_t>(currentEncVal) - static_cast<int16_t>(prevEncVal)) > threshold) {
            float tempRpm = this->calRpm(currentEncVal, prevEncVal, timeSinceLastUpdate);
            provisionalRpms.push_back(tempRpm);
            prevEncVal = currentEncVal;
            timeSinceLastUpdate = 0;
            thresholdExceeded = true;
        }
    }

    if (thresholdExceeded) {
        rpm = calFinalRpm(provisionalRpms);
    } else if (timeSinceLastUpdate > RESET_TIME_US) {
        prevEncVal = currentEncVal;
        timeSinceLastUpdate = 0;
        rpm = 0.0f;
    }

    rpm = calFinalRpm(provisionalRpms);
}

void Motor::setThresholds(const std::vector<int>& newThresholds) {
    thresholds = newThresholds;
}

void Motor::setTimeInterval(float timeInterval) {
    if (timeInterval > 0) {
        TIME_INTERVAL_US = timeInterval;
    }
}

float Motor::getRpm() {
    return rpm;
}