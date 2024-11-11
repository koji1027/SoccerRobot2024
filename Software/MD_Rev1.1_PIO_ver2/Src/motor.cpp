#include "motor.hpp"

#include <stdio.h>

void Motor::init() {
    enc.init();
    driver.init();

    // enc.setEncOffsetVal(233);

    this->calibrate();
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
    enc.updateEncVal();
    uint16_t encVal = enc.getEncVal();
    uint16_t singleTurn = enc.encMax / rotorPolesNum;
    this->electricalAngle = (float)(encVal % singleTurn) / (float)singleTurn * 360.0f;
}

void Motor::setAdvancedAngle(float angleTurn0, float angleTurn1) {
    advancedAngle[0] = angleTurn0;
    advancedAngle[1] = angleTurn1;
}

void Motor::calPhase(bool turn) {
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