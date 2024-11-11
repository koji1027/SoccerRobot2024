#include "motor.hpp"

#include <math.h>
#include <stdio.h>

#include "tim.h"

#define M_PI 3.14159265358979323846

Motor::Motor() {
    encOffset = 233;
    // encOffset = 0;
    advancedAngle[0] = -180.0f / 180.0f * M_PI;
    advancedAngle[1] = 135.0f / 180.0f * M_PI;
    motorDriveFlag = false;
    encRawValue = 0;
    encValue = 0;
    rpm[0] = 0.0f;
    rpm[1] = 0.0f;
    rpm[2] = 0.0f;
    periodSlow[0] = 5;    // 5ms
    periodSlow[1] = 25;   // 25ms
    periodSlow[2] = 125;  // 125ms
    periodFast[0] = 1;    // 1ms
    periodFast[1] = 5;    // 5ms
    periodFast[2] = 20;   // 25ms
    correctedRpm = 0.0f;
    targetRpm = 300.0f;
    power = 0;
    speedFlag = 0;
    maxDuty = 500;
}

void Motor::begin() {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    this->brake();
}

void Motor::setAdvancedAngle(float angle0, float angle1) {
    advancedAngle[0] = angle0;
    advancedAngle[1] = angle1;
}

void Motor::setEncOffset(uint16_t offset) {
    encOffset = offset;
}

void Motor::calibrateEnc(uint16_t& _encRawValue) {
    uint32_t encValueSumTurn1 = 0;
    uint16_t encOffsetTurn1 = 0;
    for (int k = 0; k < 3; k++) {
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 6; j++) {
                driveSquareWave(j, 300);
                HAL_Delay(5);
            }
        }
        // printf("%d\n", _encRawValue);
        encValueSumTurn1 += _encRawValue;
    }
    encOffsetTurn1 = (uint16_t)((float)encValueSumTurn1 / 3.0f);

    HAL_Delay(200);

    uint32_t encValueSumTurn0 = 0;
    uint16_t encOffsetTurn0 = 0;
    for (int k = 0; k < 3; k++) {
        for (int i = 0; i < 7; i++) {
            for (int j = 10; j > 4; j--) {
                driveSquareWave(j % 6, 300);
                HAL_Delay(5);
            }
        }
        encValueSumTurn0 += _encRawValue;
    }
    encOffsetTurn0 = (uint16_t)((float)encValueSumTurn0 / 3.0f);

    encOffset = (float)(encOffsetTurn0 + encOffsetTurn1) / 2.0f;
    printf("encOffset: %d\n", encOffset);

    HAL_Delay(200);

    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);  // HA
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);  // LA
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);  // HB
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);  // LB
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);  // HC
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);  // LC
}

void Motor::driveSquareWave(uint8_t state, uint16_t power) {
    if (state == 0) {                                            // A -> B
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, power);      // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, power);      // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);          // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);          // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, power / 2);  // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, power / 2);  // LC
    } else if (state == 1) {                                     // A -> C
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, power);      // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, power);      // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, power / 2);  // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, power / 2);  // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);          // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);          // LC
    } else if (state == 2) {                                     // B -> C
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, power / 2);  // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, power / 2);  // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, power);      // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, power);      // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);          // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);          // LC
    } else if (state == 3) {                                     // B -> A
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);          // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);          // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, power);      // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, power);      // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, power / 2);  // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, power / 2);  // LC
    } else if (state == 4) {                                     // C -> A
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);          // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);          // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, power / 2);  // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, power / 2);  // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, power);      // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, power);      // LC
    } else if (state == 5) {                                     // C -> B
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, power / 2);  // HA
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, power / 2);  // LA
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);          // HB
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);          // LB
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, power);      // HC
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, power);      // LC
    }
}

void Motor::driveSinWave(float phase, uint16_t power) {
    uint16_t a = (uint16_t)(power / 2.0f) * (1.0f + sinf(phase));
    uint16_t b = (uint16_t)(power / 2.0f) * (1.0f + sinf(phase - 2.0f * M_PI / 3.0f));
    uint16_t c = (uint16_t)(power / 2.0f) * (1.0f + sinf(phase + 2.0f * M_PI / 3.0f));
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, a);  // HA
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, a);  // LA
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, b);  // HB
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, b);  // LB
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, c);  // HC
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, c);  // LC
}

void Motor::updateEncValue(uint16_t _encValue) {
    encRawValue = _encValue;
    encValue = (4096 + _encValue - encOffset) % 4096;
}

void Motor::brake() {
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);  // HA
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);  // LA
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);  // HB
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);  // LB
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);  // HC
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);  // LC

    correctedRpm = 0.0f;
}

void Motor::calRpm(uint16_t calRpmCnt) {
    if (!speedFlag) {                          // 低速時
        if (calRpmCnt % periodSlow[0] == 0) {  // 5ms
            static uint16_t prevEncValue = 0;
            int32_t encValueDiff = prevEncValue - encValue;
            if (encValueDiff > 2048) {
                encValueDiff -= 4096;
            } else if (encValueDiff < -2048) {
                encValueDiff += 4096;
            }
            prevEncValue = encValue;
            float _rpm = (float)encValueDiff / 4096.0f * 60.0f / (float)periodSlow[0] * 1000.0f;
            static float prevRpm[4] = {0.0f};
            float rpmSum = _rpm;
            for (int i = 0; i < 3; i++) {
                rpmSum += prevRpm[i];
                prevRpm[i] = prevRpm[i + 1];
            }
            rpmSum += prevRpm[3];
            prevRpm[3] = _rpm;
            rpm[0] = rpmSum / 5.0f;
        }
        if (calRpmCnt % periodSlow[1] == 0) {  // 25ms
            static uint16_t prevEncValue = 0;
            int32_t encValueDiff = prevEncValue - encValue;
            if (encValueDiff > 2048) {
                encValueDiff -= 4096;
            } else if (encValueDiff < -2048) {
                encValueDiff += 4096;
            }
            prevEncValue = encValue;
            float _rpm = (float)encValueDiff / 4096.0f * 60.0f / (float)periodSlow[1] * 1000.0f;
            static float prevRpm[4] = {0.0f};
            float rpmSum = _rpm;
            for (int i = 0; i < 3; i++) {
                rpmSum += prevRpm[i];
                prevRpm[i] = prevRpm[i + 1];
            }
            rpmSum += prevRpm[3];
            prevRpm[3] = _rpm;
            rpm[1] = rpmSum / 5.0f;
        }
        if (calRpmCnt % periodSlow[2] == 0) {  // 125ms
            static uint16_t prevEncValue = 0;
            int32_t encValueDiff = prevEncValue - encValue;
            if (encValueDiff > 2048) {
                encValueDiff -= 4096;
            } else if (encValueDiff < -2048) {
                encValueDiff += 4096;
            }
            prevEncValue = encValue;
            float _rpm = (float)encValueDiff / 4096.0f * 60.0f / (float)periodSlow[2] * 1000.0f;
            static float prevRpm[4] = {0.0f};
            float rpmSum = _rpm;
            for (int i = 0; i < 3; i++) {
                rpmSum += prevRpm[i];
                prevRpm[i] = prevRpm[i + 1];
            }
            rpmSum += prevRpm[3];
            prevRpm[3] = _rpm;
            rpm[2] = rpmSum / 5.0f;
        }
    } else {                                   // 高速時
        if (calRpmCnt % periodFast[0] == 0) {  // 5ms
            static uint16_t prevEncValue = 0;
            int32_t encValueDiff = prevEncValue - encValue;
            if (encValueDiff > 2048) {
                encValueDiff -= 4096;
            } else if (encValueDiff < -2048) {
                encValueDiff += 4096;
            }
            prevEncValue = encValue;
            float _rpm = (float)encValueDiff / 4096.0f * 60.0f / (float)periodFast[0] * 1000.0f;
            static float prevRpm[4] = {0.0f};
            float rpmSum = _rpm;
            for (int i = 0; i < 3; i++) {
                rpmSum += prevRpm[i];
                prevRpm[i] = prevRpm[i + 1];
            }
            rpmSum += prevRpm[3];
            prevRpm[3] = _rpm;
            rpm[0] = rpmSum / 5.0f;
        }
        if (calRpmCnt % periodFast[1] == 0) {  // 25ms
            static uint16_t prevEncValue = 0;
            int32_t encValueDiff = prevEncValue - encValue;
            if (encValueDiff > 2048) {
                encValueDiff -= 4096;
            } else if (encValueDiff < -2048) {
                encValueDiff += 4096;
            }
            prevEncValue = encValue;
            float _rpm = (float)encValueDiff / 4096.0f * 60.0f / (float)periodFast[1] * 1000.0f;
            static float prevRpm[4] = {0.0f};
            float rpmSum = _rpm;
            for (int i = 0; i < 3; i++) {
                rpmSum += prevRpm[i];
                prevRpm[i] = prevRpm[i + 1];
            }
            rpmSum += prevRpm[3];
            prevRpm[3] = _rpm;
            rpm[1] = rpmSum / 5.0f;
        }
        if (calRpmCnt % periodFast[2] == 0) {  // 125ms
            static uint16_t prevEncValue = 0;
            int32_t encValueDiff = prevEncValue - encValue;
            if (encValueDiff > 2048) {
                encValueDiff -= 4096;
            } else if (encValueDiff < -2048) {
                encValueDiff += 4096;
            }
            prevEncValue = encValue;
            float _rpm = (float)encValueDiff / 4096.0f * 60.0f / (float)periodFast[2] * 1000.0f;
            static float prevRpm[4] = {0.0f};
            float rpmSum = _rpm;
            for (int i = 0; i < 3; i++) {
                rpmSum += prevRpm[i];
                prevRpm[i] = prevRpm[i + 1];
            }
            rpmSum += prevRpm[3];
            prevRpm[3] = _rpm;
            rpm[2] = rpmSum / 5.0f;
        }
    }

    // if (calRpmCnt % 100 == 0) {
    //     printf("rpm: %d, %d, %d, %d\n", (int)rpm[0], (int)rpm[1], (int)rpm[2], (int)correctedRpm);
    // }
}

void Motor::correctRpm() {
    // rpm[0], rpm[1], rpm[2] から信頼できるrpmを計算
    // 基本的に3つの値の平均値を使用する
    float rpmMax = rpm[0];
    float rpmMid = rpm[0];
    float rpmMin = rpm[0];
    if (rpmMax >= 0 && rpmMid >= 0 && rpmMin >= 0) {
        for (int i = 1; i < 2; i++) {
            if (rpm[i] > rpmMax) {
                rpmMax = rpm[i];
            } else if (rpm[i] < rpmMin) {
                rpmMin = rpm[i];
            } else {
                rpmMid = rpm[i];
            }
        }
        if (abs(rpmMax / rpmMid) < 1.1f) {
            if (abs(rpmMid / rpmMin) < 1.1f) {
                correctedRpm = (rpmMax + rpmMid + rpmMin) / 3.0f;
            } else {
                correctedRpm = (rpmMax + rpmMid) / 2.0f;
            }
        } else {
            if (abs(rpmMid / rpmMin) < 1.1f) {
                correctedRpm = (rpmMid + rpmMin) / 2.0f;
            }
            // それ以外の場合は信頼できるrpmがないので何もしない
        }
    } else if (rpmMax < 0 && rpmMid < 0 && rpmMin < 0) {
        for (int i = 1; i < 2; i++) {
            if (rpm[i] < rpmMax) {
                rpmMax = rpm[i];
            } else if (rpm[i] > rpmMin) {
                rpmMin = rpm[i];
            } else {
                rpmMid = rpm[i];
            }
        }
        if (abs(rpmMax / rpmMid) < 1.1f) {
            if (abs(rpmMid / rpmMin) < 1.1f) {
                correctedRpm = (rpmMax + rpmMid + rpmMin) / 3.0f;
            } else {
                correctedRpm = (rpmMax + rpmMid) / 2.0f;
            }
        } else {
            if (abs(rpmMid / rpmMin) < 1.1f) {
                correctedRpm = (rpmMid + rpmMin) / 2.0f;
            }
            // それ以外の場合は信頼できるrpmがないので何もしない
        }
    }
    if (abs(correctedRpm) > 800.0f) {
        speedFlag = 1;
    } else {
        speedFlag = 0;
    }
}

float Motor::calPhase(Motor* motor) {
    float electricAngle = motor->encValue % 575;
    electricAngle = electricAngle * 2.0f * M_PI / 575.0f;
    float phase = 0.0f;
    if (motor->targetRpm > 0) {
        phase = electricAngle + motor->advancedAngle[0];
    } else {
        phase = electricAngle + motor->advancedAngle[1];
    }
    return phase;
}

uint16_t Motor::calPower(Motor* motor) {
    if (motor->targetRpm == 0.0f) {
        return 0;
    } else {
        float diffRpm = motor->targetRpm - motor->correctedRpm;
        float p = 0.10f * diffRpm;
        static float prevDiffRpm = 0.0f;
        float d = 0.02f * (diffRpm - prevDiffRpm);
        prevDiffRpm = diffRpm;
        if (targetRpm > 0) {
            motor->power += p + d;
        } else {
            motor->power -= p + d;
        }
        if (motor->power > maxDuty) {
            motor->power = maxDuty;
        } else if (motor->power < 0) {
            motor->power = 0;
        }
        // return motor->power;
        return 400;
    }
}