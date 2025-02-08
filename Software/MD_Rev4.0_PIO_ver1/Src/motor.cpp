#include "motor.hpp"

#include <stdio.h>

#include <cmath>

#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

// モーター制御
void Motor::initAll() {
    initDriver();
    initEncoder();
    initCurrentSensor();
    fastSinfInit();

    HAL_Delay(200);

    measureVref();
    motorCalibration();

    char msg[] = "Initialized all peripherals\n";
    uint8_t len = sizeof(msg);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)msg, len);

    HAL_Delay(200);
}

float Motor::calcPhase(float elecAngle, bool turn) {
    if (turn) {
        return fmodf(fmodf(elecAngle - ADVANCED_ANGLE, TWO_PI) + TWO_PI, TWO_PI);
    } else {
        return fmodf(fmodf(elecAngle + ADVANCED_ANGLE, TWO_PI) + TWO_PI, TWO_PI);
    }
}

void Motor::driveSquareWave(uint16_t duty, float phase) {
    if (duty > PWM_RES) {
        duty = PWM_RES;
    }
    if (phase >= 0 && phase < PI_3) {  // A -> B
        uint16_t dutyArr[6] = {duty, duty, 0, 0, 0, 1000};
        outputPWM(dutyArr);
    } else if (phase >= PI_3 && phase < TWO_PI_3) {  // A -> C
        uint16_t dutyArr[6] = {duty, duty, 0, 1000, 0, 0};
        outputPWM(dutyArr);
    } else if (phase >= TWO_PI_3 && phase < PI) {  // B -> C
        uint16_t dutyArr[6] = {0, 1000, duty, duty, 0, 0};
        outputPWM(dutyArr);
    } else if (phase >= PI && phase < FOUR_PI_3) {  // B -> A
        uint16_t dutyArr[6] = {0, 0, duty, duty, 0, 1000};
        outputPWM(dutyArr);
    } else if (phase >= FOUR_PI_3 && phase < FIVE_PI_3) {  // C -> A
        uint16_t dutyArr[6] = {0, 0, 0, 1000, duty, duty};
        outputPWM(dutyArr);
    } else {  // C -> B
        uint16_t dutyArr[6] = {0, 1000, 0, 0, duty, duty};
        outputPWM(dutyArr);
    }
}

void Motor::driveSinWave(uint16_t duty, float phase) {
    if (duty > PWM_RES) {
        duty = PWM_RES;
    }
    uint16_t dutyArr[6] = {0};
    for (int i = 0; i < 3; i++) {
        dutyArr[2 * i] = (float)duty * (sinf(phase + (float)i * TWO_PI / 3.0f) + 1.0f) / 2.0f;
        dutyArr[2 * i + 1] = dutyArr[2 * i];
    }

    sinDuty[0] = dutyArr[0];
    sinDuty[1] = dutyArr[2];
    sinDuty[2] = dutyArr[4];

    outputPWM(dutyArr);
}

void Motor::driveVector(float duty, float phase) {
    phase *= -1.0f;
    float vol_d = 0.0f;
    float vol_q = 4.0f / 1000.0f * duty;

    float vol_alpha = vol_d * cosf(phase) - vol_q * sinf(phase);
    float vol_beta = vol_d * sinf(phase) + vol_q * cosf(phase);

    float vol_u = 0.81649658 * vol_alpha;
    float vol_v = -0.40824829 * vol_alpha + 0.707106781 * vol_beta;
    float vol_w = -0.40824829 * vol_alpha - 0.707106781 * vol_beta;

    uint16_t dutyArr[6] = {0};
    dutyArr[0] = 500 + vol_u / 8.0f * 1000;
    dutyArr[1] = 500 + vol_u / 8.0f * 1000;
    dutyArr[2] = 500 + vol_v / 8.0f * 1000;
    dutyArr[3] = 500 + vol_v / 8.0f * 1000;
    dutyArr[4] = 500 + vol_w / 8.0f * 1000;
    dutyArr[5] = 500 + vol_w / 8.0f * 1000;

    vectorDuty[0] = dutyArr[0];
    vectorDuty[1] = dutyArr[2];
    vectorDuty[2] = dutyArr[4];

    outputPWM(dutyArr);
}

void Motor::motorControlUpdate() {
    static uint32_t cnt = 0;
    cnt += 10;
    if (cnt >= 1000000) {
        cnt = 0;
    }
    if (cnt % CONTROL_PERIOD == 0) {
        calcCurrent();
        calcRpm();
        duty = 300;
        phase = calcPhase(elecAngle, turn);
        driveSinWave(duty, phase);

        char msg[200];
        uint8_t len = sprintf(msg, "%.1f,%.1f\n", i_d, i_q);
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)msg, len);
    } else if (cnt % CONTROL_PERIOD == CONTROL_PERIOD / 2) {
        updateEncVal();  // モーターのメインの処理と半周期ずらす
    }
}

void Motor::motorCalibration() {
    HAL_Delay(100);
    driveSquareWave(500, 0.08f);
    HAL_Delay(100);
    int32_t encValSum = 0;
    for (int i = 0; i < MOTOR_CALIBRATION_SAMPLE_NUM; i++) {
        readEncoder();
        encValSum += encVal;
        HAL_Delay(5);
    }
    driveSquareWave(500, 6.20f);
    HAL_Delay(100);
    for (int i = 0; i < MOTOR_CALIBRATION_SAMPLE_NUM; i++) {
        readEncoder();
        encValSum -= encVal;
        HAL_Delay(5);
    }
    encValOffset = encValSum / (2 * MOTOR_CALIBRATION_SAMPLE_NUM);
}

void Motor::calcRpm() {
    static uint16_t encValPrev = 0;
    encDiffBuf[encDiffBufIdx] = encVal - encValPrev;
    if (encDiffBuf[encDiffBufIdx] > ENC_RES / 2) {
        encDiffBuf[encDiffBufIdx] -= ENC_RES;
    } else if (encDiffBuf[encDiffBufIdx] < -ENC_RES / 2) {
        encDiffBuf[encDiffBufIdx] += ENC_RES;
    }
    encDiffBufIdx = (encDiffBufIdx + 1) % RPM_FILTER_WINDOW_SIZE;
    encValPrev = encVal;
    int32_t encDiffSum = 0;
    for (int i = 0; i < RPM_FILTER_WINDOW_SIZE; i++) {
        encDiffSum += encDiffBuf[i];
    }
    rpm = (float)encDiffSum / (RPM_FILTER_WINDOW_SIZE * CONTROL_PERIOD) * 60000000.0f / ENC_RES;
}

void Motor::pidRpmControl(float targetRpm) {
    static float integral = 0.0f;
    static float prevError = 0.0f;
    float error = targetRpm - rpm;
    integral += error;
    float derivative = error - prevError;
    duty += RPM_P_GAIN * error + RPM_I_GAIN * integral + RPM_D_GAIN * derivative;
    if (duty > PWM_RES) {
        duty = PWM_RES;
    } else if (duty < 0) {
        duty = 0;
    }
    prevError = error;
}

void Motor::currentControl(float targetCurrent) {
    static float integral = 0.0f;
    static float prevError = 0.0f;
    float error = targetCurrent - currentNet;
    integral += error;
    float derivative = error - prevError;
    duty += CURRENT_P_GAIN * error + CURRENT_I_GAIN * integral + CURRENT_D_GAIN * derivative;
    if (duty > PWM_RES) {
        duty = PWM_RES;
    } else if (duty < 0) {
        duty = 0;
    }
    prevError = error;
}

// モータードライバー
void Motor::initDriver() {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);  // HA
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // LA
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  // HB
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // LB
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // HC
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // LC
}

void Motor::outputPWM(uint16_t duty[6]) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, duty[0]);  // HA
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty[1]);  // LA
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty[2]);  // HB
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty[3]);  // LB
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty[4]);  // HC
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty[5]);  // LC
}

// エンコーダー
void Motor::initEncoder() {
    // エンコーダーの設定
}

void Motor::readEncoder() {
    // エンコーダーの値を読み取る
    uint8_t data[2];
    HAL_I2C_Mem_Read(&hi2c2, ENC_ADDR << 1, ENC_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT, data, 2, 1);
    encRawVal = (data[0] << 8) | data[1];
}

void Motor::updateEncVal() {
    readEncoder();
    encVal = ((-encRawVal + encValOffset) % ENC_RES + ENC_RES) % ENC_RES;
    elecAngle = fmodf(fmodf((float)encVal, (float)ELEC_ANGLE) + (float)ELEC_ANGLE, (float)ELEC_ANGLE) / (float)ELEC_ANGLE * TWO_PI;
}

// 電流センサ
void Motor::initCurrentSensor() {
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adcVal, 3);
}

void Motor::calcCurrent() {
    // 電流センサの値を読み取る
    float currentAvg = 0.0f;
    for (int i = 0; i < 3; i++) {
        adcValBuf[i][adcValBufIdx] = (float)adcVal[i] - vref[i];
        adcValBufIdx = (adcValBufIdx + 1) % CURRENTS_FILTER_WINDOW_SIZE;
        int32_t adcValSum = 0;
        for (int j = 0; j < CURRENTS_FILTER_WINDOW_SIZE; j++) {
            adcValSum += adcValBuf[i][j];
        }
        current[i] = (float)adcValSum / (CURRENTS_FILTER_WINDOW_SIZE * ADC2_RES) * ADC2_VREF * 1000.0f / CURRENT_GAIN[i];  // mA
        currentAvg += current[i];
    }
    currentAvg /= 3.0f;
    for (int i = 0; i < 3; i++) {
        current[i] -= currentAvg;
    }

    // uint8_t currentMinIdx = 0;
    // for (int i = 1; i < 3; i++) {
    //     if (current[i] < current[currentMinIdx]) {
    //         currentMinIdx = i;
    //     }
    // }
    // current[currentMinIdx] = -current[0] - current[1] - current[2] + current[currentMinIdx];

    float currentNetPositive = 0.0f;
    float currentNetNegative = 0.0f;
    for (int i = 0; i < 3; i++) {
        if (current[i] > 0) {
            currentNetPositive += current[i];
        } else {
            currentNetNegative += current[i];
        }
    }
    currentNetBuf[currentNetBufIdx] = (currentNetPositive - currentNetNegative) / 2.0f / CURRENT_GAIN[3];
    currentNetBufIdx = (currentNetBufIdx + 1) % CURRENT_NET_FILTER_WINDOW_SIZE;
    int32_t currentNetSum = 0;
    for (int i = 0; i < CURRENT_NET_FILTER_WINDOW_SIZE; i++) {
        currentNetSum += currentNetBuf[i];
    }
    currentNet = (float)currentNetSum / CURRENT_NET_FILTER_WINDOW_SIZE;

    float i_alpha = 1.22474487f * (current[1] - current[0] / 2.0f - current[2] / 2.0f);
    float i_beta = 1.22474487f * (current[0] * 0.8660254 - current[2] * 0.8660254);

    i_d = i_alpha * cosf(phase) + i_beta * sinf(phase);
    i_q = -i_alpha * sinf(phase) + i_beta * cosf(phase);
}

void Motor::measureVref() {
    uint32_t adcValSum[3] = {0};
    for (int i = 0; i < VREF_SAMPLE_NUM; i++) {
        for (int j = 0; j < 3; j++) {
            adcValSum[j] += adcVal[j];
        }
        HAL_Delay(2);
    }
    for (int i = 0; i < 3; i++) {
        vref[i] = (float)adcValSum[i] / VREF_SAMPLE_NUM;
    }
}

// その他
// 初期化関数
void Motor::fastSinfInit() {
    for (size_t i = 0; i < SIN_TABLE_SIZE; ++i) {
        sin_table[i] = sinf(i * STEP);
    }
}

// 高速正弦関数
float Motor::fastSinf(float radian) {
    // 入力値の正規化
    float radian_normalized = fmodf(radian, TWO_PI);

    // テーブルインデックス計算
    float index_f = radian_normalized / STEP;
    size_t index = static_cast<size_t>(index_f + 0.5f) % SIN_TABLE_SIZE;

    return sin_table[index];
}