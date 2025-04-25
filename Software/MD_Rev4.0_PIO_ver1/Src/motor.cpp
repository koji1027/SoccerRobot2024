#include "motor.hpp"

#include <stdio.h>

#include <cmath>

#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

// モーター制御
Motor::Motor() {
    enc = Encoder();
}

void Motor::initAll() {
    initDriver();
    enc.init();
    initCurrentSensor();
    fastSinfInit();
    fastCosfInit();

    HAL_Delay(200);

    measureVref();
    motorCalibration();
    // encValOffset = 550;

    HAL_Delay(2000);

    char msg[500];
    uint8_t len = sprintf(msg, "Initialized all peripherals. Encpder offset: %d\n", enc.getVal(2));
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)msg, len);

    HAL_Delay(2000);
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
        dutyArr[2 * i] = ((float)duty * sinf(phase - (float)i * TWO_PI / 3.0f) + PWM_RES) / 2.0f;
        dutyArr[2 * i + 1] = dutyArr[2 * i];
    }

    outputPWM(dutyArr);
}

void Motor::motorControlUpdate() {
    static uint32_t micros = 0;
    static uint32_t millis = 0;
    static uint32_t seconds = 0;
    micros += 100;
    if (micros >= 1000) {
        micros = 0;
        millis++;
    }
    if (millis >= 1000) {
        millis = 0;
        seconds++;
    }

    if ((1000 * millis + micros) % CONTROL_PERIOD == 0) {
        calcCurrent();
        enc.updateVal();
        calcRpm();
        phase = calcPhase(elecAngle, turn);
        duty = 150;
        driveSinWave(duty, phase);

        char msg[200];
        uint8_t len = sprintf(msg, "%f\n", rpm);
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)msg, len);
    }
}

void Motor::motorCalibration() {
    int32_t encValSum = 0;

    for (int k = 0; k < MOTOR_CALIBRATION_SAMPLE_NUM; k++) {
        for (int i = 1; i < 6 * MOTOR_POLE_PAIRS + 1; i++) {
            driveSquareWave(150, (float)(i % 6) * PI_3);
            HAL_Delay(3);
        }
        enc.updateVal();
        HAL_Delay(3);
        encValSum += enc.getVal(0);
    }
    HAL_Delay(100);
    for (int k = 0; k < MOTOR_CALIBRATION_SAMPLE_NUM; k++) {
        for (int i = 6 * MOTOR_POLE_PAIRS - 1; i > -1; i--) {
            driveSquareWave(150, (float)(i % 6) * PI_3);
            HAL_Delay(3);
        }
        enc.updateVal();
        HAL_Delay(3);
        encValSum += enc.getVal(0);
    }

    enc.setOffset(encValSum / (2 * MOTOR_CALIBRATION_SAMPLE_NUM));
}

void Motor::calcRpm() {
    static uint16_t encValPrev = 0;
    encDiffBuf[encDiffBufIdx] = enc.getVal(1) - encValPrev;
    if (encDiffBuf[encDiffBufIdx] > ENC_RES / 2) {
        encDiffBuf[encDiffBufIdx] -= ENC_RES;
    } else if (encDiffBuf[encDiffBufIdx] < -ENC_RES / 2) {
        encDiffBuf[encDiffBufIdx] += ENC_RES;
    }
    encDiffBufIdx = (encDiffBufIdx + 1) % RPM_FILTER_WINDOW_SIZE;
    encValPrev = enc.getVal(1);
    int32_t encDiffSum = 0;
    for (int i = 0; i < RPM_FILTER_WINDOW_SIZE; i++) {
        encDiffSum += encDiffBuf[i];
    }
    rpm = (float)encDiffSum / (RPM_FILTER_WINDOW_SIZE * CONTROL_PERIOD) * 60000000.0f / ENC_RES;
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

void Motor::fastCosfInit() {
    for (size_t i = 0; i < SIN_TABLE_SIZE; ++i) {
        cos_table[i] = cosf(i * STEP);
    }
}

// 高速正弦関数
float Motor::fastSinf(float radian) {
    // 入力値の正規化
    float radian_normalized = fmodf(fmodf(radian, TWO_PI) + TWO_PI, TWO_PI);

    // テーブルインデックス計算
    float index_f = radian_normalized / STEP;
    size_t index = static_cast<size_t>(index_f + 0.5f) % SIN_TABLE_SIZE;

    return sin_table[index];
}

float Motor::fastCosf(float radian) {
    // 入力値の正規化
    float radian_normalized = fmodf(fmodf(radian, TWO_PI) + TWO_PI, TWO_PI);

    // テーブルインデックス計算
    float index_f = radian_normalized / STEP;
    size_t index = static_cast<size_t>(index_f + 0.5f) % SIN_TABLE_SIZE;

    return cos_table[index];
}