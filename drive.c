#pragma once

#include "drive.h"

float calRPM(uint16_t enc) {
    static uint16_t enc_prev = ENCODER_MAX + 1;  // エンコーダは 0 ~ ENCODER_MAX の値を取るので、 ENCODER_MAX + 1 で初期化すると初回かどうかがわかる
    float rpm = 0;
    if (enc_prev != ENCODER_MAX + 1) {
        int32_t enc_diff = enc - enc_prev;
        if (enc_diff > (ENCODER_MAX / 2)) {  // エンコーダの最大値を超えた場合
            enc_diff = ENCODER_MAX - enc_diff;
        } else if (enc_diff < -(ENCODER_MAX / 2)) {  // エンコーダの最小値を超えた場合
            enc_diff = ENCODER_MAX + enc_diff;
        }
        rpm = enc_diff / 4096.0 * 60.0 / TIME_STEP;  // rpmの計算
    }
    enc_prev = enc;
    return -rpm;
}

uint16_t pidControl(float rpm, float targetRpm, uint16_t duty) {
    float rpm_diff = 0;
    if (rpm > 0) {
        rpm_diff = rpm - targetRpm;
    } else {
        rpm_diff = -rpm + targetRpm;
    }
    // P制御
    float p_control = rpm_diff * Kp;
    // D制御
    static float prev_rpm_diff = 0;
    float d_control = (rpm_diff - prev_rpm_diff) * Kd;
    prev_rpm_diff = rpm_diff;
    // I制御
    static float sum_rpm_diff = 0;
    sum_rpm_diff += rpm_diff;
    float i_control = sum_rpm_diff * Ki;

    // デューティ比の変更
    duty -= p_control + d_control + i_control;

    // デューティ比の制限
    if (duty > PWM_MAX) {
        duty = PWM_MAX;
    } else if (duty < 0) {
        duty = 0;
    }

    // デューティ比の平均値の計算（デバッグ用）
    static uint16_t dutyPrev = 0;
    if (dutyPrev == 0) {
        dutyPrev = duty;
    } else {
        if (duty > dutyPrev * (1 + DUTY_DIFF_LIMIT / 100)) {
            duty = dutyPrev * (1 + DUTY_DIFF_LIMIT / 100);
        } else if (duty < dutyPrev * (1 - DUTY_DIFF_LIMIT / 100)) {
            duty = dutyPrev * (1 - DUTY_DIFF_LIMIT / 100);
        }
    }
    static uint16_t dutyArray[DUTY_COUNT_LIMIT];
    static int cnt = 0;  // 平均値の分母
    static int sum = 0;  // 平均値の分子
    if (cnt < DUTY_COUNT_LIMIT) {
        sum += duty;
        dutyArray[cnt] = duty;
        cnt++;
    } else {
        sum -= dutyArray[cnt % DUTY_COUNT_LIMIT];
        sum += duty;
        dutyArray[cnt % DUTY_COUNT_LIMIT] = duty;
        cnt++;
    }
    float dutyAvg = sum / cnt;
    // printf("RPM: %f\tDuty: %d\n", rpm, duty_avg);

    return duty;
}

void sinWaveDrive(uint16_t duty, uint16_t enc, bool turn) {
    float electricAngle = (enc % 585) / 585.0 * 2.0 * PI;  // 1周が4096で、V2806には7組の磁石があるので、電気的な1周は4096/7≒585に相当する。
    float phase;
    if (turn) {  // 正回転
        phase = electricAngle + FORWARD_SHIFT_ANGLE;
    } else {  // 逆回転
        phase = electricAngle + REVERSE_SHIFT_ANGLE;
    }
    float u = 0.5 * duty * (1 + sin(phase));
    float v = 0.5 * duty * (1 + sin(phase + TWO_THIRD_PI));
    float w = 0.5 * duty * (1 + sin(phase - TWO_THIRD_PI));

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)u);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)v);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint16_t)w);
}