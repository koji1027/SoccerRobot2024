#include <stdbool.h>
#include <stdio.h>

#include "tim.h"

#define ENCODER_MAX 4095
#define TARGET_DIFF 100
#define TIME_STEP 50e-3f  // 50ms
#define PWM_MAX 4095
#define MOTOR_RADIUS 17.5  // mm
#define PI 3.14159265358979323846
#define Kp 5.0
#define Ki 0.0
#define Kd 0.2
#define DUTY_DIFF_LIMIT 5  // % / 1 cycle
#define DUTY_COUNT_LIMIT 10000
#define TWO_THIRD_PI 2.0f / 3.0f * PI

// 正回転、逆回転の進角制御の進む角度
#define FORWARD_SHIFT_ANGLE -150
#define REVERSE_SHIFT_ANGLE 50

void sinWaveDrive(uint16_t duty, uint16_t enc, bool turn);
float calRPM(uint16_t enc);
// 現在のRPMに応じてDuty比を変更する関数
uint16_t pidControl(float rpm, float targetRpm, uint16_t duty);