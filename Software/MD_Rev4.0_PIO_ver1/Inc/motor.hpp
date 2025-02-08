#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "main.h"
#include "tim.h"

// モーターに関する定数
#define MOTOR_POLE_PAIRS 7
#define KV 400

// モーター制御全般に関する定数
#define CONTROL_PERIOD 1000  // us
#define ADVANCED_ANGLE PI_2
#define MOTOR_CALIBRATION_SAMPLE_NUM 20
// #define RPM_CALC_PERIOD 2000  // us
#define RPM_FILTER_WINDOW_SIZE 10
#define RPM_P_GAIN 0.002f
#define RPM_I_GAIN 0.0000005f
#define RPM_D_GAIN 0.00005f
#define CURRENT_P_GAIN 0.013f
#define CURRENT_I_GAIN 0.0f
#define CURRENT_D_GAIN 0.7f

// モータードライバーに関する定数
#define PWM_FREQ 100  // kHz
#define PWM_RES 1000

// エンコーダーに関する定数
// #define ENC_UPDATE_PERIOD 200  // us
#define ENC_RES 4096
#define ENC_ADDR 0x36
#define ENC_REG_RAW_ANGLE 0x0C

// 電流センサに関する定数
// #define CURRENT_SAMPLE_PERIOD 500  // us
#define ADC2_RES 16384
#define ADC2_VREF 3.3
#define VREF_SAMPLE_NUM 50
#define CURRENTS_FILTER_WINDOW_SIZE 5
#define CURRENT_NET_FILTER_WINDOW_SIZE 5

// その他
#define PI 3.14159265358979323846f
#define TWO_PI 2.0f * PI
#define PI_2 PI / 2.0f
#define PI_3 PI / 3.0f
#define PI_4 PI / 4.0f
#define PI_6 PI / 6.0f
#define TWO_PI_3 2.0f * PI / 3.0f
#define FOUR_PI_3 4.0f * PI / 3.0f
#define FIVE_PI_3 5.0f * PI / 3.0f
#define SIN_TABLE_SIZE 1000

class Motor {
   public:
    // モーター制御
    void initAll();
    void motorControlUpdate();
    float calcPhase(float elecAngle, bool turn);
    void driveSquareWave(uint16_t duty, float phase);
    void driveSinWave(uint16_t duty, float phase);
    void driveVector(float duty, float phase);
    void motorCalibration();
    void calcRpm();
    void pidRpmControl(float targetRpm);
    void currentControl(float targetCurrent);

    float elecAngle = 0.0f;
    bool turn = 0;  // 0:正転, 1:逆転
    float phase = 0.0f;
    float rpm = 0.0f;
    int16_t encDiffBuf[RPM_FILTER_WINDOW_SIZE] = {0};
    uint8_t encDiffBufIdx = 0;
    float targetRpm = 0.0f;
    uint16_t duty = 200.0f;
    float targetCurrent = 0.0f;
    uint16_t sinDuty[3] = {0};
    uint16_t vectorDuty[3] = {0};

    // モータードライバー
    void outputPWM(uint16_t duty[6]);  // duty:HA, LA, HB, LB, HC, LC

    // エンコーダー
    void readEncoder();
    void updateEncVal();

    uint16_t encRawVal = 0;
    uint16_t encVal = 0;

    // 電流センサ
    void calcCurrent();

    uint16_t adcVal[3] = {0};  // A, B, C
    float vref[3] = {0.0f};
    float current[3] = {0};  // IA, IB, IC
    const float CURRENT_GAIN[4] = {1.0, 1.0f, 1.0f, 1.0f};
    float currentNet = 0.0f;
    float i_d = 0.0f;
    float i_q = 0.0f;

    // その他
    float fastSinf(float radian);

   private:
    // 関数
    // モーター制御
    const float ELEC_ANGLE = (float)ENC_RES / (float)MOTOR_POLE_PAIRS;

    // モータードライバー
    void initDriver();

    // エンコーダー
    void initEncoder();

    uint16_t encValOffset = 0;

    // 電流センサ
    void initCurrentSensor();
    void measureVref();

    float adcValBuf[3][CURRENTS_FILTER_WINDOW_SIZE] = {0};
    uint16_t adcValBufIdx = 0;
    float currentNetBuf[CURRENT_NET_FILTER_WINDOW_SIZE] = {0};
    uint16_t currentNetBufIdx = 0;

    // その他
    void fastSinfInit();

    float sin_table[SIN_TABLE_SIZE];
    const float STEP = TWO_PI / SIN_TABLE_SIZE;
};

#endif  // MOTOR_HPP