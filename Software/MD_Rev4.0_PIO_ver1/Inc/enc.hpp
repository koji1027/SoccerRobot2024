// インクルードガード
// #ifndef ENC_HPP
// #define ENC_HPP

#include "motor.hpp"
#include "stm32g4xx_hal.h"

#define ENC_RES 4096
#define ENC_ADDR 0x36
#define ENC_REG_RAW_ANGLE 0x0C
#define ELEC_ANGLE (float)ENC_RES / (float)MOTOR_POLE_PAIRS

class Encoder {
   public:
    Encoder();
    void init();
    void updateVal();
    void setOffset(uint16_t offset);
    uint16_t getVal(uint8_t type);

   private:
    void readRaw();

    uint16_t rawVal = 0;
    uint16_t val = 0;
    uint16_t offset = 0;

    float elecAngle = 0.0f;
};

// #endif  // ENC_HPP