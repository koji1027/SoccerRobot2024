#pragma once

#include <stdint.h>

class Driver {
   public:
    Driver(const uint16_t maxDuty) : maxDuty(maxDuty) {}
    ~Driver() {}

    void init();
    void driveSinWave(float phase, uint16_t duty);
    void driveSquareWave(uint8_t step, uint16_t duty);
    void drive(uint16_t ha, uint16_t hb, uint16_t hc, uint16_t la, uint16_t lb, uint16_t lc);

   private:
    const uint16_t maxDuty;

    uint16_t adc2values[2] = {0, 0};
};