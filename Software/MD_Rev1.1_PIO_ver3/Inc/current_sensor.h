#ifndef CURRENT_SENSOR_HPP
#define CURRENT_SENSOR_HPP

#include "main.h"

class CurrentSensor {
   public:
    CurrentSensor() {}
    ~CurrentSensor() {}

    void init();
    void update();
    void calibrate();
    float getCurrent(uint8_t phase);

   private:
    const float GAIN[3] = {-592.426044f, -548.231123f, -545.0388918f};
    uint16_t adcVal[3] = {0};
    float current[3] = {0.0f};
    float currentOffset[3] = {0.0f};
};

#endif  // CURRENT_SENSOR_HPP