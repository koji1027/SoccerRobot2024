#include "Arduino.h"
#include "ICM42688.h"

// an ICM42688 object with the ICM42688 sensor on SPI bus 0 and chip select pin 10
ICM42688 IMU(SPI, D7);

void setup() {
    // serial to display data
    Serial.begin(115200);
    while (!Serial) {
    }

    // start communication with IMU
    int status = IMU.begin();
    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while (1) {
        }
    }

    float gZ_sum = 0.0f;
    for (int i = 0; i < 500; i++) {
        IMU.getAGT();
        gZ_sum += IMU.gyrZ();
        delay(10);
    }
    IMU.setGyroBiasZ(gZ_sum / 500.0f);
    Serial.print("Calibration complete\n");
}

void loop() {
    // read the sensor
    IMU.getAGT();

    static float deg = 0.0f;
    static float prev_gz = 0.0f;
    static unsigned long prev_time = micros();
    unsigned long curr_time = micros();
    deg += (IMU.gyrZ() + prev_gz) / 2.0f * (curr_time - prev_time) / 1000000.0f;
    if (deg > 180.0f) {
        deg -= 360.0f;
    } else if (deg <= -180.0f) {
        deg += 360.0f;
    }
    prev_gz = IMU.gyrZ();
    prev_time = curr_time;
    Serial.print(deg, 6);
    Serial.print(" deg\n");
}