#ifndef imu_h
#define imu_h

#include <Arduino.h>

class IMU{

  public:
    IMU();

  private:

    // State variables
    float approxSpeed = 0.0;
    volatile uint32_t hallSensorCounter = 0;
    float accelX = 0.0;
    float accelY = 0.0;
    float accelZ = 0.0;
    float gyroRoll = 0.0;
    float gyroPitch = 0.0;
    float gyroYaw = 0.0;

  
};

#endif
