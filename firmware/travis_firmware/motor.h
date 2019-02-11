#ifndef motor_h
#define motor_h

#include <Arduino.h>
#include <Servo.h>

class Motor{

  public:

    Motor(int pin, int ms_min, int ms_max, int ms_center);
    void setAcceleration(float accel_value);
    void setSpeed(float speed_value);
    void stop(void);
    float getAcceleration(void);

  private:

    float acceleration;
    float speed;
    bool e_stop;
    int microseconds_min, microseconds_max, microseconds_stop;

    int pin;
    Servo *motor;
  
};


#endif
