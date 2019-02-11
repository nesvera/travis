#ifndef steering_h
#define steering_h

#include <Arduino.h>
#include <Servo.h>

class Steering{

  public:

    Steering(int pin, int ms_min, int ms_max, int ms_stop);
    void setPosition(float position);
    void stop(void);
    float getPosition(void);

  private:

    float position;
    bool e_stop;
    int microseconds_min, microseconds_max,  microseconds_stop;

    int pin;
    Servo *steering;
      
};

#endif
