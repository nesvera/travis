#include "motor.h"

Motor::Motor(int pin, int ms_min, int ms_max, int ms_stop){

  this->microseconds_min = ms_min;
  this->microseconds_max = ms_max;
  this->microseconds_stop = ms_stop;
  this->pin = pin;

  this->motor = new Servo();
  this->motor->attach(this->pin, this->microseconds_min, this->microseconds_max);
  this->motor->writeMicroseconds(this->microseconds_stop);

}

void Motor::setAcceleration(float accel_value){

  // acceleration range -1 (brake/reverse) -> 1 (forward)
  // multiply for 100 to work with integers
  int microseconds_value = map((accel_value*100), -100, 100, this->microseconds_min, this->microseconds_max);

  this->motor->writeMicroseconds(microseconds_value);
  
}

void Motor::stop(void){
  this->motor->writeMicroseconds(this->microseconds_stop);
}


