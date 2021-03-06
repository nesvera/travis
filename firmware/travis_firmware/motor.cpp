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

  if( accel_value > 0 ){
    this->acceleration = min(accel_value, 1);
  }else{
    this->acceleration = max(accel_value, -1);
  }

  // acceleration range -1 (brake/reverse) -> 1 (forward)
  // multiply for 100 to work with integers
  int microseconds_value = this->microseconds_stop;
  if( accel_value > 0){
    microseconds_value = map((this->acceleration*100), 0, 100, this->microseconds_min, this->microseconds_max);
  }else{
    microseconds_value = map((this->acceleration*100), -100, 0, this->microseconds_stop, this->microseconds_min);
  }

  this->motor->writeMicroseconds(microseconds_value);
  
}

void Motor::stop(void){
  this->motor->writeMicroseconds(this->microseconds_stop);
}

float Motor::getAcceleration(void){
  return this->acceleration;
}

