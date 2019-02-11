#include "steering.h"

Steering::Steering(int pin, int ms_min, int ms_max, int ms_stop){

  this->microseconds_min = ms_min;
  this->microseconds_max = ms_max;
  this->microseconds_stop = ms_stop;
  this->pin = pin;

  this->steering = new Servo();
  this->steering->attach(this->pin, this->microseconds_min, this->microseconds_max);
  this->steering->writeMicroseconds(this->microseconds_stop);

  this->position = 0;

}

void Steering::setPosition(float position){

  if( position > 0 ){
    this->position = min(position, 1);
  }else{
    this->position = max(position, -1);
  }

  // steering range -1 (left) -> 1 (right)
  // multiply for 100 to work with integers
  int microseconds_value = map((this->position*100), -100, 100, this->microseconds_min, this->microseconds_max);

  this->steering->writeMicroseconds(microseconds_value);
  
}

void Steering::stop(void){
  this->steering->writeMicroseconds(this->microseconds_stop);
}

float Steering::getPosition(void){
  return this->position;
}

