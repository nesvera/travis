#include "usb_communication.h"
#include "motor.h"
#include "steering.h"
#include "imu.h"

#define STEERING_PIN 7
#define STEERING_MIN_MS 1772
#define STEERING_CENTER_MS 1412
#define STEERING_MAX_MS 1070
#define STEERING_STOP_MS 1412

#define MOTOR_PIN 8
#define MOTOR_MIN_MS 904
#define MOTOR_CENTER_MS 1448
#define MOTOR_MAX_MS 1676
#define MOTOR_STOP_MS 904

UsbCommunication *usb_comm;
Motor *motor;
Steering *steering;
IMU *imu;

float *input_data;
float *output_data;

float steering_value, speed_value, e_stop;

void setup() {

  // Usb communication, protocol
  usb_comm = new UsbCommunication(Serial, 115200, 128);

  // Rear motor with ESC
  motor = new Motor(MOTOR_PIN, MOTOR_MIN_MS, MOTOR_MAX_MS, MOTOR_STOP_MS);

  // Steering wheel servo
  steering = new Steering(STEERING_PIN, STEERING_MIN_MS, STEERING_MAX_MS, STEERING_STOP_MS);

  // MPU6050
  imu = new IMU();

  Serial.println("Starting microcontroller");

  output_data = malloc(10*sizeof(float));

}

void loop() {
  
  // Check if receive data from pc
  usb_comm->check();

  if( usb_comm->hasNewData() ){
    input_data = usb_comm->readMessage();

    steering_value = input_data[0];
    speed_value = input_data[1];
    e_stop = input_data[2];

    //motor->setAcceleration(speed_value, e_stop);

  } 

  output_data[0] = input_data[0];
  output_data[1] = input_data[1];
  output_data[2] = input_data[2];
  output_data[3] = 10;
  output_data[4] = 11;
  output_data[5] = 12;
  output_data[6] = 13;
  output_data[7] = 14;
  output_data[8] = 15;
  output_data[9] = 16;
    
  usb_comm->writeMessage(output_data);

  delay(10);

}

