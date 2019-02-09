#include <Servo.h>

Servo steering, motor;

void setup() {
  // put your setup code here, to run once:

  steering.attach(7);
  motor.attach(8);
  motor.writeMicroseconds(1448);
  steering.writeMicroseconds(1412);

  Serial.begin(115200);

  delay(2000);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  int pos;
  for(pos = 1448; pos < 1676; pos++){
    steering.writeMicroseconds(pos);
    Serial.println(pos);
    delay(10);
  }
  delay(2000);

}
