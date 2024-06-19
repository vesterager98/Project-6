#include <ESP32Servo.h>

//Motor control variables
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

const int motorPin1 = 5;
const int motorPin2 = 23;
const int motorPin3 = 22;
const int motorPin4 = 4;

int motorSpeed1 = 0;
int motorSpeed2 = 0;
int motorSpeed3 = 0;
int motorSpeed4 = 0;


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  while (!Serial) {}

  motor1.attach(motorPin1, 1000, 2000);
  motor2.attach(motorPin2, 1000, 2000);
  motor3.attach(motorPin3, 1000, 2000);
  motor4.attach(motorPin4, 1000, 2000);

  motor1.write(0);
  motor2.write(0);
  motor3.write(0);
  motor4.write(0);

  Serial.println("Starting motor test... 10 sec");

  delay(10000);

  motor1.write(180*0.3); //4 4 4 
  delay(1000);
  motor1.write(0);

  Serial.println("Done!");

}

void loop() {
  // put your main code here, to run repeatedly:

}
