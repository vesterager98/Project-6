
#include <ESP32Servo.h>


Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;


void setup() {

  Serial.begin(115200);
  while(!Serial){}
 Serial.println(0);
 ESC1.attach(18, 1000, 2000);  // (pin, min pulse width, max pulse width in microseconds)
 ESC2.attach(19, 1000, 2000);  // (pin, min pulse width, max pulse width in microseconds)
 ESC3.attach(5, 1000, 2000);  // (pin, min pulse width, max pulse width in microseconds)
 ESC4.attach(32, 1000, 2000);  // (pin, min pulse width, max pulse width in microseconds)
  ESC1.write(0);
  ESC2.write(0);
  ESC3.write(0);
  ESC4.write(0);
  delay(5000);
  ESC1.write(30);
  ESC2.write(30);
  ESC3.write(30);
  ESC4.write(30);
  delay(1000);
  ESC1.write(0);
  ESC2.write(0);
  ESC3.write(0);
  ESC4.write(0);



  //ttach the ESC on pin 9
  //motor.attach(18, 0, 70);
  //ESC.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds)


}

void loop() {
  // put your main code here, to run repeatedly:
 
}