
#include <ESP32Servo.h>


Servo ESC;




void setup() {

pinMode(33,OUTPUT);


  // Attach the ESC on pin 9
  //motor.attach(18, 0, 70);
  //ESC.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds)

  ESC.attach(33, 500, 2500);  // (pin, min pulse width, max pulse width in microseconds)
  delay(1000);
  ESC.write(0);
  delay(5000);
  ESC.write(43);
  delay(1000);
  ESC.write(0);
}

void loop() {
  // put your main code here, to run repeatedly:

}