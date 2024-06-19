
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>

#include <ESP32Servo.h>

#define SDA_PIN 18
#define SCL_PIN 19

Adafruit_BMP280 bme;  // I2C
MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

struct vector3 {
  double x, y, z;
};

vector3 downDir = { 0, 0, 1 };
vector3 upRef = { 0, 0, -1 };
vector3 downRef = { 0, 0, 1 };

vector3 gyroDrift = { 0, 0, 0 };

float angleX = 0;
float angleY = 0;

struct ultraPinStruct {
  int trig;
  int echo;
};

const struct ultraPinStruct ultraPin = { .trig = 14, .echo = 27 };  //Ultrasonic sensor

//Motor control variables
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

const int motorPin1 = 5;
const int motorPin2 = 23;
const int motorPin3 = 22;
const int motorPin4 = 4;

int hoverSpeed = 30;
int motorSpeed1 = hoverSpeed;
int motorSpeed2 = hoverSpeed;
int motorSpeed3 = hoverSpeed;
int motorSpeed4 = hoverSpeed;

const unsigned long duration = 5000;
int frameCount = 0;

unsigned long ultraTrigTime = 0;
float distance = 0;
float heightReference = 100;
bool newDistMeas = false;

void IRAM_ATTR ultraSoundISR() {
  switch (digitalRead(ultraPin.echo)) {
    case HIGH:
      ultraTrigTime = micros();
      break;
    
    case LOW:
      int duration = micros() - ultraTrigTime;
      // Calculating the distance
      distance = duration * 0.034 / 2;
      newDistMeas = true;
      break;
    
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

#ifdef _ESP32_HAL_I2C_H_  // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
#else
  Wire.begin();
  mySensor.setWire(&Wire);
#endif

  bme.begin(0x76);
  mySensor.beginAccel();
  mySensor.beginGyro();

  motor1.attach(motorPin1, 1000, 2000);
  motor2.attach(motorPin2, 1000, 2000);
  motor3.attach(motorPin3, 1000, 2000);
  motor4.attach(motorPin4, 1000, 2000);

  motor1.write(0);
  motor2.write(0);
  motor3.write(0);
  motor4.write(0);

  delay(1000);

  //Interrupt for ultrasonic sensor
  attachInterrupt(ultraPin.echo, ultraSoundISR, CHANGE);


  //Initiate pins for ultrasonic sensors
  pinMode(ultraPin.trig, OUTPUT);
  pinMode(ultraPin.echo, INPUT);

  //Calibration
  unsigned long start = micros();
  const unsigned int calibrationTime = 1000000;
  while (micros() - start < calibrationTime) {
    static unsigned long previousTime = micros();

    long delta = micros() - previousTime;
    previousTime = micros();
    vector3 gyroRead = readGyroscope();
    gyroDrift.x += gyroRead.x * delta / 1000000;
    gyroDrift.y += gyroRead.y * delta / 1000000;
    gyroDrift.z += gyroRead.z * delta / 1000000;
  }
  //Makes it drift in degrees per second
  gyroDrift.x /= (micros() - start) / 1000000;
  gyroDrift.y /= (micros() - start) / 1000000;
  gyroDrift.z /= (micros() - start) / 1000000;

  //Calibrate down direction at the start using accelerometer
  downDir = readAccelerometer();

  /*
  setMotorSpeed(motor1, 0);
  setMotorSpeed(motor2, 0);
  setMotorSpeed(motor3, 0);
  setMotorSpeed(motor4, 0);
  */


  delay(1000);

  /*
  motor1.write(30);
  delay(1000);
  motor1.write(0);
  
  motor2.write(30);
  delay(1000);
  motor2.write(0);
  
  motor3.write(30);
  delay(1000);
  motor3.write(0);

  motor4.write(30);
  delay(1000);
  motor4.write(0);
  */


  //kill();


  delay(10000);
}

void loop() {
  static unsigned long startTime = millis();

  static unsigned long lastRun = micros();
  const unsigned long interval = 1000000 / 800;

  if (micros() - lastRun >= interval) {
    lastRun = micros();
    GyroAdjust();

    //Outline
    //Find angle to gravity around x-axis and y-axis
    //Determine difference in power to the motors on each axis
    //Substract or add half the difference from needed power to hover from the correct motors
    //

    vector3 xOrientGrav = downDir;
    xOrientGrav.x = 0;
    vector3 yOrientGrav = downDir;
    yOrientGrav.y = 0;

    angleX = getAngle(xOrientGrav, downRef);  //Gets the absolute angle between the vectors
    if (downDir.y < 0) {                      //Makes it signed
      angleX *= -1;
    }
    angleY = getAngle(yOrientGrav, downRef);
    if (downDir.x < 0) {
      angleY *= -1;
    }
      /*
     /\
    //\\
   / \/ \
   \ /\ /
    \\//
     \/
    */
    

    int xDifference = 0;  //angleControllerX(angleX);
    int yDifference = 0; //angleControllerY(angleY);

    static int ultraCount = 16;

    if (ultraCount >= 16) {
      frameCount++;
      ultraCount = 0;
      measureDist(ultraPin);
    } else {
      ultraCount++;
    }

    if (newDistMeas) {
      hoverSpeed = heightController(heightReference - distance);
      newDistMeas = false;
      Serial.println(hoverSpeed);
      // Prints the distance on the Serial Monitor
      //Serial.print("Distance: ");
      //Serial.println(distance);
    }

    //*
    static int printCount = 110;

    if (printCount >= 100) {
      printCount = 0;
      //Serial.print("X: ");
      //Serial.print(rad2deg(angleX * 1));
      //Serial.print(",");
      //Serial.print("\tY: ");
      //Serial.print(xDifference);
      //Serial.print(",");
      //Serial.print(rad2deg(angleY * 1));
      //Serial.print(",");
      //Serial.print(yDifference);
      //Serial.print(hoverSpeed);
      //Serial.println("");
    } else {
      printCount++;
    }
    //*/


    motorSpeed1 = hoverSpeed + xDifference / 2 + yDifference / 2;
    motorSpeed2 = hoverSpeed - xDifference / 2 + yDifference / 2;
    motorSpeed3 = hoverSpeed + xDifference / 2 - yDifference / 2;
    motorSpeed4 = hoverSpeed - xDifference / 2 - yDifference / 2;


    if (motorSpeed1 < 0) {
      motorSpeed1 = 0;
    }
    if (motorSpeed2 < 0) {
      motorSpeed2 = 0;
    }
    if (motorSpeed3 < 0) {
      motorSpeed3 = 0;
    }
    if (motorSpeed4 < 0) {
      motorSpeed4 = 0;
    }

    motor1.write(motorSpeed1);
    motor2.write(motorSpeed2);
    motor3.write(motorSpeed3);
    motor4.write(motorSpeed4);
  }





  if (millis() - startTime >= duration) {
    kill();
  }

  /*
  static int count = 0;

  if (count > 200) {
    count = 0;
    //*
    Serial.print("\nAngle around x: ");
    Serial.println(rad2deg(angleX));
    Serial.print("Angle around y: ");
    Serial.println(rad2deg(angleY));
    //
    
    Serial.print("\nX-oriented gravity: ");
    Serial.print(xOrientGrav.x);
    Serial.print(", ");
    Serial.print(xOrientGrav.y);
    Serial.print(", ");
    Serial.print(xOrientGrav.z);
    
  } else {
    count++;
  }
  */
}



int angleControllerX(float error) {  //Error is the angle to reference
  //error = deg2rad(error);
  static float outMem[4] = { 0, 0, 0, 0 };
  static float inMem[4] = { 0, 0, 0, 0 };

  //Tiger
  /*
  float out = error * 6.3 - inMem[0] * 25.1167 + inMem[1] * 37.57 - inMem[2] * 24.98 + inMem[3] * 6.23; //Tiger control
  out += outMem[0] * 3.09 - outMem[1] * 5.71 + outMem[2] * 3.71 - outMem[3] * 0.91;
  */


  //MAthias

  //Old
  //float out = error * 200 - inMem[0] * 595.3 + inMem[1] * 590.6 - inMem[2] * 195.3; //Mat control
  //out += outMem[0] * 2.719 - outMem[1] * 2.461 + outMem[2] * 0.7416;

  //New
  //float out = error * 0.2146 - inMem[0] * 0.4284 + inMem[1] * 0.2138;
  //out += outMem[0] * 1.962 - outMem[1] * 0.9619;

  //V3
  //float out = error - inMem[0] * 1.996 + inMem[1] * 0.9961 + outMem[0] * 1.962 - outMem[1] * 0.9619;


  //Sketchy Jonathan controller
  float out = 0.15 * error + 800 / 25 * (error - inMem[2]) / 3;

  //static float integral = 0;
  //float out = 0.15 * error + 800/30*(error - inMem[0]) + 0.01 * integral;
  //integral += error / 800;


  //Mathias PID test
  /*
  static float integral = 0;
  float out = 0.041513 * error + 800 * 0.008 * (error - inMem[0]) + 0.05 * integral;
  
  integral += error / 800;
  */



  inMem[3] = inMem[2];
  inMem[2] = inMem[1];
  inMem[1] = inMem[0];
  inMem[0] = error;

  outMem[3] = outMem[2];
  outMem[2] = outMem[1];
  outMem[1] = outMem[0];
  outMem[0] = out;


  if (out > 1) {
    out = 1;
  } else if (out < -1) {
    out = -1;
  } else {
    //integral -= error / 800;
    //outMem[2] = outMem[1];
    //outMem[1] = outMem[0];
    //outMem[0] = out;
  }

  /* else {
    out += outMem[0] * 2.719 - outMem[1] * 2.461 + outMem[2] * 0.7416;
  }*/



  //Serial.print("n1 is: "); Serial.println(outMem[0]);

  out *= 180;
  return ((int)out);
}

int angleControllerY(float error) {  //Error is the angle to reference
  //error = deg2rad(error);

  static float outMem[4] = { 0, 0, 0, 0 };
  static float inMem[4] = { 0, 0, 0, 0 };

  //Tiger controller
  /*
  float out = error * 6.3 - inMem[0] * 25.1167 + inMem[1] * 37.57 - inMem[2] * 24.98 + inMem[3] * 6.23;
  out += outMem[0] * 3.09 - outMem[1] * 5.71 + outMem[2] * 3.71 - outMem[3] * 0.91;
  */


  //Mathias controller
  //Old
  //float out = error * 200 - inMem[0] * 595.3 + inMem[1] * 590.6 - inMem[2] * 195.3; //Mat control
  //out += outMem[0] * 2.719 - outMem[1] * 2.461 + outMem[2] * 0.7416;

  //New
  //float out = error * 0.2146 - inMem[0] * 0.4284 + inMem[1] * 0.2138;
  //out += outMem[0] * 1.962 - outMem[1] * 0.9619;

  //V3
  //float out = error - inMem[0] * 1.996 + inMem[1] * 0.9961 + outMem[0] * 1.962 - outMem[1] * 0.9619;


  //Sketchy Jonathan controller
  //float out = 0.15 * error + 800/25*(error - inMem[2])/3;

  //static float integral = 0;
  //float out = 0.15 * error + 800/25*(error - inMem[2])/3 + 0.01 * integral;
  //integral += error / 800 / 20;

  //Mathias PID test

  static float integral = 0;
  float out = 0.0044 * error + 800 * 0.004 * (inMem[0] - error) + 0.000044 * integral;
  //float out = 0.044 * error + 800 * 0.004 * (inMem[0] - error) + 0.000044 * integral;

  integral += error / 800;




  inMem[3] = inMem[2];
  inMem[2] = inMem[1];
  inMem[1] = inMem[0];
  inMem[0] = error;

  outMem[3] = outMem[2];
  outMem[2] = outMem[1];
  outMem[1] = outMem[0];
  outMem[0] = out;


  if (out > 1) {
    out = 1;
  } else if (out < -1) {
    out = -1;
  } else {
    integral -= error / 800;
    //outMem[2] = outMem[1];
    //outMem[1] = outMem[0];
    //outMem[0] = out;
  }



  /*else {
    out += outMem[0] * 2.719 - outMem[1] * 2.461 + outMem[2] * 0.7416;
  }*/



  out *= 180;
  return ((int)out);
}


int heightController(float error) {
  error /= 100; //Change from cm to m
  static float outMem[4] = { 0, 0, 0, 0};                                
  static float inMem[4] = { 0, 0, 0, 0};

  static float integral = 0;
  float out = 0.0082 * error + 0.0544 * (error - inMem[0]) + 5.4 * integral;
  integral += error / 100 / 50;

  //float out = error * 9.61 - inMem[0] * 27.66 + inMem[1] * 26.54 - inMem[2] * 8.494;
  //out += outMem[0] * 2.359 - outMem[1] * 1.855 + outMem[2] * 0.486;

  inMem[3] = inMem[2];
  inMem[2] = inMem[1];
  inMem[1] = inMem[0];
  inMem[0] = error;

  outMem[3] = outMem[2];
  outMem[2] = outMem[1];
  outMem[1] = outMem[0];
  outMem[0] = out;

  if (out > 1) {
    out = 1;
  }
  else if (out < 0) {
    out = 0;
  }
  else{
    integral -= error / 100 / 50;
  }

  return ((int)(out * 180));
}



void kill() {
  Serial.println("Motor is kill.");
  Serial.print("Average framerate: ");
  Serial.println(frameCount / (duration / 1000));
  while (true) {
    motor1.write(0);
    motor2.write(0);
    motor3.write(0);
    motor4.write(0);
    /*
    setMotorSpeed(motor1, 0);
    setMotorSpeed(motor2, 0);
    setMotorSpeed(motor3, 0);
    setMotorSpeed(motor4, 0);
    */
  }
}


void setMotorSpeed(Servo motor, int speed) {
  motor.write(speed);
}


void GyroAdjust() {
  if (mySensor.gyroUpdate() == 0) {
    static unsigned long lastTime = 0;

    unsigned long delta = micros() - lastTime;
    lastTime = micros();

    static vector3 angle = { 0, 0, 0 };

    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();

    angle.x = (gX - gyroDrift.x) * delta / 1000000.0;
    angle.y = (gY - gyroDrift.y) * delta / 1000000.0;
    angle.z = (gZ - gyroDrift.z) * delta / 1000000.0;

    downDir = vectorRotateX(downDir, angle.x);
    downDir = vectorRotateY(downDir, angle.y);
    downDir = vectorRotateZ(downDir, angle.z);


    //Serial.print("\ngyroX: " + String(angle.x));
    //Serial.print("\tgyroY: " + String(angle.y));
    //Serial.print("\tgyroZ: " + String(angle.z));
    //delay(10);
    static int count = 0;
    //Serial.print("Angle to gravity: ");
    if (count >= 100) {
      //Serial.println(String(rad2deg(getAngle(upRef, downDir))));
      count = 0;
    } else {
      count++;
    }
  }
}

vector3 readGyroscope() {
  vector3 result = { 0, 0, 0 };

  if (mySensor.gyroUpdate() == 0) {
    result.x = mySensor.gyroX();
    result.y = mySensor.gyroY();
    result.z = mySensor.gyroZ();
  } else {
    result.x = NULL;
  }

  return (result);
}

vector3 readAccelerometer() {
  vector3 result = { 0, 0, 0 };

  if (mySensor.accelUpdate() == 0) {
    result.x = mySensor.accelX();
    result.y = mySensor.accelY();
    result.z = mySensor.accelZ();
  } else {
    result.x = NULL;
  }

  return (result);
}


// __________Helper functions__________

void measureDist(struct ultraPinStruct pin) {  //Returns distance measured on an unltrasonic sensor
  //                             | Pins of ultrasonic sensor to measure on (ultraPinStruct contains two pins: echo and trig)

  //Clear trigger pin
  digitalWrite(pin.trig, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(pin.trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin.trig, LOW);

  //ultraTrigTime = micros();
}

float getLength(vector3 vec) {
  return (sqrt(sq(vec.x) + sq(vec.y) + sq(vec.z)));
}

vector3 normalize(vector3 vec) {
  float length = getLength(vec);

  vector3 result;
  result.x = vec.x / length;
  result.y = vec.y / length;
  result.z = vec.z / length;

  return (result);
}

float normalizedDotProduct(vector3 vec_1, vector3 vec_2) {
  vector3 vec1 = normalize(vec_1);
  vector3 vec2 = normalize(vec_2);
  return (vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z);
}

float getAngle(vector3 vec_1, vector3 vec_2) {  //Returns angle between two vectors
  return (acos(normalizedDotProduct(vec_1, vec_2)));
}

float deg2rad(float deg) {  //Transforms degrees to radians
  return (deg / 360 * 2 * 3.14159265);
}

float rad2deg(float rad) {  //Transforms radians to degrees
  return (rad / (2 * PI) * 360);
}

vector3 vectorRotateX(vector3 vec, float angle) {  //Rotate a vector a certain amount of degrees around the x-axis
  vector3 result;

  float radAngle = deg2rad(angle);

  result.x = vec.x;
  result.y = vec.y * cos(radAngle) - vec.z * sin(radAngle);
  result.z = vec.y * sin(radAngle) + vec.z * cos(radAngle);

  return (result);
}

vector3 vectorRotateY(vector3 vec, float angle) {
  vector3 result;

  float radAngle = deg2rad(angle);

  result.x = vec.x * cos(radAngle) + vec.z * sin(radAngle);
  result.y = vec.y;
  result.z = -vec.x * sin(radAngle) + vec.z * cos(radAngle);

  return (result);
}

vector3 vectorRotateZ(vector3 vec, float angle) {
  vector3 result;

  float radAngle = deg2rad(angle);

  result.x = vec.x * cos(radAngle) - vec.y * sin(radAngle);
  result.y = vec.x * sin(radAngle) + vec.y * cos(radAngle);
  result.z = vec.z;

  return (result);
}
