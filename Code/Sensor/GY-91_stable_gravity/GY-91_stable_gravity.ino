
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>

#define SDA_PIN 21
#define SCL_PIN 22 

Adafruit_BMP280 bme; // I2C
MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

struct vector3 {
  float x, y, z;
};

vector3 downDir = {0, 0, 0};
vector3 upRef = {0, 0, -1};

void setup() {
  Serial.begin(115200);
  while (!Serial);

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
#else
  Wire.begin();
  mySensor.setWire(&Wire);
#endif

  bme.begin(0x76);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
}

float getLength(vector3 vec){
  return( sqrt(sq(vec.x) + sq(vec.y) + sq(vec.z)) );
}

vector3 normalize(vector3 vec){
  float length = getLength(vec);

  vector3 result;
  result.x = vec.x / length;
  result.y = vec.y / length;
  result.z = vec.z / length;

  return(result);
}

float normalizedDotProduct(vector3 vec_1, vector3 vec_2){
  vector3 vec1 = normalize(vec_1);
  vector3 vec2 = normalize(vec_2);
  return(vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z);
}

float getAngle(vector3 vec_1, vector3 vec_2){
  return(acos(normalizedDotProduct(vec_1, vec_2)));
}

float deg2rad(float deg){
  return(deg / 360 * 2 * 3.14159265);
}

float rad2deg(float rad){
  return(rad / (2 * PI) * 360);
}

vector3 vectorRotateX(vector3 vec, float angle){
  vector3 result;

  float radAngle = deg2rad(angle);

  result.x = vec.x;
  result.y = vec.y * cos(radAngle) - vec.z * sin(radAngle);
  result.z = vec.y * sin(radAngle) + vec.z * cos(radAngle);

  return(result);
}

vector3 vectorRotateY(vector3 vec, float angle){
  vector3 result;

  float radAngle = deg2rad(angle);

  result.x = vec.x * cos(radAngle) + vec.z * sin(radAngle);
  result.y = vec.y;
  result.z = - vec.x * sin(radAngle) + vec.z * cos(radAngle);

  return(result);
}

vector3 vectorRotateZ(vector3 vec, float angle){
  vector3 result;

  float radAngle = deg2rad(angle);

  result.x = vec.x * cos(radAngle) - vec.y * sin(radAngle);
  result.y = vec.x * sin(radAngle) + vec.y * cos(radAngle);
  result.z = vec.z;

  return(result);
}

void loop() {
  bool useGyro = false;

  if (mySensor.accelUpdate() == 0) {
    //aX = mySensor.accelX();
    //aY = mySensor.accelY();
    //aZ = mySensor.accelZ();

    //aSqrt = mySensor.accelSqrt();

    vector3 newDir;

    newDir.x = mySensor.accelX();
    newDir.y = mySensor.accelY();
    newDir.z = mySensor.accelZ();

    //Serial.print("accelX: " + String(downDir.x));
    //Serial.print("\taccelY: " + String(downDir.y));
    //Serial.print("\taccelZ: " + String(downDir.z));

    if(getLength(newDir) >= 1.2){
      useGyro = true;
      Serial.print("-");
    }
    else{
      useGyro = false;
      downDir = newDir;
    }

    //Serial.print("\taccelSqrt: " + String(aSqrt));
  }
  
  if (mySensor.gyroUpdate() == 0) {
    static unsigned long lastTime = 0;

    unsigned long delta = micros() - lastTime;
    lastTime = micros();

    static vector3 angle = {0, 0, 0};

    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();

    angle.x += gX * delta / 1000000.0;
    angle.y += gY * delta / 1000000.0;
    angle.z += gZ * delta / 1000000.0;

    if(useGyro && false){
      downDir = vectorRotateX(downDir, angle.x);
      downDir = vectorRotateY(downDir, angle.y);
      downDir = vectorRotateZ(downDir, angle.z);
    }

    //Serial.print("\ngyroX: " + String(angle.x));
    //Serial.print("\tgyroY: " + String(angle.y));
    //Serial.print("\tgyroZ: " + String(angle.z));
    //delay(10);
  }

  //Serial.print("Angle to gravity: ");
  //Serial.print(String(rad2deg(getAngle(upRef, downDir))));
  Serial.print(String(getLength(downDir)));
  /*
  if (mySensor.magUpdate() == 0) {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();
    Serial.print("\tmagX: " + String(mX));
    Serial.print("\tmaxY: " + String(mY));
    Serial.print("\tmagZ: " + String(mZ));
    Serial.print("\thorizontalDirection: " + String(mDirection));
  }

  Serial.print("\tTemperature(*C): ");
  Serial.print(bme.readTemperature());

  Serial.print("\tPressure(Pa): ");
  Serial.print(bme.readPressure());


  Serial.print("\tApproxAltitude(m): ");
  Serial.print(bme.readAltitude(1013.25)); // this should be adjusted to your local forcase
  */
  Serial.println(""); // Add an empty line
  }
