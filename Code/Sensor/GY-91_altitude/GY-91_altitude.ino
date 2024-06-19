//#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>

#define SDA_PIN 21
#define SCL_PIN 22 

Adafruit_BMP280 bme; // I2C
//MPU9250_asukiaaa mySensor;
//float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

float initialPressure = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  //mySensor.setWire(&Wire);
#else
  Wire.begin();
  //mySensor.setWire(&Wire);
#endif

  bme.begin(0x76);
  //mySensor.beginAccel();
  //mySensor.beginGyro();
  //mySensor.beginMag();

  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;

  delay(1000);

  initialPressure = bme.readPressure();

}

void loop() {
  static unsigned long lastTime = millis();
  const unsigned long delayTime = 10;

  if (millis() - lastTime < delayTime){
    return;
  }

  lastTime = millis();

  /*
  if (mySensor.accelUpdate() == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
    Serial.print("accelX: " + String(aX));
    Serial.print("\taccelY: " + String(aY));
    Serial.print("\taccelZ: " + String(aZ));
    Serial.print("\taccelSqrt: " + String(aSqrt));
  }

  if (mySensor.gyroUpdate() == 0) {
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
    Serial.print("\tgyroX: " + String(gX));
    Serial.print("\tgyroY: " + String(gY));
    Serial.print("\tgyroZ: " + String(gZ));
  }

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
  */
  //Serial.print("\tTemperature(*C): ");
  //Serial.print(bme.readTemperature());

  //Serial.print("\tPressure(Pa): ");
  //Serial.print(bme.readPressure());


  //Serial.print("\tApproxAltitude(m): ");
  Serial.print(bme.readAltitude(initialPressure/100)); // this should be adjusted to your local forcase
  Serial.print(", ");

  static int count = 0;

  if (count >= 35){
    count = 0;
    Serial.print("\n");
  }
  else{
    count ++;
  }

  //Serial.println(""); // Add an empty line
  }
