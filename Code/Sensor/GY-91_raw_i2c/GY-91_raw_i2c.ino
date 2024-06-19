#include "Wire.h"

#define SDA_PIN 21
#define SCL_PIN 22 

#define ACC_FULL_SCALE_2_G       0x00
#define ACC_FULL_SCALE_4_G       0x08
#define ACC_FULL_SCALE_8_G       0x10
#define ACC_FULL_SCALE_16_G      0x18

#define MPU9250_ADDR_ACCELCONFIG  0x1C

#define SENSOR_ADDRESS 104

void setup() {
  Serial.begin(115200);
  while (!Serial){}
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(MPU9250_ADDR_ACCELCONFIG);
  Wire.write((uint8_t)ACC_FULL_SCALE_16_G);
  Wire.endTransmission();


  delay(100);

  /*
  byte x = 0b11110000;
  Serial.print("Binary number printed with print:  ");
  Serial.println(x, BIN);
  Serial.print("Binary number printed from 0 to 7: ");
  for(int i = 0; i < 8; i++){
    Serial.print(bitRead(x, i));
  }
  while(true){}
  */

}

void loop() {
  Serial.println(readValue(104 ,63));
  delay(100);

  

}

float readValue(uint8_t address, uint8_t registerAddress){
  sendByte(address, registerAddress, false);

  Wire.requestFrom(address, 2);

  int value = 0;

  int8_t rightByte = Wire.read();
  int sign = 0;
  if (rightByte < 0){
    rightByte *= -1;
    sign = 1;
  }


  value |= (int16_t) rightByte << 8;
  value |= (int16_t) Wire.read();

  if(sign){
    value *= -1;
  }

  return(((float) -value) * (16 / 32768.0));

}

void sendByte(uint8_t address, uint8_t byte, bool stop){
  Wire.beginTransmission(address);
  Wire.write(byte);
  Wire.endTransmission(stop);
}


