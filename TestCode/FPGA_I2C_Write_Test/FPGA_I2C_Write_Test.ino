// Testcode for ESP32 (FPGA I2C Write test)

#include "Wire.h"

#define SDA 23
#define SCL 22

//When the ESP recieves data
void onReceive(int len)
{
  
  char C[2];
  int i = 0;
  
  //While the wire is available send it on to the PC
  while(Wire.available())
  {
    C[i] = Wire.read();
    i++;
  }
  Serial.print(C[0], BIN);
  Serial.print(" ");
  Serial.print(C[1], BIN);
  Serial.println("");
}

void setup() {

  //Init serial
  Serial.begin(115200);

  //Hook up Wire
  Wire.onReceive(onReceive);

  //Init Wire (Address = 53, Frequency = 400 kHz)
  Wire.begin((uint8_t)53, SDA, SCL, uint32_t(400000));

}

void loop() {
  
  delay(1000);

}
