// Testcode for ESP32 (FPGA I2C Read test)

#include "Wire.h"

#define SDA 23
#define SCL 22

//Data for sending
char D[1] = {char(0)};

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

//When the data is requested
void onRequest()
{
  //Send the data
  //Wire.print(1);
  //Prep new data
  D[0] = char(int(D[0]) + 1);
  Wire.slaveWrite((uint8_t *)D, 1);
  Serial.print("Next: "); Serial.println(D[0], BIN);
}

void setup() {

  //Init serial
  Serial.begin(115200);

  //Hook up Wire
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  //Init Wire (Address = 53, Frequency = 400 kHz)
  Wire.begin((uint8_t)53, SDA, SCL, uint32_t(400000));

  //Load first data value
  Wire.slaveWrite((uint8_t *)D, 1);

}

void loop() {
  
  delay(1000);

}