


// defines pins
const int trigPin = 14;
const int echoPin = 27;

// vars
unsigned long ultraTime = 0;

volatile float distance = 0;
volatile unsigned long duration = 0;

volatile int newVal = 0;

void IRAM_ATTR ultraSoundISR() {
  
  switch(digitalRead(echoPin)){
    case HIGH:
      duration = micros();
    case LOW:
      distance = (micros() - duration) * 0.034 / 2;
      newVal = 1;
  }
}


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  while (!Serial) {}

  //Initiate pins for ultrasonic sensors
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 

  //Attach interrupt
  attachInterrupt(echoPin, ultraSoundISR, CHANGE);

}

void loop() {
  

  
  if (millis() > (ultraTime + 2000)){

    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    ultraTime = millis();

  }

  if (newVal == 1){

    Serial.println(distance);
    newVal = 0;

  }
  

  

}
