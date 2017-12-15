#include <mavlink.h>
#include <SoftwareSerial.h>

#define RX_PIN 2 //used for Mavlink, reads input from Pixhawk
#define TX_PIN 3 //used for Mavlink, transmits to Pixhawk

SoftwareSerial MavSerial = SoftwareSerial(RX_PIN, TX_PIN);

void setup()
{
  Serial.begin(57600);
  Serial.println("Connecting");

  MavSerial.begin(57600);
  while (!MavSerial.isListening())
  {
    Serial.println("Connecting");
  }
  
  Serial.println("Ready");
  
}
void loop()    
{
  //Serial.println(MavSerial.available());
  //Serial.println(MavSerial.available() > 0);    
  while (MavSerial.available() > 0 ) 
  {
    uint8_t c = MavSerial.read();
    Serial.println(c);
    Serial.print("Num Bytes: ");
    Serial.println(MavSerial.available());
  }
  delay(1000);
  Serial.println("Not yet");
}
