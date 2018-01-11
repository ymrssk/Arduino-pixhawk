#include <mavlink.h>
#include <common.h>
#include <SoftwareSerial.h>

#define RX_PIN 2 //used for Mavlink, reads input from Pixhawk
#define TX_PIN 3 //used for Mavlink, transmits to Pixhawk

SoftwareSerial MavSerial = SoftwareSerial(RX_PIN, TX_PIN);

uint8_t buf[MAVLINK_MAX_PACKET_LEN];

uint8_t system_id = 100;
uint8_t component_id = 50;


void setup() {
  MavSerial.begin(57600); //RXTX from Pixhawk (Port 18,19 Arduino Mega)
  Serial.begin(57600); //Main serial port to read the port
}


void loop() {
  mavlink_message_t msg;
  float battery_voltage = 1.0;
  float battery_current = 1.0;
  float battery_power = 1.0;
  int battery_remaining = 10;
  float sap_voltage = 1.0;
  float sap_current = 10.0;
  float sap_power = 1.0; 
  float sap_str1_voltage = 1.0;
  float sap_str1_current = 1.0;
  float sap_str1_power = 1.0;   
  float sap_str2_voltage = 1.0;
  float sap_str2_current = 1.0;
  float sap_str2_power = 1.0;   
  float sap_str3_voltage = 1.0;
  float sap_str3_current = 1.0;
  float sap_str3_power = 1.0;   
  float load_current = 1.0;
  float load_voltage = 1.0;
  float load_power = 1.0;   
  int component_status = 10;
  mavlink_msg_sel_solar_plane_pack(system_id, component_id, &msg, battery_voltage, battery_current, battery_power, battery_remaining, sap_voltage, sap_current, sap_power, sap_str1_voltage, sap_str1_current, sap_str1_power, sap_str2_voltage, sap_str2_current, sap_str2_power, sap_str3_voltage, sap_str3_current, sap_str3_power, load_current, load_voltage, load_power, component_status);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  /*for(int i;i<len;i++){
  Serial.print(buf[i]);
  Serial.print(" ");
  }
  Serial.println("");
  */
  MavSerial.write(buf,len);
}
