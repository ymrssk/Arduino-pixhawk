#include <mavlink.h>

#include <common.h>
#include <mavlink_msg_ca_trajectory.h>
#include <mavlink_msg_heartbeat.h>

#include <math.h>
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
  uint64_t timestamp=1; /*< */
  uint16_t time_start_usec=2; /*< */
  uint16_t time_stop_usec=1; /*< */
  uint8_t coefficients[28];
  for(int i=4;i<32;i++)
  coefficients[i] =i;
  uint16_t seq_id = 5;
  mavlink_msg_ca_trajectory_pack(system_id, component_id, &msg, timestamp, time_start_usec, time_stop_usec, coefficients, seq_id);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.println("ca_trajectory");
  MavSerial.write(buf,len);
}
