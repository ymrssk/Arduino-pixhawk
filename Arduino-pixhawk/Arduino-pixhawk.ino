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
uint8_t type = 0; //GCS
uint8_t autopilot = 0; //generic

uint8_t received_sysid;//51 ;
uint8_t received_compid;// = 68;
uint8_t GCS_UNITS = 0;
int a = 0;


void setup() {
  MavSerial.begin(57600); //RXTX from Pixhawk (Port 18,19 Arduino Mega)
  Serial.begin(57600); //Main serial port to read the port
}

void loop() {
  // Define the system type (see mavlink_types.h for list of possible types)op 
  if (a==0) {
    delay(2000);
    present_msg();
    delay(2000);
    //start_feeds();
    a=1; 
  }
  delay(1000);
  present_msg();
}


void present_msg() {
  mavlink_message_t msg;

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(system_id, component_id, &msg, type, autopilot, 16, 0, 3);
  uint64_t timestamp=1; /*< */
  uint16_t time_start_usec=2; /*< */
  uint16_t time_stop_usec=1; /*< */
  uint8_t coefficients[28];
  for(int i=4;i<32;i++)
  coefficients[i] =i;
  uint16_t seq_id = 5;
  
  mavlink_msg_heartbeat_pack(system_id, component_id, &msg, type, autopilot, 16, 0, 3);
  //send_message(&msg);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  delay(1000);

  Serial.println("");
  Serial.println("Heartbeat start");
  for(int i=0;i<len;i++){
  Serial.print(buf[i]);
  Serial.print(" ");
}  

  Serial.println(""); 
  Serial.write(buf,len);
 
  //Serial.write(buf,len);
  Serial.println("ca_trajectory");
  mavlink_msg_ca_trajectory_pack(system_id, component_id, &msg, timestamp, time_start_usec, time_stop_usec, coefficients, seq_id);

  len = mavlink_msg_to_send_buffer(buf, &msg);

  for(int i=0;i<len;i++){
  Serial.print(buf[i]);
  Serial.print(" ");  
  }
  Serial.println("");

  MavSerial.write(buf,len);

  Serial.println("Debug");
  mavlink_msg_debug_pack(system_id,component_id, &msg, 100,12,1.0);
  len = mavlink_msg_to_send_buffer(buf, &msg);


  Serial.write(buf,len);
/*
  //send_message(&msg);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  delay(1000);
  Serial.println("ca_trajectory start");
  for(int i=0;i<len;i++){
  Serial.print(buf[i]);
  Serial.print(" ");
}  
  MavSerial.write(buf,len);  

  Serial.println("");  
  Serial.println("ca_trajectory stop");
*/  
  //Serial.write(buf,len);  
  //MavSerial.write(buf,len);  

  delay(1000);

  //receive_msg();
}


void start_feeds() //request data
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_request_data_stream_pack(system_id, component_id, &msg, received_sysid, received_compid, 30 ,1000, 1); //here its my problem the last digit
  //  send_message(&msg);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  delay(1000);
  MavSerial.write(buf,len);

}

////////////////////////////////////////////////////////////////////////////////////////
//void receive_msg()
//{ //receive data over serial
  // Serial.println(MavSerial.available());
  //  Serial.println(" ");
//  while (MavSerial.available() > 0){   
//    mavlink_message_t msg;
//    mavlink_status_t status;
//    uint8_t rec =  buf;//Serial.read();//Show bytes send from the pixhawk
    //Serial.print("len = ");
    //Serial.print(" Data = ");
//    Serial.print(rec);
//    Serial.print(" ");

    /* if read failed, this loop won't execute */
  //  if(mavlink_parse_char(MAVLINK_COMM_0, rec, &msg, &status)){ 

    //  Serial.println(" ");
      //Serial.print("message:");
    //  Serial.println(msg.msgid);
    //  gcs_handleMessage(&msg);
   // }
 // }
 //     Serial.println(" ");
//}

  


void gcs_handleMessage(mavlink_message_t* msg) //read varaible
{
  //  Serial.println();
  // Serial.print("Message ID: ");
  // Serial.println(msg->msgid);
  switch (msg->msgid) {
  case MAVLINK_MSG_ID_HEARTBEAT:
    {
      mavlink_heartbeat_t packet;
      mavlink_msg_heartbeat_decode(msg, &packet);
      if ((*msg).sysid != 0xff) { // do not process mission planner heartbeats if we have two receiver xbees
        received_sysid = (*msg).sysid; // save the sysid and compid of the received heartbeat for use in sending new messages
        received_compid = (*msg).compid;
        Serial.println("heartbeat");
      }
      break; 
    }
  case MAVLINK_MSG_ID_ATTITUDE:
    {
      // decode
      mavlink_attitude_t packet;
      mavlink_msg_attitude_decode(msg, &packet);
      float pitch = packet.pitch;
      float yaw = packet.yaw;
      float roll = packet.roll;
      Serial.println("Im in  Attitude");
      Serial.print("pitch: ");
      Serial.println(pitch);
      Serial.print("yaw: ");
      Serial.println(yaw);
      Serial.print("roll: ");      
      Serial.println(roll);
      break;
    }
  case MAVLINK_MSG_ID_PING:
    {
      Serial.println("System_time");
      break;
    }

  case MAVLINK_MSG_ID_GPS_RAW_INT:
    {
      // decode
      mavlink_gps_raw_int_t packet;
      mavlink_msg_gps_raw_int_decode(msg, &packet);
      uint8_t  gpsfix = packet.fix_type;
      uint8_t  mav_utime = packet.time_usec;
      uint8_t  numSats = packet.satellites_visible;
      uint8_t  cog = packet.cog;
      Serial.println("GPS");
      break;
    }
  case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
      // decode
      mavlink_global_position_int_t packet;
      mavlink_msg_global_position_int_decode(msg, &packet);
      uint8_t  latitude = packet.lat;
      uint8_t  longitude = packet.lon;
      if (GCS_UNITS == 0)uint8_t altitude = packet.alt / 1000;
      else if ( (GCS_UNITS == 1) || (GCS_UNITS == 2) ) uint8_t altitude = (packet.alt / 1000) * 3.28084;
      Serial.println("msg");
      break;
    }
  case MAVLINK_MSG_ID_GPS_STATUS:
    {
      mavlink_gps_status_t packet;
      mavlink_msg_gps_status_decode(msg, &packet);
      Serial.println("GPS STATUS");
      break;
    }
  case MAVLINK_MSG_ID_VFR_HUD:
    {
      mavlink_vfr_hud_t packet;
      mavlink_msg_vfr_hud_decode(msg, &packet);
      uint8_t  heading = packet.heading;
      if (GCS_UNITS == 0) uint8_t ias = packet.airspeed * 3.6;
      else if (GCS_UNITS == 1)uint8_t ias = packet.airspeed * 2.24;
      else if (GCS_UNITS == 2)uint8_t ias = packet.airspeed * 1.94;
      if (GCS_UNITS == 0)uint8_t grs = packet.groundspeed * 3.6;
      else if (GCS_UNITS == 1)uint8_t grs = packet.groundspeed * 2.24;
      else if (GCS_UNITS == 2)uint8_t grs = packet.groundspeed * 1.94;
      if (GCS_UNITS == 0)uint8_t vsi = packet.climb;
      else if ( (GCS_UNITS == 1) || (GCS_UNITS == 2) )uint8_t vsi = packet.climb * 3.28084;
      Serial.println("id vfr");
      break;
    }
  case MAVLINK_MSG_ID_RAW_PRESSURE:
    {
      // decode
      mavlink_raw_pressure_t packet;
      mavlink_msg_raw_pressure_decode(msg, &packet);
      Serial.println("raw preasure");
      break;
    }
  case MAVLINK_MSG_ID_SYS_STATUS:
    {

      mavlink_sys_status_t packet;
      mavlink_msg_sys_status_decode(msg, &packet);
      uint8_t  vbat = packet.voltage_battery;
      Serial.print("Status");
      break;
    }
  }

}


//void heartbeat_msg()//MAVLINK_MSG_ID_HEARTBEAT 0
//{
//  uint8_t system_type = MAV_TYPE_QUADROTOR;
//  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
//  uint8_t base_mode =MAV_MODE_FLAG_ENUM_END;
//  uint8_t custom_mode = 0 ;
//  uint8_t system_status = MAV_STATE_STANDBY ;
//
//  mavlink_message_t msg;// Initialize the required buffers
//  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//  // mavlink_message_heartbeat_pack(system id, component id, message container, system type, MAV_AUTOPILOT_GENERIC)
//  mavlink_msg_heartbeat_pack(system_id, component_id, &msg, system_type, autopilot_type, base_mode, custom_mode, system_status);// Pack the message
//  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);// Copy the message to send buffer
//  delay(1000);
//  MavSerial.write(buf, len);// Send the message (.write sends as bytes)
//}


