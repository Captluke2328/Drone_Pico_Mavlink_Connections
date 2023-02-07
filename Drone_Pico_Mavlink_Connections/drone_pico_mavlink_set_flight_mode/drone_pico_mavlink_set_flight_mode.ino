#include "C:\Users\Lukas\Documents\Arduino\libraries\ArduinoMAVLink\heartbeat_004\mavlinkv2\common\mavlink.h"

unsigned long HeartbeatTime = 0;

void setup() {
  Serial1.begin(57600);

  Serial.begin(57600);
  Serial.println("MAVLink starting.");


}

void loop() {

  if ( (millis() - HeartbeatTime) > 1000 ) {
    HeartbeatTime = millis();
    PIX_HEART_BEAT();
  }

   while (Serial.available() == 0) {
  }
//ARM();

  
  //TAKEOFF();
  
  // Working
  // int menu = Serial.parseInt();
  //CHANGEMODE(menu);
}

mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint16_t len;
 
void PIX_HEART_BEAT()
{
  mavlink_msg_heartbeat_pack(1, 0, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0, 1, 0);    // System ID = 255 = GCS
  len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial1.write(buf, len);
}

void ARM()
{

  mavlink_message_t messaggio;
  mavlink_command_long_t comando; //struttura dati relativa al comando mavlink long
  char buf[300];
  comando.command = MAV_CMD_COMPONENT_ARM_DISARM; //settaggio valori della struct relativa al comando
  comando.target_system = 1;
  comando.target_component = 0;
  comando.confirmation = true;
  comando.param1 = 1;
  comando.param2 = 21196;
  comando.param3 = 0;
  comando.param4 = 0;
  comando.param5 = 0;
  comando.param6 = 0;
  comando.param7 = 0;

  mavlink_msg_command_long_encode(255, 151, &messaggio, &comando);
  unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &messaggio);
  Serial1.write(buf, len);

}

void TAKEOFF()
{

mavlink_message_t messaggio; //messaggio da inviare
mavlink_command_long_t comando; //struttura dati relativa al comando mavlink long
char buf[300];

comando.command = MAV_CMD_NAV_TAKEOFF; //settaggio valori della struct relativa al comando
comando.target_system = 1;
comando.target_component = 255;
comando.confirmation = true;
comando.param1 = 0;
comando.param2 = 0;
comando.param3 = 0;
comando.param4 = 0;
comando.param5 = 0;
comando.param6 = 0;
comando.param7 = 1;

mavlink_msg_command_long_encode(255, 151, &messaggio, &comando);
unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &messaggio);
Serial1.write(buf, len);

//    uint8_t _system_id = 255;
//   mavlink_command_long_t takeoff_cmd;
//   takeoff_cmd.command = MAV_CMD_NAV_TAKEOFF;
//   takeoff_cmd.param1 = 0;
//   takeoff_cmd.param4 = 0;
//   takeoff_cmd.param5 = 0; //current_messages.gps_raw_int.lat;
//   takeoff_cmd.param6 = 0;//current_messages.gps_raw_int.lon;
//   takeoff_cmd.param7 = 0;//(current_messages.gps_raw_int.alt)/1000 + hauteur;
//   takeoff_cmd.target_system = _system_id;
//   takeoff_cmd.confirmation = true;

//     // Initialize the required buffers
//   mavlink_command_long_t msg;
//   uint8_t buf[MAVLINK_MAX_PACKET_LEN];

//   // Pack the message
// mavlink_msg_command_long_encode(1, 250, &messaggio, &comando);
// unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &messaggio);
//   // Copy the message to the send buffer
//   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//   // Send the message (.write sends as bytes)
//   Serial1.write(buf, len);

}

void CHANGEMODE(int menu)
{

  //Set message variables
  uint8_t _system_id = 255; // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 151; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t _base_mode = 1;
  uint32_t _custom_mode =menu; //10 = auto mode
  
/*
Flight / Driving Modes (change custom mode above)

0 - stabilize
1 - acro
2 - althold
3 - auto
4 - guided
5 - loiter
6 - RTL
7 - Circle
8 - 
9 - Land
10 - 
11 - Drift
12
*/

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_set_mode_pack(_system_id, _component_id, &msg, _target_system, _base_mode, _custom_mode);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)
  Serial.print("\nsending set mode command...");
  Serial1.write(buf, len); //Write data to serial port

 
}





