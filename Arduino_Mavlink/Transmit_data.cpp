void mav_heartbeat_pack() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  /**
   * @brief Pack a heartbeat message
   * @param system_id ID of this system
   * @param component_id ID of this component (e.g. 200 for IMU)
   * @param msg The MAVLink message to compress the data into
   *
   * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
   * @param autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
   * @param base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
   * @param custom_mode A bitfield for use for autopilot-specific flags.
   * @param system_status System status flag, see MAV_STATE ENUM
   * @return length of the message in bytes (excluding serial stream start sign)
   */
  mavlink_msg_heartbeat_pack(255,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void mav_arm_pack(boolean state) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  //Arm the drone
  //400 stands for MAV_CMD_COMPONENT_ARM_DISARM
  // 1 an 8'th argument is for ARM (0 for DISARM)
  if(state) {
    //ARM
    mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, 400, 1,1.0,0,0,0,0,0,0);
  }else {
    //DISARM
    mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, 400, 1,0.0,0,0,0,0,0,0);
  }
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void mav_set_mode() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  //Set flight mode 'Stabilize'
  mavlink_msg_set_mode_pack(0xFF, 0xBE, &msg, 1, 209, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void mav_override_rc(int roll, int pitch, int throttle, int yaw) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_rc_channels_override_pack(0xFF, 0xBE, &msg, 1, 1, roll, pitch, throttle, yaw, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}
