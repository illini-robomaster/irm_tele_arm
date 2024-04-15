/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2024 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#include <Dynamixel2Arduino.h>
#include <minipc_protocol.h>
#include <utility>
#include <vector>
#include <map>

// Use OpenRB-150.
// OpenRB does not require the DIR control pin.
#define DXL_SERIAL Serial1
#define USB_SERIAL Serial
const int DXL_DIR_PIN = -1;
 
const float DXL_PROTOCOL_VERSION = 2.0;
std::map<uint8_t, float> ANGLES = {
  {0, 0.0},
  {1, 0.0},
  {2, 0.0},
  {3, 0.0},
  {4, 0.0},
  {5, 0.0}
};
const std::map<uint8_t, float> STARTUP_ANGLES = {
  {0, 100.0},
  {1, 100.0},
  {2, 100.0},
  {3, 100.0},
  {4, 100.0},
  {5, 100.0}
};
const std::map<uint8_t, std::pair<float, float>> MIN_MAX_ANGLES = {
  {0, std::make_pair(180, 45)},
  {1, std::make_pair(180, 45)},
  {2, std::make_pair(180, 45)},
  {3, std::make_pair(180, 45)},
  {4, std::make_pair(180, 45)},
  {5, std::make_pair(180, 45)}
};
const std::vector<uint8_t> DXL_IDS = {0, 1, 2, 3, 4, 5};

uint8_t DXL_ID;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

communication::gimbal_data_t gimbal_data;
communication::color_data_t color_data;
communication::chassis_data_t chassis_data;
communication::selfcheck_data_t selfcheck_data;
communication::arm_data_t arm_data;
auto minipc_session = communication::MinipcPort();

const communication::status_data_t* status_data;
char possible_packet_char[minipc_session.MAX_PACKET_LENGTH];  // Char array for buffering Serial.readBytes(). TODO: Fix.
uint8_t possible_packet[minipc_session.MAX_PACKET_LENGTH];
uint8_t packet_to_send[minipc_session.MAX_PACKET_LENGTH];
const uint8_t* data = possible_packet;  // Point to possible_packet
uint8_t recv_cmd_id;
int32_t length_;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  USB_SERIAL.begin(115200);
  while(!USB_SERIAL);
  USB_SERIAL.println("SETUP BEGIN");

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Initialize our motors.
  for (const uint8_t& DXL_ID : DXL_IDS) {
    // Get DYNAMIXEL information
    dxl.ping(DXL_ID);

    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_ID);
    dxl.ledOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION);
    dxl.torqueOn(DXL_ID);
    dxl.ledOn(DXL_ID);

    // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 100);

    // Initial position
    dxl.setGoalPosition(DXL_ID, STARTUP_ANGLES.at(DXL_ID), UNIT_DEGREE);
  }
  delay(1500);
  for (const uint8_t& DXL_ID : DXL_IDS) {
    dxl.torqueOff(DXL_ID);
    dxl.ledOff(DXL_ID);
  }
  USB_SERIAL.println("SETUP END");
}

void loop() {
  // Disable torque mode when goal position reached.
  for (const uint8_t& DXL_ID : DXL_IDS) {
    if (dxl.getTorqueEnableStat(DXL_ID)
        && dxl.readControlTableItem(MOVING_STATUS, DXL_ID) & 1) {
      dxl.torqueOff(DXL_ID);
      dxl.ledOff(DXL_ID);
    }
  }

  // Handle incoming packets.
  length_ = min(USB_SERIAL.available(), minipc_session.MAX_PACKET_LENGTH);
  if (length_ > 0){
    USB_SERIAL.readBytes(possible_packet_char, length_);
    // Copy char array into uint8_t array.
    for (int i = 0; i < minipc_session.MAX_PACKET_LENGTH; i++) {
      possible_packet[i] = (uint8_t)possible_packet_char[i];
    }
    minipc_session.ParseUartBuffer(data, length_);

    recv_cmd_id = minipc_session.GetCmdId();
    status_data = minipc_session.GetStatus();

    
    switch (recv_cmd_id) {
      case communication::GIMBAL_CMD_ID:
        // Forward gimbal data
        break;
      case communication::COLOR_CMD_ID:
        // Forward color data
        break;
      case communication::CHASSIS_CMD_ID:
        // Forward gimbal data
        break;
      case communication::SELFCHECK_CMD_ID:
        if (status_data->mode == 1){
          selfcheck_data.mode = 1;
          selfcheck_data.debug_int = status_data->debug_int;

          minipc_session.Pack(packet_to_send, (void*)&selfcheck_data, communication::SELFCHECK_CMD_ID);
          USB_SERIAL.write(packet_to_send, minipc_session.GetPacketLen(communication::SELFCHECK_CMD_ID));
        } else if (status_data->mode == 2){
          // Respond with ID:
          //  BRD: 129 (129 - 127 = 2)
          //  ARM: 130 (130 - 127 = 3)
          //  See repo irm_tele_arm `config.py` for details
          selfcheck_data.mode = 2;
          selfcheck_data.debug_int = 130;

          minipc_session.Pack(packet_to_send, (void*)&selfcheck_data, communication::SELFCHECK_CMD_ID);
          USB_SERIAL.write(packet_to_send, minipc_session.GetPacketLen(communication::SELFCHECK_CMD_ID));
        }
        break;
      // Take position.
      case communication::ARM_CMD_ID:
        for (int i = 0; i < 6; i++) {
          DXL_ID = DXL_IDS.at(i);
          arm_data.floats[i] = status_data->floats[i];
          if (!dxl.getTorqueEnableStat(DXL_ID)) {
            dxl.torqueOn(DXL_ID);
            dxl.ledOn(DXL_ID);
          };
          dxl.setGoalPosition(DXL_ID, arm_data.floats[i], UNIT_DEGREE);
        }

        // Respond with current position
        minipc_session.Pack(packet_to_send, (void*)&arm_data, communication::ARM_CMD_ID);
        USB_SERIAL.write(packet_to_send, minipc_session.GetPacketLen(communication::ARM_CMD_ID));
        break;
      default:
        selfcheck_data.mode = status_data->mode;
        selfcheck_data.debug_int = 130;

        minipc_session.Pack(packet_to_send, (void*)&selfcheck_data, communication::SELFCHECK_CMD_ID);
        USB_SERIAL.write(packet_to_send, minipc_session.GetPacketLen(communication::SELFCHECK_CMD_ID));
        break;
    }
  }
}
