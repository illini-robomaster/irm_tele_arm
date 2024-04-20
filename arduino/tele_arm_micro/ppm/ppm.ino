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

#include <minipc_protocol.h>
#include <wfly_ppm.h>

#define USB_SERIAL Serial

#define SERVOS 6  // Arm has 6 servos.
#define OFFSET 5  // First 4 channels are used for movement.

communication::arm_data_t arm_data;
communication::selfcheck_data_t selfcheck_data;
auto minipc_session = communication::MinipcPort();

const communication::status_data_t* status_data;
char possible_packet_char[minipc_session.MAX_PACKET_LENGTH];  // Char array for buffering Serial.readBytes(). TODO: Fix.
uint8_t possible_packet[minipc_session.MAX_PACKET_LENGTH];
const uint8_t* data = possible_packet;
uint8_t packet_to_send[minipc_session.MAX_PACKET_LENGTH];
uint8_t recv_cmd_id;
int32_t length_;

wfly_ppm::WFly wflyer(1100, 1950, 1500, -360.0, 360.0);

bool f;  // Flag to enable ppm output after data first recieved.

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  USB_SERIAL.begin(115200);  // Should match tele_arm/arm_only baudrate.
  //while (!USB_SERIAL);
  //USB_SERIAL.println("SETUP BEGIN");

  // Initalize ppm.
  wflyer.init();

  f = false;
  //f = true;

  //wflyer.enable_output();

  //USB_SERIAL.println("SETUP END");

  delay(500);
}

void loop() {
  // Handle incoming packets.
  length_ = min(USB_SERIAL.available(), minipc_session.MAX_PACKET_LENGTH);
  if (length_ > 0) {
    USB_SERIAL.readBytes(possible_packet_char, length_);
    // Copy char array into uint8_t array.
    for (int i = 0; i < minipc_session.MAX_PACKET_LENGTH; i++) {
      possible_packet[i] = (uint8_t)possible_packet_char[i];
    }
    minipc_session.ParseUartBuffer(data, length_);

    recv_cmd_id = minipc_session.GetCmdId();
    status_data = minipc_session.GetStatus();

    // Take position.
    if (recv_cmd_id == communication::ARM_CMD_ID) {
      for (int i = 0; i < 6; i++) {
        arm_data.floats[i] = status_data->floats[i];
      }
      if (!f) {
        wflyer.enable_output();
        f = true;
      }
      // Insert angles.
      wflyer.insert(arm_data.floats, SERVOS, OFFSET);
    } else if (recv_cmd_id == communication::SELFCHECK_CMD_ID) {
      if (status_data->mode == 1){
        selfcheck_data.mode = 1;
        selfcheck_data.debug_int = status_data->debug_int;
      
        minipc_session.Pack(packet_to_send, (void*)&selfcheck_data, communication::SELFCHECK_CMD_ID);
        USB_SERIAL.write(packet_to_send, minipc_session.GetPacketLen(communication::SELFCHECK_CMD_ID));
      } else if (status_data->mode == 2){
        // Respond with ID:
        //  BRD: 131 (131 - 127 = 4)
        //  See repo irm_tele_arm `config.py` for details
        selfcheck_data.mode = 2;
        selfcheck_data.debug_int = 131;
      
        minipc_session.Pack(packet_to_send, (void*)&selfcheck_data, communication::SELFCHECK_CMD_ID);
        USB_SERIAL.write(packet_to_send, minipc_session.GetPacketLen(communication::SELFCHECK_CMD_ID));
      }
    }
  } else {
    minipc_session.Pack(packet_to_send, (void*)&arm_data, communication::ARM_CMD_ID);
      USB_SERIAL.write(packet_to_send, minipc_session.GetPacketLen(communication::ARM_CMD_ID));
  }

  sei();
}
