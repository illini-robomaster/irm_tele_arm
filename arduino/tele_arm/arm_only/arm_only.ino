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
#include <wfly_ppm.h>
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

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  USB_SERIAL.begin(115200);
  while (!USB_SERIAL);
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
  /* PPM CODE
  */

  // Disable torque mode when goal position reached.
  for (const uint8_t& DXL_ID : DXL_IDS) {
    if (dxl.getTorqueEnableStat(DXL_ID)
        && dxl.readControlTableItem(MOVING_STATUS, DXL_ID) & 1) {
      dxl.torqueOff(DXL_ID);
      dxl.ledOff(DXL_ID);
    }
  }
}
