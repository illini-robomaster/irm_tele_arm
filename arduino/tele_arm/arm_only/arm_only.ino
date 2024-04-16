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
//#include <minipc_protocol.h>
#include <wfly_ppm.h>
#include <utility>
#include <vector>
#include <map>

// Use OpenRB-150.
// OpenRB does not require the DIR control pin.
#define DXL_SERIAL Serial1
#define USB_SERIAL Serial
const int DXL_DIR_PIN = -1;

#define SERVOS 6  // Arm has 6 servos.
#define OFFSET 4  // First 4 channels are used for movement.

const float DXL_PROTOCOL_VERSION = 2.0;
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

// If there are unused spaces between channels fill those positions with `bad'
// (unused/invalid) ids. They will automatically be masked and its
// corresponding angle from STARTUP_ANGLES will be sent.
const std::vector<uint8_t> DXL_IDS = {0, 1, 2, 3, 4, 5};
// Note: Masking an id freezes its value.
//       Masking an id does not skip instances of disabling torque/led.
std::vector<bool> mask = {1, 1, 1, 1, 1, 1};
int counter;  // For filling in angles. Ignores when > SERVOS.

float angles[SERVOS];

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
wfly_ppm::WFly wflyer(1100, 1950, -180.0, 180.0);

//This namespace is required to use Control table item names.
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

  // Initalize ppm.
  wflyer.init();

  // Initialize our motors.
  counter = -1;
  for (const uint8_t& DXL_ID : DXL_IDS) {
    // Get DYNAMIXEL information and mask unresponsive ids.
    if (!dxl.ping(DXL_ID)) mask[DXL_ID] = 0;

    // Turn off torque when configuring items in EEPROM area.
    dxl.torqueOff(DXL_ID);
    dxl.ledOff(DXL_ID);

    if (!mask[DXL_ID]) continue;

    dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION);
    dxl.torqueOn(DXL_ID);
    dxl.ledOn(DXL_ID);

    // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed.
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 100);

    // Set initial angles.
    // Enforce MIN_MAX_ANGLES.
    dxl.setGoalPosition(
      DXL_ID,
      wfly_ppm::clamp(STARTUP_ANGLES.at(DXL_ID),
                      MIN_MAX_ANGLES.at(DXL_ID).first,
                      MIN_MAX_ANGLES.at(DXL_ID).second),
      UNIT_DEGREE);
  }

  delay(500);

  counter = -1;
  for (const uint8_t& DXL_ID : DXL_IDS) {
    dxl.torqueOff(DXL_ID);
    dxl.ledOff(DXL_ID);

    counter++;
    if (!mask[DXL_ID]) {
      angles[counter] = STARTUP_ANGLES.at(DXL_ID);
      continue;
    } else if (counter > SERVOS - 1) continue;

    // Enforce MIN_MAX_ANGLES.
    angles[counter] = wfly_ppm::clamp(
      dxl.getPresentPosition(DXL_ID, UNIT_DEGREE),
      MIN_MAX_ANGLES.at(DXL_ID).first,
      MIN_MAX_ANGLES.at(DXL_ID).second);
  }
  wflyer.enable_ppm();

  USB_SERIAL.println("SETUP END");
}

void loop() {
  // Insert ppm values into send buffer.
  counter = -1;
  for (const uint8_t& DXL_ID : DXL_IDS) {
      counter++;
      if (counter > SERVOS - 1) continue;
      else if (!mask[DXL_ID]) continue;

      angles[counter] = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
  }
  wflyer.insert(angles, SERVOS, OFFSET);

  // Disable torque mode when goal position reached.
  for (const uint8_t& DXL_ID : DXL_IDS) {
    if (dxl.getTorqueEnableStat(DXL_ID)
        && dxl.readControlTableItem(MOVING_STATUS, DXL_ID) & 1) {
      dxl.torqueOff(DXL_ID);
      dxl.ledOff(DXL_ID);
    }
  }
}
