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
#include <vector>
#include <map>

// Arduino uses -std=gnu++11. std::clamp is introduced in c++17.
// Reimplement a simplified version here.
template<class T>
const T& clamp(const T& v, const T& lo, const T& hi) {
  return v < lo ? lo : hi < v ? hi : v;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// Use OpenRB-150.
// OpenRB does not require the DIR control pin.
#define DXL_SERIAL Serial1
#define USB_SERIAL Serial
const int DXL_DIR_PIN = -1;

#define SERVOS 6  // Arm has 6 servos.
#define STATIC_CURRENT 300
#define MOVING_CURRENT 15

communication::arm_data_t arm_data;
communication::selfcheck_data_t selfcheck_data;
auto minipc_session = communication::MinipcPort();

const communication::status_data_t* status_data;
char possible_packet_char[minipc_session.MAX_PACKET_LENGTH];  // Char array for buffering Serial.readBytes(). TODO: Fix.
uint8_t possible_packet[minipc_session.MAX_PACKET_LENGTH];
const uint8_t* data = possible_packet;
uint8_t packet_to_send[minipc_session.MAX_PACKET_LENGTH];
uint8_t recv_cmd_id;
uint32_t length_;

const float DXL_PROTOCOL_VERSION = 2.0;
const std::map<uint8_t, float> STARTUP_ANGLES = {
  {0, 90.0},
  {1, 140.0},
  {2, 120.0},
  {3, 120.0},
  {4, 120.0},
  {5, 120.0}
};
const std::map<uint8_t, std::pair<float, float>> MIN_MAX_ANGLES = {
  {0, std::make_pair(-360, 360)},
  {1, std::make_pair(-360, 360)},
  {2, std::make_pair(-360, 360)},
  {3, std::make_pair(-360, 360)},
  {4, std::make_pair(-360, 360)},
  {5, std::make_pair(-360, 360)}
};

bool threshold_crossed[SERVOS] = {false, false, false, false, false, false};

float level_angles[SERVOS];
float horizon_angles[SERVOS];

float torque_threshold_constants[SERVOS];
float torque_thresholds[SERVOS];

// If there are unused spaces between channels fill those positions with `bad'
// (unused/invalid) ids. They will automatically be masked and its
// corresponding (clamped) angle from STARTUP_ANGLES will be sent.
// Order: Base servo -> Top servo
const std::vector<uint8_t> DXL_IDS = {0, 1, 2, 3, 4, 5};
// Note: Masking an id freezes its value.
//       Masking an id does not skip instances of disabling torque/led.
std::vector<bool> mask = {1, 1, 1, 1, 1, 1};
int counter;  // For filling in angles. Ignores when > SERVOS.

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names.
using namespace ControlTableItem;

// Safely set operating mode of motor.
void set_operating_mode(uint8_t DXL_ID, uint8_t mode) {
  // [t]orque_[e]nabled
  bool t_e = dxl.getTorqueEnableStat(DXL_ID);
  // Turn off torque when configuring items in EEPROM area.
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, mode);
  // Reenable if torque was on prior.
  if (t_e) dxl.torqueOn(DXL_ID);
}

float get_level_angle(uint8_t DXL_ID) {
  // When arm is ready: fill in details.
  //case
  return dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
}

void get_level_angles() {
  counter = -1;
  for (auto DXL_ID : DXL_IDS) {
    counter++;
    level_angles[counter] = get_level_angle(DXL_ID);
  }
}

// Get angles relative to horizon.
void get_horizon_angles() {
  get_level_angles();
  for (int i = 0; i < SERVOS; i++) {
    if (i == 0) horizon_angles[i] = level_angles[i];
    else horizon_angles[i] = horizon_angles[i-1] + level_angles[i];
  }
}

// Calculate torque threshold constants. Docment later.
void calculate_torque_threshold_constants(float* torque_threshold_constants) {
  // Set startup angles, get leveled angles.
  for (auto DXL_ID : DXL_IDS) {
    dxl.ledOn(DXL_ID);
    dxl.torqueOn(DXL_ID);
    set_operating_mode(DXL_ID, OP_CURRENT_BASED_POSITION);
    dxl.setGoalCurrent(DXL_ID, STATIC_CURRENT);
    dxl.setGoalPosition(
      DXL_ID,
      clamp(STARTUP_ANGLES.at(DXL_ID),
            MIN_MAX_ANGLES.at(DXL_ID).first,
            MIN_MAX_ANGLES.at(DXL_ID).second),
      UNIT_DEGREE);
    delay(50);
    dxl.ledOff(DXL_ID);
  }
  // Wait until motors are in position.
  delay(500);

  // Get angles relative to horizon.
  get_horizon_angles();

  // Calculate torque constants.
  float accum = 0.0;
  float m_t;
  counter = 6;
  for (auto rDXL_ID = DXL_IDS.rbegin(); rDXL_ID != DXL_IDS.rend(); ++rDXL_ID) {
    counter--;
    if (counter < 0) break;

    // [m]easured_[t]orque
    m_t = dxl.getPresentCurrent(*rDXL_ID);
    // Avoid division by 0.
    torque_threshold_constants[counter] = (m_t - accum)/(cos(level_angles[counter] * M_PI/180) || 0.001);
    accum += m_t;
  }
}

void calculate_torque_thresholds(float* torque_thresholds) {
  float accum = 0.0;
  float c_t;
  counter = 6;
  get_horizon_angles();
  for (auto rDXL_ID = DXL_IDS.rbegin(); rDXL_ID != DXL_IDS.rend(); ++rDXL_ID) {
    counter--;
    if (counter < 0) break;

    // [c]alculated_[t]orque
    c_t = torque_threshold_constants[counter]*cos(level_angles[counter] * M_PI/180);
    torque_thresholds[counter] = c_t + accum;
    accum += c_t;
  }
}

bool in_quadrant_I_IV(uint8_t DXL_ID) {
  float level_angle = get_level_angle(DXL_ID);
  return (level_angle < 90 || level_angle > 270);
}

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  USB_SERIAL.begin(115200);  // Should match tele_arm_micro/ppm baudrate.
  while (!USB_SERIAL);
  //USB_SERIAL.println("SETUP BEGIN");

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Initialize our motors.
  for (auto DXL_ID : DXL_IDS) {
    // Get DYNAMIXEL information and mask unresponsive ids.
    if (!dxl.ping(DXL_ID)) mask[DXL_ID] = 0;
    if (!mask[DXL_ID]) continue;
  }

  // Switches to OP_CURRENT_BASED_POSITION and sets current to STATIC_CURRENT.
  // Remain in this mode.
  calculate_torque_threshold_constants(torque_threshold_constants);

  counter = -1;
  for (auto DXL_ID : DXL_IDS) {
    counter++;
    if (counter > SERVOS - 1) break;
    else if (!mask[DXL_ID]) {
      // Fill in angles before entering loop.
      arm_data.floats[counter] = clamp(
        STARTUP_ANGLES.at(DXL_ID),
        MIN_MAX_ANGLES.at(DXL_ID).first,
        MIN_MAX_ANGLES.at(DXL_ID).second);
      continue;
    }

    // Enforce MIN_MAX_ANGLES.
    arm_data.floats[counter] = clamp(
      dxl.getPresentPosition(DXL_ID, UNIT_DEGREE),
      MIN_MAX_ANGLES.at(DXL_ID).first,
      MIN_MAX_ANGLES.at(DXL_ID).second);
  }

  //USB_SERIAL.println("SETUP END");
}

void loop() {
  float pres_v;
  float pres_t;
  bool right_half;
  // Threshold behavior.
  calculate_torque_thresholds(torque_thresholds);
  counter = -1;
  for (auto DXL_ID : DXL_IDS) {
    counter++;
    if (counter > SERVOS - 1) break;
    else if (!mask[DXL_ID]) continue;

    if (dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) < MIN_MAX_ANGLES.at(DXL_ID).first ||
        dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) > MIN_MAX_ANGLES.at(DXL_ID).second) {
      dxl.ledOn(DXL_ID);
      set_operating_mode(DXL_ID, OP_CURRENT_BASED_POSITION);

      dxl.setGoalPosition(DXL_ID,
        clamp(
          dxl.getPresentPosition(DXL_ID, UNIT_DEGREE),
          MIN_MAX_ANGLES.at(DXL_ID).first,
          MIN_MAX_ANGLES.at(DXL_ID).second),
        UNIT_DEGREE);
      dxl.setGoalCurrent(DXL_ID, STATIC_CURRENT);
    } else {
      // Threshold behavior.
      pres_v = dxl.getPresentVelocity(DXL_ID);
      pres_t = dxl.getPresentCurrent(DXL_ID);
      if (threshold_crossed[DXL_ID]) {
        if (abs(pres_v) < 15) {
          set_operating_mode(DXL_ID, OP_CURRENT_BASED_POSITION);

          dxl.setGoalPosition(DXL_ID,
            //clamp(
                dxl.getPresentPosition(DXL_ID, UNIT_DEGREE),
            //  MIN_MAX_ANGLES.at(DXL_ID).first,
            //  MIN_MAX_ANGLES.at(DXL_ID).second),
            UNIT_DEGREE);
          dxl.setGoalCurrent(DXL_ID, STATIC_CURRENT);

          threshold_crossed[DXL_ID] = false;
        } else {
          dxl.ledOn(DXL_ID);
          dxl.setGoalCurrent(DXL_ID, MOVING_CURRENT * sgn(pres_v));
        }
      } else {
        dxl.ledOff(DXL_ID);
        right_half = in_quadrant_I_IV(DXL_ID);
        if ((!right_half && pres_t < torque_thresholds[counter] - 25) ||
            (right_half && pres_t > torque_thresholds[counter] + 25)) {
          dxl.ledOn(DXL_ID);
          dxl.setGoalPosition(DXL_ID,
            //clamp(
                dxl.getPresentPosition(DXL_ID, UNIT_DEGREE),
            //  MIN_MAX_ANGLES.at(DXL_ID).first,
            //  MIN_MAX_ANGLES.at(DXL_ID).second),
            UNIT_DEGREE);
          dxl.ledOff(DXL_ID);
        }
        if (abs(pres_v) > 20) {
          // Large adjustments.
          threshold_crossed[DXL_ID] = true;
          set_operating_mode(DXL_ID, OP_CURRENT);
        }
      }
    }
  }

  // Read angles.
  counter = -1;
  for (auto DXL_ID : DXL_IDS) {
    counter++;
    if (counter > SERVOS - 1) break;
    else if (!mask[DXL_ID]) continue;

    arm_data.floats[counter] = clamp(
      dxl.getPresentPosition(DXL_ID, UNIT_DEGREE),
      MIN_MAX_ANGLES.at(DXL_ID).first,
      MIN_MAX_ANGLES.at(DXL_ID).second);
    //arm_data.floats[counter] = dxl.getPresentCurrent(counter);
  }
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

    if (recv_cmd_id == communication::SELFCHECK_CMD_ID) {
      if (status_data->mode == 1){
        selfcheck_data.mode = 1;
        selfcheck_data.debug_int = status_data->debug_int;
      
        minipc_session.Pack(packet_to_send, (void*)&selfcheck_data, communication::SELFCHECK_CMD_ID);
        USB_SERIAL.write(packet_to_send, minipc_session.GetPacketLen(communication::SELFCHECK_CMD_ID));
      } else if (status_data->mode == 2){
        // Respond with ID:
        //  ARM: 130 (130 - 127 = 2)
        //  See repo irm_tele_arm `config.py` for details
        selfcheck_data.mode = 2;
        selfcheck_data.debug_int = 130;
      
        minipc_session.Pack(packet_to_send, (void*)&selfcheck_data, communication::SELFCHECK_CMD_ID);
        USB_SERIAL.write(packet_to_send, minipc_session.GetPacketLen(communication::SELFCHECK_CMD_ID));
      }
    }
  }

  //arm_data.floats[1] = torque_thresholds[1];
  //arm_data.floats[2] = torque_thresholds[2];
  //arm_data.floats[3] = torque_thresholds[3];
  arm_data.floats[4] = torque_thresholds[2];
  //arm_data.floats[5] = torque_thresholds[5];
  //arm_data.floats[1] = torque_threshold_constants[2];
  //arm_data.floats[2] = -1*cos(level_angles[2] * M_PI/180);
  //arm_data.floats[3] = level_angles[2];
  //arm_data.floats[4] = torque_threshold_constants[2]*-1*cos(level_angles[2] * M_PI/180);
  arm_data.floats[5] = dxl.getPresentCurrent(2);
  // Send angles.
  minipc_session.Pack(packet_to_send, (void*)&arm_data, communication::ARM_CMD_ID);
  USB_SERIAL.write(packet_to_send, minipc_session.GetPacketLen(communication::ARM_CMD_ID));
}
