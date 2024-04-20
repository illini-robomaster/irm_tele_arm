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
// DO NOT USE ON OPENRB-150!!!

#define ASR

// In microseconds
#define PPM_FRAME_LEN 35000
#define PPM_PULSE_LEN 200
// Counter will be in 0.5 microsecond intervals
#define MICROSECOND_SCALAR 2
// PPM settings
#define CHANNELS 16
#define SIGPIN 10
#define ON 1
#define OFF ~ON

extern uint32_t ppm[CHANNELS];
extern uint8_t current_channel;
extern uint32_t ppm_delay;
extern uint32_t time_elapsed;
extern bool enabled;
extern bool state;

#ifdef SAMD 
void TC4_Handler(void);
#elif defined ASR
#endif

namespace wfly_ppm {

// Arduino uses -std=gnu++11. std::clamp is introduced in c++17.
// Reimplement a simplified version here.
template<class T>
const T& clamp(const T& v, const T& lo, const T& hi) {
  return v < lo ? lo : hi < v ? hi : v;
}

class WFly {
 public:
  /**
   * @param min_d: Minimum delay to next pulse (in microseconds)
   *        max_d: Maximum delay to next pulse (in microseconds)
   *        min_v: Minimum value to send (used in scaling)
   *        max_v: Maximum value to send (used in scaling)
   * @note Scaling assumes all data sent to be in the same range per object.
   *       Data out of range will be clamped.
   */
  WFly(uint32_t min_d, uint32_t max_d, uint32_t default_d,
       float min_v, float max_v);

  /**
   * @brief Initialize pins and clock registers.
   * @note Disabled by default; needs to be enabled after initialization.
   */
  void init();

  /**
   * @brief Convert values between min_v and max_v to delays between
   *        min_d and max_d. Inserts delays into send buffer.
   * @param data:   pointer to buffer
   *        len:    length to retrieve
   *        offset: ppm[i + offset] = data[i]
   */
  void insert(float* data, int len, int offset = 0);

  /**
   * @brief Directly modify the send buffer.
   * @param data:   pointer to buffer
   *        len:    length to retrieve
   *        offset: ppm[i + offset] = data[i]
   */
  void insert(uint32_t* data, int len, int offset = 0);

  /**
   * @brief Disable PPM output.
   */
  void disable_output();

  /**
   * @brief (Re)enable PPM output.
   */
  void enable_output();

  /**
   * @breif Toggle PPM output on/off. Returns new state.
   */
  bool toggle_output();

 private:
  /**
   * @brief Initialization helper method.
   */
  void initialize_pins();
  // DO NOT FLASH ON OPENRB-150 - DOES NOT WORK - WILL BREAK OPENRB-150!!!!
  // Any attempt to modify GCLK register blocks everything including uploading.
  // TO FIX: Flash saveme while pressing reset button at the right time (good luck).
  void initialize_gclk();

  uint32_t min_d;
  uint32_t max_d;
  uint32_t default_d;
  float min_v;
  float max_v;
  uint32_t interval_d;
  uint32_t interval_v;
}; /* class WFly */

} /* namespace wfly_ppm */
