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

// In microseconds
#define PPM_FRAME_LEN 22500
#define PPM_PULSE_LEN 300
// Counter will be in 0.5 microsecond intervals
#define MICROSECOND_SCALAR 2
// PPM settings
#define CHANNELS 16
#define SIGPIN 10
#define ON 1
#define OFF ~ON

static uint32_t ppm[CHANNELS];
static uint8_t current_channel;
static uint32_t ppm_delay;
static uint32_t time_elapsed;
static bool state;

void TC4_Handler(void);

namespace wfly_ppm {

class WFly {
 public:
  WFly();

  /**
   * @brief Initialize pins and clock registers.
   * @note Disabled by default; needs to be enabled after initialization.
   */
  void init();

  /**
   * @brief Disable PPM.
   */
  void disable_ppm();

  /**
   * @brief (Re)enable PPM.
   */
  void enable_ppm();

  /**
   * @brief Set the data to send.
   */
  void set(int* data, int len = 16, int offset = 0);

 private:
  /**
   * @brief Initialization helper methods.
   */
  void initialize_pins();
  void initialize_gclk();

}; /* class WFly */

} /* namespace wfly_ppm */
