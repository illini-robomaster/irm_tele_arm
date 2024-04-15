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

#include <Arduino.h>

#include "wfly_ppm.h"

using namespace wfly_ppm;

void TC4_Handler(void) {
  if (state) {
    digitalWrite(SIGPIN, state);
    // Set wait to PPM_PULSE_LEN microseconds.
    TC4->COUNT32.CC[0].reg = (uint32_t) PPM_PULSE_LEN * MICROSECOND_SCALAR;
  } else {
    digitalWrite(SIGPIN, state);
    // Set wait:
    if (current_channel > CHANNELS) {
      // to the end of the frame and reset.
      TC4->COUNT32.CC[0].reg = (uint32_t) (PPM_FRAME_LEN -
                                           PPM_PULSE_LEN -
                                           time_elapsed) * MICROSECOND_SCALAR;
      current_channel = 0;
      time_elapsed = 0;
    } else {
      // to the next pulse.
      TC4->COUNT32.CC[0].reg = (uint32_t) (ppm[current_channel] -
                                           PPM_PULSE_LEN) * MICROSECOND_SCALAR;
      time_elapsed += ppm[current_channel++];
    }
  }
  state = ~state;  // Toggle state.
}


WFly::WFly() {};

void WFly::init() {
  current_channel = 0;
  time_elapsed = 0;
  state = ON;

  initialize_pins();
  initialize_gclk();
}

void disable_ppm() {
  TC4->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC4
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);     // Wait for synchronization
  TC4->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;      // Reset TC4
  while (TC4->COUNT32.CTRLA.bit.SWRST);         // Wait for synchronization
}

void enable_ppm() {
  TC4->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;  // Enable TC4
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);   // Wait for synchronization
}

void WFly::set(int* data, int len, int offset) {
  for (int i = 0; i < len; i++) {
    ppm[offset + i] = data[i];
  }
}

void WFly::initialize_pins() {
  pinMode(SIGPIN, OUTPUT);
  digitalWrite(SIGPIN, OFF);
}

void WFly::initialize_gclk() {
  //
  // Generic Clock Initialisation
  //
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(3) |            // Set clock divisor to 3 -> 48MHz/3 = 16MHz
                     GCLK_GENDIV_ID(0);              // Select GCLK0
                     
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |           // Enable generic clock
                      GCLK_CLKCTRL_GEN_GCLK0 |       // GCLK0 at 48MHz
                      GCLK_CLKCTRL_ID(GCM_TC4_TC5);  // As a clock source for TC4 and TC5.
  while (GCLK->STATUS.bit.SYNCBUSY);                 // Wait for synchronization.

  //
  // TC4 Initialisation
  //
  TC4->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 |   // 32-bit timer on TC4
                           TC_CTRLA_PRESCALER_DIV8;  // Prescaler of 8 -> 0.5us
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);          // Wait for synchronization

  //
  // Configure interrupt request
  //
  NVIC_DisableIRQ(TC4_IRQn);
  NVIC_ClearPendingIRQ(TC4_IRQn);
  NVIC_SetPriority(TC4_IRQn, 0);
  NVIC_EnableIRQ(TC4_IRQn);

  TC4->COUNT32.INTENSET.bit.MC0 = 1;         // Enable the TC4 interrupt request
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);  // Wait for synchronization
}
