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
// See !!!NOTICE!!!

#include <Arduino.h>

#include "wfly_ppm.h"

using namespace wfly_ppm;

uint32_t ppm[CHANNELS];
uint8_t current_channel;
uint32_t ppm_delay;
uint32_t time_elapsed;
bool enabled = false;
bool state;

#ifdef SAMD
void TC4_Handler(void) {
#elif defined ASR
ISR(TIMER1_COMPA_vect) {
  TCNT1 = 0;  // Zero timer.
#endif
  // Wait for frame to end.
  if (!enabled && current_channel == 0) {
    ppm_delay = PPM_FRAME_LEN * MICROSECOND_SCALAR;
#ifdef SAMD
    TC4->COUNT32.CC[0].reg = ppm_delay;
#elif defined ASR
    OCR1A = ppm_delay;
#endif
    // Get data.
    cli();
    return;
  }

  digitalWrite(SIGPIN, state);

  if (state) {
    // Set wait to PPM_PULSE_LEN microseconds.
    ppm_delay = PPM_PULSE_LEN * MICROSECOND_SCALAR;
  } else {
    // Set wait:
    if (current_channel > CHANNELS - 1) {
      // to the end of the frame and reset.
      ppm_delay = (PPM_FRAME_LEN -
                   PPM_PULSE_LEN -
                   time_elapsed) * MICROSECOND_SCALAR;
      current_channel = 0;
      time_elapsed = 0;
      // Get data.
      cli();
    } else {
      // to the next pulse.
      ppm_delay = (ppm[current_channel] - PPM_PULSE_LEN) * MICROSECOND_SCALAR;
      time_elapsed += ppm[current_channel++];
    }
  }

#ifdef SAMD
  TC4->COUNT32.CC[0].reg = ppm_delay;  // Write to register.
#elif defined ASR
  OCR1A = ppm_delay;
#endif
  state = !state;                      // Toggle state.
}


WFly::WFly(uint32_t min_d_, uint32_t max_d_, uint32_t default_d_,
           float min_v_, float max_v_) {
  min_d = min_d_;
  max_d = max_d_;
  default_d = default_d_;
  min_v = min_v_;
  max_v = max_v_;
  interval_d = max_d - min_d;
  interval_v = max_v - min_v;
  for (int i = 0; i < CHANNELS; i++) {
    ppm[i] = default_d;
  }
};


void WFly::init() {
  current_channel = 0;
  time_elapsed = 0;
  enabled = false;
  state = ON;

  initialize_pins();
  initialize_gclk();
}


void WFly::insert(float* data, int len, int offset) {
  float scalar;  // scalar < 1
  for (int i = 0; i < len; i++) {
    scalar = (clamp(data[i], min_v, max_v) - min_v) / interval_v;
    ppm[offset + i] = (uint32_t) (min_d + scalar * interval_d + 0.5);
  }
}

void WFly::insert(uint32_t* data, int len, int offset) {
  for (int i = 0; i < len; i++) {
    ppm[offset + i] = data[i];
  }
}

void WFly::disable_output() {
  enabled = false;
}

void WFly::enable_output() {
  enabled = true;
}

bool WFly::toggle_output() {
  enabled = !enabled;
  return enabled;
}

void WFly::initialize_pins() {
  pinMode(SIGPIN, OUTPUT);
  digitalWrite(SIGPIN, OFF);
}

// !!!NOTICE!!!
// WILL BREAK OPENRB-150!!!!
// Any attempt to modify GCLK register blocks everything including uploading.
// TO FIX STUCK BOARD 赛博电灯泡:
//  Flash tele_arm/saveme while pressing reset button at the right time (good luck).
// TIPS:
//  Try holding reset down and releasing it when you feel the time is right.
//  This may rename your device to usb-ARDUINO_LLC*OpenRB*.
//  This went away (for me) after successfully uploading another sketch.

void WFly::initialize_gclk() {
#ifdef SAMD
  //
  // Generic Clock Initialisation
  //
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(3) |            // Set clock divisor to 3 -> 48MHz/3 = 16MHz.
                     GCLK_GENDIV_ID(0);              // Select GCLK0.
                     
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |           // Enable generic clock
                      GCLK_CLKCTRL_GEN_GCLK0 |       // GCLK0 at 48MHz
                      GCLK_CLKCTRL_ID(GCM_TC4_TC5);  // As a clock source for TC4 and TC5.
  while (GCLK->STATUS.bit.SYNCBUSY);                 // Wait for synchronization.

  //
  // TC4 Initialisation
  //
  TC4->COUNT32.CTRLA.reg = TC_CTRLA_ENABLE |         // Enable TC4
                           TC_CTRLA_MODE_COUNT32 |   // As 32-bit timer on TC4
                           TC_CTRLA_PRESCALER_DIV8;  // With prescaler of 8 -> 0.5us.
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);          // Wait for synchronization.

  //
  // Configure interrupt request
  //
  NVIC_DisableIRQ(TC4_IRQn);
  NVIC_ClearPendingIRQ(TC4_IRQn);
  NVIC_SetPriority(TC4_IRQn, 0);
  NVIC_EnableIRQ(TC4_IRQn);

  TC4->COUNT32.INTENSET.bit.MC0 = 1;         // Enable the TC4 interrupt request.
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);  // Wait for synchronization.
#elif defined ASR
  cli();
  // Zero TCCR1 register.
  TCCR1A = 0;
  TCCR1B = 0;

  OCR1A = 100;

  TCCR1B |= (1 << WGM12);   // Turn on CTC mode
  TCCR1B |= (1 << CS11);    // 8 prescaler: 0.5 microseconds at 16 MHz
  TIMSK1 |= (1 << OCIE1A);  // Enable timer compare interrupt
  state = ON;

  sei();
#endif
}
