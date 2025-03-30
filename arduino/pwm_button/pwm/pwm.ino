/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2025 RoboMaster.                                          *
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

// Configurable variables
const int servoPin = 9;
const int buttonPin = 2; // Button connected to D2
const int ledPin = 13;   // Built-in LED on pin 13
const unsigned long clockFreq = 16000000;
const unsigned long outputFreq = 50;
float duty = 0.025;
bool buttonPressed = false;

void setup() {
  pinMode(servoPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Enable internal pull-up resistor for the button
  pinMode(ledPin, OUTPUT);          // Configure LED pin as output

  // Configure Timer1 for PWM generation
  TCCR1A = _BV(COM1A1) | _BV(WGM11); // Non-inverting mode, Fast PWM mode (TOP = ICR1)
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // Fast PWM mode, prescaler = 8
  ICR1 = (clockFreq / (outputFreq * 8)) - 1; // Set TOP value for frequency
  OCR1A = (unsigned long)(duty * clockFreq) / (8 * outputFreq); // Set duty cycle

  digitalWrite(ledPin, LOW);
}


void loop() {
  // Check for button press (LOW because of pull-up)
  if (digitalRead(buttonPin) == LOW && !buttonPressed) {
    buttonPressed = true;
    duty = (duty == 0.07) ? 0.025 : 0.07; // Toggle between 0.5 and 0.0
    OCR1A = (unsigned long)(duty * clockFreq) / (8 * outputFreq); // Update duty cycle
    digitalWrite(ledPin, duty == 0.5 ? HIGH : LOW); // Turn LED on if duty is 0.5, off otherwise
    delay(100);
  } else if (digitalRead(buttonPin) == HIGH) {
    buttonPressed = false;
    delay(100);
  }
}
