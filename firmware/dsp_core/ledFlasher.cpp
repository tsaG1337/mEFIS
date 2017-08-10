/*
  mEICAS - my Engine Indicating Crew Alerting System.
  Copyright (C) 2017 Patrick Bihn <pbihn at uni minus bremen dot de>

  You can find the full repo here:
  https://github.com/tsaG1337/mEFIS

  This program is free software: you can redistribute it and/or modify
  it under the terms of the version 3 GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#include "Arduino.h"
#include "ledFlasher.h"

ledFlasher::ledFlasher(char PinNumber) {
  _ledPin = PinNumber;
  pinMode(_ledPin, OUTPUT);
}

void ledFlasher::pattern(int patternType) {
  ledPattern = patternType;
}

void ledFlasher::update() {
  // check to see if it's time to change the state of the LED
  unsigned long currentMillis = millis();
  switch (ledPattern) {
    case 0:
      if ((_ledState == HIGH) && (currentMillis - _previousMillis >= _state1OnTime))
      {
        _ledState = LOW;  // Turn it off
        _previousMillis = currentMillis;  // Remember the time
        digitalWrite(_ledPin, _ledState);  // Update the actual LED
      }
      else if ((_ledState == LOW) && (currentMillis - _previousMillis >= _state1OffTime))
      {
        _ledState = HIGH;  // turn it on
        _previousMillis = currentMillis;   // Remember the time
        digitalWrite(_ledPin, _ledState);   // Update the actual LED
      }
      break;

    case 1:
      if (_state2PatternState <= 3) {
        if (((_state2PatternState == 0) || (_state2PatternState == 2)) && (currentMillis - _previousMillis >= _state2OffTime))
        {
          _ledState = HIGH;  // Turn it off
          _previousMillis = currentMillis;  // Remember the time
          digitalWrite(_ledPin, _ledState);  // Update the actual LED
          _state2PatternState++;

        }
        else if (((_state2PatternState == 1) || (_state2PatternState == 3)) && (currentMillis - _previousMillis >= _state2OnTime))
        {
          _ledState = LOW;  // turn it on
          _previousMillis = currentMillis;   // Remember the time
          digitalWrite(_ledPin, _ledState);   // Update the actual LED
          _state2PatternState++;
        }
      }
      else if ((_state2PatternState == 4) && (currentMillis - _previousMillis >= (_state2DelayTime)))
      {
        _ledState = LOW;  // turn it on
        _previousMillis = currentMillis;   // Remember the time
        digitalWrite(_ledPin, _ledState);   // Update the actual LED
        _state2PatternState = 0;
      }
      break;

    case 2: {
        digitalWrite(_ledPin, LOW);
      }
      break;
  }
}


