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

class ledFlasher
{
  private:

    int _ledPin = 0;      // the number of the LED pin
    int ledPattern = 2;

    int state1Val = 0;
    long _state1OnTime = 500;     // milliseconds of on-time
    long _state1OffTime = 500;    // milliseconds of off-time

    int state2Val = 0;
    int _state2PatternState = 0;
    long _state2OnTime = 100;     // milliseconds of on-time
    long _state2OffTime = 200;    // milliseconds of off-time
    int state3Val = 0;

    // These maintain the current state
    int _ledState;                 // ledState used to set the LED
    unsigned long _previousMillis;   // will store last time LED was updated

  public:
    ledFlasher(char PinNumber);
    void pattern(int patternType);
    void update();

};

