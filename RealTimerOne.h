/**
 * RealTimerOne - A Wiring library to easily set up a repeated task using Timer1
 * Copyright (C) 2012 Stefan Rado
 * 
 * Version 1.0
 * 
 * Losely based on TimerOne for Arduino by Jesse Tane and others:
 * http://code.google.com/p/arduino-timerone/
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef RealTimerOne_h
#define RealTimerOne_h

#include <Wiring.h>

class RealTimerOne {
  public:
    void setPeriod(unsigned long microseconds);

    void attachInterrupt(void (*isr)(), long microseconds = -1);
    void detachInterrupt();

    void start();
    void resume();
    void stop();

    void pwm(uint8_t pin, uint16_t duty, long microseconds = -1);
    void setPwmDuty(uint8_t pin, uint16_t duty);
    void disablePwm(uint8_t pin);

    unsigned long read();

  private:
    unsigned long cycles;
    uint8_t clockSource;
};

extern RealTimerOne RealTimer1;

#endif
