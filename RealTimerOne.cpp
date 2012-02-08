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
#include "RealTimerOne.h"

#define RESOLUTION 65536  // Timer1 is 16 bit

RealTimerOne RealTimer1;  // preinstatiate

void RealTimerOne::setPeriod(unsigned long microseconds) {
  Timer1.setClockSource(CLOCK_STOP); // stop the timer
  Timer1.setMode(0b1000);  // set mode 8: phase and frequency correct pwm

  cycles = (F_CPU / 2000000) * microseconds;                                // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
  if (cycles < RESOLUTION)              clockSource = CLOCK_NO_PRESCALE;    // no prescale, full xtal
  else if ((cycles >>= 3) < RESOLUTION) clockSource = CLOCK_PRESCALE_8;     // prescale by /8
  else if ((cycles >>= 3) < RESOLUTION) clockSource = CLOCK_PRESCALE_64;    // prescale by /64
  else if ((cycles >>= 2) < RESOLUTION) clockSource = CLOCK_PRESCALE_256;   // prescale by /256
  else if ((cycles >>= 2) < RESOLUTION) clockSource = CLOCK_PRESCALE_1024;  // prescale by /1024
  else         cycles = RESOLUTION - 1, clockSource = CLOCK_PRESCALE_1024;  // request was out of bounds, set as maximum

  ICR1 = cycles;     // ICR1 is TOP in p & f correct pwm mode

  Timer1.setClockSource(clockSource);   // reset clock select register, and starts the clock
}

void RealTimerOne::attachInterrupt(void (*isr)(), long microseconds) {
  if (microseconds > 0)
    setPeriod(microseconds);
  Timer1.attachInterrupt(INTERRUPT_OVERFLOW, isr, true);   // register the user's callback with the real ISR and sets the timer overflow interrupt enable bit
}

void RealTimerOne::detachInterrupt() {
  Timer1.setInterrupt(INTERRUPT_OVERFLOW, false);   // clears the timer overflow interrupt enable bit
}


void RealTimerOne::start() {
  TIMSK1 &= ~_BV(TOIE1);
  GTCCR |= _BV(PSRSYNC);  // reset prescaler (NB: shared with all 16 bit timers);

  TCNT1 = 0;
  while (TCNT1 == 0);     // Nothing -- wait until timer moved on from zero - otherwise get a phantom interrupt

  TIFR1 = 0xff;           // Clear interrupt flags
  TIMSK1 = _BV(TOIE1);    // sets the timer overflow interrupt enable bit
}

void RealTimerOne::resume() {
  Timer1.setClockSource(clockSource);
}

void RealTimerOne::stop() {
  Timer1.setClockSource(CLOCK_STOP);
}


void RealTimerOne::pwm(uint8_t pin, uint16_t duty, long microseconds) {  // expects duty cycle to be 10 bit (1024)
  if (microseconds > 0)
    setPeriod(microseconds);

  setPwmDuty(pin, duty);

  pinMode(pin, OUTPUT);
  switch (digitalPinToTimer(pin)) {
    case TIMER1A:
      Timer1.setOutputMode(CHANNEL_A, 0b10);
      break;
    case TIMER1B:
      Timer1.setOutputMode(CHANNEL_B, 0b10);
      break;
    case TIMER1C:
      Timer1.setOutputMode(CHANNEL_C, 0b10);
      break;
  }

  resume();
}

void RealTimerOne::setPwmDuty(uint8_t pin, uint16_t duty) {
  unsigned long dutyCycle = (cycles * duty) >> 10;

  switch (digitalPinToTimer(pin)) {
    case TIMER1A:
      Timer1.setOCR(CHANNEL_A, dutyCycle);
      break;
    case TIMER1B:
      Timer1.setOCR(CHANNEL_B, dutyCycle);
      break;
    case TIMER1C:
      Timer1.setOCR(CHANNEL_C, dutyCycle);
      break;
  }
}

void RealTimerOne::disablePwm(uint8_t pin) {
  switch (digitalPinToTimer(pin)) {
    case TIMER1A:
      Timer1.setOutputMode(CHANNEL_A, 0b00);
      break;
    case TIMER1B:
      Timer1.setOutputMode(CHANNEL_B, 0b00);
      break;
    case TIMER1C:
      Timer1.setOutputMode(CHANNEL_C, 0b00);
      break;
  }
}


/**
 * Returns the value of the timer in microseconds.
 */
unsigned long RealTimerOne::read() {
  unsigned int oldTCNT1 = TCNT1;

  uint8_t scale = 0;
  switch (clockSource) {
    case CLOCK_NO_PRESCALE:   scale =  0; break;
    case CLOCK_PRESCALE_8:    scale =  3; break;
    case CLOCK_PRESCALE_64:   scale =  6; break;
    case CLOCK_PRESCALE_256:  scale =  8; break;
    case CLOCK_PRESCALE_1024: scale = 10; break;
  }

  unsigned long tcnt1;
  do {
    tcnt1 = TCNT1;
  } while (TCNT1 == oldTCNT1);

  //if we are counting down add the top value to how far we have counted down
  if (tcnt1 < oldTCNT1)
    tcnt1 = (long)ICR1 + ((long)ICR1 - tcnt1);
  return ((tcnt1 * 1000L) / (F_CPU / 1000L)) << scale;
}
