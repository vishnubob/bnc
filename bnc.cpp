/**
 ** Author: Giles F. Hall 
 ** Email: <ghall -at- csh (dot) rit (dot) edu>
 **/

#ifndef F_CPU
#define F_CPU           16000000UL  // 16 MHz
#endif

#include                <stdlib.h>
#include                <avr/io.h>
#include                <avr/interrupt.h>
#include                <util/delay.h>
#include                <arduino/Arduino.h>

// arduino redefines int, which bugs out stdio.h (needed for sscanf)
// see: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1207028702/3
#undef int
#include                <stdio.h>

// The timers are configured with the prescaler set to 8, which means every
// 8 clock cycles equals on tick on the counter.  This is a constant to help
// convert timer cycles back to real time.
#define FREQ_DIV_8      (F_CPU / 8)

// Defines how many ticks a millisecond equals on our clock
#define MILLITICK       (FREQ_DIV_8 * .001)

void spin();

// I have found that the sensor in my rig is prone to triggering the interrupt
// immediately after a real signal, likely related to capactive effects.  This
// tricks the system into thinking the platter is suddenly traveling a lot
// faster, and can cause visible glitches.  A 15K RPM drive spins 250 times a
// second, where a single revolution requires 4ms.  4ms is exactly 8000 timer
// cycles on our timer, which means anything half of 8000 is going faster than
// any known HD.  If such a value is captured, it is ignored.
#define SPURIOUS_INT    4000

// Helper macros for frobbing bits
#define bitset(var,bitno) ((var) |= (1 << (bitno)))
#define bitclr(var,bitno) ((var) &= ~(1 << (bitno)))
#define bittst(var,bitno) (var& (1 << (bitno)))

#define TACH_PIN 2
#define STROBE_PIN 3

// The period of the platter in timer ticks
int period;

/**
 ** Setup
 **/

// Configure the Serial Port, LED Pins, Timers, and Hardware Interrupt
void SetupHardware(void)
{
  // setup serial
  Serial.begin(115200);

  Serial.print("[");
  // setup output
  //pinMode(BLU, OUTPUT);
  Serial.print("L");

  // disable global interrupts
  cli();

  // setup timer0 - 8bit
  // resonsible for timing the LEDs
  TCCR2A = 0;
  TCCR2B = 0;  
  // select CTC mode
  bitset(TCCR2A, WGM21);
  // select prescaler clk / 8
  bitset(TCCR2B, CS21);
  // enable compare interrupt
  OCR2A = 100;
  bitset(TIMSK2, OCIE2A);
  Serial.print("2");
  
  // setup timer1 - 16bit
  // responsible for timing the rotation of the platter
  TCCR1B = 0;
  TCCR1A = 0;
  // select prescaler clk / 8
  bitset(TCCR1B, CS10);
  bitset(TCCR1B, CS11);
  // reset timer
  TCNT1 = 0;
  // enable overflow interrupt
  //bitset(TIMSK1, TOIE1);
  Serial.print("1");

  pinMode(STROBE_PIN, OUTPUT);
  
  pinMode(TACH_PIN, INPUT);
  digitalWrite(TACH_PIN, HIGH);
  attachInterrupt(0, spin, RISING);
  Serial.print("G");

  // set the rotational period to 0
  period = 0;
  
  // enable global interrupts
  sei();

  Serial.println("]");
  Serial.flush();
}


/**
 ** Serial Output
 **/

// Report the status of the system, plus important variables over the serial
// port
void report_status_to_serial(void)
{
}

/**
 ** Serial Input
 **/

// Sit and spin until there is serial data available
void __inline__ wait_for_serial_input()
{
    while (Serial.available() < 1) {}
}

/**
 ** Setup
 **/

// Top level setup, called by the Arduino core
void setup(void)
{
    SetupHardware();
}

/**
 ** Main Loop
 **/

// Top level loop, call by the Arduino core
void loop(void)
{
    int cmd;
    int slice, value;

    Serial.print("~ ");
    wait_for_serial_input();

    cmd = Serial.read();
    Serial.println("");
    switch(cmd)
    {
        case 'p':
            Serial.println(period);
            Serial.println(TCNT1);
            break;
        default:
            Serial.print("Unknown command: ");
            Serial.println(cmd);
            break;
    }
    Serial.print("> ");
    Serial.flush();
}


/**
 ** Interrupts
 **
 ** The interrupts provide the accurate timing needed for this project.
 ** 
 **/

// This interrupt is called on every pulse of the sensor, which represents
// one full rotation of the platter.  It is responsible for timing the
// revolution, and for initiating the first draw instance.
void spin()
{
  // Capture the 16bit count on timer1, this represents one revolution
  size_t putative_period = TCNT1;
  // If the period is shorter than our threshold, don't do anything
  if(putative_period < SPURIOUS_INT)
  {
    return;
  }
  period = putative_period;
  //digitalWrite(STROBE_PIN, HIGH);
  PORTD |= _BV(3);
  TCNT1 = 0;
  TCNT2 = 0;
  OCR2A = 100;
  // Write out the first slice to the LEDs to PORTD
  //PORTD = FrameBuffer[page_visible][0];
  // Divide the time of single platter rotation by the number of drawable
  // divisions
}

// This interrupt is called every time timer0 counts up to the 8bit value
// stored in the register "ORC0A", which is configured in INT0 interrupt.
// It is responsible for drawing out all the slices of the frame buffer
// during the exact moment the slot is in its proper rotational position.
// Using a 7200RPM drive with 255 divisions, this interrupt is called 
// 31,200 times a second.
ISR(TIMER2_COMPA_vect) {
  PORTD &= ~(_BV(3));
}

// If the platter spin time overflows timer1, this is called
ISR(TIMER1_OVF_vect) {
}
