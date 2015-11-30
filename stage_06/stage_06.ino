/** Stage 06. The ultimate version
 *  Added extra modes
 *  Complex arithmetic has been dropped for sake
 *  of the predefined modes
 */

#define DEBUG 1

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

// Arduino standard (AVR) methods for setting (sbi) or
// clearing (cbi) bits in PORT (and other) variables
// Use _MMIO_BYTE(addr) with pointers
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Read bit from PORT (and other) variables
#define rbi(sfr, bit) (_SFR_BYTE(sfr) & _BV(bit))

// Compare 32-bit arrays (ca, cb are pointers)
#define cmp32(ca, cb) (!( *((uint8_t *) ca) ^ \
  *((uint8_t *) cb)     | *((uint8_t *) ca + 1) ^ \
  *((uint8_t *) cb + 1) | *((uint8_t *) ca + 2) ^ \
  *((uint8_t *) cb + 2) | *((uint8_t *) ca + 3) ^ \
  *((uint8_t *) cb + 3)))

#define SEQN 8 // Maximum sequence index (should be n^2)
#define LEDS 4 // Maximum led count (have 4 of them)
#define MODC 4 // Total modes count (4 as well)
#define CMDC 7 // Total commands count (7 yet)
#define CMDX 4 // Maximum command's size (4 bytes)

// Custom types to handle PORT (and other) variables
typedef struct {
  uint8_t bit;
  uint16_t port;
  uint16_t pin;
  uint16_t ddr;
} pin_t;

// Leds' (PWM) array: 5 = PD5, 6 = PD6, 9 = PB1, 10 = PB2
const pin_t led[LEDS] = {
  {PD5, (uint16_t) &PORTD,
    (uint16_t) &PIND, (uint16_t) &DDRD},
  {PD6, (uint16_t) &PORTD,
    (uint16_t) &PIND, (uint16_t) &DDRD},
  {PB1, (uint16_t) &PORTB,
    (uint16_t) &PINB, (uint16_t) &DDRB},
  {PB2, (uint16_t) &PORTB,
    (uint16_t) &PINB, (uint16_t) &DDRB}
};

// Leds' order (Output Control Registers for PWM)
const uint16_t lord[LEDS] = {
  (uint16_t) &OCR0B, //  5 = OC0B (Timer0 OC B)
  (uint16_t) &OCR0A, //  6 = OC0A (Timer0 OC A)
  (uint16_t) &OCR1A, //  9 = OC1A (Timer1 OC A)
  (uint16_t) &OCR1B  // 10 = OC1B (Timer1 OC B)
};

// Leds' mode matrix (values in leds' order)
const uint8_t mode[MODC][SEQN][LEDS] = {
  {   // Mode 1. Waves
    { /* SEQ1 */ 0, 85, 255, 85},
    { /* SEQ2 */ 85, 0, 85, 255},
    { /* SEQ3 */ 255, 85, 0, 85},
    { /* SEQ4 */ 85, 255, 85, 0},
    { /* SEQ5 */ 0, 85, 255, 85},
    { /* SEQ6 */ 85, 0, 85, 255},
    { /* SEQ7 */ 255, 85, 0, 85},
    { /* SEQ8 */ 85, 255, 85, 0}
  },{ // Mode 2. Collisions
    { /* SEQ1 */ 255, 127, 63, 0},
    { /* SEQ2 */ 255, 191, 0, 0},
    { /* SEQ3 */ 191, 255, 0, 0},
    { /* SEQ4 */ 63, 127, 255, 0},
    { /* SEQ5 */ 0, 63, 127, 255},
    { /* SEQ6 */ 0, 0, 191, 255},
    { /* SEQ7 */ 0, 0, 255, 191},
    { /* SEQ8 */ 0, 255, 127, 63}
  },{ // Mode 3. Drops
    { /* SEQ1 */ 255, 0, 0, 255},
    { /* SEQ2 */ 191, 63, 63, 191},
    { /* SEQ3 */ 127, 127, 127, 127},
    { /* SEQ4 */ 63, 191, 191, 63},
    { /* SEQ5 */ 0, 255, 255, 0},
    { /* SEQ6 */ 63, 191, 191, 63},
    { /* SEQ7 */ 127, 127, 127, 127},
    { /* SEQ8 */ 191, 63, 63, 191}
  },{ // Mode 4. Ouroboros
    { /* SEQ1 */ 255, 127, 0, 63},
    { /* SEQ2 */ 127, 63, 255, 0},
    { /* SEQ3 */ 63, 0, 127, 255},
    { /* SEQ4 */ 0, 255, 63, 127},
    { /* SEQ5 */ 255, 127, 0, 63},
    { /* SEQ6 */ 127, 63, 255, 0},
    { /* SEQ7 */ 63, 0, 127, 255},
    { /* SEQ8 */ 0, 255, 63, 127}
  }
};

// PushButton's pin (PWM): 3 = PD3
const pin_t button = { PD3,
  (uint16_t) &PORTD, (uint16_t) &PIND, (uint16_t) &DDRD
};

// Serial message strings
const char *STR_SUB = "Entering setup...";
const char *STR_SUE = "Setup finished.";
const char *STR_SSP = "Lights suspended...";
const char *STR_RSM = "Lights' run resumed.";
const char *STR_M_1 = "Mode 1. Waves.";
const char *STR_M_2 = "Mode 2. Collisions.";
const char *STR_M_3 = "Mode 3. Drops.";
const char *STR_M_4 = "Mode 4. Ouroboros.";
#if (DEBUG == 1)
const char *STR_DBG = "Input: ";
#endif

// Serial commands
const char command[CMDC][CMDX] = {
  {'o', 'n', 0, 0},     // Lights on
  {'o', 'f', 'f', 0},   // Lights off
  {'m', 'o', 'd', 'e'}, // Next mode
  {'m', '1', 0, 0},     // Mode 1 (m1)
  {'m', '2', 0, 0},     // Mode 2 (m2)
  {'m', '3', 0, 0},     // Mode 3 (m3)
  {'m', '4', 0, 0}      // Mode 4 (m4)
};

// This is not OOP - so global variables are ok
volatile uint8_t lrun = 0; // Lights' run-stop variable
volatile uint8_t bmod = 0; // Mode change trigger variable
// Indices for the mode, the sequence, and the rest
volatile uint8_t mod = 0, seq = 0, i = 0;

char str_in[CMDX] = {0, 0, 0, 0}; // Serial input string

/** ISR (Interrupt Service Routine) setup macro
 *  Used to set handler for interrupts from
 *  the timer (on overflow). delay() and milis()
 *  fuctions should not be used here (as they
 *  disable global interrupts)
 */
ISR(TIMER2_OVF_vect) {
  // Check for lights' run enabled
  if (!lrun) return;

  // Trim led index to be always in range
  seq &= (SEQN - 1);
  // Set each of 4 leds
  _MMIO_BYTE(lord[0]) = mode[mod][seq][0];
  _MMIO_BYTE(lord[1]) = mode[mod][seq][1];
  _MMIO_BYTE(lord[2]) = mode[mod][seq][2];
  _MMIO_BYTE(lord[3]) = mode[mod][seq][3];
  seq ++; // Increment the index variable
}

/** ISR (Interrupt Service Routine) setup macro
 *  Used to set handler for interrupts from PushButton
 *  on pin 3 (PD3 line). It will handle interrupts
 *  on ANY CHANGE of PushButton's pin state
 */
ISR(INT1_vect) {
  if (rbi(_MMIO_BYTE(button.pin), button.bit)) {
    // Button released: allow lights to run again
    lrun = 1;
    mod = (mod + 1) & (MODC - 1); // Shift to the next mode
    bmod = 1; // Trigger the mode change
  } else {
    // Button pressed: suspend running lights
    lrun = 0;
  }
}

/** Setup routine
 *  All configuration trcks go here
 */
void setup() {
  // Set up Serial console
  Serial.begin(9600);
  Serial.println(STR_SUB);

  cli(); // Turn off global interrupts

  // Initialize pins (set for OUTPUT and clear for INPUT)
  for (i = 0; i < LEDS; i ++) {
    // Set Led's pin as OUTPUT
    sbi(_MMIO_BYTE(led[i].ddr), led[i].bit);
  }
  i = 0; // Reset index for loop()
  // Set PushButton's pin (PD3) as INPUT
  cbi(_MMIO_BYTE(button.ddr), button.bit);

  // Reset Timers (Timer Counter Control Registers)
  TCCR0A = TCCR0B = 0; // Timer0 disabled and detached
  TCCR1A = TCCR1B = 0; // Timer1 disabled and detached
  TCCR2A = TCCR2B = 0; // Timer2 disabled and detached

  // Disable interrupts for Timer0 & Timer1
  TIMSK0 = TIMSK1 = 0;
  // Enable Fast PWM for Timer0 & Timer1 (with 0xFF limit)
  sbi(TCCR0A, WGM00); sbi(TCCR0A, WGM01); // Fast PWM
  sbi(TCCR1A, WGM10); sbi(TCCR1B, WGM12); // Fast PWM 8-bit
#if (DEBUG == 1)
  // Enable timers (at (16000000 / 8) / 256 = 7812.5 Hz)
  sbi(TCCR0B, CS01); // with /8 prescaler
  sbi(TCCR1B, CS11); // with /8 prescaler
#else
  // Enable timers (at (16000000 / 1) / 256 = 62500 Hz)
  sbi(TCCR0B, CS00); // Timer0 enabled (with /1 prescaler)
  sbi(TCCR1B, CS10); // Timer1 enabled (with /1 prescaler)
#endif
  // Set Compare match Output Mode for Timer0 & Timer1
  sbi(TCCR0A, COM0A1); // Timer0 attached to OC0A output
  sbi(TCCR0A, COM0B1); // Timer0 attached to OC0B output
  sbi(TCCR1A, COM1A1); // Timer1 attached to OC1A output
  sbi(TCCR1A, COM1B1); // Timer1 attached to OC1B output
  // Set Compare Register value for PWM on Timer0 & Timer1
  OCR0A = OCR0B = 0; // Timer0 1/256 of duty cycle
  OCR1A = OCR1B = 0; // Timer1 1/256 of duty cycle

  // Enable interrupts for Timer2
  sbi(TIMSK2, TOIE2); // Interrupts from overflow enabled
  // Enable Phase-Correct PWM (with OCR2A limit)
  sbi(TCCR2B, WGM22); sbi(TCCR2A, WGM20);
  // Enable Timer2 (at (16000000 / 1024) / 312 = 50 Hz)
  // Bits CSn0, CSn1, CSn2 for /1024 async prescaler
  sbi(TCCR2B, CS20); sbi(TCCR2B, CS21); sbi(TCCR2B, CS22);
  // Set Compare Register value for PWM on Timer2
  OCR2A = 155; // (155 + 1) * 2 = 312

  // Enable INT1 for PD3 (PushButton)
  sbi(EICRA, ISC10); // Trigger INT1 on ANY CHANGE
  sbi(EIMSK, INT1); // Turn on INT1

  sei(); // Turn global interrupts on again
  
  Serial.println(STR_SUE);
}

/** Loop routine
 *  This is the main code to be repeated -
 *  it may hold all the costly piece of code
 */
void loop() {
  if (bmod) {
    // The mode change has been triggered
    switch (mod) {
      case 0: Serial.println(STR_M_1); break;
      case 1: Serial.println(STR_M_2); break;
      case 2: Serial.println(STR_M_3); break;
      case 3: Serial.println(STR_M_4);
    }
    bmod = 0; // Release the mode change flag
  }
  // Check serial input buffer -
  // commands are supposed to be CMDX characters long
  if (Serial.available() && (i < CMDX)) {
    // Proceed: Serial input buffer has data
    // and input string has space to read into
    str_in[i] = Serial.read();
    // Skip all zeroes read
    if (str_in[i]) i ++;
    // Pause application for at least 1ms
    // for the Serial to be ready
    delayMicroseconds(1000);
  } else {
    // Proceed: Serial input buffer is empty
    // or input string is filled
    if (i) {
      // Display contents of input string
#if (DEBUG == 1)
      Serial.print(STR_DBG);
      Serial.println(str_in); // This is not safe
#endif
      // Find command in the input
      if (cmp32(str_in, command[0])) {
          // Run lights
          lrun = 1;
          Serial.println(STR_RSM);
      } else if (cmp32(str_in, command[1])) {
          // Suspend lights
          lrun = 0;
          Serial.println(STR_SSP);
      } else if (cmp32(str_in, command[2])) {
          // Switch to the next mode
          mod = (mod + 1) & (MODC -1);
          bmod = 1; // Trigger the mode change
      } else if (cmp32(str_in, command[3])) {
          // Select Mode 1 - Waves
          mod = 0;
          bmod = 1; // Trigger the mode change
      } else if (cmp32(str_in, command[4])) {
          // Select Mode 2 - Collisions
          mod = 1;
          bmod = 1; // Trigger the mode change
      } else if (cmp32(str_in, command[5])) {
          // Select Mode 3 - Drops
          mod = 2;
          bmod = 1; // Trigger the mode change
      } else if (cmp32(str_in, command[6])) {
          // Select Mode 4 - Ouroboros
          mod = 3;
          bmod = 1; // Trigger the mode change
      }
      // Clear the rest of Serial buffer and input string
      while (Serial.available()) {
        Serial.read();
        delayMicroseconds(1000);
      }
      *((uint32_t *) str_in) = i = 0; // Flush input string
    }
  }
}
