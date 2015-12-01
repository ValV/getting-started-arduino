/** Stage 06. The ultimate Leonardo version
 *  Added extra modes
 *  Complex arithmetic has been dropped for sake
 *  of the predefined modes
 *  Timer2 replaced with Timer3
 *  Timer0 (PWM) replaced with Timer4 (PWM)
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
#define cmp32(ca, cb) (!(( *((uint8_t *) ca) ^ \
  *((uint8_t *) cb))     | (*((uint8_t *) ca + 1) ^ \
  *((uint8_t *) cb + 1)) | (*((uint8_t *) ca + 2) ^ \
  *((uint8_t *) cb + 2)) | (*((uint8_t *) ca + 3) ^ \
  *((uint8_t *) cb + 3))))

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

// Leds' (PWM) array: 5 = PC6, 6 = PD7, 9 = PB5, 10 = PB6
const pin_t led[LEDS] = {
  {PC6, (uint16_t) &PORTC,
    (uint16_t) &PINC, (uint16_t) &DDRC},
  {PD7, (uint16_t) &PORTD,
    (uint16_t) &PIND, (uint16_t) &DDRD},
  {PB5, (uint16_t) &PORTB,
    (uint16_t) &PINB, (uint16_t) &DDRB},
  {PB6, (uint16_t) &PORTB,
    (uint16_t) &PINB, (uint16_t) &DDRB}
};

// Leds' order (Output Control Registers for PWM)
const uint16_t lord[LEDS] = {
  (uint16_t) &OCR4A, //  5 = #OC4A (Timer4 OC A)
  (uint16_t) &OCR4D, //  6 =  OC4D (Timer4 OC D)
  (uint16_t) &OCR1A, //  9 =  OC1A (Timer1 OC A)
  (uint16_t) &OCR1B  // 10 =  OC1B (Timer1 OC B)
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

// PushButton's pin (PWM): 3 = PD1 (INT1)
const pin_t button = { PD1,
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

unsigned int us = 4000; // Delay 4k asm loops (1ms)

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
ISR(TIMER3_OVF_vect) {
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
  }
  i = 0; // Reset index for loop()
  // Set PushButton's pin (PD3) as INPUT
  cbi(_MMIO_BYTE(button.ddr), button.bit);
  // Set pins for Timer1 as output
  sbi(_MMIO_BYTE(led[2].ddr), led[2].bit);
  sbi(_MMIO_BYTE(led[3].ddr), led[3].bit);

  // Reset Timers (Timer Counter Control Registers)
  TCCR1A = TCCR1B = 0; // Timer1 disabled and detached
  TCCR3A = TCCR3B = 0; // Timer3 disabled and detached
  // Timer4 disabled and detached
  TCCR4A = TCCR4B = TCCR4C = TCCR4D = TCCR4E = 0;

  // Disable interrupts for Timer1, Timer3, Timer4
  TIMSK1 = TIMSK3 = TIMSK4 = 0;
  // Enable Fast PWM for Timer1 (with 0xFF limit)
  sbi(TCCR1A, WGM10); sbi(TCCR1B, WGM12); // Fast PWM 8-bit
  // Enable PWM mode for Timer4 (with OCR4C TOP limit)
  sbi(TCCR4A, PWM4A); // Fast PWM is the default PWM mode
  sbi(TCCR4C, PWM4D); // Fast PWM is the default PWM mode
  OCR4C = 0xFF; // TOP limit value for TCNT4 (8-bit)
#if (DEBUG == 1)
  // Enable timers (at (16000000 / 8) / 256 = 7812.5 Hz)
  sbi(TCCR1B, CS11); // with /8 prescaler
  sbi(TCCR4B, CS41); // with /8 prescaler
#else
  // Enable timers (at (16000000 / 1) / 256 = 62500 Hz)
  sbi(TCCR1B, CS10); // Timer1 enabled (with /1 prescaler)
  sbi(TCCR4B, CS40); // Timer4 enabled (with /1 prescaler)
#endif
  // Set Compare match Output Mode for Timer1
  sbi(TCCR1A, COM1A1); // Timer1 attached to OC1A output
  sbi(TCCR1A, COM1B1); // Timer1 attached to OC1B output
  // Set Compare match Output Mode for Timer4 (OCW4x)
  sbi(TCCR4B, PWM4X); // Invert all PWM (OCW4x) output
  sbi(TCCR4A, COM4A0); // Enabled #OC4A complementary output
  sbi(TCCR4C, COM4D1); // Enabled OC4D output
  sbi(TCCR4C, COM4D0); // Enabled OC4D output (inverted PWM)
  // Set pins for Timer4 as output (DDR4x after OCW4x)
  sbi(_MMIO_BYTE(led[0].ddr), led[0].bit);
  sbi(_MMIO_BYTE(led[1].ddr), led[1].bit);
  // Set Compare Register value for PWM on Timer0 & Timer1
  OCR1A = OCR1B = 0; // Timer1 1/256 of duty cycle
  OCR4A = OCR4D = 0; // Timer4 1/256 of duty cycle

  // Enable interrupts for Timer0
  sbi(TIMSK3, TOIE3); // Interrupts from overflow enabled
  // Enable Phase-Correct PWM (with OCR3A limit)
  sbi(TCCR3A, WGM30); sbi(TCCR3B, WGM33);
  // Set Compare Register value for PWM on Timer0
  OCR3A = 155; // (155 + 1) * 2 = 312
  // Enable Timer0 (at (16000000 / 1024) / 312 = 50 Hz)
  sbi(TCCR3B, CS30); sbi(TCCR3B, CS32); // /1024 prescaler

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
    asm volatile (
      "1: sbiw %0,1" "\n\t"
      "brne 1b" : "=w" (us) : "0" (us)
    );
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
        //delayMicroseconds(1000);
        asm volatile (
          "1: sbiw %0,1" "\n\t"
          "brne 1b" : "=w" (us) : "0" (us)
        );
      }
      *((uint32_t *) str_in) = i = 0; // Flush input string
    }
  }
}
