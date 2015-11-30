/** Stage 05. Low-level IO
 *  Replaced all digitalRead(), digitalWrite(),
 *  analogWrite() etc. fuctions with low-level
 *  read-writes to the ports
 *  Timer2 is used instead of Timer1- so minimal
 *  frequency is ~30.5Hz (Phase-Correct PWM)
 */

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

#define SEQN 4 // Maximum sequence index (should be n^2)
#define LEDS 4 // Maximum led count (have 4 of them)

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

// Leds' out order (5 = OC0B, 6 = OC0A, 9 = OC1A, 10 = OC1B)
const uint16_t lord[LEDS] = {
  (uint16_t) &OCR0B, (uint16_t) &OCR0A,
  (uint16_t) &OCR1A, (uint16_t) &OCR1B};

// PushButton's pin (pwm): 3 = PD3
const pin_t button = {
  PD3, (uint16_t) &PORTD,
  (uint16_t) &PIND, (uint16_t) &DDRD};

// Serial message strings
const char *STR_SSP = "Lights suspended...";
const char *STR_RSM = "Lights' run resumed.";

// This is not OOP - so global variables are ok
volatile char lrun = 0; // Lights' run-stop variable
// Indices
volatile int seq = 0, i = 0, n = 0, c = 0;
volatile int x = 0, y = 0; // Arithmetic variables

const char z = ((sizeof x) * 8 - 1); // For fast arithmetic

// Serial input string
char str_in[4] = {'\0', '\0', '\0', '\0'};

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
  // Set each led
  for (i = 0; i < LEDS; i ++) {
    // Calculate duty cycle for each led (use PWM)
    x = seq - i;
    y = (x + (x >> z)) ^ (x >> z); // y = abs(x)
    x = (int) ((LEDS / 2) - y);
    y = (x + (x >> z)) ^ (x >> z); // y = abs(x)
    x = (unsigned char) ((255.0 / (LEDS / 2)) * y);
    // Write calcualted duty cycle value to the port
    _MMIO_BYTE(lord[i]) = (uint8_t) x;
  }
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
    Serial.println(STR_RSM);
  } else {
    // Button pressed: suspend running lights
    lrun = 0;
    Serial.println(STR_SSP);
  }
}

/** Setup routine
 *  All configuration trcks go here
 */
void setup() {
  // Set up Serial console
  Serial.begin(9600);
  Serial.println("Entering setup...");

  cli(); // Turn off global interrupts

  // Initialize the pins (set for OUTPUT and clear for INPUT)
  for (int i = 0; i < LEDS; i ++) {
    // Set Led's pin as OUTPUT
    sbi(_MMIO_BYTE(led[i].ddr), led[i].bit);
  }
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
  // Enable timers (at (16000000 / 1) / 256 = 62500 Hz)
  sbi(TCCR0B, CS00); // Timer0 enabled (with /1 prescaler)
  sbi(TCCR1B, CS10); // Timer1 enabled (with /1 prescaler)
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
  
  Serial.println("Setup finished.");
}

/** Loop routine
 *  This is the main code to be repeated -
 *  it may hold all the costly piece of code
 */
void loop() {
  // Check serial input buffer -
  // commands are supposed to be 3 characters long
  if (Serial.available() && (n < 3)) {
    // Proceed: Serial input buffer has data
    // and input string has space to read into
    str_in[n] = Serial.read();
    // Skip any garbage read
    if (((str_in[n] >= ' ') && (str_in[n] <= '~'))
        || ((str_in[n] >= 'a') && (str_in[n] <= 'z'))
        || ((str_in[n] >= 'A') && (str_in[n] <= 'Z'))) {
      n ++;
    }
    // Pause application for at least 1ms
    // for the Serial to be ready
    delayMicroseconds(1000);
  } else {
    // Proceed: Serial input buffer is empty
    // or input string is filled
    if (n) {
      // Display contents of input string
      Serial.print("Input: ");
      Serial.println(str_in);

      // Find command in the input
      if ((str_in[0] == 'o')
          && (str_in[1] == 'n')) {
        // Turn lights' run on
        lrun = 1;
        Serial.println(STR_RSM);
      } else if ((str_in[0] == 'o')
                 && (str_in[1] == 'f')
                 && (str_in[2] == 'f')) {
        // Turn lights' run off
        lrun = 0;
        Serial.println(STR_SSP);
      }
      // Clear the rest of Serial buffer and input string
      while (Serial.available()) {
        Serial.read();
        delayMicroseconds(1000);
      }
      for (c = n = 0; c < 4; c ++) str_in[c] = '\0';
    }
  }
}
