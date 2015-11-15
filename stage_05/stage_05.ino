/** Stage 05. Low-level IO
 *  Replaced all digitalRead(), digitalWrite(),
 *  analogWrite() etc. fuctions with low-level
 *  read-writes to the ports
 *  Timer2 is used instead of Timer1- so minimal
 *  frequency is ~30.5Hz (Phase Correct PWM)
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

#define MAXID 4 // Maximum led index (should be power of 2)

// Custom types to handle PORT (and other) variables
//typedef unsigned int ptr_t;
typedef struct {
  uint8_t bit;
  uint16_t port;
  uint16_t pin;
  uint16_t ddr;
} pin_t;

const int tfreq = 50; // Hz - timer frequency: once per 20ms
// Leds' digital pin array:
// 5(pwm)=PD5, 6(pwm)=PD6, 9(pwm)=PB1, 10(pwm)=PB2
const pin_t led[MAXID] = {
  {PD5, (uint16_t) &PORTD, (uint16_t) &PIND, (uint16_t) &DDRD},
  {PD6, (uint16_t) &PORTD, (uint16_t) &PIND, (uint16_t) &DDRD},
  {PB1, (uint16_t) &PORTB, (uint16_t) &PINB, (uint16_t) &DDRB},
  {PB2, (uint16_t) &PORTB, (uint16_t) &PINB, (uint16_t) &DDRB}};
// Leds' sequence array (5=OC0B, 6=OC0A, 9=OC1A, 10=OC1B)
const uint16_t seq[MAXID] = {
  (uint16_t) &OCR0B, (uint16_t) &OCR0A,
  (uint16_t) &OCR1A, (uint16_t) &OCR1B};
// PushButton's pin 3(pwm)=PD3
const pin_t button = {
  PD3, (uint16_t) &PORTD,
  (uint16_t) &PIND, (uint16_t) &DDRD};

volatile int idx = 0, i = 0, n = 0, c = 0; // Indices
volatile int x = 0, y = 0, w = 0; // Arithmetic variables

const char z = ((sizeof x) * 8 - 1); // V for fast arithmetic

volatile char lrun = 0; // Lights' run-stop variable

char str_in[4] = {'\0', '\0', '\0', '\0'}; // Input string
const char *STR_SSP = "Lights suspended...";
const char *STR_RSM = "Lights' run resumed.";

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
  idx &= (MAXID - 1);
  // Set each led
  for (i = 0; i < MAXID; i ++) {
    // Calculate duty cycle for each led (use PWM)
    x = idx - i;
    y = (x + (x >> z)) ^ (x >> z); // y = abs(x)
    x = (int) ((MAXID / 2) - y);
    y = (x + (x >> z)) ^ (x >> z); // y = abs(x)
    x = (unsigned char) ((255.0 / (MAXID / 2)) * y);
    //_MMIO_BYTE(seq[i]) = (uint8_t) x;
    // TODO: Change to low-level IO
    //analogWrite(seq[i], x); // Set duty cycle for the led
  }
  sbi(_MMIO_BYTE(led[idx].port), led[idx].bit);
  idx ++; // Increment the index variable
}

/** Push button interrupt handler
 *  This one will handle all interrupts on
 *  every change of PushButton's pin state
 */
void btnSwChange() {
  if (rbi(_MMIO_BYTE(button.pin), button.bit)) {
    // Allow lights to run again
    lrun = 1;
    Serial.println(STR_RSM);
  } else {
    // Forbid running lights
    lrun = 0;
    Serial.println(STR_SSP);
  }
}

/** Setup routine
 *  All configuration trcks go here
 */
void setup() {
  // Initialize the pins (set for OUTPUT and clear for INPUT)
  for (int i = 0; i < MAXID; i ++) {
    // Set Led's pin as OUTPUT
    sbi(_MMIO_BYTE(led[i].ddr), led[i].bit);
  }
  // Set PushButton's pin (PD3) as INPUT
  cbi(_MMIO_BYTE(button.ddr), button.bit);

  // Set up Serial console
  Serial.begin(9600);
  //while(!Serial);
  Serial.println("Entering setup...");

  cli(); // Turn off global interrupts

  // Reset Timers (Timer Counter Control Registers)
  TCCR0A = 0; TCCR0B = 0; // Timer0 disabled and detached
  TCCR1A = 0; TCCR1B = 0; // Timer1 disabled and detached
  TCCR2A = 0; TCCR2B = 0; // Timer2 disabled and detached

  // Disable interrupts for Timer0 & Timer1
  TIMSK0 = TIMSK1 = 0;
  // Enable PWM for Timer0 & Timer1
  sbi(TCCR0A, WGM00); sbi(TCCR0A, WGM01); // Fast PWM
  sbi(TCCR1A, WGM10); sbi(TCCR1B, WGM12); // Fast PWM 8-bit
  // Set Compare Register value for PWM on Timer0 & Timer1
  OCR0A = 0; OCR0B = 0; // 1/256 of duty cycle
  OCR1A = 0; OCR1B = 0; // 1/256 of duty cycle
  // Set Compare match Output Mode for Timer0 & Timer1
  //sbi(TCCR0A, COM0A1); // Timer0 attached to OC0A output
  //sbi(TCCR0A, COM0B1); // Timer0 attached to OC0B output
  //sbi(TCCR1A, COM1A1); // Timer1 attached to OC1A output
  //sbi(TCCR1A, COM1B1); // Timer1 attached to OC1B output
  // Enable timers (at (16000000 / 1) / 256 = 62500 Hz)
  sbi(TCCR0B, CS00); // Timer0 enabled (with /1 prescaler)
  sbi(TCCR1B, CS10); // Timer1 enabled (with /1 prescaler)
  //sbi(TIMSK0, TOIE0); //sbi(TIMSK0, OCIE0B);
  //sbi(TIMSK1, TOIE1); //sbi(TIMSK1, OCIE1B);

  // Enable Phase-Correct PWM (with OCR2A limit)
  sbi(TCCR2B, WGM22); sbi(TCCR2A, WGM20);
  sbi(TCCR2A, COM2A0); // Timer2 attached to OC2A output
  // Set Compare Register value for PWM on Timer2
  OCR2A = 155; // (155 + 1) * 2 = 312
  // Enable Timer2 (at (16000000 / 1024) / 312 = 50 Hz)
  sbi(TCCR2B, CS20); sbi(TCCR2B, CS22); // /1024 prescaler
  // Enable interrupts for Timer2
  //sbi(TIMSK2, OCIE2A); // Interrupts from OC2A line enabled
  sbi(TIMSK2, TOIE2); // Interrupts from overflow enabled

  // Enable INT1 for PD3 (PushButton)
  x = digitalPinToInterrupt(3);
  attachInterrupt(x, btnSwChange, CHANGE);

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
