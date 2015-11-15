/* Stage 03. Interrupts
 * Leds' manipulation moved to the
 * interrupts handler. Added frequency
 * calculations
 */

#define MAXID 4 // Maximum led index (should be power of 2)

const int tfreq = 50; // Hz - once per 20ms (1 / 0.020)
const int led[MAXID] = {5, 6, 9, 10}; // Leds' pin array
volatile int idx = 0; // Indices
volatile int x = 0, y = 0; // Arithmetic variables

/** ISR (Interrupt Service Routine) setup macro
 *  Used to set handler for interrupts from
 *  the timer (on overflow). delay() and milis()
 *  fuctions should not be used here (as they
 *  disable global interrupts)
 */
ISR(TIMER1_OVF_vect) {
  // All delay() fuctions (an alike) must not be used here
  TCNT1 = 65536 - 16000000 / 256 / tfreq; // Reset Counter
  idx &= (MAXID - 1); // Trim led index to be always in range
  for (int i = 0; i < MAXID; i ++) {
    // Set each led
    x = idx - i;
    y = (x + (x >> 31)) ^ (x >> 31); // y = abs(x)
    x = (int) ((MAXID / 2) - y);
    y = (x + (x >> 31)) ^ (x >> 31); // y = abs(x)
    x = (unsigned char) ((255.0 / (MAXID / 2)) * y);
    analogWrite(led[i], x);
  }
  idx ++; // Increment the index variable
}

/** Setup routine
 *  All configuration trcks go here
 */
void setup() {
  // Initialize the pins
	for (int i = 0; i < MAXID; i ++)
    // Set Led's pin as OUTPUT
	  pinMode(led[i], OUTPUT);

	noInterrupts(); // Turn off interrupts

  // Reset Timers (Timer Counter Control Registers)
	TCCR1A = 0;
	TCCR1B = 0;

  // Set Counter start value (maximum 65535 for T1)
	TCNT1 = 65536 - 16000000 / 256 / tfreq;
	// Set CS12 bit for 256-prescaler
	TCCR1B |= (1 << CS12);
  // Set interrupt mask for overflow interrupt from timer
	TIMSK1 |= (1 << TOIE1);

	interrupts(); // Turn interrupts on again
}

/** Loop routine
 *  This is the main code to be repeated
 *  until it is not the end :)
 */
void loop() {
	// Do nothing
}
