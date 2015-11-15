/** Stage 04. External interrupt handling
 *  Added external interrupt handling for
 *  PushButton (for leds' run management);
 *  Serial console support: status echoing,
 *  serial input parsing (also for leds' run
 *  management)
 */

#define MAXID 4 // Maximum led index (should be power of 2)

const int tfreq = 50; // Hz - timer frequency: once per 20ms
const int led[MAXID] = {5, 6, 9, 10}; // Leds' pin array
const int button = 3; // PushButton's pin

volatile int idx = 0, i = 0, n = 0, c = 0; // Indices
volatile int x = 0, y = 0; // Arithmetic variables

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
ISR(TIMER1_OVF_vect) {
  // Reset the Counter start value
  TCNT1 = 65536 - 16000000 / 256 / tfreq;

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
    analogWrite(led[i], x); // Set duty cycle for the led
  }
  idx ++; // Increment the index variable
}

/** Push button interrupt handler
 *  This one will handle all interrupts on
 *  every change of PushButton's pin state
 */
void btnSwChange() {
  if (digitalRead(button) == HIGH) {
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
  // Initialize the pins
  for (int i = 0; i < MAXID; i ++)
    // Set Led's pin as OUTPUT
    pinMode(led[i], OUTPUT);
  // Set PushButton's pin as INPUT
  pinMode(button, INPUT);

  // Set up Serial console
  Serial.begin(9600);
  Serial.println("Entering setup...");

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

  // Attach interrupt handlers to the PushButton
  x = digitalPinToInterrupt(button);
  attachInterrupt(x, btnSwChange, CHANGE);

  interrupts(); // Turn interrupts on again

  Serial.println("Setup finished.");
}

/** Loop routine
 *  This is the main code to be repeated
 *  until it is not the end :)
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
