/* Stage 02. Port management
 * Added facilities for *running lights*
 * (led shifting and fading)
 */

int led[4] = {5, 6, 9, 10}; // Leds' pin array
int idx = 0; // Indices

/** Setup routine
 *  All configuration trcks go here
 */
void setup() {
  // Initialize the pins
  for (int i = 0; i < 4; i ++)
    // Set Led's pin as OUTPUT
    pinMode(led[i], OUTPUT);
}

/** Loop routine
 *  This is the main code to be repeated
 *  until it is not the end :)
 */
void loop() {
	idx &= 3; // Trim led index to be always in range {0..3}
	for (int fval = 0; fval <= 255; fval += 5) {
		// Fade in the led
		analogWrite(led[idx], fval);
		delay(30);
	}
	for (int fval = 255; fval >= 0; fval -= 5) {
		// Fade out the led
		analogWrite(led[idx], fval);
		delay(30);
	}
	idx ++; // Increment the index variable
}
