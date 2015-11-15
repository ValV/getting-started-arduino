/** Stage 01. Introduction
 *  Simple example that makes the Led blink
 */

int led = 10; // Led's pin

/** Setup routine
 *  All configuration trcks go here
 */
void setup() {
  // Initialize the pin as OUTPUT
  pinMode(led, OUTPUT);
}

/** Loop routine
 *  This is the main code to be repeated
 *  until it is not the end :)
 */
void loop() {
  // Turn the Led on (HIGH is the voltage level)
  digitalWrite(led, HIGH);
  // Wait for 850 ms
  delay(850);
  // Turn the Led off by making the voltage LOW
  digitalWrite(led, LOW);
  // wait for 550 ms
  delay(550);
}
