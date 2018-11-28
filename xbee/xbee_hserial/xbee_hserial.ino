/*
 * @file Serial_sserial.ino
 * @author Connor Novak
 * @date 2018-11-26
 *
 * Sets up Serial softwareserial connection to turn onboard LED on and off
 * Tested with Arduino Uno, Sparkfun Wireless Motor Driver Shield
 * Note: Ensure SerialSelect Switch is set to SW_SER when uploading code,
 * HW_SER when using radio functionality
 */

const int led_pin = 13;       // LED output pin

char inbit;                   // Serial input storage for single characters
bool prev_state = false;      // LED previous state
bool led_state = false;       // LED state

void setup() {

  Serial.begin(9600);         // Start Serial link with 9600 baud
  pinMode(led_pin, OUTPUT); // Set led pin to output
  draw();                   // Write instructions to serial

}

void loop() {

  // Wait for serial input
  if(Serial.available()) {

    inbit = Serial.read();

    // Switch case based on read char
    switch(inbit){

      case '0': // Turn LED off
        led_state = false;
        Serial.println("Turning light off");
        break;

      case '1': // Turn LED on
        led_state = true;
        Serial.println("Turning light on");
        break;
    }
  }

  // Write val to led if it is new
  if (led_state != prev_state) {digitalWrite(led_pin, led_state);}
  prev_state = led_state;
}

void draw() {
  Serial.println("                           ");
  Serial.println(" ------------------------- ");
  Serial.println(" |       |       |       | ");
  Serial.println(" |   0   |   1   |       | ");
  Serial.println(" |  off  |  on   |       | ");
  Serial.println(" ------------------------- ");
}
