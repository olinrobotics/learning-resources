/*
 * @file xbee_sserial.ino
 * @author Connor Novak
 * @date 2018-11-26
 *
 * Sets up xbee softwareserial connection to turn onboard LED on and off
 * Tested with Arduino Uno and Sparkfun's Wireless Motor Driver Shield and
 * XBee Shield
 */

#include <SoftwareSerial.h>   // Allows serial comms using other Arduino pins

const int led_pin = 13;       // LED output pin

char inbit;                   // Serial input storage for single characters
bool prev_state = false;      // LED previous state
bool led_state = false;       // LED state

/* If using Wireless Motor Driver Shield:
 * Set Serial Select Switch to SW_SER
 * Uncomment Wireless Motor Driver Shield
 * Comment XBee Shield
 */
//SoftwareSerial XBee(A0, A1);  // WIRELESS MOTOR DRIVER SHIELD

/* If using XBee Shield:
 * Set Switch to DLINE
 * Uncomment XBee Shield
 * Comment Wireless Motor Driver Shield
 */
SoftwareSerial XBee(2, 3);    //XBEE SHIELD

void setup() {

  XBee.begin(9600);         // Start Serial link with 9600 baud
  pinMode(led_pin, OUTPUT); // Set led pin to output
  draw();                   // Write instructions to serial

}

void loop() {

  // Wait for serial input
  if(XBee.available()) {

    inbit = XBee.read();

    // Switch case based on read char
    switch(inbit){

      case '0': // Turn LED off
        led_state = false;
        XBee.println("Turning light off");
        break;

      case '1': // Turn LED on
        led_state = true;
        XBee.println("Turning light on");
        break;
    }
  }

  // Write val to led if it is new
  if (led_state != prev_state) {digitalWrite(led_pin, led_state);}
  prev_state = led_state;
}

void draw() {
  XBee.println("                           ");
  XBee.println(" ------------------------- ");
  XBee.println(" |       |       |       | ");
  XBee.println(" |   0   |   1   |       | ");
  XBee.println(" |  off  |  on   |       | ");
  XBee.println(" ------------------------- ");
}
