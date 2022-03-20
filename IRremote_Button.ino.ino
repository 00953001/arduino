void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}#include <Arduino.h>

/* Defines of GPIO_Interrupt.ino */
const byte ledPin = 2;       // Builtin-LED pin
const byte interruptPin = 0; // BOOT/IO0 button pin
volatile byte state = LOW;

/* Defines of SendAndReceive.ino */
// select only NEC and the universal decoder for pulse width or pulse distance protocols
#define DECODE_NEC          // Includes Apple and Onkyo
#define DECODE_DISTANCE     // in case NEC is not received correctly

#include "PinDefinitionsAndMore.h"
#define INFO // To see valuable informations from universal decoder for pulse width or pulse distance protocols

#include <IRremote.hpp>

#define DELAY_AFTER_SEND 2000
#define DELAY_AFTER_LOOP 5000
 
void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), IRsender, FALLING);
  
  /* get code from setup() in SendAndReceive.ino */
  
}

void loop() {
  Serial.println("IR receiving...");
  /* get Receiver code from SendAndReceive.ino */
}

void IRsender() {
  Serial.println("IR sending...");
  /* get Sender code from SendAndReceiv.ino */
}
