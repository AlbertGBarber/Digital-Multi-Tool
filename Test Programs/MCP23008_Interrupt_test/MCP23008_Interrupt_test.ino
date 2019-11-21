/*
 * Configuration and Interrupt handling sample
 * Copyright (C) 2018 Pat Deegan, https://flyingcarsandstuff.com
 * 
 * Released as part of the Adafruit MCP23008 library, under the same
 * BSD license.
 * 
 * 
 * Hookup:
 *
 * Tie i2c lines (i.e. analog 4=SDA/DATA and analog 5=SCL/CLOCK on a Uno), 
 * using 2-5k pullups as required.
 * 
 * Power the unit and tie INTERRUPT_PIN to the MCP's interrupt output (defined below).
 * This pin must actually support interrupts (e.g. pin 2 or 3 on a Uno). 
 * 
 * 
 */

#include <Wire.h>
#include <Adafruit_MCP23008.h>

/*
   Defines used below for Serial, baudrate and interrupt pin to monitor
*/

#define SERIAL_TO_USE      Serial
#define SERIALPORT_BAUDRATE   115200
#define INTERRUPT_PIN     D5



// Our global MCP object
Adafruit_MCP23008 mcp;

/*
   Our interrupt flag / service routine.
*/
volatile bool inputsChanged = false;
void inputsHaveChanged() {
  inputsChanged = true;
}


void setup() {
  Serial.begin(115200);
  mcp.begin();      // use default address 0

  /*
     We want to configure interrupts a bit, just to
     show how it's done (though this is the default IOCONF setup anyway)
  */

  // IOConfiguration acts like a struct with the same fields
  // as the IOCON register.  Each field uses an enum for
  // clear and error free settings.  See the
  Adafruit_MCP23008::IOConfiguration config = mcp.configuration();
  config.intpol = Adafruit_MCP23008::ActiveHighInterrupt; // (or ActiveHighInterrupt)
  config.odr = Adafruit_MCP23008::ActiveDriver; // (or Adafruit_MCP23008::OpenDrain)

  mcp.setConfiguration(config);
  /*
      Could also use individual setters, e.g.
      mcp.setInterruptPolarity(Adafruit_MCP23008::ActiveLowInterrupt);
      mcp.setInterruptOutput(Adafruit_MCP23008::ActiveDriver);
      etc
      less efficient, maybe clearer...
   */
  config = mcp.configuration();

  mcp.setInterruptOnChange(0); // clear all interrupts-on-change
  // you could mcp.setDefaultValue(0xff) and compare to that, but we'll 
  // just compare to the previous value.

    // everybody an input, no pullup
    mcp.pinMode(1, INPUT);
    mcp.pullUp(1, HIGH);
    mcp.setInterruptOnChangePin(1, true, Adafruit_MCP23008::CompareToPreviousValue);


    mcp.pinMode(0, OUTPUT);
  

  
  /* Setup our local interrupt handler */
  //pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), inputsHaveChanged, RISING);

}




uint8_t lc = 0;
void loop() {
  if (! inputsChanged ) {
    // nothing going on...
    delay(50);
    if (lc++ > 100) {
      lc = 0;

    }
    return;
  }


  // inputs have changed! exciting!

  // Read all values at once, 'cause we're efficient that way...

  // could use mcp.interruptCapture() to clear flags, but readGPIO() returns
  // current state rather than the event that triggered the interrupt
  uint8_t val = mcp.readGPIO(); // clears the interrupt flag on the MCP (INT pin goes high)

  inputsChanged = false; // we clear our own flag
  mcp.digitalWrite(0, bitRead(val, 1));

} 
