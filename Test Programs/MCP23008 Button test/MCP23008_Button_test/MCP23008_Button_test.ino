#include <Wire.h>
#include "Adafruit_MCP23008.h"

// Basic pin reading and pullup test for the MCP23008 I/O expander
// public domain!

// Connect pin #1 of the expander to Analog 5 (i2c clock)
// Connect pin #2 of the expander to Analog 4 (i2c data)
// Connect pins #3, 4 and 5 of the expander to ground (address selection)
// Connect pin #6 and 18 of the expander to 5V (power and reset disable)
// Connect pin #9 of the expander to ground (common ground)

// Input #0 is on pin 10 so connect a button or switch from there to ground

Adafruit_MCP23008 mcp;

void setup() {
  Serial.begin(115200);
  mcp.begin();      // use default address 0

  mcp.pinMode(0, OUTPUT);
  mcp.pinMode(1, INPUT);
  mcp.pullUp(1, HIGH);  // turn on a 100K pullup internally

  pinMode(LED_BUILTIN, OUTPUT);  // use the p13 LED as debugging
}



void loop() {
  // The LED will 'echo' the button

//  mcp.digitalWrite(0, HIGH);
//  delay(1000);
//  mcp.digitalWrite(0, LOW);
//  delay(1000);
   // Serial.println(mcp.readGPIO());
    Serial.println(mcp.digitalRead(1));
    mcp.digitalWrite(0, mcp.digitalRead(1));
}
