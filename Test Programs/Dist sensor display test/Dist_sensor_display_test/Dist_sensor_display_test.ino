#include <Wire.h>
#include <PDQ_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1// Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include <VL53L1X.h>

VL53L1X sensor;

uint16_t rangeSum = 0;
uint16_t rangeAvg;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  Serial.begin(115200);

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();

  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(WHITE);

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(100000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  //
  for (int i = 0; i < 5; i++) {
    sensor.read(true);
    rangeSum += sensor.ranging_data.range_mm;
  }
  rangeAvg = rangeSum / 5;
  rangeSum = 0;
  //
  //  display.clearDisplay();
  //  display.setTextSize(2);
  //  display.setCursor(0, 20);
  //  display.print("range: ");
  //  display.setTextSize(1);
  //  display.setCursor(0, 35);
  //  display.print("\tstatus: ");
  //  display.setCursor(0, 45);
  //  display.print("\tpeak signal: ");
  //  display.setCursor(0, 55);
  //  display.print("\tambient: ");
  //
  display.setTextSize(2);
  display.setCursor(70, 20);
  display.print(rangeAvg);
  display.setTextSize(1);
  display.setCursor(80, 35);
  // display.print(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
  display.setCursor(80, 45);
  display.print(sensor.ranging_data.peak_signal_count_rate_MCPS);
  display.setCursor(80, 55);
  display.print(sensor.ranging_data.ambient_count_rate_MCPS);
  display.display();

}
