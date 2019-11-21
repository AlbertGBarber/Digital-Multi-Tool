//This code is intended to be pair with the Digital Multi-tool found at
//www.instructables.com/id/Digital-Multi-Tool

//This code is placed under the MIT license
//Copyright (c) 2019 Albert Barber
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in
//all copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP23008.h>
#include <Adafruit_MLX90614.h>
#include <Encoder.h>
#include <VL53L1X.h>
#include <Battery.h>

// ================================================================
// ===                   IMU OFFSETS                            ===
// ================================================================
//offsets for the Imu's gyro and accel, you'll need to supply your own
//by running the IMU_Zero sketch found under the MPU6050 examples
//make sure to read the instuctions in the example
#define X_GYRO_OFFSET  69
#define Y_GYRO_OFFSET -30
#define Z_GYRO_OFFSET  71

#define X_ACCEL_OFFSET 365
#define Y_ACCEL_OFFSET 121
#define Z_ACCEL_OFFSET 1404

// ================================================================
// ===                         MODES                            ===
// ================================================================
//Defines the order of modes, you can re-order modes as you wish
//you can remove modes by setting the mode number to greater than the NUM_MODES ex: 999
//if you do this don't forget to decrease NUM_MODES to match the number of modes remaining
//and also change the order of the remaining modes (going from 0 to however remain)
#define ROLL_LEVEL       0  //IMU
#define XY_LEVEL         1  //IMU
#define PROTRACTOR       2  //IMU
#define LASER_RULER      3  //VL53L1X
#define WHEEL_RULER      4  //Encoder
#define WHEEL_TACHOMETER 5  //Encoder
#define IR_THERMOMETER   6  //MLX90614
#define IR_TACHOMETER    7  //IR LED & Photodiode
#define LASER_POINTER    8  //Laser Diode

#define NUM_MODES        9 //total number of active modes

volatile uint8_t mode = 0; //inital mode
// ================================================================
// ===                         PIN SETUP                        ===
// ================================================================

//wemos d1 mini pins
#define INTERRUPT_PIN_IMU D8
#define INTERRUPT_PIN_IO  D5
#define ENCODER_PIN_1     D6
#define ENCODER_PIN_2     D7
#define IR_TACT_PIN       D4
#define BAT_SENSE_PIN     A0

//MCP23008 pins
#define MODE_BUTTON_PIN  2
#define ZERO_BUTTON_PIN  1
#define EXTRA_BUTTON_PIN 0
#define LASER_PIN        3
#define TACT_PIN         4

// ================================================================
// ===                     BAT SENSE SETUP                      ===
// ================================================================
//our range for lipo voltage is 4.2-3.4V, after 3.4 the wemos LDO will become unreliable
//so we should recharge
Battery battery(3400, 4200, BAT_SENSE_PIN);
const unsigned long batteryUpdateTime = 5000; //how often we update the battery level in ms
unsigned long prevBatReadTime = 0; //the last time we read the battery in ms
uint8_t batteryLvl; //the battery percentage

// ================================================================
// ===                     SCREEN SETUP                         ===
// ================================================================
#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     64
#define YELLOW_BAR_HEIGHT 16
#define BASE_FONT_LENGTH  6
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1// Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ================================================================
// ===                  PROGRAM GLOBAL VARS                     ===
// ================================================================
boolean laserOn = false;
boolean extraButtonToggle = false;
boolean extraButtonRun = true;
boolean zeroButtonToggle = false;
boolean zeroButtonRun = true;

const float cmToIn = 0.393701; //conversion from cm to in
const int32_t usInMin = 60 * 1000000; //amount of microseconds in a min

//defualt units for length
String units = "cm";

//presets for drawing a cicle of maximum radius on the blue portion of the screen
const uint8_t circleYcenter =  SCREEN_HEIGHT - (SCREEN_HEIGHT - YELLOW_BAR_HEIGHT) / 2;
const uint8_t circleRadius = 23;

// ================================================================
// ===                   TACHOMETER SETUP                       ===
// ================================================================

boolean tachOn = false; //if a tachometer mode is active
boolean tactReadingStarted = false; //if we're reading from rpm
volatile long irTacRevs = 0; //number of IR Tachometer revolutions (encoder revs are measured using the encoder's vars)
unsigned long rpm;
unsigned long tacStartTime = 0; //start time of tachometer reading
boolean tachStartTimeSet = false;  //starting time is only set once the tach detects motion, flag for it that has happened
const unsigned long measureIntervalMaxSec = 10; //maxium measurement period in seconds
const unsigned long measureIntervalMax = measureIntervalMaxSec * 1000000; //maxium measurement period in us //10 sec
const String measureIntervalMaxString = String(measureIntervalMaxSec); //maxium measurement period as a string in sec

// ================================================================
// ===                      ENCODER SETUP                       ===
// ================================================================

//both work as interrupts so we should get good performance
Encoder myEnc(ENCODER_PIN_1, ENCODER_PIN_2);

#define WHEEL_DIAM 2.5 //encoder wheel diameter in cm
#define ENC_STEPS_PER_REV 48 //number of encoder steps per revolution
//distance covered per encoder step in cm
const float encStepLength = (M_PI * WHEEL_DIAM) / ENC_STEPS_PER_REV;

// ================================================================
// ===            MLX90614 IR THERMOMETER SETUP                 ===
// ================================================================
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// ================================================================
// ===                VL53L1X DIST SENSOR SETUP                 ===
// ================================================================
VL53L1X distSensor;
#define INTERNAL_OFFSET 6 //distance from case front to sensor in mm

boolean distSensorOff = true;
//distance mode tracker
//0 - long
//1 - medium
//3 - short
uint8_t distMode = 0;  //inital sensor mode
#define NUM_DIST_MODES 3; //total number of modes
//max range for each distance mode
String distModeSting[] = {"Long, 4m/157in" , "Medium, 3m/118in", "Short, 1.3m/51in"};

// Use long distance mode and allow up to 100000 us (100 ms) for a measurement.
// You can change these settings to adjust the performance of the sensor, but
// the minimum timing budget is 20 ms for short distance mode and 33 ms for
// medium and long distance modes. See the VL53L1X datasheet for more
// information on range and timing limits.
#define DIST_TIMING_BUDGET 100000

//The reading rate in ms. This period should be at least as long as the timing budget.
const long laserDistRate = DIST_TIMING_BUDGET / 1000;

//the program will take DIST_LOOP_MAX_COUNT readings and average them
//(note that it will take DIST_LOOP_MAX_COUNT * laserDistRate ms for an average to be calculated )
#define DIST_LOOP_MAX_COUNT 5
uint16_t rangeSum = 0;
uint16_t rangeAvg = 0;
uint8_t distLoopCount = 0;

// ================================================================
// ===                         IMU SETUP                        ===
// ================================================================

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// MPU control/status vars
boolean imuOn = true;   // set true if we've attached the IMU interrupt (the imu setup connects the imu initally)
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprOffsets[] = {0, 0, 0};    // [yaw, pitch, roll]  offsets (in rad) for yaw/pitch/roll (for zeroing)

//interrupt routine
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                    MCP23008 SETUP                        ===
// ================================================================
Adafruit_MCP23008 mcp;

//we're using the mcp23008 in interrupt mode
//below is the flag that a button has been pushed and interrupt function to set it
volatile bool inputsChanged = false;
void inputsHaveChanged() {
  inputsChanged = true;
}

// ================================================================
// ================================================================
// ================================================================
// ===                   SETUP FUNCTION                         ===
// ================================================================
// ================================================================
// ================================================================

void setup() {
  Serial.begin(115200);
  // ================================================================
  // ===                  BATTERY CONFIG                          ===
  // ================================================================
  pinMode(BAT_SENSE_PIN, INPUT); //our analog input pin connected to the battery
  //start a battery object with 4.2 max votage using sigmodal mapping for lipos
  //the wemos already has a voltage divider on A0
  //our inclusion of a 100K resistor brings the divider ratio to exactly 1.0
  battery.begin(4200, 1.0, &sigmoidal);
  batteryLvl = battery.level(); //get an inital reading
  // ================================================================
  // ===             IR TACHOMETER CONFIG                         ===
  // ================================================================
  pinMode(IR_TACT_PIN, INPUT);

  // ================================================================
  // ===                  DISPLAY CONFIG                          ===
  // ================================================================
  //note debug messages after this are displayed on the display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  // Clear the buffer, set the default text size, and color
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // ================================================================
  // ===              MLX90614 IR THERMOMETER SETUP               ===
  // ================================================================
  mlx.begin();

  // ================================================================
  // ===                  MCP23008 CONFIG                         ===
  // ================================================================
  mcp.begin();      // use default address 0

  //config the mcp23008 for interrupts
  //ActiveDriver is needed for interrupts to work
  //ActiveHighInterrupt indicates the interrupt pin is driven high when triggered
  Adafruit_MCP23008::IOConfiguration config = mcp.configuration();
  config.intpol = Adafruit_MCP23008::ActiveHighInterrupt; // (or ActiveLowInterrupt)
  config.odr = Adafruit_MCP23008::ActiveDriver; // (or Adafruit_MCP23008::OpenDrain)

  mcp.setConfiguration(config);

  //config interrupts for push buttons
  mcp.setInterruptOnChange(0); // clear all interrupts-on-change
  mcp.pinMode(MODE_BUTTON_PIN, INPUT);
  mcp.pullUp(MODE_BUTTON_PIN, HIGH);
  //CompareToPreviousValue will trigger once when the button is pressed or released
  mcp.setInterruptOnChangePin(MODE_BUTTON_PIN, true, Adafruit_MCP23008::CompareToPreviousValue);

  mcp.pinMode(EXTRA_BUTTON_PIN, INPUT);
  mcp.pullUp(EXTRA_BUTTON_PIN, HIGH);
  mcp.setInterruptOnChangePin(EXTRA_BUTTON_PIN, true, Adafruit_MCP23008::CompareToPreviousValue);

  mcp.pinMode(ZERO_BUTTON_PIN, INPUT);
  mcp.pullUp(ZERO_BUTTON_PIN, HIGH);
  mcp.setInterruptOnChangePin(ZERO_BUTTON_PIN, true, Adafruit_MCP23008::CompareToPreviousValue);

  //config outputs (laser and IR tachometer switches)
  mcp.pinMode(LASER_PIN, OUTPUT); //Laser
  mcp.pinMode(TACT_PIN, OUTPUT); //IR tachometer

  //Setup our local interrupt handler
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_IO), inputsHaveChanged, RISING);

  // ================================================================
  // ===             VL53L1X DIST SENSOR CONFIG                   ===
  // ================================================================
  distSensor.setTimeout(500);
  if (!distSensor.init())
  {
    display.clearDisplay();
    display.setCursor(5, 10);
    display.setTextSize(1);
    display.print("Failed to detect and initialize VL53L1X sensor!");
    display.display();
    while (1);
  }
  distSensor.setDistanceMode(VL53L1X::Long);
  distSensor.setMeasurementTimingBudget(DIST_TIMING_BUDGET);

  // ================================================================
  // ===                      IMU CONFIG                          ===
  // ================================================================
  // initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN_IMU, INPUT);

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro / accel offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(X_GYRO_OFFSET);
  mpu.setYGyroOffset(Y_GYRO_OFFSET);
  mpu.setZGyroOffset(Z_GYRO_OFFSET);

  mpu.setXAccelOffset(X_ACCEL_OFFSET);
  mpu.setYAccelOffset(Y_ACCEL_OFFSET);
  mpu.setZAccelOffset(Z_ACCEL_OFFSET);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_IMU), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    display.clearDisplay();
    display.setCursor(0, 10);
    display.setTextSize(1);
    display.print(F("DMP Initialization failed (code "));
    display.print(devStatus);
    display.display();
    while (1);
  }
  //start up splash screen
  display.clearDisplay();
  drawHeader("By AGB");
  display.setCursor(15, 16);
  display.setTextSize(2);
  display.print("Digital");
  display.setCursor(5, 33);
  display.print("Multi-tool");
  display.setTextSize(1);
  display.setCursor(0, 49);
  display.print("Instructables.com/id/");
  display.setCursor(0, 57);
  display.print("Digital-Multi-Tool");
  display.display();
  delay(3000);
}

// ================================================================
// ================================================================
// ================================================================
// ===                     MAIN PROGRAM                         ===
// ================================================================
// ================================================================
// ================================================================

//Basic Function Outline
//each mode is contained in its own while loop
//until the mode is changed we continually loop through the current mode's loop
//for every mode a few things are common:
//we yeild() to let esp8266 background processes run
//we read the active sensor (or let interrupts for the sensor happen)
//we act on any button flags (changing units, zeroing, etc)
//we clear the display and redraw it (including the header)
//(we could overwrite only the changed parts of the display, but that is far more complicated, and for small display's it's quick to redraw the whole thing)
//we check any for any button presses at the end of the loop
//(this must come at the end of the loop, because if the mode changes we want to catch it at the while mode check, without doing anything in the loop)
//The zero and extra buttons have two state boolean flags, a "run" and a "toggle"
//the "toggle" changes whenever the button is pressed
//the "run" is set false whenever the button is pressed and must be cleared by the current mode
//Both flags are reset to defaults (true for "run" and false for "toggle") when a mode changes
void loop() {

  //a typical level display using the IMU, like a bubble level
  while (mode == ROLL_LEVEL) {
    driveIMU(true); //turn on IMU
    checkDMP();
    yield();
    display.clearDisplay();
    drawHeader("Roll Level");
    //We want to orient the multi-tool with the screen vertically so that the display can match the phyical roll
    //This means the IMU is rotated from its usual orientation (screen facing up)
    //so the axis of roll is different from what the library assumes,
    //and we need to caculate it ourselves
    ypr[2] = atan2(gravity.y , -gravity.x );
    //an alternative calc, might be more accurate, but only works for angles < 80 deg
    //ypr[2] =  atan2(gravity.y , sqrt(gravity.x * gravity.x + gravity.z * gravity.z));

    //if the zero button is pressed, we set a new offset (zero point)
    if (!zeroButtonRun) {
      yprOffsets[2] = ypr[2];
      zeroButtonRun = true;
    }
    //subtrace the offset and convert to degrees
    float roll = (ypr[2] - yprOffsets[2]) * 180 / M_PI;
    //draw a crosshair circle, and a line across it representing the angle using the unit circle
    float lineXOffset = circleRadius * cos(ypr[2]);
    float lineYOffset = circleRadius * sin(ypr[2]);
    uint8_t circleXcenter = circleRadius;
    drawCrosshairCircle(circleXcenter);
    display.drawLine(circleXcenter - lineXOffset, circleYcenter - lineYOffset , circleXcenter + lineXOffset, circleYcenter + lineYOffset, WHITE);
    display.setTextSize(2);
    display.setCursor(52, 30);
    //roll is negative to better match the line on the crosshair circle
    //we only display to one decimal place to avoid overflowing to the next line
    display.print(-roll, 1);
    //        display.setTextSize(1);
    //        display.setCursor(0, 55);
    //        display.print(ypr[2]);
    //        display.print(", ");
    //        display.print(yprOffsets[2]);
    //        display.print(", ");
    //        display.print(gravity.z);
    display.display();
    readButtons();
  }

  //An x-y level using the IMU, like a circular bubble level
  //also normalizes angles when the multi-tool is turned upside down,
  //keeping them between -90, 90
  while ( mode == XY_LEVEL) {
    driveIMU(true); //turn on IMU
    checkDMP();
    yield();
    display.clearDisplay();
    drawHeader("X-Y Level");
    //if the zero button is hit we'll store the current angles
    //and use them as a new zero point
    if (!zeroButtonRun) {
      yprOffsets[2] = ypr[2];
      yprOffsets[1] = ypr[1];
      zeroButtonRun = true;
    }
    //convert the angles and offsets to degrees
    float xAngleOff = yprOffsets[2] * 180 / M_PI;
    float yAngleOff = yprOffsets[1] * 180 / M_PI;
    float xAngle = ypr[2] * 180 / M_PI;
    float yAngle = ypr[1] * 180 / M_PI;

    //if we're holding the sensor upside down (ie screen facing the ground)
    //the angles will be from +-180-270
    //we shift them back to +-90 for readability
    //this also includes shifting any offsets if needed
    if (xAngle > 90 || xAngle < -90) {
      xAngle = (180 - abs(xAngle)) * getSign(xAngle);
    }

    if (yAngle > 90 || yAngle < -90) {
      yAngle = (180 - abs(yAngle)) * getSign(yAngle);
    }

    if (xAngleOff > 90 || xAngleOff < -90) {
      xAngleOff = (180 - abs(xAngleOff)) * getSign(xAngleOff);
    }

    if (yAngleOff > 90 || yAngleOff < -90) {
      yAngleOff = (180 - abs(yAngleOff)) * getSign(yAngleOff);
    }
    //subtract the offsets from the measured angle
    xAngle -= xAngleOff;
    yAngle -= yAngleOff;
    yAngle *= -1; //- to account for sensor orientation
    uint8_t circleXcenter = circleRadius;
    //display the angles and a crosshair circle
    //angles are displayed with one decimal to make them fit on one line
    display.setTextSize(2);
    display.setCursor(52, 20);
    display.print("X");
    display.print(xAngle, 1); //only display to one decimal place to avoid overflowing to the next line
    display.setCursor(52, 45);
    display.print("Y");
    display.print(-yAngle, 1);
    //draw a dot inside the crosshair circle according to the angles
    //(like a bubble level)
    display.fillCircle(circleXcenter + circleRadius  * sin(ypr[2]) , circleYcenter + circleRadius * sin(ypr[1]), 2, WHITE);
    drawCrosshairCircle(circleXcenter);
    display.display();
    readButtons();
  }

  //Presents yaw reading (z-axis rotation) from the IMU,
  //(z-axis is normal to the multi-tool's screen)
  //making the tool act like a compass/protractor
  //Unfortunatly the yaw drifts constantly (it's cannot be normalized by the acceletometer)
  //so you need to re-zero often
  while ( mode == PROTRACTOR ) {
    driveIMU(true); //turn on IMU
    checkDMP();
    yield();
    if (!zeroButtonRun) {
      yprOffsets[0] = ypr[0];
      zeroButtonRun = true;
    }
    ypr[0] -= yprOffsets[0];
    float yaw = ypr[0] * 180 / M_PI;
    //x and y co-ordinates for the angle line end/start points
    float lineXOffset = circleRadius * cos(ypr[0]);
    float lineYOffset = circleRadius * sin(ypr[0]);
    uint8_t circleXcenter = circleRadius;
    display.clearDisplay();
    drawHeader("Protractor");
    display.setTextSize(2);
    display.setCursor(52, 30);
    display.print(yaw, 1); //print the yaw value with one decimal place to avoid text overflow
    //draws a line from -lineXOffset, --lineYOffset to +lineXOffset, +lineYOffset through the cross-hair's circle center
    display.drawLine(circleXcenter - lineXOffset, circleYcenter - lineYOffset , circleXcenter + lineXOffset, circleYcenter + lineYOffset, WHITE);
    drawCrosshairCircle(circleXcenter);
    display.display();
    readButtons();
  }

  //distance sensing using the VLX53L1X TOF sensor
  //also turns on the laser pointer
  //Calculates an average range after DIST_LOOP_MAX_COUNT readings
  //note that the time period to get an average is the time length for one reading times the DIST_LOOP_MAX_COUNT
  //The sensor's range mode can be changed (Long, Medium, Short) using the zero button
  //units (mm or in) can be toggled using the extra button
  while (mode == LASER_RULER) {
    yield();
    //if the dist sensor is off, we're just starting the mode
    //we want to display something for the user while we wait for the first measurment avg
    if (distSensorOff) {
      driveLaser(true);
      display.clearDisplay();
      drawHeader("Laser Ruler");
      display.setTextSize(2);
      display.setCursor(0, 25);
      display.print("Reading...");
      display.display();
    }
    driveDistSensor(true);

    //take DIST_LOOP_MAX_COUNT readings and average them
    //(note this will take DIST_LOOP_MAX_COUNT times the measurement period time length)
    distSensor.read(true);
    rangeSum += (distSensor.ranging_data.range_mm - INTERNAL_OFFSET);
    distLoopCount++;

    //if the zero button is pressed we change measurement modes (Long, Medium, Short)
    //we'll also reset the range average to start a new one
    if (!zeroButtonRun) {
      distMode = (distMode + 1) % NUM_DIST_MODES;
      if ( distMode == 0) {
        distSensor.setDistanceMode( VL53L1X::Long );
      } else if (distMode == 1 ) {
        distSensor.setDistanceMode( VL53L1X::Medium );
      } else {
        distSensor.setDistanceMode( VL53L1X::Short );
      }
      zeroButtonRun = true;
      rangeSum = 0;
      distLoopCount = 0;
    }

    //if we've collected the DIST_LOOP_MAX_COUNT number of readings
    //we can caculate the average and display it
    //we also display the mode, and the latest status message
    if (distLoopCount >= DIST_LOOP_MAX_COUNT) {
      distLoopCount = 0;
      rangeAvg = rangeSum / DIST_LOOP_MAX_COUNT;
      rangeSum = 0;
      display.clearDisplay();
      drawHeader("Laser Ruler");
      display.setTextSize(3);
      display.setCursor(0, 20);
      int decim;
      if (extraButtonToggle) {
        rangeAvg = rangeAvg / 10 * cmToIn;
        decim = 2;
        units = "\"";
      } else {
        decim = 0;
        units = "mm";
      }
      centerString( doubleToString(rangeAvg, decim) + units, 20, 3);
      //display.print(units);
      display.setTextSize(1);
      display.setCursor(0, 45);
      display.print("Mode:");
      display.print(distModeSting[distMode]); //prints out the info for the current dist mode (see sensor config for info)
      display.setCursor(0, 55);
      //displays the status message from the sensor
      //the sensor's min range is 4cm, but there's no message for this, so we add one
      if (rangeAvg < 40) {
        display.print("Below min range");
      } else {
        display.print(VL53L1X::rangeStatusToString(distSensor.ranging_data.range_status));
      }
      display.display();
    }
    readButtons();
  }

  //displays the distance covered by the wheel and encoder (ie a ruler)
  //we always show a positive distance (although the reading can increment and decerement)
  while (mode == WHEEL_RULER) {
    yield();
    display.clearDisplay();
    drawHeader("Wheel Ruler");

    long newPosition = myEnc.read();
    //we always show a positive distance
    //the user can zero it if needed
    double displayPos = abs(newPosition) * encStepLength;
    //zeros the encoder count
    if (!zeroButtonRun) {
      myEnc.write(0);
      zeroButtonRun = true;
    }
    display.setTextSize(3);
    display.setCursor(10, 20);
    if (extraButtonToggle) {
      displayPos = displayPos * cmToIn;  //convert from cm to in
      units = "in";
    } else {
      units = "cm";
    }
    centerString( doubleToString((double)displayPos, 2), 20, 3);
    display.setCursor(46, 43);
    display.print(units);
    display.display();
    readButtons();
  }

  //a tachometer that uses the encoder wheel to measure rpm
  while (mode == WHEEL_TACHOMETER) {
    runTachometer("Wheel Tactometer", 1);
  }

  //outputs the temperature readings from the mlx9014
  //both the sensor body temp and the ir range temp are output
  while (mode == IR_THERMOMETER) {
    yield();
    display.clearDisplay();
    drawHeader("IR Thermometer");
    float tempAmb;
    float tempObj;
    if (extraButtonToggle) {
      tempAmb = mlx.readAmbientTempC();
      tempObj = mlx.readObjectTempC();
      units = "C";
    } else {
      tempAmb = mlx.readAmbientTempF();
      tempObj = mlx.readObjectTempF();
      units = "F";
    }
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.print("Amb:");
    display.print(tempAmb, 1);
    display.print(units);
    display.setCursor(0, 45);
    display.print("Obj:");
    display.print(tempObj, 1);
    display.print(units);
    display.display();
    readButtons();
  }

  //a tachometer using the ir led and photodiode to sense rotation
  //to use it correctly you need to attach some ir reflective material to the rotating body
  //and then adjust the potentiometer (the IR adj knob) until the IR ind led only lights up
  //when the multi-tool is pointing at the reflective material
  while (mode == IR_TACHOMETER) {
    runTachometer("IR Tactometer", 0);
  }

  //Laser pointer, just turns on/off the laser
  //both the extra and zero buttons will toggle the laser
  while (mode == LASER_POINTER) {
    yield();
    display.clearDisplay();
    drawHeader("Laser Pointer");
    display.setTextSize(2);
    display.setCursor(10, 30);
    //print the state of the laser
    if (laserOn) {
      display.print("Laser On");
    } else {
      display.print("Laser Off");
    }
    display.display();
    //if either of the extra or zero button's have been pressed
    //we need to turn the laser on or off
    //we then set the state of both buttons  b/c we don't know which was pressed
    if (!zeroButtonRun || !extraButtonRun) {
      driveLaser(!laserOn);
      zeroButtonRun = true;
      extraButtonRun = true;
    }
    readButtons();
  }

}

//read the mode, zero, and extra buttons
//if the buttons have been pressed set flags
//because the buttons are tied to the mcp23008, which is tied to one interrupt,
//we must read all the buttons when only one is pushed
//also we cannot read the buttons in the interrupt function directly b/c i2c is disabled during interrupts
//The zero and extra buttons have two state boolean flags, a "run" and a "toggle"
//the "toggle" changes whenever the button is pressed
//the "run" is set false whenever the button is pressed and must be cleared by the current mode
//Both flags are reset to defaults (true for "run" and false for "toggle") when a mode changes
void readButtons(void) {
  mcp.interruptCapture();
  //if the interrupt flag is set we need to read the button states
  if ( inputsChanged ) {
    delay(10); //small delay for debouncing
    inputsChanged = false;
    // Read all values at once
    uint8_t val = mcp.readGPIO(); // clears the interrupt flag on the MCP (INT pin goes high)

    //the mcp23008 reports a byte value to store the states of all the pins
    //so we need to read the bits corrosponding to the buttons
    //the buttons are on the first 3 pins, and occupy the first 3 bits
    if (bitRead(val, MODE_BUTTON_PIN)) {
      //if the mode pin is high, we need to increment the mode, wrapping is needed
      //and reset the system for the next modezsx
      resetSystemState();
      mode = (mode + 1) % NUM_MODES;
    }

    if (bitRead(val, EXTRA_BUTTON_PIN)) {
      extraButtonRun = false;
      extraButtonToggle = !extraButtonToggle;
    }

    if (bitRead(val, ZERO_BUTTON_PIN)) {
      zeroButtonRun = false;
      zeroButtonToggle = !zeroButtonToggle;
    }
  }

}

//resets button variables and turns off/resets all sensors
//sets a clean state for the next mode and avoids any sensors interfering with each other (ir beams etc)
void resetSystemState() {
  display.clearDisplay();
  extraButtonToggle = false;
  extraButtonRun = true;
  zeroButtonToggle = false;
  zeroButtonRun = true;
  driveLaser(false);
  myEnc.write(0);
  driveIMU(false);
  driveTac(false, 0);
  //if the VL53L1X is reading, turn it off to avoid clashing with IR sensor
  driveDistSensor(false);
}

//fills in the header on the screen (the yellow bar) with the current mode name and the battery charge level
//because the battery level can fluctuate with current draw / noise, we only measure it at fixed intervals
//this prevents it from changing too often on the display, confusing the user
void drawHeader(String modeName) {
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print(modeName);
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - prevBatReadTime;
  //update the battery level after batteryUpdateTime ms
  if (elapsedTime > batteryUpdateTime) {
    batteryLvl = battery.level();
    prevBatReadTime = millis();
  }
  display.setCursor(100, 0);
  display.print(batteryLvl);
  display.print(char(37)); //prints the % symbol
}

//draws a maximum sized cicle centered horizontally at x pixel
//with a vertical and horizontal line running through the center
void drawCrosshairCircle(uint16_t x) {
  display.drawFastVLine(x, YELLOW_BAR_HEIGHT + 1, circleRadius * 2, WHITE);
  display.drawFastHLine(x - circleRadius, circleYcenter, circleRadius * 2, WHITE);
  display.drawCircle(x, circleYcenter, circleRadius, WHITE);
}

//takes a double and returns a string of the double to the input decimal places
//uses the standard dtostrf() function and then trims the result to remove any leading spaces
String doubleToString(double num, int decimal) {
  const int bufferSize = 10;
  char buffer[bufferSize];
  String tmpStr = dtostrf(num, bufferSize, decimal, buffer);
  tmpStr.trim();
  return tmpStr;
}

//centers a string on the display at the specified y coordinate and text size
void centerString(String text, uint16_t yCoord, int textSize) {
  //the number of pixels needed to the left of the string
  //found by subtracting the screen width from the total length of the string in pixels,
  //then dividing whats left by two (because we want the padding to be equal on left and right)
  int16_t padding = (SCREEN_WIDTH - (text.length() * BASE_FONT_LENGTH * textSize) ) / 2;
  //if the text string is too long, the padding will be negative
  //we set it to zero to avoid this
  if (padding < 0) {
    padding = 0;
  }
  //draw the text at the input y coord
  display.setTextSize(textSize);
  display.setCursor(padding, yCoord);
  display.print(text);
}

//turn the laset on or off according to the state input
void driveLaser(boolean state) {
  mcp.digitalWrite(LASER_PIN, state);
  laserOn = state;
}

//if the imu isn't on and state is true, turn it on (attach its interrupt)
//if the imu is on and the state is false, turn it off (detach its interrupt)
//we don't need to keep attaching and detach the interrupt if it's already been done
void driveIMU(boolean state) {
  if (!imuOn && state) {
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_IMU), dmpDataReady, RISING);
    imuOn = true;
  } else if (imuOn && !state) {
    detachInterrupt(INTERRUPT_PIN_IMU);
    imuOn = false;
  }
}

//if the distance sensor isn't on and state is true, turn it on
//if the distance sensor is on and the state is false, turn it off
//we don't need to keep turing the sensor on and off if it's already been done
//b/c it takes a sec or so
void driveDistSensor(boolean state) {
  if (distSensorOff && state) {
    distSensor.startContinuous(laserDistRate);
    distSensorOff = false;
  } else if (!distSensorOff && !state) {
    distSensor.stopContinuous();
    distSensorOff = true;
  }
}

//tactType 0 -- IR Tachometer
//tactType 1 -- wheel Tachometer
//if the tactometer isn't on and state is true, turn it on (or just set the flag if using the encoder)
//if the tactometer is on and the state is false, turn it off (detach its interrupt, or just set the flag)
//we don't need to keep attaching and detach the interrupt if it's already been done
void driveTac(boolean state, int tacType) {
  if (tachOn && !state) {
    tachOn = false;
    if (tacType == 0) {
      mcp.digitalWrite(TACT_PIN, state);
      detachInterrupt(IR_TACT_PIN);
    }
  } else if (!tachOn && state) {
    tachOn = true;
    if (tacType == 0) {
      mcp.digitalWrite(TACT_PIN, state);
    }
  }
}


//tactType 0 -- IR Tachometer
//tactType 1 -- wheel Tachometer
//since the multi-tool features two types of tachometer the shared code has been consolidated into a single function
//the tactType determines which tachometer will be used
//esentially the code will wait for the user to hit zero and then start recording revolutions
//when the user hits zero again or when measureIntervalMax time has passed, we stop reading,
//divide the revolutions by the time passed and convert to minutes
void runTachometer(String displayName, int tactType) {
  yield();
  //if the tachometer isn't on we're on the first pass through the function
  //so we display a prompt for the user to start the first reading
  if (!tachOn) {
    driveTac(true, tactType);
    tactReadingStarted = false;
    display.clearDisplay();
    drawHeader(displayName);
    display.setTextSize(2);
    display.setCursor(15, 20);
    display.print("Hit Zero  to start");
    //if it's the IR tach, the user should adjust the sensetivity
    //to only pick up the ir reflection from the rotating object
    if (tactType == 0) {
      display.setTextSize(1);
      display.setCursor(0, 55);
      display.print("Please adjust IR knob");
    }
    display.display();
  }

  //if the zero button is pressed and we havn't started reading we start a new reading
  //we turn on the ir tach (if needed), reset the tach revolutions, and record the start time
  if (zeroButtonToggle && !tactReadingStarted) {
    if (tactType == 0) {
      irTacRevs = 0;
      //the ir tach revs are counted using an interrupt, which we attach to it's pin
      attachInterrupt(digitalPinToInterrupt(IR_TACT_PIN), countIRTactPulse, FALLING);
    } else {
      myEnc.write(0);
    }
    tactReadingStarted = true;
    tachStartTimeSet = false;
    display.clearDisplay();
    drawHeader(displayName);
    display.setTextSize(2);
    display.setCursor(5, 20);
    display.print("Reading...");
    display.setTextSize(1);
    display.setCursor(15, 45);
    display.print("Hit Zero to stop");
    display.setCursor(0, 55);
    display.print(String("Auto stop after " + measureIntervalMaxString + "sec"));
    display.display();
  }

  //the tach reading has started but the zero button has not been pressed again to stop
  //we only start the timer if we detect motion from either the IR photodiode or the encoder
  //we want to auto stop after measureIntervalMax, so we check how much time has passed
  //if enough time has passed, we set the zero button as pressed
  if (zeroButtonToggle && tactReadingStarted) {
    //if we have not detected motion yet, check for some
    //if we have, check if measureIntervalMax time has passed
    if (!tachStartTimeSet) {
      //if we detect motion ie either the encoder or tachRevs change
      //set the start time and flag that the time has been set
      if ( (myEnc.read() != 0 && tactType == 1) || (irTacRevs != 0 && tactType == 0) ) {
        tachStartTimeSet = true;
        tacStartTime = micros();
      }
    } else {
      unsigned long currentTime = micros();
      if ( (currentTime - tacStartTime) > measureIntervalMax) {
        zeroButtonToggle = false;
      }
    }
  }

  //if we were reading, but the zero button has been pressed, it's time to stop and caculate rpm
  if (!zeroButtonToggle && tactReadingStarted) {
    tactReadingStarted = false;
    unsigned long currentTime = micros();
    //get the total elapsed time for the measurment
    unsigned long elapsedTime = currentTime - tacStartTime;
    //if we're using the IR tach we detatch the interrupt to stop accumilating revs
    //if we're using the encoder wheel, we get it's new position
    if (tactType == 0) {
      detachInterrupt(IR_TACT_PIN);
      //convert total revs to rpm
      rpm = irTacRevs / (double)elapsedTime * usInMin;
    } else {
      long newPosition = myEnc.read();
      //convert total revs to rpm
      rpm = ( ( abs(newPosition) / ENC_STEPS_PER_REV ) / (double)elapsedTime) * usInMin;
    }
    display.clearDisplay();
    drawHeader(displayName);
    display.setTextSize(2);
    display.setCursor(10, 20);
    centerString( doubleToString( (double)rpm, 0 ), 20, 2);
    //display.print(rpm);
    display.setCursor(46, 40);
    display.print("RPM");
    display.setTextSize(1);
    display.setCursor(6, 55);
    display.print("Hit Zero to restart");
    display.display();
  }
  readButtons();
}

//ir tach interrupt counting routine
void countIRTactPulse() {
  irTacRevs++;
}

//checks if the imu is ready to be read
//we cannot read the imu in the interrupt function b/c i2c is disabled during interrupts
void checkDMP(void) {
  if (mpuInterrupt) {
    GetDMP();
  }
}

//reads the fifo packet from the imu and clears the buffer
//if the fifo count is too big or too small we won't read, and instead clear the buffer
void GetDMP() {
  //static unsigned long LastGoodPacketTime;
  mpuInterrupt = false;
  fifoCount = mpu.getFIFOCount();
  if ((!fifoCount) || (fifoCount % packetSize)) { // we have failed Reset and wait till next time!
    // digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    mpu.resetFIFO();// clear the buffer and start over
  } else {
    while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    //LastGoodPacketTime = millis();
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
  }
}

//returns the sign of the input number (either -1 or 1)
int getSign(double num) {
  if (num >= 0) {
    return 1;
  } else {
    return -1;
  }
}
