// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#include "Wire.h"


//#include <PDQ_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Battery.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

#define LED_PIN 13

// MPU control/status vars
int FifoAlive = 0; // tests if the interrupt is triggering
int IsAlive = -20;     // counts interrupt start at -20 to get 20+ good values before assuming connected
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                     SCREEN SETUP                         ===
// ================================================================
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1// Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


//our range for lipo voltage is 4.2-3.4V, after 3.4 the wemos LDO will become unreliable
//so we should recharge
Battery battery(3400, 4200, A0);
const unsigned long batteryUpdateTime = 5000; //how often we update the battery level in ms
unsigned long prevBatReadTime = 0; //the last time we read the battery in ms
uint8_t batteryLvl; //the battery percentage

void setup() {

  pinMode(LED_PIN, OUTPUT);
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

// ================================================================
  // ===                  BATTERY CONFIG                          ===
  // ================================================================
  pinMode(A0, INPUT); //our analog input pin connected to the battery
  //start a battery object with 4.2 max votage using sigmodal mapping for lipos
  //the wemos already has a voltage divider on A0
  //our inclusion of a 100K resistor brings the divider ratio to exactly 1.0
  battery.begin(4200, 1.0, &sigmoidal);
  batteryLvl = battery.level(); //get an inital reading

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(WHITE);
  display.print("HI");
  display.display();

}

void loop() {
  int mode = 1;
  if (mpuInterrupt) {
    GetDMP();

    if (mode == 1) {
      // yaw: (about X axis)
      ypr[0] =  atan2( 2 * q.z * q.y + 2 * q.w * q.x , 2 * q.w * q.w + 2 * q.z * q.z - 1);
      // pitch: (nose up/down, about Y axis)
      ypr[1] = atan2(gravity.z , -sqrt(gravity.y * gravity.y + gravity.x * gravity.x));
      // roll: (tilt left/right, about z axis)
      ypr[2] = atan2(gravity.y , sqrt(gravity.x * gravity.x + gravity.z * gravity.z));

      if (gravity.x < 0) {
        if (ypr[1] > 0) {
          ypr[1] = PI - ypr[1];
        } else {
          ypr[1] = -PI - ypr[1];
        }
      }

      float roll = ypr[2] * 180 / M_PI;
      display.clearDisplay();
      int circleYcenter = display.height() - (display.height() - 14) / 2;
      display.setTextSize(2);
      display.setCursor(52, 20);
      display.print(roll);
      display.drawFastVLine(23, 16, 23 * 2, WHITE);
      display.drawFastHLine(0, circleYcenter, 23 * 2, WHITE);
      display.drawCircle(23, circleYcenter, 23, WHITE);
      display.drawLine(23 - 23 * cos(ypr[2]), circleYcenter - 23 * sin(ypr[2]) , 23 + 23 * cos(ypr[2]), circleYcenter + 23 * sin(ypr[2]), WHITE);
      display.display();
    } else {

      int circleYcenter = display.height() - (display.height() - 14) / 2;
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(52, 20);
      display.print("X");
      display.print(ypr[2] * 180 / M_PI);
      display.setCursor(52, 45);
      display.print("Y");
      display.print(ypr[1] * 180 / M_PI);
      display.fillCircle(23 + 23 * sin(ypr[2]) , circleYcenter + 23 * sin(ypr[1]), 2, WHITE);
      display.drawFastVLine(23, 16, 23 * 2, WHITE);
      display.drawFastHLine(0, circleYcenter, 23 * 2, WHITE);
      display.drawCircle(23, circleYcenter, 23, WHITE);
      display.display();
    }
//        Serial.print("ypr\t");
//        Serial.print(ypr[0] * 180 / M_PI);
//        Serial.print("\t");
//        Serial.print(ypr[1] * 180 / M_PI);
//        Serial.print("\t");
//        Serial.println(ypr[2] * 180 / M_PI);
  }
}


void GetDMP() {
  static unsigned long LastGoodPacketTime;
  mpuInterrupt = false;
  FifoAlive = 1;
  fifoCount = mpu.getFIFOCount();
  if ((!fifoCount) || (fifoCount % packetSize)) { // we have failed Reset and wait till next time!
    //digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    mpu.resetFIFO();// clear the buffer and start over
  } else {
    while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    LastGoodPacketTime = millis();
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
   // digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
  }
}
