//Crytal LED Color
//White : Initiating
//Blue : DPFlayer does not work properly
//Green : MPU 6050 couldn't connect with arduino
//Red or Any other color set by the user : All components has been connect and
//its ready to run

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "FastLED.h"
#define DATA_PIN 7
#define DATA_BAT_PIN 8
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS 140
#define NUM_LEDS_BAT 7
#define BRIGHTNESS 962
CRGB leds[NUM_LEDS];
CRGB led_bat[NUM_LEDS_BAT];
#define buttonPin 3

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

MPU6050 mpu;

#define OUTPUT_READABLE_REALACCEL

#define INTERRUPT_PIN 2

SoftwareSerial mySoftwareSerial(11, 9); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

#define led 13
bool state = true;

unsigned long randomNum;
int sw = 0;

/***declaring RGB pins in PWM mode***/
int redPin = 4;
int greenPin = 5;
int bluePin = 6;
#define COMMON_ANODE

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[128]; // FIFO storage buffer

// orientation/motion vars
Quaternion q1;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int oldX, oldY, oldZ;

int buttonState = 0;

unsigned long PTimer = 0;

uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

void clean_up() {
  for (int i = 0; i < NUM_LEDS; ++i) {
    leds[i] = CRGB::Black;
  }
  for (int i = 0; i < NUM_LEDS_BAT; ++i) {
    led_bat[i] = CRGB::Black;
  }
  FastLED.show();
}

void blade_on(long delayTime) {
  for (int i = 0; i < NUM_LEDS; ++i) {
    leds[i] = CRGB::Red;//Define the blade color
    leds[i++] = CRGB::Red;//Lighting up two led by loop, make the animation faster
    FastLED.show();
  }
}

int g = NUM_LEDS_BAT;

float x;

void battery_led_animation() {
  switch (state) {
    case false:
      switch (g) {
        default:
          led_bat[g - 1] = CRGB::Black;
          led_bat[g++] = x;
          FastLED.show();
          break;
      }
      break;
    case true:
      switch (g) {
        default:
          led_bat[g + 1] = CRGB::Black;
          led_bat[g--] = x;
          FastLED.show();
          break;
      }
      break;
  }
  if (g >= NUM_LEDS_BAT) {
    g = NUM_LEDS_BAT;
    state = !state;
  } else if ( g < 0 ) {
    g = 0;
    state = !state;
  }
}
void blade_off(long delayTime) {
  for (int i = NUM_LEDS; i >= 0; i--) {
    leds[i] = CRGB::Black;
    FastLED.show();
  }
}

float max_voltage;
void setup() {

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  Serial.begin(115200);
  mySoftwareSerial.begin(9600);

  int sensorValue = analogRead(A0); //read the A0 pin value
  max_voltage = sensorValue * (5.00 / 1023.00) * 2; //convert the value to a true voltage.

  clean_up();

  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  while (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println(F("Unable to begin"));
    Serial.println(myDFPlayer.readState()); //read mp3 state
    Serial.println(myDFPlayer.readFileCounts()); //read all file counts in SD card
    blue();
  }

  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.volume(30);

  myDFPlayer.EQ(DFPLAYER_EQ_BASS);

  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  randomSeed(analogRead(0));

  mpu.dmpInitialize();

  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip); // initializes LED strip
  FastLED.addLeds<LED_TYPE, DATA_BAT_PIN, COLOR_ORDER>(led_bat, NUM_LEDS_BAT).setCorrection(TypicalLEDStrip); // initializes LED strip

  FastLED.setBrightness(BRIGHTNESS);
  pinMode(buttonPin, INPUT);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  while (!Serial);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));

  while (!mpu.testConnection()) {
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    green();
  }

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
    mpu.resetFIFO();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  red();//Set color of crystal led
}
void loop() {
  unsigned long currentMillis = millis();

  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {}

  buttonState = digitalRead(buttonPin);
  if (buttonState == CHANGE) {
    blink();
    switch (sw) {
      case 1:
        myDFPlayer.play(3);
        blade_off(0);
        sw = 0;
        Serial.println(F("OFF"));
        break;
      case 0:
        myDFPlayer.play(12);
        blade_on(0);
        sw = 1;
        Serial.println(F("ON"));
        break;
    }
    mpu.resetFIFO();
  }
  /*
    if (Serial.available()) {//test if
    int incomingByte = Serial.parseInt();
    switch (incomingByte) {
    case 2:
      myDFPlayer.play(3);
      blade_off(0);
      Serial.println(F("OFF"));
      break;
    case 1:
      myDFPlayer.play(12);
      blade_on(0);
      Serial.println(F("ON"));
      break;
    }
    mpu.resetFIFO();
    }*/

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();
  mpu.resetFIFO();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.resetFIFO();

    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_REALACCEL
    mpu.dmpGetQuaternion(&q1, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q1);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    if (oldX == 0 || oldY == 0 || oldZ == 0) {
      oldX = aaReal.x;
      oldY = aaReal.y;
      oldZ = aaReal.z;
    } else {
      if ( sw == 1 && (abs(oldX - aaReal.x) > 1000) || (abs(oldY - aaReal.y) > 1000) || (abs(oldZ - aaReal.z) > 1000)) {
        //myDFPlayer.play(random(8, 10));
      }
    }
    oldX = aaReal.x;
    oldY = aaReal.y;
    oldZ = aaReal.z;
#endif
  }

  if (currentMillis - PTimer > 500) {
    PTimer = currentMillis;

    if (VoltMeter() > max_voltage) {
      max_voltage = VoltMeter();
    }

    float r = VoltMeter() / max_voltage;
    //TO-DO implement discharge curve
    if (VoltMeter() >= 4 && VoltMeter() <= 5) {
      x = CRGB::Blue;
    } else if (r >= 0.8) {
      x = CRGB::Green;
    } else if (r >= 0.6) {
      x = CRGB::Yellow;
    } else if (r >= 0.5) {
      x = CRGB::Orange;
    } else if (r <= 0.3) {
      x = CRGB::Red;
    }
    battery_led_animation();
  }
}
void setColor(int red, int green, int blue) {
#ifdef COMMON_ANODE
  red = 255 - red;
  green = 255 - green;
  blue = 255 - blue;
#endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}
void red () {
  setColor(255, 0, 0);
}
void green() {
  setColor(0, 255, 0);
}
void blue() {
  setColor(0, 0, 255);
}
void yellow() {
  setColor(255, 255, 0);
}
void white() {
  setColor(255, 255, 255);
}
void led_off() {
  setColor(0, 0, 0);
}
void purple() {
  setColor(255, 0, 255);
}
void blink() {
  state = !state;
  digitalWrite(led, state);
}
float VoltMeter() {
  int sensorValue = analogRead(A1); //read the A0 pin value
  float voltage = sensorValue * (5.00 / 1023.00) * 2; //convert the value to a true voltage.
  return voltage;
}


