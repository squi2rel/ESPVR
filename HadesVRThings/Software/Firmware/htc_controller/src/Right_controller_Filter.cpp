/*
  Copyright 2023 HadesVR
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,INCLUDING BUT NOT LIMITED TO
  THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <SPI.h>
#include "RF24.h"
#include "FastIMU.h"
#include "Madgwick.h"
#include <Adafruit_NeoPixel.h>

//==========================================================================================================
//************************************ USER CONFIGURABLE STUFF HERE*****************************************
//==========================================================================================================

//#include "TFT_eSPI.h"                     // uncomment these three lines to use TFT input
//#include "Free_Fonts.h"
//#define TFT_Input

#include <WiFi.h>
#define Socket_Transport

#ifdef Socket_Transport
const char* ssid = "your ssid";
const char* password = "your password";
const int socketPort = 6699;                // be same in driver config
#endif

#define SERIAL_DEBUG
#define IMU_ADDRESS     0x68                // You can find it out by using the IMUIdentifier example
BMI160 IMU;                                // IMU type
#define CALPIN              13              //pin to start mag calibration at power on

#ifndef TFT_Input

#define SysPin              16
#define MenuPin             22
#define GripPin             17
#define JoyXPin             36
#define JoyYPin             39
#define JoyClickPin         23
#define TriggerPin          34
#define VbatPin             35

#define BatLevelMax         1500             //you need to find all of these values on your own
#define JoyXMin             2840             //check on the utils folder for sketches and instructions
#define JoyXMax             0             //that help on getting these values
#define JoyYMin             2880             //YOU NEED TO DO THIS FOR BOTH CONTROLLERS
#define JoyYMax             0             //if you use these values without changing them you MAY
#define JoyXDeadZoneMin     1250             //get stick drift
#define JoyXDeadZoneMax     1350
#define JoyYDeadZoneMin     1400
#define JoyYDeadZoneMax     1500

#else

#define JoyClickPin         2
#define VbatPin             1
#define BatLevelMax         968

#endif
//==========================================================================================================

Adafruit_NeoPixel pixels(1, 14, NEO_GRB + NEO_KHZ800);

class DataTransport {
  public:
    virtual void write(const void * buf, int len) = 0;
    virtual bool valid() = 0;
};

calData calib =
{ false,                   //data valid?
  {0, 0, 0},              //Accel bias
  {0, 0, 0},              //Gyro bias
  {0, 0, 0},              //Mag bias
  {1, 1, 1},              //Mag Scale
};

#define HTC_SysClick        0x0001
#define HTC_MenuClick       0x0002
#define HTC_ThumbstickClick 0x0004
#define HTC_GripClick       0x0008
#define HTC_ThumbstickTouch 0x0010
//==========================================================================================================
//************************************* Data packet stuff *************************************************
//==========================================================================================================
#pragma pack(push, 1)
struct ctrlData {
  int16_t qW;
  int16_t qX;
  int16_t qY;
  int16_t qZ;
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  uint16_t BTN;
  uint8_t  trigg;
  int8_t  axisX;
  int8_t  axisY;
  int8_t  trackY;
  uint8_t  vBAT;
  uint8_t  fingerThumb;
  uint8_t  fingerIndex;
  uint8_t  fingerMiddle;
  uint8_t  fingerRing;
  uint8_t  fingerPinky;
  uint8_t  gripForce;
  uint16_t Data;
};
ctrlData data;
#pragma pack(pop)
//==========================================================================================================
//**************************************** Analog inputs ***************************************************
//==========================================================================================================
int tracky;
int trackoutput;
int axisX;
int axisY;
bool joyTouch = false;
bool holding = false;
//==========================================================================================================
//**************************************** RF Data stuff ***************************************************
//==========================================================================================================
//uint64_t Pipe = 0xF0F0F0F0E1LL; //right
uint64_t Pipe = 0xF0F0F0F0D2LL; //left
#ifndef Socket_Transport
class RFTransport : public DataTransport {
  public:
    RFTransport();
    void write(const void *buf, int len);
    bool valid();
  private:
    RF24 radio;
};
#else
//==========================================================================================================
//************************************** Socket Data stuff *************************************************
//==========================================================================================================
#define TIME (esp_timer_get_time() / 1000)
constexpr int UDP_DELAY = 1000 / 45; // 45fps       to prevent packet spam
class SocketTransport : public DataTransport {
  public:
    SocketTransport();
    void write(const void *buf, int len);
    bool valid();
  private:
    WiFiUDP udp;
    IPAddress serverIP;
    bool found;
    char buffer[32] = {};
    uint64_t lastSent;
    const char *header = "HadesVR Sniffer <3";
};
#endif
//==========================================================================================================
//**************************************** IMU variables ***************************************************
//==========================================================================================================
AccelData IMUAccel;
GyroData IMUGyro;
MagData IMUMag;
//==========================================================================================================
//************************************** Filter variables **************************************************
//==========================================================================================================
Madgwick filter;
static const float MadgwickBeta = 0.16f;
float rot = 0.f;
//==========================================================================================================

#ifdef TFT_Input
TFT_eSPI tft = TFT_eSPI();
#define INBOUNDS(x, y, r) (x > r->x && x < r->x + r->w && y > r->y && y < r->y + r->h)

struct Circle {
  int16_t x, y;
  uint16_t r;
};

struct Rect {
  int16_t x, y;
  uint16_t w, h;
};

struct Point {
  int16_t x, y;
};

struct Pointf {
  float x, y;
};

struct {
  int64_t lastTouch;
  Point size;
  Point touchRawPos;
  Rect sys;
  Rect menu;
  Rect grip;
  Rect trigger;
  Rect cal;
  Circle touch;
  int16_t triggerRawX;
  Pointf touchPos;
  float triggerX;
  bool touchPressed;
  bool trigPressed;
  bool sysPressed;
  bool menuPressed;
  bool gripPressed;
  bool touchEnabled;
  time_t calPressedTime;
} tft_data {
  0,
  { 0, 0 },
  { 0, 0 },
  { 0, 0, 0, 0 },
  { 0, 0, 0, 0 },
  { 0, 0, 0, 0 },
  { 0, 0, 0, 0 },
  { 0, 0, 0, 0 },
  { 0, 0, 0 },
  0,
  false,
  false,
  false,
  false,
  false,
  0
};

void init_tft() {
  setup_t user;
  tft.getSetup(user);
  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_WHITE);

  bool calOK;
  EEPROM.get(119, calOK);
  if (!calOK || !digitalRead(CALPIN)) {
    touch_calibrate();
  } else {
    uint16_t calData[5];
    EEPROM.get(120, calData);
    tft.setTouch(calData);
  }

  Point *size = &tft_data.size;
  size->x = user.tft_width;
  size->y = user.tft_height;

  uint16_t pad = size->y - size->x;

  Rect *trigger = &tft_data.trigger;
  trigger->x = trigger->y = 0;
  trigger->w = size->x;
  trigger->h = pad - 10;

  Circle *touch = &tft_data.touch;
  touch->x = size->x / 2;
  touch->y = pad + touch->x;
  touch->r = size->x / 2 - 20;

  float l1 = sqrtf(touch->x * touch->x * 2) - touch->r;
  int16_t side = sqrtf((l1 * l1) / 2);

  Rect *grip = &tft_data.grip;
  grip->x = 0;
  grip->y = pad;
  grip->w = grip->h = side;

  Rect *menu = &tft_data.menu;
  menu->x = size->x - side;
  menu->y = pad;
  menu->w = menu->h = side;

  Rect *sys = &tft_data.sys;
  sys->x = 0;
  sys->y = size->y - side;
  sys->w = sys->h = side;

  Rect *cal = &tft_data.cal;
  cal->x = size->x - side;
  cal->y = size->y - side;
  cal->w = cal->h = side;

  tft.setTextColor(TFT_BLACK);
  tft.setTextDatum(CC_DATUM);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setCursor(0, 40);
}

void drawElements() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_BLACK);
  tft.setFreeFont(FM9);
  Rect *trigger = &tft_data.trigger;
  Circle *touch = &tft_data.touch;
  Rect *grip = &tft_data.grip;
  Rect *menu = &tft_data.menu;
  Rect *sys = &tft_data.sys;
  Rect *cal = &tft_data.cal;
  tft.fillRect(trigger->x, trigger->y, trigger->w, trigger->h, TFT_YELLOW);
  tft.drawFastVLine(trigger->w * 0.1f, trigger->y, trigger->h, TFT_BLACK);
  tft.drawFastVLine(trigger->w * 0.9f, trigger->y, trigger->h, TFT_BLACK);
  tft.drawString("Trigger", trigger->x + trigger->w / 2, trigger->y + trigger->h / 2);
  tft.fillSmoothCircle(touch->x, touch->y, touch->r, TFT_WHITE, TFT_BLACK);
  tft.drawString("Touch", touch->x, touch->y);
  tft.fillRoundRect(sys->x, sys->y, sys->w, sys->h, 5, TFT_YELLOW);
  tft.drawString("Sys", sys->x + sys->w / 2, sys->y + sys->h / 2);
  tft.fillRoundRect(grip->x, grip->y, grip->w, grip->h, 5, TFT_YELLOW);
  tft.drawString("Grip", grip->x + grip->w / 2, grip->y + grip->h / 2);
  tft.fillRoundRect(menu->x, menu->y, menu->w, menu->h, 5, TFT_YELLOW);
  tft.drawString("Menu", menu->x + menu->w / 2, menu->y + menu->h / 2);
  tft.fillRoundRect(cal->x, cal->y, cal->w, cal->h, 5, TFT_YELLOW);
  tft.drawString("Cal", cal->x + cal->w / 2, cal->y + cal->h / 2);
}

void update_tft() {
  if (!tft_data.touchEnabled) return;
  static bool touched;
  uint16_t x, y;
  bool touching = tft.getTouch(&x, &y, 100);
  if (!touching && !holding && tft_data.triggerRawX > 0) {
    uint16_t d = (TIME - tft_data.lastTouch) / 3;
    //tft_data.triggerRawX -= d;
    tft_data.triggerRawX = 0;
    if (tft_data.triggerRawX < 0) tft_data.triggerRawX = 0;
    tft_data.triggerX = (float) tft_data.triggerRawX / tft_data.trigger.w;
    tft.fillRect(tft_data.triggerRawX, tft_data.trigger.y, d, tft_data.trigger.h, TFT_YELLOW);
    tft.drawFastVLine(tft_data.trigger.w * 0.1f, tft_data.trigger.y, tft_data.trigger.h, TFT_BLACK);
    tft.drawFastVLine(tft_data.trigger.w * 0.9f, tft_data.trigger.y, tft_data.trigger.h, TFT_BLACK);
  } 
  if (!touching && TIME - tft_data.lastTouch < 50) return;
  readTouch(x, y, touching, touched);
  touching = true;
  tft_data.lastTouch = TIME;
}

void readTouch(uint16_t x, uint16_t y, bool touching, bool &touched) {
  Circle *touch = &tft_data.touch;
  Point *touchRawPos = &tft_data.touchRawPos;
  Pointf *touchPos = &tft_data.touchPos;

  if (!touching) {
    touched = false;
    tft_data.touchPressed = tft_data.trigPressed = false;
    if (touchRawPos->x != 0) {
      tft.fillCircle(touchRawPos->x, touchRawPos->y, 11, TFT_WHITE);
      touchRawPos->x = 0;
      touchPos->x = touchPos->y = 0;
    }
    Rect *btn;
    if (tft_data.gripPressed) {
      btn = &tft_data.grip;
      tft.fillRoundRect(btn->x, btn->y, btn->w, btn->h, 5, TFT_YELLOW);
      tft.drawString("Grip", btn->x + btn->w / 2, btn->y + btn->h / 2);
      tft_data.gripPressed = false;
    }
    if (tft_data.menuPressed) {
      btn = &tft_data.menu;
      tft.fillRoundRect(btn->x, btn->y, btn->w, btn->h, 5, TFT_YELLOW);
      tft.drawString("Menu", btn->x + btn->w / 2, btn->y + btn->h / 2);
      tft_data.menuPressed = false;
    }
    if (tft_data.sysPressed) {
      btn = &tft_data.sys;
      tft.fillRoundRect(btn->x, btn->y, btn->w, btn->h, 5, TFT_YELLOW);
      tft.drawString("Sys", btn->x + btn->w / 2, btn->y + btn->h / 2);
      tft_data.sysPressed = false;
    }
    return;
  }

  float dx = x - touch->x, dy = y - touch->y;
  float d = sqrtf(dx * dx + dy * dy);
  if (tft_data.touchPressed && d >= touch->r - 11.5f) {
    float r = touch->r - 12;
      x = touch->x + r * dx / d;
      y = touch->y + r * dy / d;
      d = 0;
  }
  if ((!touched || tft_data.touchPressed) && d < touch->r - 11.5f) {
    tft_data.touchPressed = true;
    if (touchRawPos->x != 0) {
      tft.fillCircle(touchRawPos->x, touchRawPos->y, 11, TFT_WHITE);
    }
    tft.fillCircle(x, y, 10, TFT_LIGHTGREY);
    tft.drawCircle(x, y, 10, TFT_DARKGREY);
    touchRawPos->x = x;
    touchRawPos->y = y;
    touchPos->x = (float) (x - touch->x) / touch->r;
    touchPos->y = (float) (y - touch->y) / touch->r;
  }

  Rect *trigger = &tft_data.trigger;
  if (tft_data.trigPressed || INBOUNDS(x, y, trigger)) {
    if (!tft_data.trigPressed) holding = false;
    tft_data.trigPressed = true;
    if (x < tft_data.triggerRawX) {
      tft.fillRect(x, trigger->y, tft_data.triggerRawX - x, trigger->h, TFT_YELLOW);
    }
    else {
      tft.fillRect(tft_data.triggerRawX, trigger->y, x - tft_data.triggerRawX, trigger->h, TFT_ORANGE);
    }
    tft.drawFastVLine(trigger->w * 0.1f, trigger->y, trigger->h, TFT_BLACK);
    tft.drawFastVLine(trigger->w * 0.9f, trigger->y, trigger->h, TFT_BLACK);
    tft_data.triggerRawX = x;
    tft_data.triggerX = (float) x / trigger->w;
    holding = holding || !digitalRead(JoyClickPin);
    return;
  }

  if (touched) return;

  Rect *btn;
  btn = &tft_data.grip;
  if (!tft_data.gripPressed && INBOUNDS(x, y, btn)) {
    tft.fillRoundRect(btn->x, btn->y, btn->w, btn->h, 5, TFT_ORANGE);
    tft.drawString("Grip", btn->x + btn->w / 2, btn->y + btn->h / 2);
    tft_data.gripPressed = true;
  }
  btn = &tft_data.menu;
  if (!tft_data.menuPressed && INBOUNDS(x, y, btn)) {
    tft.fillRoundRect(btn->x, btn->y, btn->w, btn->h, 5, TFT_ORANGE);
    tft.drawString("Menu", btn->x + btn->w / 2, btn->y + btn->h / 2);
    tft_data.menuPressed = true;
  }
  btn = &tft_data.sys;
  if (!tft_data.sysPressed && INBOUNDS(x, y, btn)) {
    tft.fillRoundRect(btn->x, btn->y, btn->w, btn->h, 5, TFT_ORANGE);
    tft.drawString("Sys", btn->x + btn->w / 2, btn->y + btn->h / 2);
    tft_data.sysPressed = true;
  }
  btn = &tft_data.cal;
  if (INBOUNDS(x, y, btn)) {
    if (TIME - tft_data.calPressedTime < 3000) return; 
    tft_data.touchEnabled = false;
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(0, 20);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.println("Accelerometer and gyroscope calibration mode.");
    tft.println("Keep IMU completely still on flat and level surface.");
    delay(8000);
    IMU.calibrateAccelGyro(&calib);
    tft.println("Accel & Gyro calibration complete!");
    delay(1000);
    calib.valid = true;
    if (IMU.hasMagnetometer()) {
      tft.println("Magnetic calibration mode.");
      tft.println("Move IMU in figure 8 until done.");
      delay(3000);
      IMU.calibrateMag(&calib);
      tft.println("Magnetic calibration complete!");
      delay(1000);
    }
    EEPROM.put(210, calib);
    tft.fillScreen(TFT_BLACK);
    printCalibration();
#ifdef ESP32
    EEPROM.commit();
#endif
    tft.println("Calibrate finished!");
    delay(3000);
    drawElements();
    tft_data.touchEnabled = true;
  } else {
    tft_data.calPressedTime = TIME;
  }
}
#endif

#ifndef Socket_Transport
RFTransport::RFTransport()
  : radio(16, 17)
{
  while(!radio.begin()) {
    Serial.println("NRF24L01 Module not detected!");
    delay(1000);
  }
  radio.setPayloadSize(32);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.openWritingPipe(Pipe);
  radio.startListening();
  radio.setAutoAck(false);
  Serial.println("NRF24L01 Module up and running!");
}

bool RFTransport::valid() {
  return true;
}

void RFTransport::write(const void *buf, int len) {
  radio.stopListening();
  radio.write(buf, len);
  radio.startListening();
}
#else
SocketTransport::SocketTransport()
  : udp(WiFiUDP()),
  serverIP(IPAddress()),
  found(false),
  lastSent(0)
{
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
#ifdef TFT_Input
  tft.println("Connecting to WiFi...");
#endif
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  pixels.setPixelColor(0, pixels.Color(128, 255, 0));
  pixels.show();
  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  udp.begin(socketPort);
  Serial.print("Listening on UDP port ");
  Serial.println(socketPort);
#ifdef TFT_Input
  tft.println("\nConnected to WiFi");
  tft.print("IP Address: ");
  tft.println(WiFi.localIP());
  tft.print("Listening on UDP port ");
  tft.println(socketPort);
  tft.println("Waiting for server");
#endif
}

bool SocketTransport::valid() {
  if (!found) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(buffer, 31);
      if (len > 0) {
        buffer[len] = 0;
      }

      if (strcmp(buffer, header) == 0) {
        serverIP = udp.remoteIP();
        udp.stop();
        found = true;
        Serial.println("Server found!");
        
#ifdef TFT_Input
        tft_data.touchEnabled = true;
        drawElements();
#endif
      }
    }
  }
  return found;
}

void SocketTransport::write(const void *buf, int len) {
  if (TIME - lastSent < UDP_DELAY) return;
  lastSent = TIME;
  udp.beginPacket(serverIP, socketPort);
  udp.write(static_cast<uint8_t>(Pipe & 0xff));
  udp.write((const uint8_t*)buf, len);
  udp.endPacket();
}
#endif



#ifndef TFT_Input
void printCalibration()
{
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  if (IMU.hasMagnetometer()) {
    Serial.println("Mag biases X/Y/Z: ");
    Serial.print(calib.magBias[0]);
    Serial.print(", ");
    Serial.print(calib.magBias[1]);
    Serial.print(", ");
    Serial.println(calib.magBias[2]);
    Serial.println("Mag Scale X/Y/Z: ");
    Serial.print(calib.magScale[0]);
    Serial.print(", ");
    Serial.print(calib.magScale[1]);
    Serial.print(", ");
    Serial.println(calib.magScale[2]);
  }
  delay(5000);
}
#else
void printCalibration()
{
  tft.setCursor(0, 40);
  tft.println("Accel biases X/Y/Z: ");
  tft.print(calib.accelBias[0]);
  tft.print(", ");
  tft.print(calib.accelBias[1]);
  tft.print(", ");
  tft.println(calib.accelBias[2]);
  tft.println("Gyro biases X/Y/Z: ");
  tft.print(calib.gyroBias[0]);
  tft.print(", ");
  tft.print(calib.gyroBias[1]);
  tft.print(", ");
  tft.println(calib.gyroBias[2]);
  if (IMU.hasMagnetometer()) {
    tft.println("Mag biases X/Y/Z: ");
    tft.print(calib.magBias[0]);
    tft.print(", ");
    tft.print(calib.magBias[1]);
    tft.print(", ");
    tft.println(calib.magBias[2]);
    tft.println("Mag Scale X/Y/Z: ");
    tft.print(calib.magScale[0]);
    tft.print(", ");
    tft.print(calib.magScale[1]);
    tft.print(", ");
    tft.println(calib.magScale[2]);
  }
  delay(5000);
}
#endif

DataTransport *conn;

void scanI2C() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
}

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
#endif

  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
  Wire.begin(19, 18);
  Wire.setClock(400000); //400khz clock
  scanI2C();

  pinMode(CALPIN, INPUT_PULLUP);
#ifndef TFT_Input
  pinMode(SysPin, INPUT_PULLUP);
  pinMode(MenuPin, INPUT_PULLUP);
  pinMode(GripPin, INPUT_PULLUP);
  pinMode(TriggerPin, INPUT_PULLUP);
#endif
  pinMode(JoyClickPin, INPUT_PULLUP);

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0)
  {
    Serial.print("IMU ERROR: ");
    Serial.println(err);
    while (true);
  }

#ifdef ESP32
  EEPROM.begin(4096);
#endif

#ifdef TFT_Input
  init_tft();
#endif

  EEPROM.get(210, calib);

  bool calDone = !calib.valid;                             //check if calibration values are on flash
#ifndef TFT_Input
  while (calDone)
  {
    delay(1000);
    Serial.print("Calibration not done!");
    if (!digitalRead(CALPIN))
    {
      calDone = false;
    }
  }
  if (!digitalRead(CALPIN)) {
    pixels.setPixelColor(0, pixels.Color(255, 255, 0));
    pixels.show();
    delay(3000);
    if (!digitalRead(MenuPin)) {
      pixels.setPixelColor(0, pixels.Color(0, 255, 0));
      pixels.show();
      Serial.println("Accelerometer and gyroscope calibration mode.");
      Serial.println("Keep IMU completely still on flat and level surface.");
      delay(8000);
      IMU.calibrateAccelGyro(&calib);
      Serial.println("Accel & Gyro calibration complete!");
      calib.valid = true;
    }
    else {
      if (IMU.hasMagnetometer()) {
        pixels.setPixelColor(0, pixels.Color(0, 0, 255));
        pixels.show();
        Serial.println("Magnetic calibration mode.");
        Serial.println("Move IMU in figure 8 until done.");
        delay(3000);
        IMU.calibrateMag(&calib);
        Serial.println("Magnetic calibration complete!");
        delay(1000);
      }
    }
    printCalibration();
    Serial.println("Writing values to EEPROM!");
    EEPROM.put(210, calib);
#ifdef ESP32
    EEPROM.commit();
#endif
    delay(3000);
  }
#endif

#ifdef Socket_Transport
  conn = new SocketTransport();
#else
  conn = new RFTransport();
#endif

  //initialize controller data.
  data.qW = 1;
  data.qX = 0;
  data.qY = 0;
  data.qZ = 0;
  data.BTN = 0;
  data.trigg = 0;
  data.axisX = 0;
  data.axisY = 0;
  data.trackY = 0;
  data.vBAT = 0;
  data.fingerThumb = 0;
  data.fingerIndex = 0;
  data.fingerMiddle = 0;
  data.fingerRing = 0;
  data.fingerPinky = 0;
  data.Data = 0xFF;

  filter.begin(MadgwickBeta);
  IMU.setIMUGeometry(3);
  err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0)
  {
    Serial.print("IMU ERROR: ");
    Serial.println(err);
    while (true);
  }
}

void loop() {

#ifdef TFT_Input
  update_tft();
#endif

  IMU.update();
  IMU.getAccel(&IMUAccel);
  IMU.getGyro(&IMUGyro);

  if (IMU.hasMagnetometer()) {
    IMU.getMag(&IMUMag);
    filter.update(IMUGyro.gyroX, -IMUGyro.gyroY, -IMUGyro.gyroZ, IMUAccel.accelX, -IMUAccel.accelY, -IMUAccel.accelZ, IMUMag.magX, -IMUMag.magY, -IMUMag.magZ);
  }
  else {
    filter.updateIMU(IMUGyro.gyroX, -IMUGyro.gyroY, -IMUGyro.gyroZ, IMUAccel.accelX, -IMUAccel.accelY, -IMUAccel.accelZ);
  }

  rot += (abs(IMUGyro.gyroX) + abs(IMUGyro.gyroY) + abs(IMUGyro.gyroZ));
  if (rot > 64000.f) rot = 64000.f;
  rot *= 0.97f;
  filter.changeBeta(rot * (1.5 - 0.1) / 64000 + 0.1);

  int btn = 0;

#ifndef TFT_Input
  axisX = analogRead(JoyXPin);
  axisY = analogRead(JoyYPin);

  if (axisX > JoyXDeadZoneMax || axisX < JoyXDeadZoneMin) {
    data.axisX = -map(axisX, JoyXMin, JoyXMax, -127, 127);
  } else {
    data.axisX = 0;
  }

  if (axisY > JoyYDeadZoneMax || axisY < JoyYDeadZoneMin) {
    data.axisY = map(axisY, JoyYMin, JoyYMax, -127, 127);
    btn |= HTC_ThumbstickTouch;
  } else {
    data.axisY = 0;
  }

  data.trigg = (map(analogRead(TriggerPin), 0, 4096, 0, 255));

  if (!digitalRead(SysPin)) {
    btn |= HTC_SysClick;
  }
  if (!digitalRead(MenuPin)) {
    btn |= HTC_MenuClick;
  }
  if (!digitalRead(JoyClickPin)) {
    btn |= HTC_ThumbstickClick;
  }
  if (!digitalRead(GripPin)) {
    btn |= HTC_GripClick;
  }
#else
  data.axisX = 127 * tft_data.touchPos.x;
  data.axisY = 127 * -tft_data.touchPos.y;

  if (tft_data.touchPressed) {
    btn |= HTC_ThumbstickTouch;
  }

  //data.trigg = 255 * tft_data.triggerX;
  data.trigg = tft_data.triggerX == 0 ? 0 : 255;

  if (tft_data.sysPressed) {
    btn |= HTC_SysClick;
  }
  if (tft_data.menuPressed) {
    btn |= HTC_MenuClick;
  }
  if (!digitalRead(JoyClickPin)) {
    btn |= HTC_ThumbstickClick;
  }
  if (tft_data.gripPressed) {
    btn |= HTC_GripClick;
  }
#endif

  data.BTN = btn;
  data.trackY = (trackoutput * 127);
  data.vBAT = (map(analogRead(VbatPin), 1000, BatLevelMax, 0, 255));
  data.qW = (int16_t)(filter.getQuatW() * 32767.f);
  data.qX = (int16_t)(filter.getQuatY() * 32767.f);
  data.qY = (int16_t)(filter.getQuatZ() * 32767.f);
  data.qZ = (int16_t)(filter.getQuatX() * 32767.f);
  data.accX = (short)(IMUAccel.accelX * 2048);
  data.accY = (short)(IMUAccel.accelY * 2048);
  data.accZ = (short)(IMUAccel.accelZ * 2048);

  if (conn->valid()) {
    conn->write(&data, sizeof(data));
  }
}

#ifdef TFT_Input
void touch_calibrate()
{
  uint8_t calDataOK = 0;
  uint16_t calData[5];

  // Calibrate
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(20, 0);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.println("Touch corners as indicated");

  tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

  tft.fillScreen(TFT_BLACK);
  Serial.printf("%d %d %d %d %d\n", calData[0], calData[1], calData[2], calData[3], calData[4]);
  Serial.println("Writing values to EEPROM!");
  EEPROM.put(119, true);
  EEPROM.put(120, calData);
#ifdef ESP32
  EEPROM.commit();
#endif
  delay(4000);
}
#endif