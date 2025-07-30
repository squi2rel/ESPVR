#include <Arduino.h>
#include <Wire.h>
#include <TC358870.h>
#include <TC358870_REG.h>
#include <TC358870_INT.h>
#include <WiFi.h>
#include <FastIMU.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>

#define RST_TC 32
#define EN_18 26
#define IMU_CAL 2
#define EN_11 25
#define INT_TC 33
#define RST_SCREEN 16
#define EN_AVDD 17
#define USR_LED 5
#define BG_LED 18

// replace with your SSID and password
#define WIFI_SSID ssid
#define WIFI_PASSWORD password

// must be same as driver config
const int socketPort = 6699;
const char *header = "HadesVR Sniffer <3";
constexpr int UDP_DELAY = 1000 / 60; // 60fps

TC358870 tc(&Wire, 0x0f);
WiFiUDP udp = WiFiUDP();
BMI160 IMU;
IPAddress serverIP;
bool connected = false;

Adafruit_NeoPixel pixels(1, USR_LED, NEO_GRB + NEO_KHZ800);

const int pwmFrequency = 50000;
const int pwmChannel = 0;
const int pwmResolution = 8;

void (*RS_Int)();

// put function declarations here:

#define TIME (esp_timer_get_time() / 1000)
time_t lastSent;

calData calib =
    {
        false,     // data valid?
        {0, 0, 0}, // Accel bias
        {0, 0, 0}, // Gyro bias
        {0, 0, 0}, // Mag bias
        {1, 1, 1}, // Mag Scale
};

void setup()
{
  Serial.begin(115200);

  // put your setup code here, to run once:
  pinMode(RST_TC, OUTPUT);
  pinMode(EN_18, OUTPUT);
  pinMode(EN_11, OUTPUT);
  pinMode(INT_TC, INPUT_PULLDOWN);
  pinMode(RST_SCREEN, OUTPUT);
  pinMode(EN_AVDD, OUTPUT);
  pinMode(USR_LED, OUTPUT);
  pinMode(BG_LED, OUTPUT);
  pinMode(IMU_CAL, INPUT_PULLUP);

  ledcSetup(0, 5000, 8);
  ledcAttachPin(BG_LED, 0);
  digitalWrite(RST_SCREEN, LOW);
  delay(10);
  digitalWrite(EN_18, HIGH);
  delay(10);
  digitalWrite(EN_AVDD, HIGH);
  delay(10);
  digitalWrite(RST_SCREEN, HIGH);
  delay(10);
  digitalWrite(RST_TC, LOW);
  delay(10);
  digitalWrite(EN_11, HIGH);
  delay(100);
  digitalWrite(RST_TC, HIGH);
  delay(100);
  delay(1);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
  log_i("Connected to WiFi");
  log_i("IP Address: %s", WiFi.localIP().toString().c_str());

  udp.begin(socketPort);
  log_i("Listening on UDP port %d", socketPort);

  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();    
  
  Wire.begin(15, 4);
  scanI2C();
  bool success = tc.init();
  if (success)
  {
    digitalWrite(RST_SCREEN, LOW);
    delay(100);
    digitalWrite(RST_SCREEN, HIGH);
    delay(100);
    log_i("TC358870 initialization successful");
    HDMI_Startup();
    ledcWrite(0, 255 * 0.7);
  }
  EEPROM.begin(4096);
  EEPROM.get(210, calib);
  IMU.setIMUGeometry(4);
  int err = IMU.init(calib, 0x68);
  if (err != 0)
  {
    log_e("IMU ERROR: %d", err);
    return;
  }
  log_i("BMI160 initialization successful");
  if (!calib.valid || !digitalRead(IMU_CAL))
  {
    digitalWrite(USR_LED, HIGH);
    calibrateIMU();
  }
  digitalWrite(USR_LED, LOW);
}

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

void findServer()
{
  static char buffer[32] = {};
  int packetSize = udp.parsePacket();
  if (packetSize)
  {
    int len = udp.read(buffer, 31);
    if (len > 0)
    {
      buffer[len] = 0;
    }

    if (strcmp(buffer, header) == 0)
    {
      serverIP = udp.remoteIP();
      connected = true;
      udp.stop();
      log_i("Server found!");
    }
  }
}

void calibrateIMU()
{
  if (IMU.hasMagnetometer())
  {
    Serial.println("Magnetic calibration mode.");
    Serial.println("Move IMU in figure 8 until done.");
    delay(3000);
    IMU.calibrateMag(&calib);
    Serial.println("Magnetic calibration complete!");
    delay(1000);
  }
  Serial.println("Accelerometer and gyroscope calibration mode.");
  Serial.println("Keep IMU completely still on flat and level surface.");
  delay(8000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Accel & Gyro calibration complete!");
  calib.valid = true;
  printCalibration();
  EEPROM.put(210, calib);
  EEPROM.commit();
}

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
  if (IMU.hasMagnetometer())
  {
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

AccelData IMUAccel;
GyroData IMUGyro;
MagData IMUMag;

#pragma pack(push, 1)
struct HMDRAWPacket
{
  int16_t AccX;
  int16_t AccY;
  int16_t AccZ;

  int16_t GyroX;
  int16_t GyroY;
  int16_t GyroZ;

  int16_t MagX;
  int16_t MagY;
  int16_t MagZ;
} HMDRawData;
#pragma pack(pop)

void loop()
{
  if (RS_Int && digitalRead(INT_TC)) RS_Int();
  if (!connected)
  {
    findServer();
    return;
  }

  IMU.update();

  if (TIME - lastSent < UDP_DELAY) return;
  lastSent = TIME;

  IMU.getAccel(&IMUAccel);
  IMU.getGyro(&IMUGyro);

  if (IMU.hasMagnetometer())
  {
    IMU.getMag(&IMUMag);
    HMDRawData.MagX = (short)(IMUMag.magX * 5);
    HMDRawData.MagY = (short)(IMUMag.magY * 5);
    HMDRawData.MagZ = (short)(IMUMag.magZ * 5);
  }
  else
  {
    HMDRawData.MagX = (short)(0);
    HMDRawData.MagY = (short)(0);
    HMDRawData.MagZ = (short)(0);
  }

  HMDRawData.AccX = (short)(IMUAccel.accelX * 2048);
  HMDRawData.AccY = (short)(IMUAccel.accelY * 2048);
  HMDRawData.AccZ = (short)(IMUAccel.accelZ * 2048);

  HMDRawData.GyroX = (short)(IMUGyro.gyroX * 16);
  HMDRawData.GyroY = (short)(IMUGyro.gyroY * 16);
  HMDRawData.GyroZ = (short)(IMUGyro.gyroZ * 16);

  udp.beginPacket(serverIP, socketPort);
  udp.write(0xAA);
  udp.write((const uint8_t*)&HMDRawData, sizeof(HMDRawData));
  udp.endPacket();
  //log_i("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", IMUMag.magX, IMUMag.magY, IMUMag.magZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ, IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ);
}

void i2c1_uh2cd_write32(uint16_t addr, uint32_t dat) {
  tc.writeU32(addr, dat);
}

void i2c1_uh2cd_write16(uint16_t addr, uint32_t dat) {
  tc.writeU16(addr, dat);
}

void i2c1_uh2cd_write8(uint16_t addr, uint32_t dat) {
  tc.writeU8(addr, dat);
}

uint32_t i2c1_uh2cd_read32(uint16_t addr) {
  return tc.readU32(addr);
}

uint16_t i2c1_uh2cd_read16(uint16_t addr) {
  return tc.readU16(addr);
}

uint8_t i2c1_uh2cd_read8(uint16_t addr) {
  return tc.readU8(addr);
}

void Waitx1ms(uint32_t t) {
  tc.delay_ms(t);
}

void Waitx1us(uint32_t t) {
  tc.delay_us(t);
}

void Waitx1s(uint32_t t) {
  tc.delay_sec(t);
}

void HDMI_Startup(void)
{
  // Software Reset
  tc.writeU16(0x0004, 0x0004); // ConfCtl0
  tc.writeU16(0x0002, 0x3F01); // SysCtl
  tc.writeU16(0x0002, 0x0000); // SysCtl
  tc.writeU16(0x0006, 0x0008); // ConfCtl1
// DSI-TX0 Transition Timing
tc.writeU32(0x0108,0x00000001); // DSI_TX_CLKEN
tc.writeU32(0x010C,0x00000001); // DSI_TX_CLKSEL
tc.writeU32(0x02A0,0x00000001); // MIPI_PLL_CONTROL
tc.writeU32(0x02AC,0x0000B0F9); // MIPI_PLL_CNF
tc.delay_us();
tc.writeU32(0x02A0,0x00000003); // MIPI_PLL_CONTROL
tc.writeU32(0x0118,0x00000014); // LANE_ENABLE
tc.writeU32(0x0120,0x00002710); // LINE_INIT_COUNT
tc.writeU32(0x0124,0x00000000); // HSTX_TO_COUNT
tc.writeU32(0x0128,0x00000101); // FUNC_ENABLE
tc.writeU32(0x0130,0x00010000); // DSI_TATO_COUNT
tc.writeU32(0x0134,0x00005000); // DSI_PRESP_BTA_COUNT
tc.writeU32(0x0138,0x00010000); // DSI_PRESP_LPR_COUNT
tc.writeU32(0x013C,0x00010000); // DSI_PRESP_LPW_COUNT
tc.writeU32(0x0140,0x00010000); // DSI_PRESP_HSR_COUNT
tc.writeU32(0x0144,0x00010000); // DSI_PRESP_HSW_COUNT
tc.writeU32(0x0148,0x00001000); // DSI_PR_TO_COUNT
tc.writeU32(0x014C,0x00010000); // DSI_LRX-H_TO_COUNT
tc.writeU32(0x0150,0x00000141); // FUNC_MODE
tc.writeU32(0x0154,0x00000001); // DSI_RX_VC_ENABLE
tc.writeU32(0x0158,0x000000C8); // IND_TO_COUNT
tc.writeU32(0x0168,0x0000002A); // DSI_HSYNC_STOP_COUNT
tc.writeU32(0x0170,0x0000036A); // APF_VDELAYCNT
tc.writeU32(0x017C,0x00000081); // DSI_TX_MODE
tc.writeU32(0x018C,0x00000001); // DSI_HSYNC_WIDTH
tc.writeU32(0x0190,0x0000003C); // DSI_HBPR
tc.writeU32(0x01A4,0x00000000); // DSI_RX_STATE_INT_MASK
tc.writeU32(0x01C0,0x00000015); // DSI_LPRX_THRESH_COUNT
tc.writeU32(0x0214,0x00000000); // APP_SIDE_ERR_INT_MASK
tc.writeU32(0x021C,0x00000080); // DSI_RX_ERR_INT_MASK
tc.writeU32(0x0224,0x00000000); // DSI_LPTX_INT_MASK
tc.writeU32(0x0254,0x00000007); // LPTXTIMECNT
tc.writeU32(0x0258,0x00320609); // TCLK_HEADERCNT
tc.writeU32(0x025C,0x0012000C); // TCLK_TRAILCNT
tc.writeU32(0x0260,0x0018000A); // THS_HEADERCNT
tc.writeU32(0x0264,0x00009C40); // TWAKEUPCNT
tc.writeU32(0x0268,0x00000012); // TCLK_POSTCNT
tc.writeU32(0x026C,0x0010000A); // THS_TRAILCNT
tc.writeU32(0x0270,0x00000020); // HSTXVREGCNT
tc.writeU32(0x0274,0x0000001F); // HSTXVREGEN
tc.writeU32(0x0278,0x0007000C); // BTA_COUNT
tc.writeU32(0x027C,0x00000002); // DPHY_TX ADJUST
tc.writeU32(0x011C,0x00000001); // DSITX_START
// DSI-TX1 Transition Timing
tc.writeU32(0x0308,0x00000001); // DSI_TX_CLKEN
tc.writeU32(0x030C,0x00000001); // DSI_TX_CLKSEL
tc.writeU32(0x04A0,0x00000001); // MIPI_PLL_CONTROL
tc.writeU32(0x04AC,0x0000B0F9); // MIPI_PLL_CNF
tc.delay_us();
tc.writeU32(0x04A0,0x00000003); // MIPI_PLL_CONTROL
tc.writeU32(0x0318,0x00000014); // LANE_ENABLE
tc.writeU32(0x0320,0x00002710); // LINE_INIT_COUNT
tc.writeU32(0x0324,0x00000000); // HSTX_TO_COUNT
tc.writeU32(0x0328,0x00000101); // FUNC_ENABLE
tc.writeU32(0x0330,0x00010000); // DSI_TATO_COUNT
tc.writeU32(0x0334,0x00005000); // DSI_PRESP_BTA_COUNT
tc.writeU32(0x0338,0x00010000); // DSI_PRESP_LPR_COUNT
tc.writeU32(0x033C,0x00010000); // DSI_PRESP_LPW_COUNT
tc.writeU32(0x0340,0x00010000); // DSI_PRESP_HSR_COUNT
tc.writeU32(0x0344,0x00010000); // DSI_PRESP_HSW_COUNT
tc.writeU32(0x0348,0x00001000); // DSI_PR_TO_COUNT
tc.writeU32(0x034C,0x00010000); // DSI_LRX-H_TO_COUNT
tc.writeU32(0x0350,0x00000141); // FUNC_MODE
tc.writeU32(0x0354,0x00000001); // DSI_RX_VC_ENABLE
tc.writeU32(0x0358,0x000000C8); // IND_TO_COUNT
tc.writeU32(0x0368,0x0000002A); // DSI_HSYNC_STOP_COUNT
tc.writeU32(0x0370,0x0000036A); // APF_VDELAYCNT
tc.writeU32(0x037C,0x00000081); // DSI_TX_MODE
tc.writeU32(0x038C,0x00000001); // DSI_HSYNC_WIDTH
tc.writeU32(0x0390,0x0000003C); // DSI_HBPR
tc.writeU32(0x03A4,0x00000000); // DSI_RX_STATE_INT_MASK
tc.writeU32(0x03C0,0x00000015); // DSI_LPRX_THRESH_COUNT
tc.writeU32(0x0414,0x00000000); // APP_SIDE_ERR_INT_MASK
tc.writeU32(0x041C,0x00000080); // DSI_RX_ERR_INT_MASK
tc.writeU32(0x0424,0x00000000); // DSI_LPTX_INT_MASK
tc.writeU32(0x0454,0x00000007); // LPTXTIMECNT
tc.writeU32(0x0458,0x00320609); // TCLK_HEADERCNT
tc.writeU32(0x045C,0x0012000C); // TCLK_TRAILCNT
tc.writeU32(0x0460,0x0018000A); // THS_HEADERCNT
tc.writeU32(0x0464,0x00009C40); // TWAKEUPCNT
tc.writeU32(0x0468,0x00000012); // TCLK_POSTCNT
tc.writeU32(0x046C,0x0010000A); // THS_TRAILCNT
tc.writeU32(0x0470,0x00000020); // HSTXVREGCNT
tc.writeU32(0x0474,0x0000001F); // HSTXVREGEN
tc.writeU32(0x0478,0x0007000C); // BTA_COUNT
tc.writeU32(0x047C,0x00000002); // DPHY_TX ADJUST
tc.writeU32(0x031C,0x00000001); // DSITX_START


// Command Transmission Before Video Start
tc.writeU32(0x0110,0x00000016); // MODE_CONFIG
tc.writeU32(0x0310,0x00000016); // MODE_CONFIG
// LCD Initialization
tc.writeU16(0x0500,0x0004); // CMD_SEL

  // Soft Reset
  tc.writeU16(0x0504, 0x0005); // DCSCMD_Q
  tc.writeU16(0x0504, 0x0001); // DCSCMD_Q
  tc.delay_ms(5);

  // MCAP
  tc.writeU16(0x0504, 0x0023); // DCSCMD_Q
  tc.writeU16(0x0504, 0x00B0); // DCSCMD_Q
  tc.delay_ms(32);
  // Remove NVM
  tc.writeU16(0x0504, 0x0023); // DCSCMD_Q
  tc.writeU16(0x0504, 0x01D6); // DCSCMD_Q
  tc.delay_ms(32);
  // Interface Setting
  tc.writeU16(0x0504, 0x8029); // DCSCMD_Q
  tc.writeU16(0x0504, 0x0004); // DCSCMD_Q
  tc.writeU16(0x0504, 0x10B3); // DCSCMD_Q
  tc.writeU16(0x0504, 0x0000); // DCSCMD_Q
  tc.delay_ms(32);
  // MCAP
  tc.writeU16(0x0504, 0x0023); // DCSCMD_Q
  tc.writeU16(0x0504, 0x03B0); // DCSCMD_Q
  tc.delay_ms(32);

  // Set all Pixels On
  // tc.writeU16(0x0504, 0x0005); // DCSCMD_Q
  // tc.writeU16(0x0504, 0x0023); // DCSCMD_Q
  tc.delay_ms(1);


// Split Control
tc.writeU16(0x5000,0x0000); // STX0_CTL
tc.writeU16(0x500C,0x0000); // STX0_FPX
tc.writeU16(0x500E,0x02CF); // STX0_LPX
tc.writeU16(0x5080,0x0000); // STX1_CTL
tc.writeU16(0x508C,0x02D0); // STX1_FPX
tc.writeU16(0x508E,0x059F); // STX1_LPX


  // HDMI PHY
  tc.writeU8(0x8410, 0x03); // PHY CTL
  tc.writeU8(0x8413, 0x3F); // PHY_ENB
  tc.writeU8(0x84F0, 0x31); // APLL_CTL
  tc.writeU8(0x84F4, 0x01); // DDCIO_CTL
  // HDMI Clock
  tc.writeU16(0x8540, 0x12C0); // SYS_FREQ0_1
  tc.writeU8(0x8630, 0x00);    // LOCKDET_FREQ0
  tc.writeU16(0x8631, 0x0753); // LOCKDET_REF1_2
  tc.writeU8(0x8670, 0x02);    // NCO_F0_MOD
  tc.writeU16(0x8A0C, 0x12C0); // CSC_SCLK0_1
  // HDMI Interrupt Mask, Clear
  tc.writeU8(0x8502, 0xFF); // SYS_INT
  tc.writeU8(0x8512, 0xFE); // SYS_INTM
  // Interrupt Control (TOP level)
  tc.writeU16(0x0014, 0x0FBF); // IntStatus
  tc.writeU16(0x0016, 0x0DBF); // IntMask

// EDID
tc.writeU8(0x85E0,0x01); // EDID_MODE
tc.writeU16(0x85E3,0x0100); // EDID_LEN1_2
// EDID Data
tc.writeU8(0x8C00,0x00); // EDID_RAM
tc.writeU8(0x8C01,0xFF); // EDID_RAM
tc.writeU8(0x8C02,0xFF); // EDID_RAM
tc.writeU8(0x8C03,0xFF); // EDID_RAM
tc.writeU8(0x8C04,0xFF); // EDID_RAM
tc.writeU8(0x8C05,0xFF); // EDID_RAM
tc.writeU8(0x8C06,0xFF); // EDID_RAM
tc.writeU8(0x8C07,0x00); // EDID_RAM
tc.writeU8(0x8C08,0x52); // EDID_RAM
tc.writeU8(0x8C09,0x62); // EDID_RAM
tc.writeU8(0x8C0A,0x88); // EDID_RAM
tc.writeU8(0x8C0B,0x88); // EDID_RAM
tc.writeU8(0x8C0C,0x00); // EDID_RAM
tc.writeU8(0x8C0D,0x88); // EDID_RAM
tc.writeU8(0x8C0E,0x88); // EDID_RAM
tc.writeU8(0x8C0F,0x88); // EDID_RAM
tc.writeU8(0x8C10,0x1C); // EDID_RAM
tc.writeU8(0x8C11,0x15); // EDID_RAM
tc.writeU8(0x8C12,0x01); // EDID_RAM
tc.writeU8(0x8C13,0x03); // EDID_RAM
tc.writeU8(0x8C14,0x80); // EDID_RAM
tc.writeU8(0x8C15,0x00); // EDID_RAM
tc.writeU8(0x8C16,0x00); // EDID_RAM
tc.writeU8(0x8C17,0x78); // EDID_RAM
tc.writeU8(0x8C18,0x0A); // EDID_RAM
tc.writeU8(0x8C19,0x0D); // EDID_RAM
tc.writeU8(0x8C1A,0xC9); // EDID_RAM
tc.writeU8(0x8C1B,0xA0); // EDID_RAM
tc.writeU8(0x8C1C,0x57); // EDID_RAM
tc.writeU8(0x8C1D,0x47); // EDID_RAM
tc.writeU8(0x8C1E,0x98); // EDID_RAM
tc.writeU8(0x8C1F,0x27); // EDID_RAM
tc.writeU8(0x8C20,0x12); // EDID_RAM
tc.writeU8(0x8C21,0x48); // EDID_RAM
tc.writeU8(0x8C22,0x4C); // EDID_RAM
tc.writeU8(0x8C23,0x00); // EDID_RAM
tc.writeU8(0x8C24,0x00); // EDID_RAM
tc.writeU8(0x8C25,0x00); // EDID_RAM
tc.writeU8(0x8C26,0x01); // EDID_RAM
tc.writeU8(0x8C27,0x01); // EDID_RAM
tc.writeU8(0x8C28,0x01); // EDID_RAM
tc.writeU8(0x8C29,0x01); // EDID_RAM
tc.writeU8(0x8C2A,0x01); // EDID_RAM
tc.writeU8(0x8C2B,0x01); // EDID_RAM
tc.writeU8(0x8C2C,0x01); // EDID_RAM
tc.writeU8(0x8C2D,0x01); // EDID_RAM
tc.writeU8(0x8C2E,0x01); // EDID_RAM
tc.writeU8(0x8C2F,0x01); // EDID_RAM
tc.writeU8(0x8C30,0x01); // EDID_RAM
tc.writeU8(0x8C31,0x01); // EDID_RAM
tc.writeU8(0x8C32,0x01); // EDID_RAM
tc.writeU8(0x8C33,0x01); // EDID_RAM
tc.writeU8(0x8C34,0x01); // EDID_RAM
tc.writeU8(0x8C35,0x01); // EDID_RAM
tc.writeU8(0x8C36,0xF0); // EDID_RAM
tc.writeU8(0x8C37,0x55); // EDID_RAM
tc.writeU8(0x8C38,0xA0); // EDID_RAM
tc.writeU8(0x8C39,0xA0); // EDID_RAM
tc.writeU8(0x8C3A,0x50); // EDID_RAM
tc.writeU8(0x8C3B,0x00); // EDID_RAM
tc.writeU8(0x8C3C,0x08); // EDID_RAM
tc.writeU8(0x8C3D,0xA0); // EDID_RAM
tc.writeU8(0x8C3E,0x64); // EDID_RAM
tc.writeU8(0x8C3F,0x1E); // EDID_RAM
tc.writeU8(0x8C40,0x42); // EDID_RAM
tc.writeU8(0x8C41,0x00); // EDID_RAM
tc.writeU8(0x8C42,0xA0); // EDID_RAM
tc.writeU8(0x8C43,0x00); // EDID_RAM
tc.writeU8(0x8C44,0x5A); // EDID_RAM
tc.writeU8(0x8C45,0x00); // EDID_RAM
tc.writeU8(0x8C46,0x00); // EDID_RAM
tc.writeU8(0x8C47,0x18); // EDID_RAM
tc.writeU8(0x8C48,0xF0); // EDID_RAM
tc.writeU8(0x8C49,0x55); // EDID_RAM
tc.writeU8(0x8C4A,0xA0); // EDID_RAM
tc.writeU8(0x8C4B,0xA0); // EDID_RAM
tc.writeU8(0x8C4C,0x50); // EDID_RAM
tc.writeU8(0x8C4D,0x00); // EDID_RAM
tc.writeU8(0x8C4E,0x08); // EDID_RAM
tc.writeU8(0x8C4F,0xA0); // EDID_RAM
tc.writeU8(0x8C50,0x64); // EDID_RAM
tc.writeU8(0x8C51,0x1E); // EDID_RAM
tc.writeU8(0x8C52,0x42); // EDID_RAM
tc.writeU8(0x8C53,0x00); // EDID_RAM
tc.writeU8(0x8C54,0xA0); // EDID_RAM
tc.writeU8(0x8C55,0x00); // EDID_RAM
tc.writeU8(0x8C56,0x5A); // EDID_RAM
tc.writeU8(0x8C57,0x00); // EDID_RAM
tc.writeU8(0x8C58,0x00); // EDID_RAM
tc.writeU8(0x8C59,0x18); // EDID_RAM
tc.writeU8(0x8C5A,0x00); // EDID_RAM
tc.writeU8(0x8C5B,0x00); // EDID_RAM
tc.writeU8(0x8C5C,0x00); // EDID_RAM
tc.writeU8(0x8C5D,0xFC); // EDID_RAM
tc.writeU8(0x8C5E,0x00); // EDID_RAM
tc.writeU8(0x8C5F,0x7A); // EDID_RAM
tc.writeU8(0x8C60,0x61); // EDID_RAM
tc.writeU8(0x8C61,0x6B); // EDID_RAM
tc.writeU8(0x8C62,0x6F); // EDID_RAM
tc.writeU8(0x8C63,0x7A); // EDID_RAM
tc.writeU8(0x8C64,0x61); // EDID_RAM
tc.writeU8(0x8C65,0x6B); // EDID_RAM
tc.writeU8(0x8C66,0x6F); // EDID_RAM
tc.writeU8(0x8C67,0x0A); // EDID_RAM
tc.writeU8(0x8C68,0x20); // EDID_RAM
tc.writeU8(0x8C69,0x20); // EDID_RAM
tc.writeU8(0x8C6A,0x20); // EDID_RAM
tc.writeU8(0x8C6B,0x20); // EDID_RAM
tc.writeU8(0x8C6C,0x00); // EDID_RAM
tc.writeU8(0x8C6D,0x00); // EDID_RAM
tc.writeU8(0x8C6E,0x00); // EDID_RAM
tc.writeU8(0x8C6F,0xFD); // EDID_RAM
tc.writeU8(0x8C70,0x00); // EDID_RAM
tc.writeU8(0x8C71,0x17); // EDID_RAM
tc.writeU8(0x8C72,0x3D); // EDID_RAM
tc.writeU8(0x8C73,0x0F); // EDID_RAM
tc.writeU8(0x8C74,0x8C); // EDID_RAM
tc.writeU8(0x8C75,0x17); // EDID_RAM
tc.writeU8(0x8C76,0x00); // EDID_RAM
tc.writeU8(0x8C77,0x0A); // EDID_RAM
tc.writeU8(0x8C78,0x20); // EDID_RAM
tc.writeU8(0x8C79,0x20); // EDID_RAM
tc.writeU8(0x8C7A,0x20); // EDID_RAM
tc.writeU8(0x8C7B,0x20); // EDID_RAM
tc.writeU8(0x8C7C,0x20); // EDID_RAM
tc.writeU8(0x8C7D,0x20); // EDID_RAM
tc.writeU8(0x8C7E,0x01); // EDID_RAM
tc.writeU8(0x8C7F,0x86); // EDID_RAM
tc.writeU8(0x8C80,0x02); // EDID_RAM
tc.writeU8(0x8C81,0x03); // EDID_RAM
tc.writeU8(0x8C82,0x17); // EDID_RAM
tc.writeU8(0x8C83,0x74); // EDID_RAM
tc.writeU8(0x8C84,0x47); // EDID_RAM
tc.writeU8(0x8C85,0x04); // EDID_RAM
tc.writeU8(0x8C86,0x13); // EDID_RAM
tc.writeU8(0x8C87,0x03); // EDID_RAM
tc.writeU8(0x8C88,0x02); // EDID_RAM
tc.writeU8(0x8C89,0x07); // EDID_RAM
tc.writeU8(0x8C8A,0x06); // EDID_RAM
tc.writeU8(0x8C8B,0x01); // EDID_RAM
tc.writeU8(0x8C8C,0x23); // EDID_RAM
tc.writeU8(0x8C8D,0x09); // EDID_RAM
tc.writeU8(0x8C8E,0x07); // EDID_RAM
tc.writeU8(0x8C8F,0x01); // EDID_RAM
tc.writeU8(0x8C90,0x66); // EDID_RAM
tc.writeU8(0x8C91,0x03); // EDID_RAM
tc.writeU8(0x8C92,0x0C); // EDID_RAM
tc.writeU8(0x8C93,0x00); // EDID_RAM
tc.writeU8(0x8C94,0x30); // EDID_RAM
tc.writeU8(0x8C95,0x00); // EDID_RAM
tc.writeU8(0x8C96,0x80); // EDID_RAM
tc.writeU8(0x8C97,0x8C); // EDID_RAM
tc.writeU8(0x8C98,0x0A); // EDID_RAM
tc.writeU8(0x8C99,0xD0); // EDID_RAM
tc.writeU8(0x8C9A,0xF0); // EDID_RAM
tc.writeU8(0x8C9B,0x55); // EDID_RAM
tc.writeU8(0x8C9C,0xA0); // EDID_RAM
tc.writeU8(0x8C9D,0xA0); // EDID_RAM
tc.writeU8(0x8C9E,0x50); // EDID_RAM
tc.writeU8(0x8C9F,0x00); // EDID_RAM
tc.writeU8(0x8CA0,0x08); // EDID_RAM
tc.writeU8(0x8CA1,0xA0); // EDID_RAM
tc.writeU8(0x8CA2,0x64); // EDID_RAM
tc.writeU8(0x8CA3,0x1E); // EDID_RAM
tc.writeU8(0x8CA4,0x42); // EDID_RAM
tc.writeU8(0x8CA5,0x00); // EDID_RAM
tc.writeU8(0x8CA6,0xA0); // EDID_RAM
tc.writeU8(0x8CA7,0x00); // EDID_RAM
tc.writeU8(0x8CA8,0x5A); // EDID_RAM
tc.writeU8(0x8CA9,0x00); // EDID_RAM
tc.writeU8(0x8CAA,0x00); // EDID_RAM
tc.writeU8(0x8CAB,0x18); // EDID_RAM
tc.writeU8(0x8CAC,0xF0); // EDID_RAM
tc.writeU8(0x8CAD,0x55); // EDID_RAM
tc.writeU8(0x8CAE,0xA0); // EDID_RAM
tc.writeU8(0x8CAF,0xA0); // EDID_RAM
tc.writeU8(0x8CB0,0x50); // EDID_RAM
tc.writeU8(0x8CB1,0x00); // EDID_RAM
tc.writeU8(0x8CB2,0x08); // EDID_RAM
tc.writeU8(0x8CB3,0xA0); // EDID_RAM
tc.writeU8(0x8CB4,0x64); // EDID_RAM
tc.writeU8(0x8CB5,0x1E); // EDID_RAM
tc.writeU8(0x8CB6,0x42); // EDID_RAM
tc.writeU8(0x8CB7,0x00); // EDID_RAM
tc.writeU8(0x8CB8,0xA0); // EDID_RAM
tc.writeU8(0x8CB9,0x00); // EDID_RAM
tc.writeU8(0x8CBA,0x5A); // EDID_RAM
tc.writeU8(0x8CBB,0x00); // EDID_RAM
tc.writeU8(0x8CBC,0x00); // EDID_RAM
tc.writeU8(0x8CBD,0x18); // EDID_RAM
tc.writeU8(0x8CBE,0xF0); // EDID_RAM
tc.writeU8(0x8CBF,0x55); // EDID_RAM
tc.writeU8(0x8CC0,0xA0); // EDID_RAM
tc.writeU8(0x8CC1,0xA0); // EDID_RAM
tc.writeU8(0x8CC2,0x50); // EDID_RAM
tc.writeU8(0x8CC3,0x00); // EDID_RAM
tc.writeU8(0x8CC4,0x08); // EDID_RAM
tc.writeU8(0x8CC5,0xA0); // EDID_RAM
tc.writeU8(0x8CC6,0x64); // EDID_RAM
tc.writeU8(0x8CC7,0x1E); // EDID_RAM
tc.writeU8(0x8CC8,0x42); // EDID_RAM
tc.writeU8(0x8CC9,0x00); // EDID_RAM
tc.writeU8(0x8CCA,0xA0); // EDID_RAM
tc.writeU8(0x8CCB,0x00); // EDID_RAM
tc.writeU8(0x8CCC,0x5A); // EDID_RAM
tc.writeU8(0x8CCD,0x00); // EDID_RAM
tc.writeU8(0x8CCE,0x00); // EDID_RAM
tc.writeU8(0x8CCF,0x18); // EDID_RAM
tc.writeU8(0x8CD0,0xF0); // EDID_RAM
tc.writeU8(0x8CD1,0x55); // EDID_RAM
tc.writeU8(0x8CD2,0xA0); // EDID_RAM
tc.writeU8(0x8CD3,0xA0); // EDID_RAM
tc.writeU8(0x8CD4,0x50); // EDID_RAM
tc.writeU8(0x8CD5,0x00); // EDID_RAM
tc.writeU8(0x8CD6,0x08); // EDID_RAM
tc.writeU8(0x8CD7,0xA0); // EDID_RAM
tc.writeU8(0x8CD8,0x64); // EDID_RAM
tc.writeU8(0x8CD9,0x1E); // EDID_RAM
tc.writeU8(0x8CDA,0x42); // EDID_RAM
tc.writeU8(0x8CDB,0x00); // EDID_RAM
tc.writeU8(0x8CDC,0xA0); // EDID_RAM
tc.writeU8(0x8CDD,0x00); // EDID_RAM
tc.writeU8(0x8CDE,0x5A); // EDID_RAM
tc.writeU8(0x8CDF,0x00); // EDID_RAM
tc.writeU8(0x8CE0,0x00); // EDID_RAM
tc.writeU8(0x8CE1,0x18); // EDID_RAM
tc.writeU8(0x8CE2,0x00); // EDID_RAM
tc.writeU8(0x8CE3,0x00); // EDID_RAM
tc.writeU8(0x8CE4,0x00); // EDID_RAM
tc.writeU8(0x8CE5,0x00); // EDID_RAM
tc.writeU8(0x8CE6,0x00); // EDID_RAM
tc.writeU8(0x8CE7,0x00); // EDID_RAM
tc.writeU8(0x8CE8,0x00); // EDID_RAM
tc.writeU8(0x8CE9,0x00); // EDID_RAM
tc.writeU8(0x8CEA,0x00); // EDID_RAM
tc.writeU8(0x8CEB,0x00); // EDID_RAM
tc.writeU8(0x8CEC,0x00); // EDID_RAM
tc.writeU8(0x8CED,0x00); // EDID_RAM
tc.writeU8(0x8CEE,0x00); // EDID_RAM
tc.writeU8(0x8CEF,0x00); // EDID_RAM
tc.writeU8(0x8CF0,0x00); // EDID_RAM
tc.writeU8(0x8CF1,0x00); // EDID_RAM
tc.writeU8(0x8CF2,0x00); // EDID_RAM
tc.writeU8(0x8CF3,0x00); // EDID_RAM
tc.writeU8(0x8CF4,0x00); // EDID_RAM
tc.writeU8(0x8CF5,0x00); // EDID_RAM
tc.writeU8(0x8CF6,0x00); // EDID_RAM
tc.writeU8(0x8CF7,0x00); // EDID_RAM
tc.writeU8(0x8CF8,0x00); // EDID_RAM
tc.writeU8(0x8CF9,0x00); // EDID_RAM
tc.writeU8(0x8CFA,0x00); // EDID_RAM
tc.writeU8(0x8CFB,0x00); // EDID_RAM
tc.writeU8(0x8CFC,0x00); // EDID_RAM
tc.writeU8(0x8CFD,0x00); // EDID_RAM
tc.writeU8(0x8CFE,0x00); // EDID_RAM
tc.writeU8(0x8CFF,0xF4); // EDID_RAM



// HDCP Setting
i2c1_uh2cd_write8(0x85EC,0x01); // 
i2c1_uh2cd_write8(0x8560,0x24); // HDCP_MODE
i2c1_uh2cd_write8(0x8563,0x11); // 
i2c1_uh2cd_write8(0x8564,0x0F); // 
// HDMI SYSTEM
i2c1_uh2cd_write8(0x8543,0x02); // DDC_CTL
i2c1_uh2cd_write8(0x8544,0x10); // HPD_CTL
// HDMI Audio Setting
i2c1_uh2cd_write8(0x8600,0x00); // AUD_Auto_Mute
i2c1_uh2cd_write8(0x8602,0xF3); // Auto_CMD0
i2c1_uh2cd_write8(0x8603,0x02); // Auto_CMD1
i2c1_uh2cd_write8(0x8604,0x0C); // Auto_CMD2
i2c1_uh2cd_write8(0x8606,0x05); // BUFINIT_START
i2c1_uh2cd_write8(0x8607,0x00); // FS_MUTE
i2c1_uh2cd_write8(0x8652,0x61); // SDO_MODE1
i2c1_uh2cd_write32(0x8671,0x020C49BA); // NCO_48F0A_D
i2c1_uh2cd_write32(0x8675,0x01E1B089); // NCO_44F0A_D
i2c1_uh2cd_write8(0x8680,0x00); // AUD_MODE

  // Let HDMI Source start access
  tc.writeU8(0x854A, 0x01); // INIT_END
  // Start Video TX
  tc.writeU16(0x0004, 0x0C17); // ConfCtl0
  tc.writeU16(0x0006, 0x0000); // ConfCtl1
  // Command Transmission After Video Start.
  tc.writeU32(0x0110, 0x00000006); // MODE_CONFIG
  tc.writeU32(0x0310, 0x00000006); // MODE_CONFIG
  tc.delay_ms(5000);

  /* SleepOut */
  tc.writeU16(0x0504, 0x0005); // DCSCMD_Q
  tc.writeU16(0x0504, 0x0011); // DCSCMD_Q
  tc.delay_ms(150);
  // Set Display On
  tc.writeU16(0x0504, 0x0005); // DCSCMD_Q
  tc.writeU16(0x0504, 0x0029); // DCSCMD_Q
  tc.delay_ms(1);
}
