//Lowkey ignore most-->

        // IMPORTANT: LCDWIKI_SPI LIBRARY MUST BE SPECIFICALLY
        // CONFIGURED FOR EITHER THE TFT SHIELD OR THE BREAKOUT BOARD.

        //This program is a demo of how to display scroll

        //when using the BREAKOUT BOARD only and using these hardware spi lines to the LCD,
        //the SDA pin and SCK pin is defined by the system and can't be modified.
        //if you don't need to control the LED pin,you can set it to 3.3V and set the pin definition to -1.
        //other pins can be defined by youself,for example
        //pin usage as follow:                                   (LED always on rn)
        //                   CS  DC/RS  RESET  SDI/MOSI  SCK  SDO/MISO  LED    VCC     GND    
        //ESP32-WROOM-32E:   25    32      33      23     18      19     -1      5V     GND  
        //                   SCL  SCA    INT      RST
        //TOUCHSCREEN:       26    27      35      14

        //Pressure sensor on Bus 1: 19, 22
        //NDIR on whatever the hell that one is on, i'll figure it out later :)

        //Remember to set the pins to suit your display module!

        /***********************************************************************************
        * @attention
        *
        * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
        * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
        * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
        * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
        * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
        * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
        **********************************************************************************/ 

//DK what that all is lowkey, some stuff left over from the demo 

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//NOW DONT IGNORE THIS MY GOATS -->
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*
__README___________________________________________________________________________

Collects and processes data from (currently) NDIR CO2 sensor and pressure sensor. 
Isolates EtCO2 and generates CO2 waveform. 
Displays waveform and EtCO2 value on TFT LCD display.
Mute button functional as of now, edit button is not yet.

FLOW SENSOR (SFM3400-33-D via Nicolay RS-232 cable + MAX3232):
  - Wired to UART0 (RX0/TX0, GPIO3/GPIO1) on the PCB
  - Serial Monitor IS NOT AVAILABLE because UART0 is used by the sensor
  - All Serial.print() debug calls REMOVED for this reason
  - Flow indicator on GUI:
      GREEN  → flow > 40 mL/min (0.040 SLPM)
      YELLOW → flow is a valid numerical reading but below threshold
      RED    → cable communication error / no valid reading

UPLOAD WORKFLOW (because UART0 is shared with the USB-Serial chip):
  1. Disconnect the flow cable from RX0/TX0
  2. Upload sketch from Arduino IDE
  3. Reconnect the cable
  4. Press RST on the ESP32

TODO:
- 'Edit' functionality
- FICO2 calculation and baseline set
- Integrate Temp/RH sensor (once hardware integration working)
- High CO2 alarm, not sure how that will work
- Add battery life display on GUI
- Check accuracy with pressure comp vs without

*/

#include <Arduino.h>
#include <TFT_eSPI.h> 
#include <SPI.h>
#include <string>
#include <Vector.h>
#include <vector>
#include <Wire.h>

#include "Adafruit_MPRLS.h"
#include "font.h"
#include "touch.h"

//_________________________________
//Ste up all the I2C busses
//TwoWire I2C_1 = TwoWire(0);  //pressure & display
TwoWire I2C_2 = TwoWire(1);  //battery monitor

//__________________________________
//TFT LCD Screen set up
TFT_eSPI my_lcd = TFT_eSPI(); 

//__________________________________
//Set up pressure sensor
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
unsigned long lastPressureRead = 0;
float p = 0; //pressure

const int customSDA = 27; 
const int customSCL = 26; 

//___________________________________
//Settig up RH/Temp sensor
#define SENSOR_ADDR 0x44
float temperature = 25; //placeholder temp [C]
float humidity = 50; //placeholder val



// #define TFT_CS 25
// #define TFT_RST 33
// #define TFT_RS 32
// #define CTP_INT 35
// #define CTP_RST 14
// #define CTP_SDA 27
// #define CTP_SCL 26
// #define SD_CS 5
// #define TFT_SCK 18
// #define TFT_MISO 19
// #define TFT_MOSI 23


#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

//__________________________________
//NDIR CO2 sensor setup (on UART1 / GPIO16-17)
#define SENSOR_RX_PIN 16   // ESP32 RX <- sensor TX
#define SENSOR_TX_PIN  17  // ESP32 TX -> sensor RX

//__________________________________
// FLOW SENSOR setup (Nicolay RS-232 cable on UART0 / RX0=GPIO3, TX0=GPIO1)
// Uses the default `Serial` object since UART0 is the default Serial.
#define FLOW_SLAVE_ADDRESS     0x01
#define FLOW_UART_BAUD         115200
#define FLOW_CMD_GET_FLOW      0x10   // 32-bit signed, units mSLM
#define FLOW_CMD_TEST          0x05   // Returns fixed 0x55 0xAA
#define FLOW_RESPONSE_TIMEOUT_MS  100
#define FLOW_DATA_BYTES        4
// Display threshold — green if above this (in SLPM)
// 40 mL/min = 0.040 SLPM
#define FLOW_THRESHOLD_SLPM    0.040f

// Public state — read elsewhere in the sketch
float g_flowSLPM     = 0.0f;
bool  g_flowValid    = false;
uint8_t g_flowConsecutiveFailures = 0;
#define FLOW_FAILURE_THRESHOLD 10

//__________________________________
// --- Graph configuration ---
#define CO2_MIN       0.0f    // mmhg floor (typical ambient)
#define CO2_MAX       60.0f   // mmhg ceiling
#define GRAPH_X       20      // left edge of graph area
#define GRAPH_Y       195     // top edge (leaves room for label)
#define LABEL_HEIGHT  36     // pixels reserved for the text labe

#define PPM_TO_MMHG  0.000750062f 

//__________________________________
//Set up the battery SOC monitor BOOMBA
// MAX17048/49 fixed I2C address (not configurable)
#define MAX17049_ADDR     0x36

// Register addresses (from datasheet Table 1)
#define REG_VCELL         0x02   // Cell voltage
#define REG_SOC           0x04   // State of Charge
#define REG_MODE          0x06   // Mode (quick start, hibernate)
#define REG_VERSION       0x08   // IC version
#define REG_CONFIG        0x0C   // Configuration
#define REG_CRATE         0x16   // Charge/discharge rate
#define REG_COMMAND       0xFE   // Command register

float soc        = 0;
float soc_past = 0;
float voltage    = 0;
float chargeRate = 0;

//__________________________________
//Set up pressure sensor
// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
//#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
//#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin

//Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
//__________________________________

//__________________________________
// Scrolling graph buffer — one entry per pixel column
int graphWidth  = 0;
int graphHeight = 0;
int* scrollBuf  = nullptr;   // allocated in setup() once screen size is known
int  scrollHead = 0;         // index of the oldest (leftmost) sample

std::vector<float> waveVals;
//float past_co2val = 0;
bool mute = 0;
bool edit = 0;
bool edit_past = 0;
bool mode = 0;
bool status = 0;
bool prev_status = 0;

struct FailResult {
  bool flowFail;
  bool tempFail;
  bool humFail;
  bool battFail;
};

float EtCO2 = 0;
unsigned long counts_sec = 0;
//__________________________________


float pressureMbar = 800.0;   // fallback: standard sea-level pressure, changes when reads pressure sensor


//For pressing Mute and Edit buttons
unsigned long lastPress = 0;
const unsigned long DEBOUNCE_MS = 300;

//__________________________________
//Apnea alarm setup
enum AlarmState { ALARM_OFF, ALARM_APNEA };
AlarmState alarmState = ALARM_OFF;
unsigned long alarmFlashT = 0;
bool alarmVisible = false;
bool alarmAcked = false;   // mute pressed during alarm

unsigned long apneaStartT = 0;   // when CO2 first drops below threshold
bool apneaTracking = false;
bool apneaYes = 0;

#define ALARM_PIN 13

int high = 45;
int low = 30;
int prev_high_y = -1;  // screen Y of last drawn high line
int prev_low_y  = -1;  // screen Y of last drawn low line
//__________________________________

HardwareSerial SensorSerial(1);

String lineBuffer = "";
int scaleFactor = 10;   // default guess until sensor tells us
unsigned long lastQuery = 0;

int filteredCO2 = 0;   // stores the latest filtered CO2 reading in ppm
int rawCO2 = 0;        // stores the latest raw CO2 reading in ppm
float FiCO2 = 5;       //NEED TO UPDATE WITH FICO2 CALCULATION :)

const char *aspect_name[] = {"PORTRAIT", "LANDSCAPE", "PORTRAIT_REV", "LANDSCAPE_REV"};
const char *color_name[] = { "BLUE", "GREEN", "RED", "WHITE" ,"CYAN","MAGENTA","YELLOW"};


// float calcY(float c1) {
//   if (c1 < 1500.0f) {
//     // Low-concentration polynomial (< 1500 ppm)
//     return  2.6661e-16f * powf(c1, 4)
//            -1.1146e-12f * powf(c1, 3)
//            +1.7397e-9f  * powf(c1, 2)
//            -1.2556e-6f  * c1
//            -9.8754e-4f;
//   } else {
//     // High-concentration polynomial (>= 1500 ppm)
//     return  2.811e-38f  * powf(c1, 6)
//            -9.817e-32f  * powf(c1, 5)
//            +1.304e-25f  * powf(c1, 4)
//            -8.126e-20f  * powf(c1, 3)
//            +2.311e-14f  * powf(c1, 2)
//            -2.195e-9f   * c1
//            -1.471e-3f;
//   }
// }

// int compensateCO2(int rawPpm, float pressureMbar) {
//   float c1  = (float)rawPpm;
//   float Y   = calcY(c1);
//   float denom = 1.0f + Y * (1013.0f - pressureMbar);

//   // Guard against divide-by-zero or nonsensical denominator
//   if (fabsf(denom) < 1e-6f) return rawPpm;

//   float corrected = c1 / denom;
//   return (int)roundf(corrected);
// }


//__________________________________
// FLOW SENSOR — CRC-8 lookup table (poly 0x31, init 0x00)
static const uint8_t FLOW_CRC8_TABLE[256] = {
  0x00,0x31,0x62,0x53,0xC4,0xF5,0xA6,0x97,0xB9,0x88,0xDB,0xEA,0x7D,0x4C,0x1F,0x2E,
  0x43,0x72,0x21,0x10,0x87,0xB6,0xE5,0xD4,0xFA,0xCB,0x98,0xA9,0x3E,0x0F,0x5C,0x6D,
  0x86,0xB7,0xE4,0xD5,0x42,0x73,0x20,0x11,0x3F,0x0E,0x5D,0x6C,0xFB,0xCA,0x99,0xA8,
  0xC5,0xF4,0xA7,0x96,0x01,0x30,0x63,0x52,0x7C,0x4D,0x1E,0x2F,0xB8,0x89,0xDA,0xEB,
  0x3D,0x0C,0x5F,0x6E,0xF9,0xC8,0x9B,0xAA,0x84,0xB5,0xE6,0xD7,0x40,0x71,0x22,0x13,
  0x7E,0x4F,0x1C,0x2D,0xBA,0x8B,0xD8,0xE9,0xC7,0xF6,0xA5,0x94,0x03,0x32,0x61,0x50,
  0xBB,0x8A,0xD9,0xE8,0x7F,0x4E,0x1D,0x2C,0x02,0x33,0x60,0x51,0xC6,0xF7,0xA4,0x95,
  0xF8,0xC9,0x9A,0xAB,0x3C,0x0D,0x5E,0x6F,0x41,0x70,0x23,0x12,0x85,0xB4,0xE7,0xD6,
  0x7A,0x4B,0x18,0x29,0xBE,0x8F,0xDC,0xED,0xC3,0xF2,0xA1,0x90,0x07,0x36,0x65,0x54,
  0x39,0x08,0x5B,0x6A,0xFD,0xCC,0x9F,0xAE,0x80,0xB1,0xE2,0xD3,0x44,0x75,0x26,0x17,
  0xFC,0xCD,0x9E,0xAF,0x38,0x09,0x5A,0x6B,0x45,0x74,0x27,0x16,0x81,0xB0,0xE3,0xD2,
  0xBF,0x8E,0xDD,0xEC,0x7B,0x4A,0x19,0x28,0x06,0x37,0x64,0x55,0xC2,0xF3,0xA0,0x91,
  0x47,0x76,0x25,0x14,0x83,0xB2,0xE1,0xD0,0xFE,0xCF,0x9C,0xAD,0x3A,0x0B,0x58,0x69,
  0x04,0x35,0x66,0x57,0xC0,0xF1,0xA2,0x93,0xBD,0x8C,0xDF,0xEE,0x79,0x48,0x1B,0x2A,
  0xC1,0xF0,0xA3,0x92,0x05,0x34,0x67,0x56,0x78,0x49,0x1A,0x2B,0xBC,0x8D,0xDE,0xEF,
  0x82,0xB3,0xE0,0xD1,0x46,0x77,0x24,0x15,0x3B,0x0A,0x59,0x68,0xFF,0xCE,0x9D,0xAC
};

uint8_t flowComputeCRC8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc = FLOW_CRC8_TABLE[crc ^ data[i]];
  }
  return crc;
}

// Send command to flow sensor via UART0 (Serial)
void flowSendCommand(uint8_t funcCode) {
  uint8_t frame[4];
  frame[0] = FLOW_SLAVE_ADDRESS;
  frame[1] = funcCode;
  frame[2] = 0x00;
  frame[3] = flowComputeCRC8(frame, 3);
  while (Serial.available()) Serial.read();
  Serial.write(frame, 4);
}

// Receive a response from flow sensor on UART0 (Serial)
bool flowReceiveResponse(uint8_t funcCode, uint8_t *buf, uint8_t expectedDataBytes) {
  uint8_t totalBytes = 4 + expectedDataBytes;
  uint8_t raw[16];
  uint32_t start = millis();
  uint8_t idx = 0;
  while (idx < totalBytes) {
    if (millis() - start > FLOW_RESPONSE_TIMEOUT_MS) return false;
    if (Serial.available()) {
      uint8_t b = (uint8_t)Serial.read();
      if (idx == 0 && b != FLOW_SLAVE_ADDRESS) continue;
      raw[idx++] = b;
    }
  }
  if (raw[0] != FLOW_SLAVE_ADDRESS) return false;
  if (raw[1] & 0x80)                return false;
  if (raw[1] != funcCode)           return false;
  if (raw[2] != expectedDataBytes)  return false;
  uint8_t receivedCRC = raw[totalBytes - 1];
  uint8_t computedCRC = flowComputeCRC8(raw, totalBytes - 1);
  if (receivedCRC != computedCRC)   return false;
  for (uint8_t i = 0; i < expectedDataBytes; i++) buf[i] = raw[3 + i];
  return true;
}

// Read flow in SLPM. Updates g_flowSLPM and g_flowValid.
void readFlowSensor() {
  flowSendCommand(FLOW_CMD_GET_FLOW);
  uint8_t buf[FLOW_DATA_BYTES];
  if (!flowReceiveResponse(FLOW_CMD_GET_FLOW, buf, FLOW_DATA_BYTES)) {
    g_flowValid = false;
    g_flowSLPM = 0.0f;
    g_flowConsecutiveFailures++;
    return;
  }
  int32_t raw = (int32_t)(
    (uint32_t)buf[0]        |
    (uint32_t)buf[1] << 8   |
    (uint32_t)buf[2] << 16  |
    (uint32_t)buf[3] << 24
  );
  if (raw == 0x7FFFFFFF) {  // Sensor unreadable sentinel
    g_flowValid = false;
    g_flowSLPM = 0.0f;
    g_flowConsecutiveFailures++;
    return;
  }
  g_flowValid = true;
  g_flowSLPM = (float)raw / 1000.0f;
  g_flowConsecutiveFailures = 0;
}

// Spam test commands at startup to wake up the cable
void flowSpamAndSettle(uint32_t spamDurationMs) {
  uint8_t testCmd[4] = {0x01, 0x05, 0x00, 0x31};
  uint32_t spamStart = millis();
  while (millis() - spamStart < spamDurationMs) {
    Serial.write(testCmd, 4);
    delay(1);
  }
  while (Serial.available()) Serial.read();
  delay(500);
  while (Serial.available()) Serial.read();
}

// Recovery if cable becomes unresponsive
void flowRecover() {
  flowSpamAndSettle(2000);
  g_flowConsecutiveFailures = 0;
}

// Decide flow indicator color (3-state)
// Returns: GREEN if flow > threshold, YELLOW if any valid numeric reading,
// RED if reading is invalid / error state
uint16_t getFlowIndicatorColor() {
  if (!g_flowValid) return RED;
  if (g_flowSLPM > FLOW_THRESHOLD_SLPM) return GREEN;
  return YELLOW;
}


//For Battery monitor
// Read a 16-bit register (MSB first, as per datasheet)
uint16_t readRegister(uint8_t reg) {
  I2C_2.beginTransmission(MAX17049_ADDR);
  I2C_2.write(reg);
  if (I2C_2.endTransmission(false) != 0) return 0xFFFF; // NACK — device not found
  I2C_2.requestFrom(MAX17049_ADDR, 2);
  if (I2C_2.available() < 2) return 0xFFFF;
  uint16_t value = (I2C_2.read() << 8) | I2C_2.read();
  return value;
}

//For Battery monitor
// Write a 16-bit register
bool writeRegister(uint8_t reg, uint16_t value) {
  I2C_2.beginTransmission(MAX17049_ADDR);
  I2C_2.write(reg);
  I2C_2.write((value >> 8) & 0xFF); // MSB
  I2C_2.write(value & 0xFF);        // LSB
  return I2C_2.endTransmission() == 0;
}

// VCELL: per datasheet Table 2, multiply the full 16-bit word by 78.125µV.
// No bit shift — the entire 16-bit register value is used directly.
float readVoltage() {
  uint16_t raw = readRegister(REG_VCELL);
  if (raw == 0xFFFF) return -1.0;
  return raw * 0.000078125 * 2; // 78.125µV per LSB, ×2 for MAX17049 2S pack
}

// SOC: MSB = whole percent, LSB = 1/256 percent (fixed-point)
float readSOC() {
  uint16_t raw = readRegister(REG_SOC);
  if (raw == 0xFFFF) return -1.0;
  return (raw >> 8) + ((raw & 0xFF) / 256.0);
}

// CRATE: signed 16-bit, LSB = 0.208%/hr. Negative = discharging, positive = charging.
float readChargeRate() {
  uint16_t raw = readRegister(REG_CRATE);
  if (raw == 0xFFFF) return 0.0;
  int16_t signed_raw = (int16_t)raw;
  return signed_raw * 0.208; // %/hour
}

void drawGUI(int drawBoxes, int redrawMute, bool editing, bool editmode, bool flowFail, bool tempFail, bool humFail, bool battFail) {
  //if (alarmVisible) return;

  // Static variables to track previous state - only redraw on change
  static int prev_mute = -1;  // -1 forces draw on first call
  static int prev_etco2 = -1;

  if (drawBoxes == 1) {
    my_lcd.setTextColor(0xFFFF);
    my_lcd.setTextSize(3);
    my_lcd.setFreeFont();
    my_lcd.drawRect(10, 160, 460, 151, 0xFFFF);
    my_lcd.drawRect(10, 22, 170, 135, 0xFFFF);
    //my_lcd.drawRect(200, 22, 121, 67, 0xFFFF);
    //my_lcd.drawRect(200, 90, 121, 67, 0xFFFF);
    my_lcd.drawRect(339, 22, 131, 66, 0xFFFF);
    //my_lcd.drawString("High", 225, 46);
    //my_lcd.drawString("Low", 235, 46+68);
    my_lcd.drawString("EtCO2:", 17, 25);
  }

  // update when mute button pressed or etco2 updates
  if(status != prev_status || redrawMute) {
    EtCO2 = 0;
    updateEtCO2(EtCO2);
    my_lcd.fillRect(13, 132, 165, 20, BLACK);
    my_lcd.fillRect(270,5,80, 15, BLACK);
    

    if(status) {
      my_lcd.fillRect(339, 22+68, 131, 66, WHITE);
      my_lcd.drawRect(339, 22+68, 131, 66, 0xFFFF);
      my_lcd.setTextColor(BLACK);
      my_lcd.setTextSize(3);
      my_lcd.setFreeFont();
      my_lcd.drawString("Stop", 370, 46+68);
    } else{
      my_lcd.fillRect(339, 22+68, 131, 66, BLACK);
      my_lcd.drawRect(339, 22+68, 131, 66, 0xFFFF);
      my_lcd.setTextColor(0xFFFF);
      my_lcd.setTextSize(3);
      my_lcd.setFreeFont();
      my_lcd.drawString("Start", 363, 46+68);
    }
    prev_status = status;
  }
  if (mute != prev_mute || redrawMute) {

    if (mute == 0) {
      my_lcd.fillRect(339, 22, 131, 66, BLACK);
      my_lcd.drawRect(339, 22, 131, 66, 0xFFFF);
      my_lcd.setTextColor(0xFFFF);
      my_lcd.setTextSize(3);
      my_lcd.setFreeFont();
      my_lcd.drawString("Mute", 369, 46);
    } else {
      my_lcd.fillRect(339, 22, 131, 66, 0xFFFF);
      my_lcd.setTextColor(BLACK);
      my_lcd.setTextSize(3);
      my_lcd.setFreeFont();
      my_lcd.drawString("Unmute", 355, 46);
    }
    prev_mute = mute;
  }
  if(edit == 0) {
    if(edit != edit_past || redrawMute) {
      my_lcd.fillRect(410, 170, 50, 50, BLACK);
      my_lcd.fillRect(410, 230, 50, 50, BLACK);
      my_lcd.fillRect(200, 22, 121, 67, BLACK);
      my_lcd.fillRect(200, 90, 121, 67, BLACK);
      my_lcd.drawRect(200, 22, 121, 67, 0xFFFF);
      my_lcd.drawRect(200, 90, 121, 67, 0xFFFF);
      my_lcd.setTextColor(0xFFFF);
      my_lcd.setTextSize(3);
      my_lcd.setFreeFont();
      my_lcd.drawString("High", 225, 46);
      my_lcd.drawString("Low", 235, 46+68);
    }
  } else {
    my_lcd.fillRect(410, 170, 50, 50, BLACK);
    my_lcd.drawRect(410, 170, 50, 50, WHITE); //plus box
    my_lcd.fillRect(410, 230, 50, 50, BLACK);
    my_lcd.drawRect(410, 230, 50, 50, WHITE); //minus box
    my_lcd.fillRect(420, 251, 30, 8, WHITE); //draw minus sign
    my_lcd.fillRect(420, 191, 30, 8, WHITE);
    my_lcd.fillRect(431, 180, 8, 30, WHITE); //draw plus sign


    my_lcd.setTextColor(BLACK);
    my_lcd.setTextSize(3);
    my_lcd.setFreeFont();
      if(mode) {
        my_lcd.fillRect(200, 22, 121, 67, WHITE);
        my_lcd.drawRect(200, 22, 121, 67, 0xFFFF);
        my_lcd.drawString("High", 225, 46);

        my_lcd.fillRect(200, 90, 121, 67, BLACK);
        my_lcd.drawRect(200, 90, 121, 67, 0xFFFF);
        my_lcd.setTextColor(0xFFFF);
        my_lcd.drawString("Low", 235, 46+68);
      } else {
        my_lcd.fillRect(200, 90, 121, 67, WHITE);
        my_lcd.drawRect(200, 90, 121, 67, 0xFFFF);
        my_lcd.drawString("Low", 235, 46+68);

        my_lcd.fillRect(200, 22, 121, 67, BLACK);
        my_lcd.drawRect(200, 22, 121, 67, 0xFFFF);
        my_lcd.setTextColor(0xFFFF);
        my_lcd.drawString("High", 225, 46);
      }

  }
  edit_past = edit;
  //draw battery SOC

  uint16_t COLOR;
  if(soc > 50) {
    COLOR = GREEN;
  }
  else if (soc > 20 && soc <= 50) {
    COLOR = YELLOW;
  }
  else {
    COLOR = RED;
  }

  if (soc != soc_past) {
    my_lcd.fillRect(410, 2, 50, 18, BLACK);
  }

  
  my_lcd.fillRect(350, 2, 50, 18, BLACK);
  my_lcd.fillRect(410, 2, 50*soc/100, 18, COLOR);
  my_lcd.setTextSize(2);
  char SOCText[30];
  sprintf(SOCText, "%.0f%%", soc);
  if (soc == 100) {
    my_lcd.drawString(SOCText, 360, 4);
  }
  else {
    my_lcd.drawString(SOCText, 370, 4);
  }
  
  my_lcd.drawRect(410, 2, 50, 18, WHITE);
  my_lcd.drawRect(460, 6, 5, 10, WHITE);

  soc_past = soc;

  // ----- FLOW INDICATOR (3-state: GREEN / YELLOW / RED) -----
  // Uses g_flowValid and g_flowSLPM directly instead of the flowFail flag
  uint16_t flowColor = getFlowIndicatorColor();
  my_lcd.setTextColor(WHITE);
  my_lcd.drawCircle(15, 10, 5, flowColor);
  my_lcd.fillCircle(15, 10, 5, flowColor);
  my_lcd.setTextSize(2);
  my_lcd.setFreeFont();
  my_lcd.drawString("Flow", 25, 5);
  my_lcd.setTextColor(WHITE);

  if(tempFail) {
    my_lcd.setTextColor(WHITE);
    my_lcd.drawCircle(85, 10, 5, RED);
    my_lcd.fillCircle(85, 10, 5, RED);
  } else {
    my_lcd.setTextColor(WHITE);
    my_lcd.drawCircle(85, 10, 5, GREEN);
    my_lcd.fillCircle(85, 10, 5, GREEN);
  }
  my_lcd.setTextSize(2);
  my_lcd.setFreeFont();
  my_lcd.drawString("Temp", 95, 5);
  my_lcd.setTextColor(WHITE);

  if(humFail) {
    my_lcd.setTextColor(WHITE);
    my_lcd.drawCircle(155, 10, 5, RED);
    my_lcd.fillCircle(155, 10, 5, RED);
  } else {
    my_lcd.setTextColor(WHITE);
    my_lcd.drawCircle(155, 10, 5, GREEN);
    my_lcd.fillCircle(155, 10, 5, GREEN);
  }
  my_lcd.setTextSize(2);
  my_lcd.setFreeFont();
  my_lcd.drawString("Hum", 165, 5);
  my_lcd.setTextColor(WHITE);

  if(battFail) {
    my_lcd.fillRect(270,5,80, 15, RED);
    my_lcd.setTextColor(YELLOW);
    my_lcd.setTextSize(2);
    my_lcd.setFreeFont();
    my_lcd.drawString("CHARGE", 275, 6);
  } else if(!battFail) {
    my_lcd.fillRect(270,5,80, 15, BLACK);
  }
}

void processLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  // NOTE: Serial.print() debug calls removed — UART0 is the flow sensor now

  if (line.startsWith(".")) {
    int spaceIdx = line.indexOf(' ');
    if (spaceIdx > 0) {
      scaleFactor = line.substring(spaceIdx + 1).toInt();
      if (scaleFactor <= 0) scaleFactor = 1;
    }
    return;
  }

  int zFilteredIdx = line.indexOf("Z ");
  if (zFilteredIdx != -1) {
    filteredCO2 = line.substring(zFilteredIdx + 2).toInt() * scaleFactor; // ← write global
  }

  int zRawIdx = line.indexOf("z ");
  if (zRawIdx != -1) {
    rawCO2 = line.substring(zRawIdx + 2).toInt() * scaleFactor; // ← write global
  }
}

void readSensor() {
  while (SensorSerial.available()) {
    char c = SensorSerial.read();

    if (c == '\n') {
      processLine(lineBuffer);
      lineBuffer = "";
    } else if (c != '\r') {
      lineBuffer += c;
    }
  }

  //read battery monitor
  soc        = readSOC();
  voltage    = readVoltage();
  chargeRate = readChargeRate();

  // ----- Read flow sensor -----
  readFlowSensor();

  // Trigger recovery if too many consecutive flow failures
  if (g_flowConsecutiveFailures >= FLOW_FAILURE_THRESHOLD) {
    flowRecover();
  }
}

void text_test(int co2Val)
{

    my_lcd.setTextColor(GREEN);
    my_lcd.setTextSize(2);
    char text[30];
    sprintf(text, "CO2: %d ppm", co2Val);
    my_lcd.drawString(text, 10, 56, 4);

}

FailResult checkFails() {
  int battMin = 10;
  int tempMax = 50;
  int humMax = 80;
  int flowMin = 1;
  
  FailResult result;
  result.battFail = (soc < battMin);
  result.tempFail = (temperature > tempMax);
  result.humFail  = (humidity > humMax);
  result.flowFail = !g_flowValid;   // used by anything else that checks the flag
  return result;
}


void updateEtCO2(float EtCO2) {
  my_lcd.fillRect(15, 55, 150, 80, BLACK);
  my_lcd.setTextColor(YELLOW);
  char text[30];
  my_lcd.setTextSize(20);
  sprintf(text, "%.0f", EtCO2);
  my_lcd.drawString(text, 25, 75);
  my_lcd.setTextSize(2);
  my_lcd.drawString("mmHg", 120, 115);

}

void check_Et(float co2val) {
  waveVals.push_back(co2val);
  // updateEtCO2(waveVals.size());


  if(waveVals.size() >= 2) {
    if(waveVals[waveVals.size()-1] < waveVals[waveVals.size()-2]) {
      EtCO2 = waveVals[0]; // Initialize with the first element
    
      for (size_t i = 1; i < waveVals.size(); i++) {
        if (waveVals[i] > EtCO2) {
          EtCO2 = waveVals[i]; // Update if a larger value is found
        }
      }
      updateEtCO2(EtCO2);
    }
  }
  // int i = 0;

  // if (co2val > currMax) {
  //   currMax = co2val;
  // }
  // if (co2val-past_co2val < 0) {
  //   //i += 1;
  //   EtCO2 = currMax;
  //   currMax = 0;
  //   updateEtCO2(EtCO2);
  // }
  // else {
  //   i = 0;
  // }
  // past_co2val = co2val;
  // if (i > 3) {
  //   EtCO2 = currMax;
  //   currMax = 0;
  //   updateEtCO2(EtCO2);
  // }
}

void apneaAlarm() {
  if (alarmState != ALARM_APNEA) {
    // if (alarmVisible) {
    //   my_lcd.fillRect(90, 85, 300, 150, BLACK);
    //   drawGUI(1);
    // }
    return;
  } else {
    edit= 0;
  }

  if(apneaYes) {

    if (millis() - alarmFlashT >= 300) { //flash "Apnea Detected" on screen
      alarmFlashT  = millis();
      alarmVisible = !alarmVisible;

      if (alarmVisible) {
        if (!alarmAcked && mute == 0) { //pulse the alarm buzzer as the screen flashes
          digitalWrite(ALARM_PIN, HIGH);
        }
        my_lcd.fillRect(90, 85, 300, 150, RED);
        my_lcd.setTextColor(YELLOW);
        my_lcd.setTextSize(4);
        my_lcd.drawString("APNEA",    185, 125);
        my_lcd.drawString("DETECTED", 150, 175);
        my_lcd.setTextColor(WHITE);

        
      } else {
        digitalWrite(ALARM_PIN, LOW);
        my_lcd.fillRect(90, 85, 300, 150, BLACK);
        FailResult fails = checkFails();
        drawGUI(1, 1, edit, mode, fails.flowFail, fails.tempFail, fails.humFail, fails.battFail);
        updateEtCO2(EtCO2);
      }
    }
  } else 
    {
      if (millis() - alarmFlashT >= 800) { //flash "Apnea Detected" on screen
      alarmFlashT  = millis();
      alarmVisible = !alarmVisible;

      if (alarmVisible) {
        if (!alarmAcked && mute == 0) { //pulse the alarm buzzer as the screen flashes
          digitalWrite(ALARM_PIN, HIGH);
        }
        my_lcd.fillRect(90, 85, 300, 150, RED);
        my_lcd.setTextColor(YELLOW);
        my_lcd.setTextSize(4);
        my_lcd.drawString("LOW", 200, 125);
        my_lcd.drawString("CO2", 200, 175);
        my_lcd.setTextColor(WHITE);
        
        
      } else {
        digitalWrite(ALARM_PIN, LOW);
        my_lcd.fillRect(90, 85, 300, 150, BLACK);
        FailResult fails = checkFails();
        drawGUI(1, 1, edit, mode, fails.flowFail, fails.tempFail, fails.humFail, fails.battFail);
        updateEtCO2(EtCO2);
      }
    
    }
  }
}

void alarmCount(float co2mmhg) {
  if (co2mmhg < 3) {
    if (!apneaTracking) {
      apneaTracking = true;
      apneaStartT = millis();   // start the 15s timer
    }
    // Timer already running — check if 15s elapsed
    if (millis() - apneaStartT >= 15000UL && alarmState == ALARM_OFF) {
      alarmState = ALARM_APNEA;
      alarmVisible = false;
      alarmFlashT = millis();
      alarmAcked = false;
    }
    if(millis() - apneaStartT >= 20000UL) {
      apneaYes = 1;
    }
    else {
      apneaYes = 0;
    }
  } else { //reset if CO2 detected above threshold
    if (alarmState == ALARM_APNEA) {
      digitalWrite(ALARM_PIN, LOW);                        // make sure buzzer is off
      my_lcd.fillRect(90, 85, 300, 150, BLACK);     // erase the red box
      FailResult fails = checkFails();
      drawGUI(1, 1, edit, mode, fails.flowFail, fails.tempFail, fails.humFail, fails.battFail);                                // redraw boxes/labels over it
    }
    apneaTracking = false;
    apneaStartT = 0;
    alarmState = ALARM_OFF;
    alarmVisible  = false;
    alarmAcked = false;
  }

  if(!status) {
    if (alarmState == ALARM_APNEA) {
      digitalWrite(ALARM_PIN, LOW);                        // make sure buzzer is off
      my_lcd.fillRect(90, 85, 300, 150, BLACK);     // erase the red box
      FailResult fails = checkFails();
      drawGUI(1, 1, edit, mode, fails.flowFail, fails.tempFail, fails.humFail, fails.battFail);                                // redraw boxes/labels over it
    }
    apneaTracking = false;
    apneaStartT = 0;
    alarmState = ALARM_OFF;
    alarmVisible  = false;
    alarmAcked = false;
    apneaYes = 0;
  }
}

float pressureComp(float co2uncomp) {
  pressureMbar = mpr.readPressure();
  float Y = 0;
  float co2comp = 0;
  if (co2uncomp < 1500) {
    Y = ((2.6661 * pow(10, -16)) * pow(co2uncomp,4))-((1.1146 * pow(10, -12)) * pow(co2uncomp, 3))-((1.7397 * pow(10, -9)) * pow(co2uncomp, 2))-((1.2556 * pow(10, -6)) * co2uncomp)-(9.8754 * pow(10, -4));
    co2comp = co2uncomp/(1+(Y*(1013-pressureMbar)));
  }
  else {
    Y = ((2.811 * pow(10, -38)) * pow(co2uncomp, 6))-((9.817 * pow(10, -32)) * pow(co2uncomp, 5))-((1.304 * pow(10, -25)) * pow(co2uncomp, 4))-((8.126 * pow(10, -20)) * pow(co2uncomp, 3))-(2.311 * pow(10, -14) * pow(co2uncomp, 2))-(2.195 * pow(10, -9) * co2uncomp)-(1.471 * pow(10, -3));
    co2comp = co2uncomp/(1+(Y*(1013-pressureMbar)));
  }
  return co2comp;
}

void drawLimitLines() {
  
  // Convert mmHg value to graph Y pixel position
  auto mmhgToY = [](float mmhg) -> int {
    float clamped = constrain(mmhg, CO2_MIN, CO2_MAX);
    int barH = map(clamped, CO2_MIN, CO2_MAX, 10, 125);
    return GRAPH_Y + (graphHeight - barH);
  };

  int high_y = mmhgToY((float)high);
  int low_y  = mmhgToY((float)low);

  // Erase old lines only if position changed
  if (prev_high_y != high_y) {
    my_lcd.drawFastHLine(GRAPH_X, prev_high_y, graphWidth, BLACK);
    prev_high_y = high_y;
  }
  if (prev_low_y != low_y) {
    my_lcd.drawFastHLine(GRAPH_X, prev_low_y, graphWidth, BLACK);
    prev_low_y = low_y;
  }

  // Draw dotted lines (every other 4px segment)
  if(!alarmVisible) {
    for (int x = GRAPH_X; x < GRAPH_X + graphWidth; x += 8) {
      if(edit && mode) {
        my_lcd.drawFastHLine(x, high_y, 4, RED);
      } else{
        my_lcd.drawFastHLine(x, high_y, 4, WHITE);
      }
      if(edit && !mode) {
        my_lcd.drawFastHLine(x, low_y,  4, RED);
      } else{
        my_lcd.drawFastHLine(x, low_y,  4, WHITE);
      }
    }
  }
}

// Push a new sample, redraw only the changed columns (fast, no full clear)
void updateGraph(int co2val) {
  // --- 1. Store new sample in circular buffer ---
  //co2val = pressureComp(co2val);
  float co2mmhg = (float)co2val * PPM_TO_MMHG;
  float newBarH = map(constrain(co2mmhg, CO2_MIN, CO2_MAX),
                    CO2_MIN, CO2_MAX, 10, 125);
  scrollBuf[scrollHead] = newBarH;
  scrollHead = (scrollHead + 1) % graphWidth;

  // --- 2. Redraw every column from the buffer ---
  for (int col = 0; col < graphWidth; col++) {
    int bufIdx   = (scrollHead + col) % graphWidth;
    int barH     = scrollBuf[bufIdx];
    int x        = GRAPH_X + col;
    int fillTop  = GRAPH_Y + (graphHeight - barH);

    // Sky (empty) portion — above the fill
    if (fillTop > GRAPH_Y) {
      if (!alarmVisible) {
        my_lcd.drawFastVLine(x, GRAPH_Y, fillTop - GRAPH_Y+15, BLACK);
      }
    }

    if (barH > 0) {
      my_lcd.drawFastVLine(x, fillTop, barH, YELLOW);
    }
  }

  // --- 3. Redraw text label (only this strip is erased, not the graph) ---
  if (!alarmVisible) {
    my_lcd.fillRect(16, 165, 400, 50, BLACK);
    char text[30];
    char hightext[30];
    char lowtext[30];
    my_lcd.setTextSize(2);
    sprintf(text, "CO2: %.0f mmHg", co2mmhg);
    sprintf(hightext, "H: %d mmHg", high);
    sprintf(lowtext, "L: %d mmHg", low);
    //my_lcd.drawString(text, 16, 165);
    if(edit && mode) {
      my_lcd.setTextColor(RED);
    } else{
      my_lcd.setTextColor(WHITE);
    }
    my_lcd.drawString(hightext, 16, 165);

    if(edit && !mode) {
      my_lcd.setTextColor(RED);
    } else {
      my_lcd.setTextColor(WHITE);
    }
    my_lcd.drawString(lowtext, 156, 165);

    if(EtCO2 < low) {
      my_lcd.setTextColor(YELLOW);
      if(status){
        my_lcd.fillRect(13, 132, 165, 20, RED);
        my_lcd.drawString("LOW CO2 LEVEL", 16, 135);
      }
    } else if(EtCO2 > high) {
      my_lcd.setTextColor(YELLOW);
      if(status) {
        my_lcd.fillRect(13, 132, 165, 20, RED);
        my_lcd.drawString("HIGH CO2 LVL", 16, 135);
      }
    } else {
      my_lcd.fillRect(13, 132, 165, 20, BLACK);

    }
    my_lcd.setTextColor(WHITE);


  }

  check_Et(co2mmhg);

  if(co2mmhg <  FiCO2) {     //NEED TO IMPLEMENT FICO2 CALCULATION INSTEAD OF 5
    waveVals.clear();

  }


  alarmCount(co2mmhg);

  
  drawLimitLines();
}

void readTempRHSensor() {
  I2C_2.beginTransmission(SENSOR_ADDR);
  I2C_2.write(0x2C); // MSB Command
  I2C_2.write(0x06); // LSB Command
  if (I2C_2.endTransmission() != 0) {
    // Sensor didn't ACK the measurement command
    return; 
  }

  // 2. Give the sensor a brief moment to actually measure (e.g., 15-20ms)
  delay(20);
  
  I2C_2.requestFrom(SENSOR_ADDR, 6);
  if (I2C_2.available() == 6) {
    // Read temperature data
    uint16_t temp_raw = (I2C_2.read() << 8) | I2C_2.read();
    I2C_2.read();

    // Read humidity data
    uint16_t humi_raw = (I2C_2.read() << 8) | I2C_2.read();
    I2C_2.read();

    temperature = -45.0 + 175.0 * (temp_raw / 65535.0);
    humidity = 100.0 * (humi_raw / 65535.0);
  }
}

void setup()
{
  // UART0 (Serial) is now dedicated to the flow sensor — NOT debug output.
  // NDIR CO2 sensor uses UART1 (SensorSerial).
  // Boot delay lets the ESP32 ROM finish dumping its boot chatter on TX0
  // before we initialize Serial for the flow sensor.
  delay(1000);
  Serial.begin(FLOW_UART_BAUD);

  // Initialize NDIR CO2 sensor on UART1
  SensorSerial.begin(9600, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);

  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, LOW);

  my_lcd.init();
  my_lcd.fillScreen(BLACK);
  uint16_t rotation,n;
  my_lcd.setRotation(3);
  touch_init(my_lcd.width(), my_lcd.height(),my_lcd.getRotation());

  // Wake up the flow cable with spam-and-settle
  // (must happen after Serial.begin and before the first real command)
  delay(1500);  // Let cable boot
  flowSpamAndSettle(3000);

  //initially draw GUI
  FailResult fails = checkFails();
  drawGUI(1, 1, edit, mode, fails.flowFail, fails.tempFail, fails.humFail, fails.battFail);  
  //my_lcd.drawString("CO2", 16, 105);
  // [END lopaka generated]

  // Derive graph dimensions from screen size now that rotation is set
  graphWidth  = 440;
  graphHeight = 100;

  // Allocate and zero the scrolling buffer
  scrollBuf = new int[graphWidth]();

  //I2C_1.begin(customSDA, customSCL);
  Wire.begin(customSDA, customSCL);
  mpr.begin();
  I2C_2.begin(21, 22); // SDA = pin 21, SCL = pin 22
  

  // Battery monitor check — note: Serial.println() was removed because
  // UART0 is now the flow sensor cable, not debug. If MAX17049 missing,
  // we just continue without it rather than halting.
  uint16_t version = readRegister(REG_VERSION);
  (void)version;

  //check connection with pressure sensor
  // if (! mpr.begin()) {
  //   Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
  //   while (1) {
  //     delay(10);
  //   }
  // }
  updateEtCO2(EtCO2);
}

void loop() 
{
  readSensor();
  readTempRHSensor();
  //for battery monitor

  //fail safe for if connection is bad
  // (Serial.println() removed — UART0 is the flow sensor cable now)

  // Only read pressure every 5 seconds
  // if (millis() - lastPressureRead >= 5000UL) {
  //   p = mpr.readPressure();
  //   if (p > 0) pressureMbar = p;   // only update if valid
  //   lastPressureRead = millis();
  // }
  // Serial.print("Pressure (mbar): "); Serial.println(pressure_mbar);
  // Serial.print("Pressure (PSI): "); Serial.println(pressure_mbar / 68.947572932);
  //text_test(filteredCO2);
  //int correctedCO2 = compensateCO2(filteredCO2, pressureMbar);
  updateGraph(filteredCO2);
  if(touch_touched()) {
    if (touch_last_x >= 339 && touch_last_x <= 470 && touch_last_y >= 22 && touch_last_y <= 88) {
      unsigned long now = millis();
      if (now - lastPress >= DEBOUNCE_MS) { //debounce check
        mute = !mute;
        lastPress = now;
      }

      touch_last_x = 0;
      touch_last_y = 0;
    }
    else if (touch_last_x >= 200 && touch_last_x <= 321 && touch_last_y >= 22 && touch_last_y <= 88) {
      unsigned long now = millis();
      if (now - lastPress >= DEBOUNCE_MS) { //debounce check
        if(edit && mode) {
          edit = !edit;
          lastPress = now;
        } else if (edit && !mode) {
          mode = !mode;
          lastPress = now;
        } else if (!edit) {
          edit = !edit;
          mode = 1;
          lastPress = now;
        } 
      }


      touch_last_x = 0;
      touch_last_y = 0;
    }
    else if (touch_last_x >= 200 && touch_last_x <= 321 && touch_last_y >= 90 && touch_last_y <= 157) {
      unsigned long now = millis();
      if (now - lastPress >= DEBOUNCE_MS) { //debounce check
        if(edit && !mode) {
          edit = !edit;
          lastPress = now;
        } else if (edit && mode) {
          mode = !mode;
          lastPress = now;
        } else if (!edit) {
          edit = !edit;
          mode = 0;
          lastPress = now;
        } 
      }


      touch_last_x = 0;
      touch_last_y = 0;
    } else if (touch_last_x >= 339 && touch_last_x <= 470 && touch_last_y >= 90 && touch_last_y <= 157) {
      unsigned long now = millis();
      if (now - lastPress >= DEBOUNCE_MS) { //debounce check
        status = !status;
        lastPress = now;
      }
      touch_last_x = 0;
      touch_last_y = 0;
    }
    if(edit) {
      if(mode) {  //editing higher bound
        if(touch_last_x >= 410 && touch_last_x <= 460 && touch_last_y >= 170 && touch_last_y <= 220) {
          high = high + 1;
          if(high >= 50) {
            high = 50;
          }
          my_lcd.fillRect(410, 170, 50, 50, WHITE);
          delay(100);

        } else if (touch_last_x >= 410 && touch_last_x <= 460 && touch_last_y >= 230 && touch_last_y <= 280) {
          high = high - 1;
          if(high <= low) {
            high = low+1;
          }
          my_lcd.fillRect(410, 230, 50, 50, WHITE);
          delay(100);

        }
      } else if (!mode) { //editing lower bound
        if(touch_last_x >= 410 && touch_last_x <= 460 && touch_last_y >= 170 && touch_last_y <= 220) {
          low = low + 1;
          if(low >= high) {
            low = high-1;
          }
          my_lcd.fillRect(410, 170, 50, 50, WHITE);
          delay(100);

        } else if (touch_last_x >= 410 && touch_last_x <= 460 && touch_last_y >= 230 && touch_last_y <= 280) {
          low = low - 1;
          if(low <= 0) {
            low = 0;
          }
          my_lcd.fillRect(410, 230, 50, 50, WHITE);
          delay(100);

        }
      }
    }
  }

  FailResult fails = checkFails();
  drawGUI(0, 0, edit, mode, fails.flowFail, fails.tempFail, fails.humFail, fails.battFail);
  apneaAlarm();

  delay(50);
  //my_lcd.fillScreen(BLACK);

}
