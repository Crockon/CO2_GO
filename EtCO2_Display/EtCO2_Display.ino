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

TODO:
- 'Edit' functionality
- FICO2 calculation and baseline set
- Integrate flow and Temp/RH sensor (once hardware integration working)
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
//NDIR CO2 sensor setup
#define SENSOR_RX_PIN 16   // ESP32 RX <- sensor TX
#define SENSOR_TX_PIN  17  // ESP32 TX -> sensor RX

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
bool mode = 0;

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

void drawGUI(int drawBoxes, int redrawMute, bool editing, bool editmode) {
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
  
}

void processLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  Serial.print("RAW: ");
  Serial.println(line);

  if (line.startsWith(".")) {
    int spaceIdx = line.indexOf(' ');
    if (spaceIdx > 0) {
      scaleFactor = line.substring(spaceIdx + 1).toInt();
      if (scaleFactor <= 0) scaleFactor = 1;
      Serial.print("Scale factor = ");
      Serial.println(scaleFactor);
    }
    return;
  }

  int zFilteredIdx = line.indexOf("Z ");
  if (zFilteredIdx != -1) {
    filteredCO2 = line.substring(zFilteredIdx + 2).toInt() * scaleFactor; // ← write global
    Serial.print("Filtered CO2 = ");
    Serial.print(filteredCO2);
    Serial.println(" ppm");
  }

  int zRawIdx = line.indexOf("z ");
  if (zRawIdx != -1) {
    rawCO2 = line.substring(zRawIdx + 2).toInt() * scaleFactor; // ← write global
    Serial.print("Unfiltered CO2 = ");
    Serial.print(rawCO2);
    Serial.println(" ppm");
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
  
}

void text_test(int co2Val)
{

    my_lcd.setTextColor(GREEN);
    my_lcd.setTextSize(2);
    char text[30];
    sprintf(text, "CO2: %d ppm", co2Val);
    my_lcd.drawString(text, 10, 56, 4);

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
  }

  if(apneaYes) {

    if (millis() - alarmFlashT >= 200) { //flash "Apnea Detected" on screen
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
        
        
      } else {
        digitalWrite(ALARM_PIN, LOW);
        my_lcd.fillRect(90, 85, 300, 150, BLACK);
        drawGUI(1, 1, edit, mode);
      }
    }
  } else 
    {
      if (millis() - alarmFlashT >= 500) { //flash "Apnea Detected" on screen
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
        
        
      } else {
        digitalWrite(ALARM_PIN, LOW);
        my_lcd.fillRect(90, 85, 300, 150, BLACK);
        drawGUI(1, 1, edit, mode);
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
      drawGUI(1, 1, edit, mode);                                // redraw boxes/labels over it
    }
    apneaTracking = false;
    apneaStartT = 0;
    alarmState = ALARM_OFF;
    alarmVisible  = false;
    alarmAcked = false;
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
  Serial.println(co2uncomp);
  Serial.println(co2comp);

  return co2comp;
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
    my_lcd.fillRect(16, 165, 200, 20, BLACK);
    my_lcd.setTextColor(WHITE);
    char text[30];
    my_lcd.setTextSize(2);
    sprintf(text, "CO2: %.0f mmHg", co2mmhg);
    my_lcd.drawString(text, 16, 165);
  }

  check_Et(co2mmhg);

  if(co2mmhg <  FiCO2) {     //NEED TO IMPLEMENT FICO2 CALCULATION INSTEAD OF 5
    waveVals.clear();

  }

    alarmCount(co2mmhg);
  
}



void setup()
{
  Serial.begin(115200);
  // Initialize sensor serial — adjust baud rate to match your sensor's spec
  SensorSerial.begin(9600, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);

  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, LOW);

  my_lcd.init();
  my_lcd.fillScreen(BLACK);
  uint16_t rotation,n;
  my_lcd.setRotation(3);
  touch_init(my_lcd.width(), my_lcd.height(),my_lcd.getRotation());

  //initially draw GUI
  drawGUI(1, 0, edit, mode);
  
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
  

  //initialize battery display
  uint16_t version = readRegister(REG_VERSION);
  if (version == 0xFFFF) {
    Serial.println("ERROR: MAX17049 not found on I2C bus!");
    Serial.println("Check wiring and pull-up resistors on SDA/SCL.");
    while (1) delay(1000); // halt
  }
  Serial.print("MAX17049 detected. IC version: 0x");
  Serial.println(version, HEX);
  Serial.println();

  //check connection with pressure sensor
  // if (! mpr.begin()) {
  //   Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
  //   while (1) {
  //     delay(10);
  //   }
  // }
}

void loop() 
{
  readSensor();
  //for battery monitor

  //fail safe for if connection is bad
  if (soc < 0 || voltage < 0) {
    Serial.println("Read error — check I2C connection.");
  } 

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
    }
  }
  drawGUI(0, 0, edit, mode);
  apneaAlarm();
  delay(50);
  //my_lcd.fillScreen(BLACK);

}
