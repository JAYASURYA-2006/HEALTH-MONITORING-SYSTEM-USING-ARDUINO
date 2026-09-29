#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---------------- MAX30102 register map ----------------
#define MAX_ADDR       0x57
#define REG_FIFO_WR    0x04
#define REG_FIFO_OVF   0x05
#define REG_FIFO_RD    0x06
#define REG_FIFO_DATA  0x07
#define REG_FIFO_CFG   0x08
#define REG_MODE       0x09
#define REG_SPO2_CFG   0x0A
#define REG_LED1_PA    0x0C
#define REG_LED2_PA    0x0D
#define REG_PART_ID    0xFF
#define PART_ID_VALUE  0x15

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------------- Processing parameters ----------------
const uint32_t FINGER_THRESHOLD = 20000UL;  // IR level that means "finger present"
const uint16_t SETTLE_SAMPLES   = 200;      // 2 s at 100 samples/s
const uint16_t WINDOW_SAMPLES   = 200;      // SpO2 window (2 s)
#define BEAT_BUF 4

// ---------------- State ----------------
enum Screen { SCR_NONE, SCR_NO_FINGER, SCR_MEASURING, SCR_READING, SCR_ERROR };
Screen screen = SCR_NONE;

bool sensorOk = false;
bool fingerPresent = false;

uint32_t sampleCount = 0;
bool dcInit = false;
float irDc = 0;
bool armed = false;
bool haveLastBeat = false;
uint32_t lastBeat = 0;
uint16_t beatIntervals[BEAT_BUF];
uint8_t beatIdx = 0, beatCount = 0;

uint32_t redHist[4], irHist[4];
uint8_t histIdx = 0, histFill = 0;

uint32_t redSum, irSum, redMin, redMax, irMin, irMax;
uint16_t winCount = 0;

float spo2Avg = 0, bpmAvg = 0;
bool spo2Valid = false, bpmValid = false;

unsigned long lastReport = 0;

// ---------------- I2C helpers ----------------
bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MAX_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readReg(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(MAX_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MAX_ADDR, (uint8_t)1) != 1) return false;
  val = Wire.read();
  return true;
}

bool beginSensor() {
  uint8_t id = 0;
  if (!readReg(REG_PART_ID, id)) return false;
  if (id != PART_ID_VALUE) return false;

  if (!writeReg(REG_MODE, 0x40)) return false;   // soft reset
  delay(50);
  if (!writeReg(REG_FIFO_CFG, 0x0F)) return false;
  if (!writeReg(REG_SPO2_CFG, 0x27)) return false; // 100 sps, 411 us pulse
  if (!writeReg(REG_LED1_PA, 0x24)) return false;  // RED
  if (!writeReg(REG_LED2_PA, 0x24)) return false;  // IR
  writeReg(REG_FIFO_WR, 0x00);
  writeReg(REG_FIFO_OVF, 0x00);
  writeReg(REG_FIFO_RD, 0x00);
  if (!writeReg(REG_MODE, 0x03)) return false;     // SpO2 mode (RED + IR)
  return true;
}

// ---------------- Processing ----------------
void resetProcessing() {
  sampleCount = 0;
  dcInit = false;
  armed = false;
  haveLastBeat = false;
  beatIdx = 0;
  beatCount = 0;
  histIdx = 0;
  histFill = 0;
  winCount = 0;
  spo2Valid = false;
  bpmValid = false;
}

void processSample(uint32_t red, uint32_t ir) {
  // Finger detection by IR level
  if (ir < FINGER_THRESHOLD) {
    fingerPresent = false;
    resetProcessing();
    return;
  }
  if (!fingerPresent) {
    fingerPresent = true;
    resetProcessing();
  }

  sampleCount++;

  // 4-point moving average
  redHist[histIdx] = red;
  irHist[histIdx] = ir;
  histIdx = (histIdx + 1) & 3;
  if (histFill < 4) histFill++;
  if (histFill < 4) return;
  uint32_t redS = (redHist[0] + redHist[1] + redHist[2] + redHist[3]) >> 2;
  uint32_t irS  = (irHist[0] + irHist[1] + irHist[2] + irHist[3]) >> 2;

  // Slow DC tracker
  if (!dcInit) { irDc = (float)irS; dcInit = true; }
  else irDc += ((float)irS - irDc) / 64.0f;

  if (sampleCount < SETTLE_SAMPLES) return;   // let the filters settle

  // ---- BPM: rising crossing with hysteresis ----
  float hp = (float)irS - irDc;
  if (hp < -200.0f) {
    armed = true;
  } else if (armed && hp > 200.0f) {
    armed = false;
    if (haveLastBeat) {
      uint32_t iv = sampleCount - lastBeat;    // in 10 ms units
      if (iv >= 30 && iv <= 200) {
        beatIntervals[beatIdx] = (uint16_t)iv;
        beatIdx = (beatIdx + 1) % BEAT_BUF;
        if (beatCount < BEAT_BUF) beatCount++;
        if (beatCount >= BEAT_BUF) {
          float sum = 0;
          for (uint8_t i = 0; i < BEAT_BUF; i++) sum += beatIntervals[i];
          float bpmInst = 6000.0f / (sum / BEAT_BUF);
          bpmAvg = bpmValid ? (0.7f * bpmAvg + 0.3f * bpmInst) : bpmInst;
          bpmValid = true;
        }
      }
    }
    lastBeat = sampleCount;
    haveLastBeat = true;
  }

  // ---- SpO2: ratio of ratios over a 2 s window ----
  if (winCount == 0) {
    redSum = 0; irSum = 0;
    redMin = redMax = redS;
    irMin = irMax = irS;
  }
  redSum += redS;
  irSum += irS;
  if (redS < redMin) redMin = redS;
  if (redS > redMax) redMax = redS;
  if (irS < irMin) irMin = irS;
  if (irS > irMax) irMax = irS;
  winCount++;

  if (winCount >= WINDOW_SAMPLES) {
    float redMean = (float)redSum / winCount;
    float irMean  = (float)irSum / winCount;
    float redPP = (float)(redMax - redMin);
    float irPP  = (float)(irMax - irMin);
    if (irPP > 0 && redMean > 0 && irMean > 0) {
      float R = (redPP / redMean) / (irPP / irMean);
      float s = 110.0f - 25.0f * R;
      if (s > 100.0f) s = 100.0f;
      if (s < 70.0f) s = 70.0f;
      spo2Avg = spo2Valid ? (0.6f * spo2Avg + 0.4f * s) : s;
      spo2Valid = true;
    }
    winCount = 0;
  }
}

bool readSamples() {
  uint8_t wr, rd;
  if (!readReg(REG_FIFO_WR, wr)) return false;
  if (!readReg(REG_FIFO_RD, rd)) return false;
  uint8_t avail = (uint8_t)(wr - rd) & 0x1F;
  if (avail > 20) avail = 20;

  while (avail--) {
    Wire.beginTransmission(MAX_ADDR);
    Wire.write(REG_FIFO_DATA);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)MAX_ADDR, (uint8_t)6) != 6) return false;
    uint8_t b[6];
    for (uint8_t i = 0; i < 6; i++) b[i] = Wire.read();
    uint32_t red = ((uint32_t)(b[0] & 0x03) << 16) | ((uint32_t)b[1] << 8) | b[2];
    uint32_t ir  = ((uint32_t)(b[3] & 0x03) << 16) | ((uint32_t)b[4] << 8) | b[5];
    processSample(red, ir);
  }
  return true;
}

// ---------------- LCD helpers ----------------
void printRow(uint8_t row, const char *text) {
  char buf[17];
  uint8_t n = 0;
  while (text[n] && n < 16) { buf[n] = text[n]; n++; }
  while (n < 16) buf[n++] = ' ';
  buf[16] = 0;
  lcd.setCursor(0, row);
  lcd.print(buf);
}

void showScreen(Screen s) {
  if (s == screen) return;
  screen = s;
  switch (s) {
    case SCR_NO_FINGER:
      printRow(0, "Place Finger");
      printRow(1, "on MAX30102");
      Serial.println(F("Place finger on MAX30102..."));
      break;
    case SCR_MEASURING:
      printRow(0, "Measuring...");
      printRow(1, "Hold still");
      Serial.println(F("Finger detected, measuring..."));
      break;
    case SCR_ERROR:
      printRow(0, "Sensor Error");
      printRow(1, "Check wiring");
      Serial.println(F("ERROR: MAX30102 not responding (check power/wiring)"));
      break;
    default:
      break;
  }
}

void showReading() {
  int spo2 = (int)(spo2Avg + 0.5f);
  int bpm  = (int)(bpmAvg + 0.5f);
  char line[17];
  snprintf(line, sizeof(line), "SpO2: %d %%", spo2);
  printRow(0, line);
  snprintf(line, sizeof(line), "BPM : %d", bpm);
  printRow(1, line);
  screen = SCR_READING;

  if (millis() - lastReport >= 1000) {
    lastReport = millis();
    char sbuf[40];
    snprintf(sbuf, sizeof(sbuf), "SpO2: %d %%   BPM: %d", spo2, bpm);
    Serial.println(sbuf);
  }
}

// ---------------- Arduino entry points ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  printRow(0, "Health Monitor");
  printRow(1, "Initializing...");
  Serial.println(F("Health Monitor - SpO2 / BPM"));
  delay(2000);

  sensorOk = beginSensor();
  if (sensorOk) {
    lcd.clear();
    printRow(0, "MAX30102 Ready");
    printRow(1, "Place Finger");
    Serial.println(F("MAX30102 initialised at 0x57"));
    delay(1500);
    screen = SCR_NONE;
    resetProcessing();
  } else {
    lcd.clear();
    screen = SCR_NONE;
    showScreen(SCR_ERROR);
  }
}

void loop() {
  if (!sensorOk) {
    delay(1000);
    if (beginSensor()) {
      sensorOk = true;
      fingerPresent = false;
      resetProcessing();
      lcd.clear();
      screen = SCR_NONE;
      printRow(0, "MAX30102 Ready");
      printRow(1, "Place Finger");
      Serial.println(F("MAX30102 re-initialised"));
      delay(1500);
      screen = SCR_NONE;
    }
    return;
  }

  if (!readSamples()) {          // I2C failure (e.g. power switch turned off)
    sensorOk = false;
    fingerPresent = false;
    resetProcessing();
    lcd.clear();
    screen = SCR_NONE;
    showScreen(SCR_ERROR);
    return;
  }

  if (!fingerPresent)              showScreen(SCR_NO_FINGER);
  else if (!(spo2Valid && bpmValid)) showScreen(SCR_MEASURING);
  else                             showReading();

  delay(20);
}