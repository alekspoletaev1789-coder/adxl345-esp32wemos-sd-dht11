// ============================================================
// ESP32_ADXL345_I2C_STREAM_SD.ino
// Board: ESP32 (WROOM/WROVER) 30-pin, CH340 Type-C
//
// Features:
// - ADXL345 via I2C (SDA=21, SCL=22) direct register read (fast)
// - Binary frames over Serial at 921600 baud
// - Commands over Serial (ASCII lines):
//     I   -> print INFO (#ADDR, #DEVID, #ADXL_OK, #SD_OK)
//     S   -> start continuous stream
//     X   -> stop stream
//     1   -> send ONE frame
//     R10/R30/R60/R300 -> stream+record for N seconds then stop
//     SDON / SDOFF -> enable/disable SD logging (streaming can continue)
// - Button toggles REC 30s (press -> start timed record, press again -> stop)
// - Optional DHT11 is NOT used here to keep timing stable at high fs
//
// Frame format (little-endian):
//   HDR = <IIHHhh>  (magic,u32 t_us,u16 fs,u16 n,i16 temp10,i16 hum10)
//   payload: n samples of [x,y,z] int16 interleaved => 3*n*2 bytes
//
// Notes:
// - For ADXL345 3200 Hz ODR you must set BW_RATE=0x0F.
// - For vibration spectra up to 1600 Hz you need fs>=3200 and real sampling
//   must be stable. This code samples by micros() and reads 6 bytes per sample.
// - SD logging at 3200Hz*3axes can be heavy; we write frames (512 samples).
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

// ---------------- Serial ----------------
static const uint32_t BAUD = 921600;

// ---------------- I2C pins for classic ESP32 ----------------
#define I2C_SDA 21
#define I2C_SCL 22

// ---------------- Button ----------------
#define BTN_PIN 4   // button to GND, INPUT_PULLUP

// ---------------- SD SPI pins (your mapping) ----------------
#define SD_CS   18
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK  16

// ---------------- Sampling config ----------------
static const uint16_t FS_HZ     = 3200;  // target sampling
static const uint16_t N_SAMPLES = 512;   // samples per frame
static const uint32_t PERIOD_US = 1000000UL / FS_HZ;

// ---------------- ADXL345 ----------------
// I2C address depends on SDO:
//  SDO=GND => 0x53
//  SDO=3.3V => 0x1D
static const uint8_t  ADXL_ADDR = 0x53;
static const uint32_t MAGIC     = 0xA55A5AA5;

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t t_us;
  uint16_t fs;
  uint16_t n;
  int16_t  temp10;   // -32768 = none
  int16_t  hum10;    // -32768 = none
} frame_hdr_t;

// frame buffers
static int16_t bx[N_SAMPLES];
static int16_t by[N_SAMPLES];
static int16_t bz[N_SAMPLES];

static volatile bool adxl_ok   = false;
static volatile bool sd_ok     = false;

static volatile bool streaming = false;    // stream enabled
static volatile uint32_t frames_left = 0;  // timed record frames left (0=infinite)

static uint16_t idx = 0;
static uint32_t nextUs = 0;

// SD logging control
static volatile bool sd_log_enabled = true;
static File logFile;
static String logName = "";

// button debounce
static uint32_t lastBtnMs = 0;
static bool lastBtnState = true;

// command line
static String cmdLine;

// ---------------- I2C helpers ----------------
static bool i2cWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cReadMulti(uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom((int)ADXL_ADDR, (int)n, (int)true);
  if (got != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool adxlInit() {
  uint8_t devid = 0;
  if (!i2cReadMulti(0x00, &devid, 1)) return false;
  if (devid != 0xE5) return false;

  // BW_RATE (0x2C): 0x0F = 3200 Hz, 0x0E = 1600 Hz
  if (!i2cWrite8(0x2C, 0x0F)) return false;

  // DATA_FORMAT (0x31): FULL_RES=1 + range=±16g => 0x0B
  if (!i2cWrite8(0x31, 0x0B)) return false;

  // POWER_CTL (0x2D): Measure=1 => 0x08
  if (!i2cWrite8(0x2D, 0x08)) return false;

  return true;
}

static inline void readAdxlXYZ(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t raw[6];
  if (!i2cReadMulti(0x32, raw, 6)) { x = y = z = 0; return; }
  x = (int16_t)((raw[1] << 8) | raw[0]);
  y = (int16_t)((raw[3] << 8) | raw[2]);
  z = (int16_t)((raw[5] << 8) | raw[4]);
}

// ---------------- SD helpers ----------------
static void sdCloseFile() {
  if (logFile) {
    logFile.flush();
    logFile.close();
  }
  logName = "";
}

static String makeLogName(uint16_t sec) {
  // simple name with millis; you can improve to RTC later
  char buf[64];
  snprintf(buf, sizeof(buf), "/vib_%lu_%uHz_%us.bin", (unsigned long)millis(), FS_HZ, sec);
  return String(buf);
}

static bool sdOpenFileForRecord(uint16_t sec) {
  if (!sd_ok) return false;
  sdCloseFile();
  logName = makeLogName(sec);
  logFile = SD.open(logName.c_str(), FILE_WRITE);
  return (bool)logFile;
}

// ---------------- stream control ----------------
static void start_stream_inf() {
  streaming = true;
  frames_left = 0;
}

static void start_record_seconds(uint16_t sec) {
  streaming = true;

  // open SD file for this record if enabled
  if (sd_ok && sd_log_enabled) {
    sdOpenFileForRecord(sec);
  } else {
    sdCloseFile();
  }

  float frame_sec = (float)N_SAMPLES / (float)FS_HZ;
  frames_left = (uint32_t)ceilf((float)sec / frame_sec);
  if (frames_left == 0) frames_left = 1;
}

static void stop_stream() {
  streaming = false;
  frames_left = 0;
  sdCloseFile();
}

// ---------------- send frame ----------------
static void send_frame(uint32_t now_us) {
  frame_hdr_t hdr;
  hdr.magic  = MAGIC;
  hdr.t_us   = now_us;
  hdr.fs     = FS_HZ;
  hdr.n      = N_SAMPLES;
  hdr.temp10 = (int16_t)-32768;
  hdr.hum10  = (int16_t)-32768;

  // write to Serial
  Serial.write((const uint8_t*)&hdr, sizeof(hdr));
  for (uint16_t i = 0; i < N_SAMPLES; i++) {
    Serial.write((const uint8_t*)&bx[i], 2);
    Serial.write((const uint8_t*)&by[i], 2);
    Serial.write((const uint8_t*)&bz[i], 2);
  }

  // write to SD (same binary) if file opened
  if (logFile && sd_log_enabled) {
    logFile.write((const uint8_t*)&hdr, sizeof(hdr));
    for (uint16_t i = 0; i < N_SAMPLES; i++) {
      logFile.write((const uint8_t*)&bx[i], 2);
      logFile.write((const uint8_t*)&by[i], 2);
      logFile.write((const uint8_t*)&bz[i], 2);
    }
    // flush occasionally to reduce wear but keep data safer
    static uint32_t lastFlushMs = 0;
    uint32_t ms = millis();
    if (ms - lastFlushMs > 500) {
      logFile.flush();
      lastFlushMs = ms;
    }
  }
}

// ---------------- command parser ----------------
static void printInfo() {
  Serial.println("#INFO");
  Serial.print("#ADDR=0x"); Serial.println(ADXL_ADDR, HEX);

  uint8_t devid = 0;
  bool ok = i2cReadMulti(0x00, &devid, 1);
  Serial.print("#DEVID=0x"); Serial.println(devid, HEX);
  Serial.print("#ADXL_OK="); Serial.println((ok && devid == 0xE5) ? 1 : 0);

  Serial.print("#SD_OK="); Serial.println(sd_ok ? 1 : 0);
  Serial.print("#SD_LOG="); Serial.println(sd_log_enabled ? 1 : 0);
  if (logFile) {
    Serial.print("#SD_FILE="); Serial.println(logName);
  }
  Serial.println();
}

static void handle_serial_cmds() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdLine.trim();
      if (cmdLine.length() > 0) {
        if (cmdLine == "S") start_stream_inf();
        else if (cmdLine == "X") stop_stream();
        else if (cmdLine == "1") {
          streaming = true;
          frames_left = 1;
          // no SD file for one-shot by default
          sdCloseFile();
        }
        else if (cmdLine == "I") {
          printInfo();
        }
        else if (cmdLine == "SDON") {
          sd_log_enabled = true;
          Serial.println("#SD_LOG=1");
        }
        else if (cmdLine == "SDOFF") {
          sd_log_enabled = false;
          sdCloseFile();
          Serial.println("#SD_LOG=0");
        }
        else if (cmdLine.length() >= 2 && cmdLine[0] == 'R') {
          int sec = cmdLine.substring(1).toInt();
          if (sec <= 0) sec = 10;
          start_record_seconds((uint16_t)sec);
        }
      }
      cmdLine = "";
    } else {
      if (cmdLine.length() < 32) cmdLine += c;
    }
  }
}

// ---------------- physical button ----------------
static void handle_button() {
  bool s = digitalRead(BTN_PIN); // true=not pressed, false=pressed
  uint32_t ms = millis();
  if (ms - lastBtnMs < 30) return;
  if (s != lastBtnState) {
    lastBtnMs = ms;
    lastBtnState = s;
    if (s == false) { // pressed
      if (!streaming) start_record_seconds(30);
      else stop_stream();
    }
  }
}

// ---------------- SD init ----------------
static void initSD() {
  // Use explicit SPI pins
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  // start with moderate SPI freq (4 MHz)
  sd_ok = SD.begin(SD_CS, SPI, 4000000);
  if (!sd_ok) {
    // fallback to slower
    sd_ok = SD.begin(SD_CS, SPI, 2000000);
  }
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
  pinMode(BTN_PIN, INPUT_PULLUP);

  Serial.begin(BAUD);
  delay(200);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // ADXL
  adxl_ok = adxlInit();

  // SD
  initSD();

  // initial info
  printInfo();

  nextUs = micros();
}

void loop() {
  handle_serial_cmds();
  handle_button();

  if (!adxl_ok) { delay(50); return; }
  if (!streaming) { delay(2); return; }

  uint32_t now = micros();

  // fixed-rate sampling
  if ((int32_t)(now - nextUs) >= 0) {
    nextUs += PERIOD_US;

    int16_t x, y, z;
    readAdxlXYZ(x, y, z);

    bx[idx] = x;
    by[idx] = y;
    bz[idx] = z;
    idx++;

    if (idx >= N_SAMPLES) {
      idx = 0;
      send_frame(now);

      if (frames_left > 0) {
        frames_left--;
        if (frames_left == 0) stop_stream();
      }
    }
  }
}
