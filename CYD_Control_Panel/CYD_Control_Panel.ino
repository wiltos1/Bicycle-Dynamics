#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Bitbang.h>
#include <SPIFFS.h>
#include <Ticker.h>
#include <SD.h>

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// Constants & configuration
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
constexpr uint8_t  XPT2046_IRQ     = 36;
constexpr uint8_t  XPT2046_MOSI    = 32;
constexpr uint8_t  XPT2046_MISO    = 39;
constexpr uint8_t  XPT2046_CLK     = 25;
constexpr uint8_t  XPT2046_CS      = 33;
constexpr bool     RERUN_CALIBRATE = false;

#define RXD2 22  // (not used elsewhere, but kept for reference)
#define TXD2 27  // (not used elsewhere, but kept for reference)

// SD card chip‐select
constexpr uint8_t  SD_CS           = 5;
constexpr uint16_t LCD_WIDTH       = 320;
constexpr uint16_t LCD_HEIGHT      = 240;
constexpr uint32_t SAMPLING_FREQUENCY = 10;                // Hz
constexpr uint32_t SAMPLE_INTERVAL_MS = 1000 / SAMPLING_FREQUENCY;

unsigned long lastDataUpdate = 0;  // Timestamp of last update
const unsigned long dataInterval = 1000;  // 1000 ms = 1 second

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// Globals
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// Display + touchscreen + UI
XPT2046_Bitbang ts(XPT2046_MOSI, XPT2046_MISO, XPT2046_CLK, XPT2046_CS);
TFT_eSPI       tft = TFT_eSPI();
TFT_eSPI_Button buttons[6];
Ticker         sampleTicker;
File           logFile;

// UART to Nano (we’ll call this SerialNano).  We’ll map it to pins 22 (RX) and 27 (TX).
HardwareSerial SerialNano(2);

// App state
enum State { Home, Calibrate, Record };
State CYD_State = Home;

// Labels
char homeLabels[2][11] = { "Calibrate", "Record" };
char calLabels [3][12] = { "Cal Pitch", "Cal Roll", "Home" };
char recLabels [3][24] =  { "Start New", "Stop&Save", "Home" };

// Sensor data from Nano (14 values total)
float frontOriX, frontOriY, frontOriZ;
float rearOriX,  rearOriY,  rearOriZ;
float frontAccX, frontAccY, frontAccZ;
float rearAccX,  rearAccY,  rearAccZ;
float rpm_front, rpm_rear;

// Pitch & roll biases (set via calibrateBike)
float pitchBias = 0.0f;
float rollBias  = 0.0f;

// Sampling & logging
volatile bool sample_ready = false;
bool record_data   = false;
unsigned long packet_count = 0;

// Forward declarations
void drawHomeScreen();
void drawCalibrationScreen();
void drawRecordScreen();
void resetButtonStates(int count);
String generateFilename();
void collectData();
void SDCardWrite();
void sampleTick();
void writeContext();
void updateLiveData();
void calibrateBike(int i);
void calibrateSensors();

void setup() {
  // Serial for debug
  Serial.begin(57600);

  // _Hardware_ UART to Nano at 57600 baud, using pins RX=22, TX=27
  SerialNano.begin(57600, SERIAL_8N1, 22, 27);

  // Touchscreen & SPIFFS
  tft.init();
  tft.setRotation(1);
  SPI.begin();
  if (!SPIFFS.begin(true)) {
    tft.fillScreen(TFT_RED);
    tft.setCursor(10, 10);
    tft.setTextColor(TFT_WHITE);
    tft.println("SPIFFS mount failed!");
    while (1);
  }
  ts.begin();
  if (!ts.loadCalibration() || RERUN_CALIBRATE) {
    ts.calibrate();
    ts.saveCalibration();
  }
  pinMode(XPT2046_IRQ, INPUT);

  // SD card
  if (!SD.begin(SD_CS)) {
    tft.fillScreen(TFT_RED);
    tft.setCursor(10, 10);
    tft.setTextColor(TFT_WHITE);
    tft.println("SD init failed!");
    while (1);
  }

  // Clear screen, run sensor calibration, then go to Home
  tft.fillScreen(TFT_BLACK);
  calibrateSensors();
  drawHomeScreen();

  // Start the sampling ticker (fires every SAMPLE_INTERVAL_MS ms)
  sampleTicker.attach_ms(SAMPLE_INTERVAL_MS, sampleTick);
}

void loop() {
  writeContext();

  // If we’re in Record state, periodically update live data on screen
  if (CYD_State == Record) {
    unsigned long now = millis();
    if (now - lastDataUpdate >= dataInterval) {
      lastDataUpdate = now;
      updateLiveData();
    }
  }

  // 1) UI touch & navigation
  bool touched = (digitalRead(XPT2046_IRQ) == LOW);
  int btnCount = (CYD_State == Home ? 2 : 3);

  if (touched) {
    Point p = ts.getTouch();
    // rotation=1 means x and y get swapped by the library
    int16_t tx = p.y, ty = p.x;
    for (int i = 0; i < btnCount; i++) {
      buttons[i].press(buttons[i].contains(tx, ty));
    }
  } else {
    for (int i = 0; i < btnCount; i++) {
      buttons[i].press(false);
    }
  }

  for (int i = 0; i < btnCount; i++) {
    if (buttons[i].justReleased()) {
      if (CYD_State == Home) {
        CYD_State = (i == 0 ? Calibrate : Record);
        if (CYD_State == Calibrate) drawCalibrationScreen();
        else                  drawRecordScreen();
      }
      else if (CYD_State == Calibrate) {
        if (i != 2) {
          calibrateBike(i);  // 0 → pitch, 1 → roll
        } else {
          CYD_State = Home;
          drawHomeScreen();
        }
      }
      else { // Record screen
        if (i == 0) {
          // Start Recording
          record_data = true;
          packet_count = 0;
          String fn = generateFilename();
          logFile = SD.open(fn.c_str(), FILE_WRITE);
          if (logFile) {
            logFile.println(
              "time_us,"
              "fOriX,fOriY,fOriZ,"
              "rOriX,rOriY,rOriZ,"
              "fAccX,fAccY,fAccZ,"
              "rAccX,rAccY,rAccZ,"
              "rpmFront,rpmRear"
            );
            Serial.println("Logging to " + fn);
          } else {
            Serial.println("Log file open failed");
          }
        }
        else if (i == 1) {
          // Stop & Save
          record_data = false;
          if (logFile) {
            logFile.flush();
            logFile.close();
          }
          Serial.println("Recording stopped");
        }
        else if (i == 2) {
          // Back to Home
          record_data = false;
          if (logFile) {
            logFile.flush();
            logFile.close();
          }
          CYD_State = Home;
          drawHomeScreen();
        }
      }
    }
  }

  // 2) Data polling & logging
  if (sample_ready) {
    sample_ready = false;
    collectData();
    if (record_data) {
      SDCardWrite();
    }
  }
}

//––– UI Screens –––––––––––––––––––––––––––––––––––––––––––––––––––––––––
void drawHomeScreen() {
  tft.fillScreen(TFT_BLACK);
  resetButtonStates(2);
  for (int i = 0; i < 2; i++) {
    buttons[i].initButton(&tft,
      LCD_WIDTH/2, 80 + i*80, 200, 50,
      TFT_WHITE, (i == 0 ? TFT_BLUE : TFT_GREEN), TFT_WHITE,
      homeLabels[i], 2
    );
    buttons[i].drawButton();
  }
}

void drawCalibrationScreen() {
  tft.fillScreen(TFT_BLACK);
  resetButtonStates(3);
  uint32_t cols[3] = { TFT_BLUE, TFT_BLUE, TFT_RED };
  for (int i = 0; i < 3; i++) {
    buttons[i].initButton(&tft,
      LCD_WIDTH/2, 60 + i*60, 200, 50,
      TFT_WHITE, cols[i], TFT_WHITE,
      calLabels[i], 2
    );
    buttons[i].drawButton();
  }
}

void drawRecordScreen() {
  tft.fillScreen(TFT_BLACK);
  resetButtonStates(3);
  uint32_t cols[3] = { TFT_GREEN, TFT_RED, TFT_BLUE };
  for (int i = 0; i < 3; i++) {
    int x, y, w, h;
    if (i == 2) {
      // “Home” button in upper left
      x = 40; y = 30; w = 80; h = 40;
    } else {
      // “Start New” (i=0) and “Stop&Save” (i=1) on right half
      x = (LCD_WIDTH * 3 / 4) + (i - 1) * (LCD_WIDTH / 2);
      y = 200;
      w = 150; h = 60;
    }
    buttons[i].initButton(&tft,
      x, y, w, h,
      TFT_WHITE, cols[i], TFT_WHITE,
      recLabels[i], 2
    );
    buttons[i].drawButton();
  }
}

void resetButtonStates(int count) {
  for (int i = 0; i < count; i++) {
    buttons[i].press(false);
  }
}

String generateFilename() {
  char buf[32];
  snprintf(buf, sizeof(buf), "/log_%lu.csv", millis());
  return String(buf);
}

void writeContext() {
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);

  if (CYD_State == Home) {
    tft.setCursor(5, 10); 
    tft.println("Bicycle Dynamics Home Page");
  }
  else if (CYD_State == Calibrate) {
    tft.setCursor(10, 20);
    tft.println("Select axis to calibrate");
  }
  else { // Record screen
    if (record_data) {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setCursor(120, 10);
      tft.println("Recording!");
    } else {
      tft.fillRect(100, 10, 240, 20, TFT_BLACK);
    }
  }
}

void updateLiveData() {
  tft.setCursor(10, 60);
  char buffer[22];
  snprintf(buffer, sizeof(buffer), "RPM: %.2f", rpm_rear);
  tft.println(buffer);

  tft.setCursor(10, 80);
  snprintf(buffer, sizeof(buffer), "Roll: %.2f", rearOriZ);
  tft.println(buffer);

  tft.setCursor(10, 100);
  snprintf(buffer, sizeof(buffer), "Absolute Yaw: %.2f", frontOriX - rearOriX);
  tft.println(buffer);

  tft.setCursor(10, 120);
  snprintf(buffer, sizeof(buffer), "Pitch: %.2f", rearOriY);
  tft.println(buffer);
}

//––– Data collection from Nano –––––––––––––––––––––––––––––––––––––––––
void sampleTick() {
  sample_ready = true;
}

void collectData() {
  SerialNano.write('R');

  unsigned long start = millis();
  while (!SerialNano.available() && millis() - start < 50);

  if (!SerialNano.available()) {
    Serial.println("No response from Nano");
    return;
  }

  char line[128];
  size_t len = SerialNano.readBytesUntil('\n', line, sizeof(line) - 1);
  line[len] = '\0';

  // Tokenize into 14 floats
  float v[14];
  int idx = 0;
  char* tok = strtok(line, ",");
  while (tok && idx < 14) {
    v[idx++] = atof(tok);
    tok = strtok(NULL, ",");
  }
  if (idx < 14) return;  // incomplete

  // Assign them
  frontOriX  = v[0];
  frontOriY  = v[1] - pitchBias;
  frontOriZ  = v[2] - rollBias;
  rearOriX   = v[3];
  rearOriY   = v[4] - pitchBias;
  rearOriZ   = v[5] - rollBias;
  frontAccX  = v[6];
  frontAccY  = v[7];
  frontAccZ  = v[8];
  rearAccX   = v[9];
  rearAccY   = v[10];
  rearAccZ   = v[11];
  rpm_front  = v[12];
  rpm_rear   = v[13];
}

void SDCardWrite() {
  if (!logFile) return;

  char buf[200];
  unsigned long ts = micros();
  int n = snprintf(buf, sizeof(buf),
    "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
    ts,
    frontOriX, frontOriY, frontOriZ,
    rearOriX,  rearOriY,  rearOriZ,
    frontAccX, frontAccY, frontAccZ,
    rearAccX,  rearAccY,  rearAccZ,
    rpm_front, rpm_rear
  );
  logFile.write((uint8_t*)buf, n);
  packet_count++;
}

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// User‐driven pitch/roll calibration on the bike.  i==0 → pitch; i==1 → roll
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
void calibrateBike(int i) { 
  sampleTicker.detach();  // Pause ongoing sampling

  const int numSamples = 20;
  float sum = 0.0f;

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(10, 10);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.println("Calibrating...");

  // Collect 20 readings of (front + rear) for either pitch or roll
  for (int j = 0; j < numSamples; j++) {
    SerialNano.write('R');
    unsigned long start = millis();
    while (!SerialNano.available() && millis() - start < 50);
    if (!SerialNano.available()) {
      tft.setCursor(10, 40);
      tft.println("No response!");
      delay(1000);
      drawCalibrationScreen();
      sampleTicker.attach_ms(SAMPLE_INTERVAL_MS, sampleTick);
      return;
    }

    char line[128];
    size_t len = SerialNano.readBytesUntil('\n', line, sizeof(line) - 1);
    line[len] = '\0';

    float v[14];
    int idx = 0;
    char* tok = strtok(line, ",");
    while (tok && idx < 14) {
      v[idx++] = atof(tok);
      tok = strtok(NULL, ",");
    }
    if (idx < 14) {
      // incomplete data, bail out
      drawCalibrationScreen();
      sampleTicker.attach_ms(SAMPLE_INTERVAL_MS, sampleTick);
      return;
    }

    if (i == 0) {
      // pitch = average of frontOriY and rearOriY (before bias)
      sum += (v[1] + v[4]) / 2.0f;
    }
    else if (i == 1) {
      // roll  = average of frontOriZ and rearOriZ (before bias)
      sum += (v[2] + v[5]) / 2.0f;
    }
    delay(50);
  }

  // Compute the average (bias)
  float bias = sum / float(numSamples);

  if (i == 0) {
    pitchBias = bias;
  } else if (i == 1) {
    rollBias = bias;
  }

  tft.setCursor(10, 40);
  tft.print("Bias: ");
  tft.println(bias, 2);
  delay(1000);

  drawCalibrationScreen();                     // Return to calibration UI
  sampleTicker.attach_ms(SAMPLE_INTERVAL_MS, sampleTick);
}

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// “C” command to the Nano: rotate both IMUs until they report (3,3) in
// gyro & accel.  Once both read fully calibrated, exit.
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
void calibrateSensors() {
  bool frontDone = false, rearDone = false;
  uint8_t gyroFront = 0, accelFront = 0, gyroRear = 0, accelRear = 0;

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(10, 10);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.println("Rotate IMUs to calibrate");

  while (!(frontDone && rearDone)) {
    SerialNano.write('C');

    unsigned long start = millis();
    while (!SerialNano.available() && millis() - start < 200);
    if (!SerialNano.available()) {
      Serial.println("No response from Nano");
      return;
    }

    char line[64];
    size_t len = SerialNano.readBytesUntil('\n', line, sizeof(line) - 1);
    line[len] = '\0';

    // Parse 4 comma‐separated values: gyroFront, accelFront, gyroRear, accelRear
    float v[4];
    int idx = 0;
    char* tok = strtok(line, ",");
    while (tok && idx < 4) {
      v[idx++] = atof(tok);
      tok = strtok(NULL, ",");
    }
    if (idx < 4) return;  // incomplete

    gyroFront = (uint8_t)v[0];
    accelFront = (uint8_t)v[1];
    gyroRear  = (uint8_t)v[2];
    accelRear  = (uint8_t)v[3];

    // Show current status on screen
    char buf[64];
    sprintf(buf, "Front: %d %d", gyroFront, accelFront);
    tft.setCursor(10, 40);
    tft.fillRect(10, 40, 280, 20, TFT_BLACK);
    tft.println(buf);

    sprintf(buf, "Rear: %d %d", gyroRear, accelRear);
    tft.setCursor(10, 60);
    tft.fillRect(10, 60, 280, 20, TFT_BLACK);
    tft.println(buf);

    // Check if each IMU is fully calibrated (3,3)
    if (!frontDone && gyroFront == 3 && accelFront == 3) {
      frontDone = true;
      tft.setCursor(10, 90);
      tft.println("✅ Front Calibrated!");
    }
    if (!rearDone && gyroRear == 3 && accelRear == 3) {
      rearDone = true;
      tft.setCursor(10, 110);
      tft.println("✅ Rear Calibrated!");
    }

    delay(250);
  }

  delay(1000);
  tft.fillScreen(TFT_BLACK);  // Clear before showing Home screen
}
