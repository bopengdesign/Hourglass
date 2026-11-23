#include <Arduino.h>
#include <Wire.h>
#include <WiFiMulti.h>
#include <Adafruit_VL53L0X.h>
#include <AdafruitIO_WiFi.h>

// ---------- Wi-Fi Credentials ----------
#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASS "WIFI_PASSWORD"

// ---------- Adafruit IO Configuration ----------
#define IO_USERNAME    "IO_USERNAME"
#define IO_KEY         "IO_KEY"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *t_calc_feed = io.feed("t_calculated");
AdafruitIO_Feed *t_act_feed  = io.feed("t_actual");
AdafruitIO_Feed *phase_feed  = io.feed("phase");

// ---------- Pins (Nano ESP32, Arduino numbering) ----------
const int ENCODER_CLK = 2;
const int ENCODER_DT  = 3;
const int ENCODER_SW  = 4;
const int MOSFET_LED  = 5;
const int BUZZER      = 6;
const int XSHUT_TOF1  = 7;
const int XSHUT_TOF2  = 8;

// ---------- Hopper geometry (cm, g) ----------
const float R_CM = 4.5f;    // cylinder and cone radius
const float RHO  = 1.4735f;   // g/cm3
const float H0   = 2.2f;    // top -> initial sand surface
const float H1   = 4.0f;    // sand height in the cylinder
const float H2   = 7.5f;    // cone height
const float Hx   = 9.65f;   // uncut cone height

const float SENSOR_OFFSET_CM = 2.8f;

// ---------- Phase tables (index 0..5 <-> encoder 0,4,8,12,16,20) ----------
const float    W_TABLE[6]    = {2.267f, 1.133f, 0.756f, 0.567f, 0.453f, 0.378f};  // g/s
const uint8_t  TARGET_MIN[6] = {4, 9, 14, 19, 24, 29};                // time duration - 1 in minutes

// ---------- Timing / detection ----------
const uint32_t SAMPLE_MS      = 1000;    // ToF + log period
const uint32_t UPLOAD_MS      = 5000;    // Adafruit IO period
const uint32_t CONFIRM_HOLD   = 2000;    // button hold for phase-0 confirm
const uint32_t FINISH_FLASH   = 60000;   // fast flash duration
const float    DROP_FROM_CM   = 5.0f;    // previous reading must be >= this
const float    DROP_TO_CM     = 1.0f;    // new reading must be within 0 ~ this
const uint8_t  SURGE_CONFIRM  = 1;       // consecutive samples (1 = no debounce)

// ---------- Encoder ----------
// Tune these three constants by watching the "[ENC] step=..." serial lines while turning the knob.
const int ENC_DIR                   = - 1;  // - 1 = counter-clockwise increases step (this knob); 1 = clockwise increases
const int ENC_TRANSITIONS_PER_COUNT = 2;  // quadrature transitions per click (detent). 2 for this knob (30 clicks/turn, 12 deg each);
                                          // use 4 for encoders that give a full cycle per click
const int COUNTS_PER_PHASE          = 4;  // counted steps per phase (phase values 0,4,8,...)
const int ENC_MAX_STEP              = 21;

volatile int     encoderStep = 0;         // counted steps, 0..ENC_MAX_STEP
volatile int     encRaw      = 0;         // raw transitions
volatile uint8_t encPrev     = 0;

static const int8_t DRAM_ATTR ENC_TABLE[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};

void ARDUINO_ISR_ATTR readEncoder() {
  encPrev = ((encPrev << 2) | (digitalRead(ENCODER_CLK) << 1) | digitalRead(ENCODER_DT)) & 0x0F;
  int8_t d = ENC_TABLE[encPrev];
  if (d == 0) return;
  int raw = encRaw + d * ENC_DIR;
  int maxRaw = ENC_MAX_STEP * ENC_TRANSITIONS_PER_COUNT;
  if (raw < 0) raw = 0;
  if (raw > maxRaw) raw = maxRaw;
  encRaw = raw;
  encoderStep = raw / ENC_TRANSITIONS_PER_COUNT;
}

void encoderZero() {
  noInterrupts();
  encRaw = 0;
  encoderStep = 0;
  encPrev = ((digitalRead(ENCODER_CLK) << 1) | digitalRead(ENCODER_DT)) & 0x03;
  interrupts();
}

int phaseIndex() {
  int idx = (encoderStep + 1) / COUNTS_PER_PHASE;
  return constrain(idx, 0, 5);
}

// ---------- ToF ----------
Adafruit_VL53L0X tof[2];
const uint8_t TOF_ADDR[2]  = {0x30, 0x31};
const int     XSHUT_PIN[2] = {XSHUT_TOF1, XSHUT_TOF2};
bool     tofOk[2]    = {false, false};
bool     hValid[2]   = {false, false};
float    hCm[2]      = {0, 0};
uint8_t  rawStatus[2] = {0, 0};
uint16_t rawMm[2]     = {0, 0};

void initTof() {
  for (int i = 0; i < 2; i++) { pinMode(XSHUT_PIN[i], OUTPUT); digitalWrite(XSHUT_PIN[i], LOW); }
  delay(50);
  for (int i = 0; i < 2; i++) {          // wake one sensor at a time
    digitalWrite(XSHUT_PIN[i], HIGH);
    delay(50);
    tofOk[i] = tof[i].begin(TOF_ADDR[i]);
  }
}

// Invalid readings are flagged (hValid=false), never turned into 0.
void sampleTof() {
  for (int i = 0; i < 2; i++) {
    hValid[i] = false;
    if (!tofOk[i]) continue;
    VL53L0X_RangingMeasurementData_t m = {};
    tof[i].rangingTest(&m, false);
    rawStatus[i] = m.RangeStatus;
    rawMm[i]     = m.RangeMilliMeter;
    if (m.RangeStatus == 0 && m.RangeMilliMeter < 1000) {
      float h = m.RangeMilliMeter / 10.0f - SENSOR_OFFSET_CM;
      hCm[i] = (h < 0) ? 0 : h;
      hValid[i] = true;
    }
  }
}

// ---------- Height -> elapsed time ----------
// h = distance from the top plane to the sand surface (cm)
float calcPassedTime(float h, float W) {
  if (W <= 0 || h <= H0 - SENSOR_OFFSET_CM) return 0;

  const float A = PI * R_CM * R_CM;
  
  const float vMax = A * H1 + A * Hx / 3.0f - A * (Hx - H2) * (Hx - H2) * (Hx - H2) / (3.0f * Hx * Hx);
  
  float vNow;
  if (h <=  H0 + H1 - SENSOR_OFFSET_CM) {                     // surface still in the cylinder
    vNow = A * (H0 + H1 - SENSOR_OFFSET_CM - h) + (A * Hx / 3.0f) - A * (Hx - H2) * (Hx - H2) * (Hx - H2) / (3.0f * Hx * Hx);
  } else {                           // surface in the cone
    float rem = H0 + H1 + Hx - h - SENSOR_OFFSET_CM;
    vNow = A * rem * rem * rem / (3.0f * Hx * Hx) - A * (Hx - H2) * (Hx - H2) * (Hx - H2) / (3.0f * Hx * Hx);
    if (vNow < 0) vNow = 0;
  }
  return (vMax - vNow) * RHO / W;           // seconds
}

// ---------- Shared with the network task ----------
volatile bool     g_uploadActive = false;
volatile bool     g_flush        = false;
volatile float    g_tCalc        = 0;
volatile uint32_t g_tAct         = 0;
volatile int      g_phaseVal     = 0;       // 0,4,8,12,16,20
volatile bool     g_sendPhase    = false;   // send once at task start

void netTask(void *) {
  io.connect();
  uint32_t lastSend = 0;
  for (;;) {
    io.run();
    bool connected = (io.status() == AIO_CONNECTED);
    uint32_t now = millis();
    bool due = g_uploadActive && (now - lastSend >= UPLOAD_MS);
    if (connected && (due || g_flush)) {
      lastSend = now;
      t_calc_feed->save((float)g_tCalc);
      t_act_feed->save((float)g_tAct);
    }
    if (connected && g_sendPhase) {          // stays pending until actually sent
      int finalVal = (g_phaseVal * 5) / 4 + 5; // Convert phase into minutes
      phase_feed->save(finalVal);
      g_sendPhase = false;
    }
    g_flush = false;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ---------- State machine ----------
enum State { ST_WAIT_ZERO, ST_MONITOR, ST_RUNNING, ST_FINISH };
State state = ST_WAIT_ZERO;

bool     welcomed = false;
bool     swPressed = false;
uint32_t swPressStart = 0;
bool     swWaitRelease = false;   // ignore the button until it has been released
int      lastPhShown = 0;
uint32_t lastSample = 0;
uint32_t lastLedToggle = 0;
bool     ledState = false;

float    lastH[2]    = {0, 0};          // last valid reading per sensor
bool     hasLast[2]  = {false, false};
uint8_t  surgeCnt[2] = {0, 0};
uint32_t surgeStart[2] = {0, 0};

int      trigCh = 0;      // sensor that dropped (starts the task)
int      activeCh = 0;    // the other sensor measures the sand
int      phaseLocked = 0;
float    wLocked = 2.0f;
uint32_t taskStartMs = 0;
uint32_t targetMs = 0;
uint32_t finishStart = 0;
float    tCalc = 0;
float    hShift = 0; 

// 1 = passive buzzer (needs a square wave), 0 = active buzzer (just HIGH)
#define BUZZER_PASSIVE 1
#define BUZZER_FREQ    2700   // Hz

void beep(uint16_t ms) {
#if BUZZER_PASSIVE
  tone(BUZZER, BUZZER_FREQ);
  delay(ms);
  noTone(BUZZER);
  digitalWrite(BUZZER, LOW);
#else
  digitalWrite(BUZZER, HIGH);
  delay(ms);
  digitalWrite(BUZZER, LOW);
#endif
}

void flashLed(uint32_t now, uint32_t halfPeriod) {
  if (now - lastLedToggle >= halfPeriod) { ledState = !ledState; lastLedToggle = now; }
  digitalWrite(MOSFET_LED, ledState ? HIGH : LOW);
}

void printTof(int i) {
  Serial.printf("T%d: ", i + 1);
  if (!tofOk[i])        Serial.print("INIT_FAIL");
  else if (hValid[i])   Serial.printf("%.2fcm", hCm[i]);
  else                  Serial.printf("invalid(st=%u,%umm)", rawStatus[i], rawMm[i]);
}

void logLine(const char *tag) {
  Serial.printf("[%s] ph=%d(step=%d) | ", tag, phaseIndex(), (int)encoderStep);
  printTof(0); Serial.print(" | "); printTof(1);
  if (state == ST_MONITOR) {
    Serial.println();
  } else {
    Serial.printf(" | trig=T%d meas=T%d t_act=%lus t_calc=%.0fs target=%lus\n",
                  trigCh + 1, activeCh + 1, (unsigned long)((millis() - taskStartMs) / 1000),
                  tCalc, (unsigned long)(targetMs / 1000));
  }
}

void resetDetection() {
  for (int i = 0; i < 2; i++) { hasLast[i] = false; surgeCnt[i] = 0; }
}

void startTask(int ch, uint32_t now) {
  trigCh      = ch;
  activeCh    = 1 - ch;                 // the other sensor measures the distance
  phaseLocked = phaseIndex();
  wLocked     = W_TABLE[phaseLocked];
  targetMs    = (uint32_t)TARGET_MIN[phaseLocked] * 60000UL;
  taskStartMs  = surgeStart[ch];
  hShift = hValid[activeCh] ? H0 - (hCm[activeCh] + SENSOR_OFFSET_CM) : 0;
  tCalc  = hValid[activeCh] ? calcPassedTime(hCm[activeCh] + hShift, wLocked) : 0;
  g_tCalc = tCalc;
  g_tAct  = (now - taskStartMs) / 1000;
  g_uploadActive = true;
  g_phaseVal = phaseLocked * 4;
  g_sendPhase = true;
  lastSample = now;
  state = ST_RUNNING;
  beep(150);
  Serial.printf("\n>>> T%d dropped to 0~1 cm, using T%d for height. Phase %d, W=%.1f g/s, target %u min <<<\n",
                ch + 1, activeCh + 1, phaseLocked, wLocked, TARGET_MIN[phaseLocked]);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
 
  pinMode(MOSFET_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);

  Wire.begin();           // SDA = A4, SCL = A5
  initTof();

  encoderZero();
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT),  readEncoder, CHANGE);

  xTaskCreatePinnedToCore(netTask, "net", 8192, nullptr, 1, nullptr, 0);
}

void loop() {
  uint32_t now = millis();

  // Live encoder feedback
  static int lastStepShown = -1;
  int stepNow = encoderStep;
  if (stepNow != lastStepShown) {
    lastStepShown = stepNow;
    int phNow = phaseIndex();
    if (state != ST_FINISH) Serial.printf("[ENC] step=%d -> phase %d (value %d)\n", stepNow, phNow, phNow * 4);
    if (state == ST_MONITOR && phNow != lastPhShown) beep(30);
    lastPhShown = phNow;
  }

  switch (state) {

    // ---------- Wait for phase-0 confirmation ----------
    case ST_WAIT_ZERO: {
      digitalWrite(MOSFET_LED, LOW);
      if (!welcomed) {
        welcomed = true;
        Serial.println("\n====================================================");
        Serial.printf("TOF1 %s, TOF2 %s\n", tofOk[0] ? "OK" : "INIT FAIL", tofOk[1] ? "OK" : "INIT FAIL");
        Serial.println("Turn knob to phase 0, then hold the button for 2 s.");
        Serial.println("====================================================");
      }
      if (swWaitRelease) {
        if (digitalRead(ENCODER_SW) == HIGH) swWaitRelease = false;
        break;
      }
      if (digitalRead(ENCODER_SW) == LOW) {
        if (!swPressed) { swPressed = true; swPressStart = now; }
        else if (now - swPressStart >= CONFIRM_HOLD) {
          encoderZero();
          lastPhShown = 0;
          resetDetection();
          swPressed = false;
          lastSample = now - SAMPLE_MS;
          state = ST_MONITOR;
          swWaitRelease = true;
          beep(100); delay(100); beep(100);
          Serial.println("\n>>> Phase 0 confirmed. Measuring height every 2 s. (Hold button 2 s to redo the confirmation) <<<");
        }
      } else {
        swPressed = false;
      }
      break;
    }

    // ---------- Measure only; wait for trigger 1 ----------
    case ST_MONITOR: {
      digitalWrite(MOSFET_LED, LOW);

      // hold the button 2 s to redo the phase-0 confirmation
      if (swWaitRelease) {
        if (digitalRead(ENCODER_SW) == HIGH) swWaitRelease = false;
      } else if (digitalRead(ENCODER_SW) == LOW) {
        if (!swPressed) { swPressed = true; swPressStart = now; }
        else if (now - swPressStart >= CONFIRM_HOLD) {
          swPressed = false;
          swWaitRelease = true;
          welcomed = false;
          resetDetection();
          state = ST_WAIT_ZERO;
          beep(100);
          Serial.println("\n>>> Back to phase-0 confirmation. <<<");
          break;
        }
      } else {
        swPressed = false;
      }

      if (now - lastSample >= SAMPLE_MS) {
        lastSample = now;
        sampleTof();
        for (int i = 0; i < 2; i++) {
          if (!hValid[i]) { surgeCnt[i] = 0; continue; }
          if (surgeCnt[i] == 0) {
            // sudden drop: previous reading >= 5 cm, now within 0~1 cm
            if (hasLast[i] && lastH[i] >= DROP_FROM_CM && hCm[i] <= DROP_TO_CM) {
              surgeCnt[i] = 1;
              surgeStart[i] = now;
            }
          } else if (hCm[i] <= DROP_TO_CM) {
            if (surgeCnt[i] < 255) surgeCnt[i]++;
          } else {
            surgeCnt[i] = 0;
          }
          lastH[i] = hCm[i];
          hasLast[i] = true;
        }
        logLine("MONITOR");
        for (int i = 0; i < 2; i++) {
          if (surgeCnt[i] >= SURGE_CONFIRM) { startTask(i, now); break; }
        }
      }
      break;
    }

    // ---------- Task running: slow flash ----------
    case ST_RUNNING: {
      uint32_t elapsed = now - taskStartMs;
      g_tAct = elapsed / 1000;

      // phase follows the knob during the task
      int ph = phaseIndex();
      if (ph != phaseLocked) {
        phaseLocked = ph;
        wLocked  = W_TABLE[ph];
        targetMs = (uint32_t)TARGET_MIN[ph] * 60000UL;
        if (hValid[activeCh]) {
          float t = calcPassedTime(hCm[activeCh] + hShift, wLocked);
          if (t > tCalc) tCalc = t;
          }
        beep(30);
        Serial.printf(">>> Phase changed to %d (W=%.1f g/s, target %u min) <<<\n", ph, wLocked, TARGET_MIN[ph]);
      }

      if (now - lastSample >= SAMPLE_MS) {
        lastSample = now;
        sampleTof();
        if (hValid[activeCh]) {
          float t = calcPassedTime(hCm[activeCh] + hShift, wLocked);
          float maxUp = (SAMPLE_MS / 1000.0f) * 2.0f;     // add sampling time
          if (t > tCalc + maxUp) t = tCalc + maxUp;
          if (t > tCalc) tCalc = t;
        }
        g_tCalc = tCalc;
        logLine("RUNNING");
      }

      if (elapsed >= targetMs) {
        g_sendPhase = false;
        finishStart = now;
        state = ST_FINISH;
        beep(500);
        Serial.println("\n>>> Target reached. Fast flashing for 1 min. <<<");
      } else {
        flashLed(now, 1000);
      }
      break;
    }

    // ---------- Fast flash 1 min, then restart ----------
    case ST_FINISH: {
      g_tAct = (now - taskStartMs) / 1000;          // t_actual keeps counting

      if (now - lastSample >= SAMPLE_MS) {
        lastSample = now;
        sampleTof();
        if (hValid[activeCh]) tCalc = calcPassedTime(hCm[activeCh] + hShift, wLocked);
        g_tCalc = tCalc;
        logLine("FINISH");
      }

      if (now - finishStart >= FINISH_FLASH) {
        g_uploadActive = false;
        g_flush = true;                             // one last upload
        beep(1200);
        digitalWrite(MOSFET_LED, LOW);

        // keep the current phase and go straight back to measuring
        resetDetection();                           // forget old readings, re-arm the trigger
        swPressed = false;
        swWaitRelease = false;
        lastPhShown = phaseIndex();
        lastSample = now;
        state = ST_MONITOR;
        Serial.printf("\n>>> Task finished. Measuring again, phase %d kept. <<<\n", phaseIndex());
      } else {
        flashLed(now, 100);
      }
      break;
    }
  }

  delay(5);
}