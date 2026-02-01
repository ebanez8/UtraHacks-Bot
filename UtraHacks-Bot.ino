#include <Arduino.h>

// =======================
// Pin Definitions (YOUR ORIGINAL)
// =======================
#define TRIG_PIN 8
#define ECHO_PIN 9

// TCS3200
#define S0_PIN 4
#define S1_PIN 5
#define S2_PIN 6
#define S3_PIN 7
#define OUT_PIN 12

// L298N Motor Driver
#define ENA_PIN A0
#define IN1_PIN A1
#define IN2_PIN A2
#define ENB_PIN A3
#define IN3_PIN 13
#define IN4_PIN A4

// =======================
// MOTOR INVERSION FLAGS
// =======================
#define LEFT_INVERT  1
#define RIGHT_INVERT 1

// =======================
// TCS3200 sampling
// =======================
#define SAMPLES_PER_CHANNEL 3
#define PULSE_TIMEOUT_US 20000UL

// =======================
// Movement tuning (SLOWER + LONGER MOVES)
// =======================
#define FWD_PWM 220
#define ROT_PWM 220

// How often we run one “decision”
#define CONTROL_PERIOD_MS 140

// How long we drive forward when we think we’re on the line
#define FWD_BURST_MS 220

// Scan pulses (probe left/right) — longer so it can “see”
#define SCAN_PULSE_MS 70

// After choosing left/right, how long to rotate before moving
#define STEER_BASE_MS 140
#define STEER_EXTRA_MS 140   // extra based on how much better one side is

// After steering, drive forward a bit so it actually progresses
#define FWD_AFTER_TURN_MS 240

// If we’re totally lost, rotate search this long each attempt
#define SEARCH_HOLD_MS 220

// Split detection stability
#define STABLE_MS 280

#define PRINT_PERIOD_MS 300
#define HEARTBEAT_MS 1200

// =======================
// Types
// =======================
enum ColorTag { C_UNKNOWN, C_WHITE, C_BLACK, C_RED, C_GREEN };
enum RobotState { START, FOLLOW_RED, FOLLOW_GREEN };

// =======================
// Data structs
// =======================
struct Periods { unsigned int rP, gP, bP, cP; };
struct NormRGB { int rN, gN, bN; };

// =======================
// Globals
// =======================
RobotState state = START;

unsigned long lastControl = 0;
unsigned long lastPrint = 0;
unsigned long lastHeartbeat = 0;

// White-floor baseline (auto at startup)
unsigned int whiteCP = 0;

// Stable color tracking for split
ColorTag stablePrev = C_UNKNOWN;
unsigned long stableSince = 0;

// Last search direction if lost (+1 right, -1 left)
int lastSearchDir = 1;

// =======================
// Motor control (ENA/ENB ON/OFF enable)
// =======================
static inline void enableLeft(bool on)  { digitalWrite(ENA_PIN, on ? HIGH : LOW); }
static inline void enableRight(bool on) { digitalWrite(ENB_PIN, on ? HIGH : LOW); }

void setDrive(int leftPWM, int rightPWM) {
  leftPWM  = constrain(leftPWM,  -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  if (LEFT_INVERT)  leftPWM  = -leftPWM;
  if (RIGHT_INVERT) rightPWM = -rightPWM;

  if (leftPWM >= 0) { digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW); }
  else              { digitalWrite(IN1_PIN, LOW);  digitalWrite(IN2_PIN, HIGH); }

  if (rightPWM >= 0) { digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW); }
  else               { digitalWrite(IN3_PIN, LOW);  digitalWrite(IN4_PIN, HIGH); }

  enableLeft(abs(leftPWM) > 0);
  enableRight(abs(rightPWM) > 0);
}

void stopDrive() {
  enableLeft(false);
  enableRight(false);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void driveForward() { setDrive(FWD_PWM, FWD_PWM); }
void rotateLeft()   { setDrive(-ROT_PWM, ROT_PWM); }
void rotateRight()  { setDrive(ROT_PWM, -ROT_PWM); }

// =======================
// TCS3200 reading (period = LOW + HIGH)
// =======================
static unsigned long readLowPulse()  { return pulseIn(OUT_PIN, LOW,  PULSE_TIMEOUT_US); }
static unsigned long readHighPulse() { return pulseIn(OUT_PIN, HIGH, PULSE_TIMEOUT_US); }

static unsigned int readPeriodSample() {
  unsigned long lo = readLowPulse();
  unsigned long hi = readHighPulse();
  if (lo == 0 || hi == 0) return (unsigned int)PULSE_TIMEOUT_US;
  unsigned long period = lo + hi;
  if (period > 65535UL) period = 65535UL;
  return (unsigned int)period;
}

static unsigned int median3(bool s2, bool s3) {
  digitalWrite(S2_PIN, s2 ? HIGH : LOW);
  digitalWrite(S3_PIN, s3 ? HIGH : LOW);
  delayMicroseconds(150);

  unsigned int a[3];
  for (int i=0;i<3;i++) a[i] = readPeriodSample();

  if (a[1] < a[0]) { unsigned int t=a[0]; a[0]=a[1]; a[1]=t; }
  if (a[2] < a[1]) { unsigned int t=a[1]; a[1]=a[2]; a[2]=t; }
  if (a[1] < a[0]) { unsigned int t=a[0]; a[0]=a[1]; a[1]=t; }

  return a[1];
}

Periods readPeriods() {
  Periods p;
  p.rP = median3(false, false); // red
  p.bP = median3(false, true);  // blue (unused)
  p.gP = median3(true, true);   // green
  p.cP = median3(true, false);  // clear
  return p;
}

NormRGB normalizeRGB(const Periods &p) {
  const unsigned long K = 100000UL;
  unsigned long rI = (p.rP == 0) ? 0 : (K / p.rP);
  unsigned long gI = (p.gP == 0) ? 0 : (K / p.gP);
  unsigned long bI = (p.bP == 0) ? 0 : (K / p.bP);

  unsigned long sum = rI + gI + bI;
  if (sum == 0) sum = 1;

  NormRGB n;
  n.rN = (int)((rI * 255UL) / sum);
  n.gN = (int)((gI * 255UL) / sum);
  n.bN = (int)((bI * 255UL) / sum);
  return n;
}

// =======================
// Preset “range” logic (no manual tuning)
// =======================
// Dark tape usually -> larger clear period than white floor
static inline bool isDark(unsigned int cP) {
  // ~25% darker than baseline counts as tape-ish
  return (cP > (unsigned int)((whiteCP * 125UL) / 100UL));
}

ColorTag classifyPreset(const Periods &p, const NormRGB &n) {
  int maxGB = max(n.gN, n.bN);
  int maxRB = max(n.rN, n.bN);

  int redDom   = n.rN - maxGB;
  int greenDom = n.gN - maxRB;

  bool dark = isDark(p.cP);

  // Strong color dominance wins even if not super dark
  if (redDom > 26 && n.rN > 105)   return C_RED;
  if (greenDom > 26 && n.gN > 105) return C_GREEN;

  // Otherwise use darkness for black vs white
  if (dark) return C_BLACK;
  return C_WHITE;
}

// Stable color (for split detection)
ColorTag stableColor(ColorTag nowC) {
  unsigned long now = millis();
  if (nowC != stablePrev) {
    stablePrev = nowC;
    stableSince = now;
    return C_UNKNOWN;
  }
  if (now - stableSince >= STABLE_MS) return nowC;
  return C_UNKNOWN;
}

// =======================
// Line score (0..1000) so we can compare left vs right
// =======================
// BLACK score: how dark vs white baseline
int scoreBlack(const Periods &p) {
  if (whiteCP == 0) return 0;
  long diff = (long)p.cP - (long)whiteCP;
  // map diff up to ~80% of whiteCP into 0..1000
  long denom = (long)whiteCP * 80L / 100L;
  if (denom < 1) denom = 1;
  long s = (diff * 1000L) / denom;
  return (int)constrain(s, 0L, 1000L);
}

// RED/GREEN score: dominance + a bit of darkness help
int scoreRed(const Periods &p, const NormRGB &n) {
  int dom = n.rN - max(n.gN, n.bN);   // positive = more red
  long s = (long)(dom - 10) * 35L;    // scale into ~0..1000
  if (isDark(p.cP)) s += 120;
  return (int)constrain(s, 0L, 1000L);
}
int scoreGreen(const Periods &p, const NormRGB &n) {
  int dom = n.gN - max(n.rN, n.bN);
  long s = (long)(dom - 10) * 35L;
  if (isDark(p.cP)) s += 120;
  return (int)constrain(s, 0L, 1000L);
}

// =======================
// REAL single-sensor line follow: PROBE L / PROBE R (slow + longer turns)
// =======================
int lineScore(ColorTag followColor) {
  Periods p = readPeriods();
  NormRGB n = normalizeRGB(p);

  if (followColor == C_BLACK) return scoreBlack(p);
  if (followColor == C_RED)   return scoreRed(p, n);
  return scoreGreen(p, n);
}

void followSingleSensor(ColorTag followColor) {
  // 1) read score at current heading
  int s0 = lineScore(followColor);

  // If decent, just drive forward for a while
  if (s0 >= 520) {
    driveForward();
    delay(FWD_BURST_MS);
    return;
  }

  // 2) probe LEFT
  stopDrive();
  rotateLeft();
  delay(SCAN_PULSE_MS);
  stopDrive();
  int sL = lineScore(followColor);

  // back to center
  rotateRight();
  delay(SCAN_PULSE_MS);
  stopDrive();

  // 3) probe RIGHT
  rotateRight();
  delay(SCAN_PULSE_MS);
  stopDrive();
  int sR = lineScore(followColor);

  // back to center
  rotateLeft();
  delay(SCAN_PULSE_MS);
  stopDrive();

  int best = max(sL, sR);

  // totally lost: rotate in last direction for longer, then try again
  if (best < 160) {
    if (lastSearchDir > 0) rotateRight();
    else rotateLeft();
    delay(SEARCH_HOLD_MS);
    stopDrive();
    return;
  }

  // steer toward better side, for LONGER than before
  int diff = abs(sL - sR);
  int extra = map(diff, 0, 700, 0, STEER_EXTRA_MS);
  int turnMs = STEER_BASE_MS + extra;

  if (sL > sR) {
    lastSearchDir = -1;
    rotateLeft();
    delay(turnMs);
  } else {
    lastSearchDir = 1;
    rotateRight();
    delay(turnMs);
  }

  // then move forward noticeably
  driveForward();
  delay(FWD_AFTER_TURN_MS);
}

// =======================
// Startup: auto-white baseline
// =======================
void autoWhiteBaseline() {
  // Put robot on WHITE floor for ~1.2s after reset
  unsigned long t0 = millis();
  unsigned long sum = 0;
  int count = 0;

  while (millis() - t0 < 1200) {
    Periods p = readPeriods();
    sum += p.cP;
    count++;
    delay(25);
  }
  if (count == 0) count = 1;
  whiteCP = (unsigned int)(sum / (unsigned long)count);

  Serial.print("WHITE baseline cP=");
  Serial.println(whiteCP);
}

// =======================
// Setup / Loop
// =======================
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 1500)) {}

  Serial.println("\nBOOT: Preset single-sensor line follow (slow + longer turns)");

  // TCS3200 pins
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S3_PIN, OUTPUT);
  pinMode(OUT_PIN, INPUT);

  // 20% scaling
  digitalWrite(S0_PIN, HIGH);
  digitalWrite(S1_PIN, LOW);

  // Motor pins
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  stopDrive();

  // Ultrasonic pins (unused for now)
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  autoWhiteBaseline();

  // Start following BLACK until split decides RED or GREEN
  state = START;
}

void loop() {
  unsigned long now = millis();

  if (now - lastHeartbeat >= HEARTBEAT_MS) {
    Serial.print("STATE=");
    Serial.println((int)state);
    lastHeartbeat = now;
  }

  if (now - lastControl < CONTROL_PERIOD_MS) return;
  lastControl = now;

  // Read once for split detection
  Periods p = readPeriods();
  NormRGB n = normalizeRGB(p);
  ColorTag tag = classifyPreset(p, n);
  ColorTag st = stableColor(tag);

  // Debug print
  if (now - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = now;
    Serial.print("TAG=");
    Serial.print(tag == C_WHITE ? "WHITE" : tag == C_BLACK ? "BLACK" : tag == C_RED ? "RED" : tag == C_GREEN ? "GREEN" : "UNK");
    Serial.print("  cP="); Serial.print(p.cP);
    Serial.print("  N=");  Serial.print(n.rN); Serial.print(","); Serial.print(n.gN); Serial.print(","); Serial.print(n.bN);
    Serial.print("  scores B/R/G=");
    Serial.print(scoreBlack(p)); Serial.print("/");
    Serial.print(scoreRed(p,n)); Serial.print("/");
    Serial.println(scoreGreen(p,n));
  }

  // Start on black, then choose a path at split
  if (state == START) {
    followSingleSensor(C_BLACK);

    if (st == C_RED) {
      Serial.println("SPLIT -> FOLLOW RED");
      state = FOLLOW_RED;
    } else if (st == C_GREEN) {
      Serial.println("SPLIT -> FOLLOW GREEN");
      state = FOLLOW_GREEN;
    }
    return;
  }

  if (state == FOLLOW_RED) {
    followSingleSensor(C_RED);
    return;
  }

  if (state == FOLLOW_GREEN) {
    followSingleSensor(C_GREEN);
    return;
  }
}
