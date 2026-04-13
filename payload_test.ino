#include <SCServo.h>
#include <Preferences.h>

HardwareSerial ServoBus(1);
SMS_STS st;
Preferences prefs;

// ===================== Hardware =====================
static const int S_RXD = D7;   // XIAO RX  <- adapter RX
static const int S_TXD = D6;   // XIAO TX  <- adapter TX
static const uint32_t SERVO_BAUD = 1000000;

// ===================== Servo IDs ====================
static const uint8_t SCOOP_ID = 1;
static const uint8_t VERT_ID  = 2;
static const uint8_t HORIZ_ID = 3;
// static const uint8_t FUTURE_ID = 4;

// ===================== Modes ========================
static const bool AUTO_CONFIG_MODES = true;
static const uint8_t MODE_SERVO = 0;
static const uint8_t MODE_STEP  = 3;

#ifndef SMS_STS_MODE
#define SMS_STS_MODE 33
#endif

static const uint8_t REG_MIN_ANGLE = 9;
static const uint8_t REG_MAX_ANGLE = 11;

// ===================== Motion tuning ================
static const uint16_t DEFAULT_SCOOP_SPEED = 1200;
static const uint16_t DEFAULT_STEP_SPEED  = 1200;
static const uint16_t DEFAULT_COMBINED_HORIZ_SPEED = DEFAULT_STEP_SPEED;
static const uint16_t DEFAULT_COMBINED_SCOOP_SPEED = DEFAULT_SCOOP_SPEED;
static const uint8_t  DEFAULT_ACC         = 50;

// Sequence runner timing.
// These are simple dwell times between actions, not true motion-complete checks.
static const uint16_t SEQ_SCOOP_DELAY_MS = 900;
static const uint16_t SEQ_STEP_DELAY_MS  = 1200;
static const uint16_t SEQ_MISC_DELAY_MS  = 300;

// Scooper setup.
// Use direct raw endpoints since these are now known from testing.
// 1500 = fully closed, 2800 = fully open. Midpoint is 2150.
static const int   SCOOP_CLOSED_RAW = 1500;
static const int   SCOOP_OPEN_RAW   = 2800;
static const int   SCOOP_ZERO_RAW   = (SCOOP_CLOSED_RAW + SCOOP_OPEN_RAW) / 2;

// Vertical translation mapping.
// 360 deg = 48 mm => 4096 steps = 48 mm.
static const float VERT_MM_PER_REV = 48.0f;
static const float STEPS_PER_REV   = 4096.0f;
static const float VERT_MIN_MM     = -70.0f;
static const float VERT_MAX_MM     =  70.0f;
static const float DEFAULT_VERT_JOG_MM = 5.0f;

// Horizontal mapping is still unknown, so jog by shaft degrees for now.
static const float DEFAULT_HORIZ_JOG_DEG = 90.0f;
static const float HORIZ_MIN_DEG_EST = -2520.0f;  // about -7 turns
static const float HORIZ_MAX_DEG_EST =  2520.0f;  // about +7 turns

// ===================== Software state ===============
long vertEstimateSteps  = 0;
long horizEstimateSteps = 0;
String lineBuffer;

void handleCommand(String cmdLine);

// Forward declarations for helpers used before their definitions.
void saveEstimates();
void moveRelativeSteps(uint8_t id, long deltaSteps, uint16_t speed = DEFAULT_STEP_SPEED, uint8_t acc = DEFAULT_ACC);
void moveScoopRaw(int rawPos, uint16_t speed = DEFAULT_SCOOP_SPEED, uint8_t acc = DEFAULT_ACC);
bool moveHorizAndScoop(float totalDeg, int targetRaw, int segments = 10, uint16_t pauseMs = 120, uint16_t horizSpeed = DEFAULT_COMBINED_HORIZ_SPEED, uint16_t scoopSpeed = DEFAULT_COMBINED_SCOOP_SPEED);

// ====================================================
// Helpers
// ====================================================
float stepsToDeg(long steps) {
  return (steps * 360.0f) / STEPS_PER_REV;
}

long degToSteps(float deg) {
  return lroundf((deg / 360.0f) * STEPS_PER_REV);
}

float stepsToVertMm(long steps) {
  return (steps * VERT_MM_PER_REV) / STEPS_PER_REV;
}

long vertMmToSteps(float mm) {
  return lroundf((mm / VERT_MM_PER_REV) * STEPS_PER_REV);
}

int clampRaw4095(int raw) {
  if (raw < 0) return 0;
  if (raw > 4095) return 4095;
  return raw;
}

String lowerTrimmed(String s) {
  s.trim();
  s.toLowerCase();
  return s;
}

String firstToken(String s) {
  s.trim();
  int space = s.indexOf(' ');
  if (space < 0) return s;
  return s.substring(0, space);
}

String restTokens(String s) {
  s.trim();
  int space = s.indexOf(' ');
  if (space < 0) return "";
  return s.substring(space + 1);
}

String secondToken(String s) {
  s = restTokens(s);
  s.trim();
  int space = s.indexOf(' ');
  if (space < 0) return s;
  return s.substring(0, space);
}

int splitTokens(String s, String tokens[], int maxTokens) {
  s.trim();
  int count = 0;
  while (s.length() > 0 && count < maxTokens) {
    int space = s.indexOf(' ');
    if (space < 0) {
      tokens[count++] = s;
      break;
    }
    String tok = s.substring(0, space);
    if (tok.length() > 0) {
      tokens[count++] = tok;
    }
    s = s.substring(space + 1);
    s.trim();
  }
  return count;
}

float firstArgAsFloat(String s, float defaultValue) {
  s = restTokens(s);
  s.trim();
  if (s.length() == 0) return defaultValue;
  int space = s.indexOf(' ');
  if (space >= 0) s = s.substring(0, space);
  return s.toFloat();
}

void saveEstimates() {
  prefs.putLong("vertSteps", vertEstimateSteps);
  prefs.putLong("horizSteps", horizEstimateSteps);
}

void loadEstimates() {
  vertEstimateSteps  = prefs.getLong("vertSteps", 0);
  horizEstimateSteps = prefs.getLong("horizSteps", 0);
}

bool pingServo(uint8_t id) {
  return st.Ping(id) == id;
}

void printServoQuick(uint8_t id, const char* name) {
  int pos  = st.ReadPos(id);
  int spd  = st.ReadSpeed(id);
  int load = st.ReadLoad(id);
  int volt = st.ReadVoltage(id);
  int temp = st.ReadTemper(id);
  int mode = st.ReadMode(id);

  Serial.print(name);
  Serial.print(" (ID ");
  Serial.print(id);
  Serial.print(")  ping=");
  Serial.print(pingServo(id) ? "OK" : "FAIL");
  Serial.print("  mode=");
  Serial.print(mode);
  Serial.print("  pos=");
  Serial.print(pos);
  Serial.print("  speed=");
  Serial.print(spd);
  Serial.print("  load=");
  Serial.print(load);
  Serial.print("  volt=");
  Serial.print(volt);
  Serial.print("  temp=");
  Serial.println(temp);
}

bool setServoMode(uint8_t id) {
  int ok = 0;
  ok += st.unLockEprom(id);
  ok += st.writeWord(id, REG_MIN_ANGLE, 0);
  ok += st.writeWord(id, REG_MAX_ANGLE, 4095);
  ok += st.writeByte(id, SMS_STS_MODE, MODE_SERVO);
  ok += st.LockEprom(id);
  delay(50);
  return ok >= 0;
}

bool setStepMode(uint8_t id) {
  int ok = 0;
  ok += st.unLockEprom(id);
  ok += st.writeWord(id, REG_MIN_ANGLE, 0);
  ok += st.writeWord(id, REG_MAX_ANGLE, 0);
  ok += st.writeByte(id, SMS_STS_MODE, MODE_STEP);
  ok += st.LockEprom(id);
  delay(50);
  return ok >= 0;
}

void configureDefaultModes() {
  Serial.println("Configuring modes...");
  setServoMode(SCOOP_ID);
  setStepMode(VERT_ID);
  setStepMode(HORIZ_ID);
  Serial.println("Mode configuration done.");
}

void moveScoopRaw(int rawPos, uint16_t speed, uint8_t acc) {
  rawPos = clampRaw4095(rawPos);
  st.WritePosEx(SCOOP_ID, rawPos, speed, acc);

  Serial.print("Scooper target raw = ");
  Serial.println(rawPos);
}

void moveScoopDeg(float deg, uint16_t speed = DEFAULT_SCOOP_SPEED, uint8_t acc = DEFAULT_ACC) {
  int raw = SCOOP_ZERO_RAW + degToSteps(deg);
  moveScoopRaw(raw, speed, acc);

  Serial.print("Scooper target deg from midpoint ~= ");
  Serial.println(deg, 2);
}


void dumpWiggle(int cycles = 3, int rawA = 2400, int rawB = 2000, uint16_t pauseMs = 250) {
  rawA = clampRaw4095(rawA);
  rawB = clampRaw4095(rawB);
  if (cycles < 1) cycles = 1;

  Serial.print("Dump wiggle: cycles=");
  Serial.print(cycles);
  Serial.print("  rawA=");
  Serial.print(rawA);
  Serial.print("  rawB=");
  Serial.print(rawB);
  Serial.print("  pause=");
  Serial.println(pauseMs);

  moveScoopRaw(rawA);
  delay(pauseMs);

  for (int i = 0; i < cycles; i++) {
    moveScoopRaw(rawB);
    delay(pauseMs);
    moveScoopRaw(rawA);
    delay(pauseMs);
  }
}

bool moveHorizAndScoop(float totalDeg, int targetRaw, int segments, uint16_t pauseMs, uint16_t horizSpeed, uint16_t scoopSpeed) {
  if (segments < 1) segments = 1;
  targetRaw = clampRaw4095(targetRaw);

  long totalSteps = degToSteps(totalDeg);
  long finalEstimate = horizEstimateSteps + totalSteps;
  if (!withinHorizLimits(finalEstimate)) {
    Serial.println("Blocked: combined horizontal move would exceed software limits.");
    return false;
  }

  int startRaw = st.ReadPos(SCOOP_ID);
  if (startRaw < 0 || startRaw > 4095) {
    startRaw = SCOOP_ZERO_RAW;
    Serial.println("Warning: could not read current scoop raw, using midpoint estimate.");
  }

  Serial.print("Combined move: horiz ");
  Serial.print(totalDeg, 2);
  Serial.print(" deg while scoop goes ");
  Serial.print(startRaw);
  Serial.print(" -> ");
  Serial.print(targetRaw);
  Serial.print(" in ");
  Serial.print(segments);
  Serial.print(" segments, pause ");
  Serial.print(pauseMs);
  Serial.print(" ms, horizSpeed ");
  Serial.print(horizSpeed);
  Serial.print(", scoopSpeed ");
  Serial.println(scoopSpeed);

  long sentSteps = 0;
  for (int i = 0; i < segments; i++) {
    long stepsThisSeg = (totalSteps * (long)(i + 1)) / segments - sentSteps;
    sentSteps += stepsThisSeg;

    int rawThisSeg = startRaw + (int)(((long)(targetRaw - startRaw) * (long)(i + 1)) / segments);

    moveRelativeSteps(HORIZ_ID, stepsThisSeg, horizSpeed, DEFAULT_ACC);
    moveScoopRaw(rawThisSeg, scoopSpeed, DEFAULT_ACC);
    delay(pauseMs);
  }

  horizEstimateSteps = finalEstimate;
  saveEstimates();

  Serial.print("Estimated horizontal shaft angle = ");
  Serial.print(stepsToDeg(horizEstimateSteps), 2);
  Serial.println(" deg");
  return true;
}

bool withinVertLimits(long newEstimateSteps) {
  float mm = stepsToVertMm(newEstimateSteps);
  return mm >= VERT_MIN_MM && mm <= VERT_MAX_MM;
}

bool withinHorizLimits(long newEstimateSteps) {
  float deg = stepsToDeg(newEstimateSteps);
  return deg >= HORIZ_MIN_DEG_EST && deg <= HORIZ_MAX_DEG_EST;
}

void moveRelativeSteps(uint8_t id, long deltaSteps, uint16_t speed, uint8_t acc) {
  st.WritePosEx(id, (int16_t)deltaSteps, speed, acc);
}

void moveVertMm(float deltaMm) {
  long deltaSteps = vertMmToSteps(deltaMm);
  long newEstimate = vertEstimateSteps + deltaSteps;

  if (!withinVertLimits(newEstimate)) {
    Serial.println("Blocked: vertical move would exceed software limits.");
    return;
  }

  moveRelativeSteps(VERT_ID, deltaSteps);
  vertEstimateSteps = newEstimate;
  saveEstimates();

  Serial.print("Vertical move command: ");
  Serial.print(deltaMm, 2);
  Serial.print(" mm  (steps ");
  Serial.print(deltaSteps);
  Serial.println(")");

  Serial.print("Estimated vertical position = ");
  Serial.print(stepsToVertMm(vertEstimateSteps), 2);
  Serial.println(" mm");
}

void moveHorizDeg(float deltaDeg) {
  long deltaSteps = degToSteps(deltaDeg);
  long newEstimate = horizEstimateSteps + deltaSteps;

  if (!withinHorizLimits(newEstimate)) {
    Serial.println("Blocked: horizontal move would exceed software limits.");
    return;
  }

  moveRelativeSteps(HORIZ_ID, deltaSteps);
  horizEstimateSteps = newEstimate;
  saveEstimates();

  Serial.print("Horizontal move command: ");
  Serial.print(deltaDeg, 2);
  Serial.print(" deg  (steps ");
  Serial.print(deltaSteps);
  Serial.println(")");

  Serial.print("Estimated horizontal shaft angle = ");
  Serial.print(stepsToDeg(horizEstimateSteps), 2);
  Serial.println(" deg");
}

void stopAxis(uint8_t id) {
  st.WritePosEx(id, 0, 0, 0);
}

void setTorqueTarget(String which, bool enable) {
  which = lowerTrimmed(which);

  if (which == "all" || which.length() == 0) {
    st.EnableTorque(SCOOP_ID, enable ? 1 : 0);
    st.EnableTorque(VERT_ID, enable ? 1 : 0);
    st.EnableTorque(HORIZ_ID, enable ? 1 : 0);
    Serial.println(enable ? "Torque ON for all" : "Torque OFF for all");
    return;
  }

  if (which == "scoop") {
    st.EnableTorque(SCOOP_ID, enable ? 1 : 0);
    Serial.println(enable ? "Torque ON for scoop" : "Torque OFF for scoop");
    return;
  }
  if (which == "vert") {
    st.EnableTorque(VERT_ID, enable ? 1 : 0);
    Serial.println(enable ? "Torque ON for vert" : "Torque OFF for vert");
    return;
  }
  if (which == "horiz") {
    st.EnableTorque(HORIZ_ID, enable ? 1 : 0);
    Serial.println(enable ? "Torque ON for horiz" : "Torque OFF for horiz");
    return;
  }

  Serial.println("Unknown torque target. Use: all, scoop, vert, horiz");
}

void zeroEstimate(String which) {
  which = lowerTrimmed(which);

  if (which == "vert") {
    vertEstimateSteps = 0;
    saveEstimates();
    Serial.println("Vertical software estimate reset to 0.");
    return;
  }

  if (which == "horiz") {
    horizEstimateSteps = 0;
    saveEstimates();
    Serial.println("Horizontal software estimate reset to 0.");
    return;
  }

  if (which == "all") {
    vertEstimateSteps = 0;
    horizEstimateSteps = 0;
    saveEstimates();
    Serial.println("All software estimates reset to 0.");
    return;
  }

  Serial.println("Use: zero vert | zero horiz | zero all");
}

void setVerticalEstimateMm(float mm) {
  long steps = vertMmToSteps(mm);
  if (!withinVertLimits(steps)) {
    Serial.println("Blocked: setvert target is outside software limits.");
    return;
  }
  vertEstimateSteps = steps;
  saveEstimates();
  Serial.print("Vertical software estimate set to ");
  Serial.print(mm, 2);
  Serial.println(" mm");
}

void setHorizontalEstimateDeg(float deg) {
  long steps = degToSteps(deg);
  if (!withinHorizLimits(steps)) {
    Serial.println("Blocked: sethoriz target is outside software limits.");
    return;
  }
  horizEstimateSteps = steps;
  saveEstimates();
  Serial.print("Horizontal software estimate set to ");
  Serial.print(deg, 2);
  Serial.println(" deg");
}

void printPing() {
  Serial.print("SCOOP: "); Serial.println(pingServo(SCOOP_ID) ? "OK" : "FAIL");
  Serial.print("VERT : "); Serial.println(pingServo(VERT_ID)  ? "OK" : "FAIL");
  Serial.print("HORIZ: "); Serial.println(pingServo(HORIZ_ID) ? "OK" : "FAIL");
}

void printStatus() {
  Serial.println();
  printServoQuick(SCOOP_ID, "SCOOP");
  printServoQuick(VERT_ID,  "VERT ");
  printServoQuick(HORIZ_ID, "HORIZ");

  int scoopRaw = st.ReadPos(SCOOP_ID);
  float scoopDeg = stepsToDeg((long)scoopRaw - SCOOP_ZERO_RAW);

  Serial.print("Scooper inferred angle from midpoint = ");
  Serial.print(scoopDeg, 2);
  Serial.println(" deg");

  Serial.print("Scooper presets: closed=");
  Serial.print(SCOOP_CLOSED_RAW);
  Serial.print("  open=");
  Serial.print(SCOOP_OPEN_RAW);
  Serial.print("  midpoint=");
  Serial.println(SCOOP_ZERO_RAW);

  Serial.print("Estimated vertical software position = ");
  Serial.print(stepsToVertMm(vertEstimateSteps), 2);
  Serial.println(" mm");

  Serial.print("Estimated horizontal software position = ");
  Serial.print(stepsToDeg(horizEstimateSteps), 2);
  Serial.println(" deg");
  Serial.println();
}

uint16_t estimateDelayForAction(const String &cmdLower) {
  if (cmdLower == "scoop" || cmdLower == "release" || cmdLower == "scoopdeg" || cmdLower == "scoopraw") {
    return SEQ_SCOOP_DELAY_MS;
  }
  if (cmdLower == "up" || cmdLower == "down" || cmdLower == "left" || cmdLower == "right" ||
      cmdLower == "vertraw" || cmdLower == "horizraw") {
    return SEQ_STEP_DELAY_MS;
  }
  if (cmdLower == "dumpwiggle" || cmdLower == "rightdump" || cmdLower == "leftdump") {
    return 150;
  }
  return SEQ_MISC_DELAY_MS;
}

bool commandNeedsValue(const String &cmdLower) {
  return cmdLower == "up" || cmdLower == "down" || cmdLower == "left" || cmdLower == "right" ||
         cmdLower == "scoopdeg" || cmdLower == "scoopraw" || cmdLower == "vertraw" || cmdLower == "horizraw";
}

bool isRunCommandWord(const String &cmdLower) {
  return cmdLower == "scoop" || cmdLower == "release" || cmdLower == "ping" || cmdLower == "status" ||
         cmdLower == "init" || cmdLower == "pause" || cmdLower == "wait" ||
         cmdLower == "up" || cmdLower == "down" || cmdLower == "left" || cmdLower == "right" ||
         cmdLower == "scoopdeg" || cmdLower == "scoopraw" || cmdLower == "vertraw" || cmdLower == "horizraw" ||
         cmdLower == "dumpwiggle" || cmdLower == "rightdump" || cmdLower == "leftdump";
}

bool isNumericToken(String s) {
  s.trim();
  if (s.length() == 0) return false;
  char buf[32];
  s.toCharArray(buf, sizeof(buf));
  char *endPtr = nullptr;
  strtod(buf, &endPtr);
  if (endPtr == buf) return false;
  while (*endPtr == ' ' || *endPtr == '	') endPtr++;
  return *endPtr == '';
}

void runSequence(String scriptLine) {
  scriptLine = lowerTrimmed(scriptLine);
  if (scriptLine.length() == 0) {
    Serial.println("Run sequence is empty.");
    return;
  }

  const int MAX_TOKENS = 64;
  String tokens[MAX_TOKENS];
  int count = splitTokens(scriptLine, tokens, MAX_TOKENS);

  Serial.println("Starting sequence...");

  int i = 0;
  while (i < count) {
    String cmd = lowerTrimmed(tokens[i]);
    String built = cmd;

    if (cmd == "pause" || cmd == "wait") {
      if (i + 1 >= count) {
        Serial.println("Sequence error: pause/wait needs milliseconds.");
        return;
      }
      long ms = lroundf(tokens[i + 1].toFloat());
      if (ms < 0) ms = 0;
      Serial.print("> ");
      Serial.println(built + " " + tokens[i + 1]);
      delay((uint32_t)ms);
      i += 2;
      continue;
    }

    if (commandNeedsValue(cmd)) {
      if (i + 1 >= count) {
        Serial.print("Sequence error: command needs value -> ");
        Serial.println(cmd);
        return;
      }
      built += " ";
      built += tokens[i + 1];
      Serial.print("> ");
      Serial.println(built);
      handleCommand(built);
      delay(estimateDelayForAction(cmd));
      i += 2;
      continue;
    }

    if (cmd == "dumpwiggle") {
      int j = i + 1;
      int extraCount = 0;
      while (j < count && extraCount < 4 && isNumericToken(tokens[j])) {
        built += " ";
        built += tokens[j];
        j++;
        extraCount++;
      }
      Serial.print("> ");
      Serial.println(built);
      handleCommand(built);
      delay(estimateDelayForAction(cmd));
      i = j;
      continue;
    }

    if (cmd == "rightdump" || cmd == "leftdump") {
      if (i + 1 >= count || !isNumericToken(tokens[i + 1])) {
        Serial.print("Sequence error: ");
        Serial.print(cmd);
        Serial.println(" needs at least <deg>.");
        return;
      }

      int j = i + 1;
      int extraCount = 0;
      while (j < count && extraCount < 6 && isNumericToken(tokens[j])) {
        built += " ";
        built += tokens[j];
        j++;
        extraCount++;
      }

      Serial.print("> ");
      Serial.println(built);
      handleCommand(built);
      delay(estimateDelayForAction(cmd));
      i = j;
      continue;
    }

    if (cmd == "scoop" || cmd == "release" || cmd == "ping" || cmd == "status" || cmd == "init") {
      Serial.print("> ");
      Serial.println(built);
      handleCommand(built);
      delay(estimateDelayForAction(cmd));
      i += 1;
      continue;
    }

    Serial.print("Sequence error: unsupported token -> ");
    Serial.println(cmd);
    Serial.println("Supported in run: scoop, release, up, down, left, right, scoopdeg, scoopraw, vertraw, horizraw, dumpwiggle, rightdump, leftdump, pause, wait, ping, status, init");
    return;
  }

  Serial.println("Sequence complete.");
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  help");
  Serial.println("  ping");
  Serial.println("  status");
  Serial.println("  init               -> set scoop=servo mode, vert/horiz=step mode");
  Serial.println("  scoop              -> close scooper to preset raw endpoint");
  Serial.println("  release            -> open scooper to preset raw endpoint");
  Serial.println("  scoopdeg <deg>");
  Serial.println("  scoopraw <0-4095>");
  Serial.println("  up [mm]            -> default 5 mm");
  Serial.println("  down [mm]          -> default 5 mm");
  Serial.println("  left [deg]         -> default 90 deg of servo shaft");
  Serial.println("  right [deg]        -> default 90 deg of servo shaft");
  Serial.println("  vertraw <steps>    -> relative vertical move in raw steps");
  Serial.println("  horizraw <steps>   -> relative horizontal move in raw steps");
  Serial.println("  stop all | stop vert | stop horiz");
  Serial.println("  zero vert | zero horiz | zero all");
  Serial.println("  setvert <mm>       -> set vertical software estimate only");
  Serial.println("  sethoriz <deg>     -> set horizontal software estimate only");
  Serial.println("  torque on [all|scoop|vert|horiz]");
  Serial.println("  torque off [all|scoop|vert|horiz]");
  Serial.println("  dumpwiggle [cycles] [rawA] [rawB] [pauseMs]");
  Serial.println("     example: dumpwiggle 4 2400 2000 250");
  Serial.println("  rightdump <deg> [targetRaw] [segments] [pauseMs] [horizSpeed] [scoopSpeed]");
  Serial.println("     example: rightdump 200 2400 10 120 1400 500");
  Serial.println("  leftdump <deg> [targetRaw] [segments] [pauseMs] [horizSpeed] [scoopSpeed]");
  Serial.println("     example: leftdump 200 2400 10 120 1400 500");
  Serial.println("  run <sequence>     -> execute chained commands with automatic waits");
  Serial.println("     example: run up 40 release down 60 scoop up 60 right 200");
  Serial.println("     example: run up 80 right 300 rightdump 200 2400 12 120 1500 500");
  Serial.println("     example: run up 80 right 300 dumpwiggle 4 2400 2000 250");
  Serial.println();
  Serial.println("Notes:");
  Serial.println("  - Scooper uses absolute position mode.");
  Serial.println("  - Vertical and horizontal are software-estimated positions for now.");
  Serial.println("  - Estimates survive ESP32 resets, but can drift if the belt skips or you move things by hand.");
  Serial.println("  - run uses simple fixed delays between actions. Add pause <ms> if an axis needs more time.");
  Serial.println();
}

void handleCommand(String cmdLine) {
  cmdLine.trim();
  if (cmdLine.length() == 0) return;

  String cmd = lowerTrimmed(firstToken(cmdLine));

  if (cmd == "help") {
    printHelp();
    return;
  }

  if (cmd == "ping") {
    printPing();
    return;
  }

  if (cmd == "status") {
    printStatus();
    return;
  }

  if (cmd == "init") {
    configureDefaultModes();
    return;
  }

  if (cmd == "scoop") {
    moveScoopRaw(SCOOP_CLOSED_RAW);
    return;
  }

  if (cmd == "release") {
    moveScoopRaw(SCOOP_OPEN_RAW);
    return;
  }

  if (cmd == "scoopdeg") {
    float deg = firstArgAsFloat(cmdLine, 0.0f);
    moveScoopDeg(deg);
    return;
  }

  if (cmd == "scoopraw") {
    int raw = (int)firstArgAsFloat(cmdLine, (float)SCOOP_ZERO_RAW);
    moveScoopRaw(raw);
    return;
  }
  if (cmd == "dumpwiggle") {
    String tokens[8];
    int n = splitTokens(cmdLine, tokens, 8);
    int cycles = (n >= 2) ? (int)lroundf(tokens[1].toFloat()) : 3;
    int rawA   = (n >= 3) ? (int)lroundf(tokens[2].toFloat()) : 2400;
    int rawB   = (n >= 4) ? (int)lroundf(tokens[3].toFloat()) : 2000;
    int pauseMs = (n >= 5) ? (int)lroundf(tokens[4].toFloat()) : 250;
    dumpWiggle(cycles, rawA, rawB, (uint16_t)pauseMs);
    return;
  }

  if (cmd == "rightdump" || cmd == "leftdump") {
    String tokens[10];
    int n = splitTokens(cmdLine, tokens, 10);
    float totalDeg   = (n >= 2) ? tokens[1].toFloat() : 200.0f;
    int targetRaw    = (n >= 3) ? (int)lroundf(tokens[2].toFloat()) : 2400;
    int segments     = (n >= 4) ? (int)lroundf(tokens[3].toFloat()) : 10;
    int pauseMs      = (n >= 5) ? (int)lroundf(tokens[4].toFloat()) : 120;
    int horizSpeed   = (n >= 6) ? (int)lroundf(tokens[5].toFloat()) : DEFAULT_COMBINED_HORIZ_SPEED;
    int scoopSpeed   = (n >= 7) ? (int)lroundf(tokens[6].toFloat()) : DEFAULT_COMBINED_SCOOP_SPEED;
    if (cmd == "leftdump") totalDeg = -fabsf(totalDeg);
    else totalDeg = fabsf(totalDeg);
    moveHorizAndScoop(totalDeg, targetRaw, segments, (uint16_t)pauseMs, (uint16_t)horizSpeed, (uint16_t)scoopSpeed);
    return;
  }


  if (cmd == "up") {
    float mm = firstArgAsFloat(cmdLine, DEFAULT_VERT_JOG_MM);
    moveVertMm(+mm);
    return;
  }

  if (cmd == "down") {
    float mm = firstArgAsFloat(cmdLine, DEFAULT_VERT_JOG_MM);
    moveVertMm(-mm);
    return;
  }

  if (cmd == "left") {
    float deg = firstArgAsFloat(cmdLine, DEFAULT_HORIZ_JOG_DEG);
    moveHorizDeg(-deg);
    return;
  }

  if (cmd == "right") {
    float deg = firstArgAsFloat(cmdLine, DEFAULT_HORIZ_JOG_DEG);
    moveHorizDeg(+deg);
    return;
  }

  if (cmd == "vertraw") {
    long steps = lroundf(firstArgAsFloat(cmdLine, 0.0f));
    long newEstimate = vertEstimateSteps + steps;
    if (!withinVertLimits(newEstimate)) {
      Serial.println("Blocked: vertical move would exceed software limits.");
      return;
    }
    moveRelativeSteps(VERT_ID, steps);
    vertEstimateSteps = newEstimate;
    saveEstimates();
    Serial.print("Vertical raw relative steps = ");
    Serial.println(steps);
    return;
  }

  if (cmd == "horizraw") {
    long steps = lroundf(firstArgAsFloat(cmdLine, 0.0f));
    long newEstimate = horizEstimateSteps + steps;
    if (!withinHorizLimits(newEstimate)) {
      Serial.println("Blocked: horizontal move would exceed software limits.");
      return;
    }
    moveRelativeSteps(HORIZ_ID, steps);
    horizEstimateSteps = newEstimate;
    saveEstimates();
    Serial.print("Horizontal raw relative steps = ");
    Serial.println(steps);
    return;
  }

  if (cmd == "stop") {
    String which = lowerTrimmed(secondToken(cmdLine));
    if (which == "vert") {
      stopAxis(VERT_ID);
      Serial.println("Stop sent to vertical axis.");
      return;
    }
    if (which == "horiz") {
      stopAxis(HORIZ_ID);
      Serial.println("Stop sent to horizontal axis.");
      return;
    }
    if (which == "all" || which.length() == 0) {
      stopAxis(VERT_ID);
      stopAxis(HORIZ_ID);
      Serial.println("Stop sent to all step axes.");
      return;
    }
    Serial.println("Use: stop all | stop vert | stop horiz");
    return;
  }

  if (cmd == "zero") {
    zeroEstimate(secondToken(cmdLine));
    return;
  }

  if (cmd == "setvert") {
    setVerticalEstimateMm(firstArgAsFloat(cmdLine, 0.0f));
    return;
  }

  if (cmd == "sethoriz") {
    setHorizontalEstimateDeg(firstArgAsFloat(cmdLine, 0.0f));
    return;
  }

  if (cmd == "torque") {
    String mode = lowerTrimmed(secondToken(cmdLine));
    String rest = restTokens(restTokens(cmdLine));
    if (mode == "on") {
      setTorqueTarget(rest, true);
      return;
    }
    if (mode == "off") {
      setTorqueTarget(rest, false);
      return;
    }
    Serial.println("Use: torque on [all|scoop|vert|horiz]");
    Serial.println("     torque off [all|scoop|vert|horiz]");
    return;
  }

  if (cmd == "run") {
    runSequence(restTokens(cmdLine));
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(cmdLine);
  Serial.println("Type 'help' for command list.");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  ServoBus.begin(SERVO_BAUD, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &ServoBus;

  prefs.begin("payloaddbg", false);
  loadEstimates();

  delay(300);
  Serial.println();
  Serial.println("ST3215 payload debug console");
  Serial.println("Type 'help' for commands.");

  if (AUTO_CONFIG_MODES) {
    configureDefaultModes();
  }

  printPing();
  printStatus();
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleCommand(lineBuffer);
      lineBuffer = "";
    } else {
      lineBuffer += c;
    }
  }
}