#include <Servo.h>

// ##biblioteki

const int NUM_STEPPERS = 6;
const int NUM_JOINTS = 7;
const int SERVO_JOINT_INDEX = 6;

// ##konfiguracja_robota

const int STEP_PINS[NUM_STEPPERS] = {2, 4, 6, 8, 10, 12};
const int DIR_PINS[NUM_STEPPERS]  = {3, 5, 7, 9, 11, 13};
const int SERVO_PIN = A0;

// ##piny

const float STEPS_PER_REV[NUM_STEPPERS] = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0};
const float MICROSTEPS[NUM_STEPPERS] = {16.0, 16.0, 16.0, 16.0, 16.0, 16.0};
const float GEAR_RATIO[NUM_STEPPERS] = {1.0, 5.0, 1.0, 1.0, 1.0, 1.0};
const bool INVERT_DIR[NUM_STEPPERS] = {false, false, false, false, false, false};

const float MIN_ANGLE_STEPPER[NUM_STEPPERS] = {-160.0, -90.0, -120.0, -180.0, -120.0, -180.0};
const float MAX_ANGLE_STEPPER[NUM_STEPPERS] = { 160.0,  90.0,  120.0,  180.0,  120.0,  180.0};

const float ANGLE_OFFSET[NUM_STEPPERS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

const int SERVO_MIN_ANGLE = 10;
const int SERVO_MAX_ANGLE = 170;
const int SERVO_START_ANGLE = 90;

// ##mechanika

unsigned long stepIntervalUs[NUM_STEPPERS] = {1200, 1200, 1200, 1000, 1000, 1000};
const unsigned int STEP_PULSE_US = 4;

// ##ruch

float target_angles[NUM_JOINTS] = {0, 0, 0, 0, 0, 0, SERVO_START_ANGLE};
long target_steps[NUM_STEPPERS] = {0, 0, 0, 0, 0, 0};
long current_steps[NUM_STEPPERS] = {0, 0, 0, 0, 0, 0};

unsigned long lastStepMicros[NUM_STEPPERS] = {0, 0, 0, 0, 0, 0};

String serialBuffer = "";
bool lineReady = false;

Servo gripperServo;
int currentServoAngle = SERVO_START_ANGLE;

// ##dane

long angleToSteps(float angle, int motor_id) {
  float correctedAngle = angle + ANGLE_OFFSET[motor_id];
  float stepsPerDegree = (STEPS_PER_REV[motor_id] * MICROSTEPS[motor_id] * GEAR_RATIO[motor_id]) / 360.0;
  return lround(correctedAngle * stepsPerDegree);
}

// ##angle_to_steps

void updateServo() {
  int servoTarget = constrain((int)lround(target_angles[SERVO_JOINT_INDEX]), SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  if (servoTarget != currentServoAngle) {
    currentServoAngle = servoTarget;
    gripperServo.write(currentServoAngle);
  }
}

// ##update_servo

void updateSteppers() {
  unsigned long now = micros();

  for (int i = 0; i < NUM_STEPPERS; i++) {
    long delta = target_steps[i] - current_steps[i];

    if (delta == 0) {
      continue;
    }

    if (now - lastStepMicros[i] < stepIntervalUs[i]) {
      continue;
    }

    bool dir = (delta > 0);

    if (INVERT_DIR[i]) {
      dir = !dir;
    }

    digitalWrite(DIR_PINS[i], dir ? HIGH : LOW);
    digitalWrite(STEP_PINS[i], HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PINS[i], LOW);

    if (delta > 0) {
      current_steps[i]++;
    } else {
      current_steps[i]--;
    }

    lastStepMicros[i] = now;
  }
}

// ##update_steppers

void parseLine(String line) {
  line.trim();

  if (!line.startsWith("A,")) {
    return;
  }

  float parsed[NUM_JOINTS];
  int parsedCount = 0;
  int startIndex = 2;

  while (parsedCount < NUM_JOINTS && startIndex < line.length()) {
    int commaIndex = line.indexOf(',', startIndex);
    String token;

    if (commaIndex == -1) {
      token = line.substring(startIndex);
      startIndex = line.length();
    } else {
      token = line.substring(startIndex, commaIndex);
      startIndex = commaIndex + 1;
    }

    token.trim();

    if (token.length() == 0) {
      return;
    }

    bool validNumber = true;

    for (int i = 0; i < token.length(); i++) {
      char c = token.charAt(i);
      if (!isDigit(c) && c != '-' && c != '+' && c != '.') {
        validNumber = false;
        break;
      }
    }

    if (!validNumber) {
      return;
    }

    parsed[parsedCount] = token.toFloat();
    parsedCount++;
  }

  if (parsedCount != NUM_JOINTS) {
    return;
  }

  for (int i = 0; i < NUM_STEPPERS; i++) {
    float safeAngle = constrain(parsed[i], MIN_ANGLE_STEPPER[i], MAX_ANGLE_STEPPER[i]);
    target_angles[i] = safeAngle;
    target_steps[i] = angleToSteps(safeAngle, i);
  }

  target_angles[SERVO_JOINT_INDEX] = constrain(parsed[SERVO_JOINT_INDEX], SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
}

// ##parse_line

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_STEPPERS; i++) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    digitalWrite(STEP_PINS[i], LOW);
    digitalWrite(DIR_PINS[i], LOW);
  }

  gripperServo.attach(SERVO_PIN);
  gripperServo.write(SERVO_START_ANGLE);

  for (int i = 0; i < NUM_STEPPERS; i++) {
    current_steps[i] = 0;
    target_steps[i] = 0;
    lastStepMicros[i] = micros();
  }
}

// ##setup

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      lineReady = true;
      break;
    }

    if (serialBuffer.length() < 120) {
      serialBuffer += c;
    } else {
      serialBuffer = "";
    }
  }

  if (lineReady) {
    parseLine(serialBuffer);
    serialBuffer = "";
    lineReady = false;
  }

  updateSteppers();
  updateServo();
}

// ##loop