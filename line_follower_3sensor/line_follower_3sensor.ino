#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

// ============================================================
// 1. PIN CONFIGURATION
// ============================================================

// 3x TCRT5000: left, center, right.
// These reuse the three middle BFD-1000 pins (35, 32, 33) so the
// physical positions match the old array. GPIO 34 and 25 are now unused.
const uint8_t NUM_SENSORS = 3;
const uint8_t SENSOR_PINS[NUM_SENSORS] = {
  35, 32, 33
};

// AS5600 is not used in this version. GPIO 21 and 22 remain unused.

// MG996R
const uint8_t SERVO_PIN = 18;

// IBT-2 / BTS7960
const uint8_t RPWM_PIN = 19;
const uint8_t LPWM_PIN = 23;
const uint8_t R_EN_PIN = 16;
const uint8_t L_EN_PIN = 17;

// Buttons
const uint8_t START_PIN = 27;
const uint8_t STOP_PIN = 14;
const uint8_t RESET_PIN = 13;
const uint8_t ESTOP_PIN = 4;


// ============================================================
// 2. MOTOR PWM CONFIGURATION
// ============================================================

const uint32_t MOTOR_PWM_FREQUENCY = 20000;
const uint8_t MOTOR_PWM_RESOLUTION = 10;
const int PWM_LIMIT = 700;

// Initial open-loop PWM settings; calibrate on the actual robot.
const int BASE_MOTOR_PWM = 300;
const int MIN_CURVE_PWM = 180;

// PWM ramp rates (PWM units per second).
const float ACCELERATION_PWM_PER_SEC = 500.0f;
const float DECELERATION_PWM_PER_SEC = 700.0f;

float targetMotorPWM = 0.0f;
float rampedMotorPWM = 0.0f;
float currentMotorPWM = 0.0f;


// ============================================================
// 3. LINE FOLLOWING CONFIGURATION
// ============================================================

// Level the sensor outputs when it is over the black line.
// Most TCRT5000 modules (LM393 comparator board) output HIGH over a
// dark surface and LOW over a reflective (white) surface.
// If the robot steers away from the line, or the Sensors: debug print
// shows the opposite of what you expect, flip this to LOW.
const uint8_t LINE_DETECTED_LEVEL = HIGH;

// Sensor positions, left to right.
const float SENSOR_WEIGHTS[NUM_SENSORS] = {
  -1, 0, 1
};

// Largest possible |error| (used to normalise the curve factor).
const float MAX_ERROR = 1.0f;

// Steering servo calibration.
const int SERVO_CENTER = 120;
const int SERVO_LEFT_LIMIT = 180;
const int SERVO_RIGHT_LIMIT = 60;

// Reverse this if steering moves in the wrong direction.
const int SERVO_DIRECTION = 1;

// Steering PID.
// Error now spans -1..+1 (was -2..+2), so KP is doubled to keep the
// same steering authority. Tune on the actual robot.
float STEERING_KP = 50.0f;
float STEERING_KI = 0.0f;
float STEERING_KD = 0.0f;

int servoCommandAngle = SERVO_CENTER;

float lineError = 0.0f;
float previousLineError = 0.0f;
float steeringIntegral = 0.0f;

bool lineDetected = false;

uint32_t lastLineSeenMs = 0;
const uint32_t LINE_LOST_TIMEOUT_MS = 150;


// ============================================================
// 4. CONTROL TIMING
// ============================================================

const uint32_t CONTROL_INTERVAL_MS = 10;

uint32_t lastControlMs = 0;
uint32_t lastDebugMs = 0;


// ============================================================
// 5. ROBOT STATE
// ============================================================

enum RobotState {
  IDLE,
  RUNNING,
  STOPPING,
  FAULT
};

RobotState robotState = IDLE;

bool emergencyLatched = false;

Servo steeringServo;


// ============================================================
// 6. BUTTON DEBOUNCING
// ============================================================

struct Button {
  uint8_t pin;

  bool stableState = HIGH;
  bool lastReading = HIGH;

  uint32_t lastChangeMs = 0;

  void begin(uint8_t gpio) {
    pin = gpio;
    pinMode(pin, INPUT_PULLUP);

    stableState = digitalRead(pin);
    lastReading = stableState;

    lastChangeMs = millis();
  }

  bool pressed() {
    bool reading = digitalRead(pin);

    if (reading != lastReading) {
      lastReading = reading;
      lastChangeMs = millis();
    }

    if (millis() - lastChangeMs >= 30) {
      if (reading != stableState) {
        stableState = reading;

        if (stableState == LOW) {
          return true;
        }
      }
    }

    return false;
  }
};

Button startButton;
Button stopButton;
Button resetButton;


// ============================================================
// 7. MOTOR DRIVER
// ============================================================

void motorDisable() {

  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, 0);

  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);

  currentMotorPWM = 0;
}

void motorEnable() {

  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
}

void applyMotorPWM(float pwm) {

  int duty = constrain(
    (int)pwm,
    0,
    PWM_LIMIT);

  if (duty > 0) {
    motorEnable();
  }

  // Forward-only propulsion.
  ledcWrite(LPWM_PIN, 0);
  ledcWrite(RPWM_PIN, duty);

  currentMotorPWM = duty;
}


// ============================================================
// 8. OPEN-LOOP MOTOR PWM RAMP
// ============================================================

void updateMotorRamp(float dt) {
  float difference = targetMotorPWM - rampedMotorPWM;

  if (difference > 0.0f) {
    float change = ACCELERATION_PWM_PER_SEC * dt;
    rampedMotorPWM += fminf(difference, change);
  } else if (difference < 0.0f) {
    float change = DECELERATION_PWM_PER_SEC * dt;
    rampedMotorPWM -= fminf(-difference, change);
  }

  if (rampedMotorPWM < 0.0f) {
    rampedMotorPWM = 0.0f;
  }

  rampedMotorPWM = constrain(rampedMotorPWM, 0.0f, (float)PWM_LIMIT);
}


// ============================================================
// 9. TCRT5000 LINE POSITION (3 SENSORS)
// ============================================================

// Possible results with 3 sensors:
//   L only          -> -1.0   (line far left)
//   L + C           -> -0.5
//   C only          ->  0.0   (centered)
//   C + R           -> +0.5
//   R only          -> +1.0   (line far right)
//   L + R (no C)    ->  0.0   (unusual; treated as centered)
//   none            -> line lost
bool readLinePosition(float &position) {

  float weightedSum = 0.0f;
  float activeCount = 0.0f;

  for (int i = 0; i < NUM_SENSORS; i++) {

    bool detected =
      (digitalRead(SENSOR_PINS[i])
       == LINE_DETECTED_LEVEL);

    if (detected) {

      weightedSum += SENSOR_WEIGHTS[i];
      activeCount += 1.0f;
    }
  }

  if (activeCount == 0) {
    return false;
  }

  position = weightedSum / activeCount;

  return true;
}


// ============================================================
// 10. STEERING PID
// ============================================================

void updateSteering(float error, float dt) {

  steeringIntegral += error * dt;

  steeringIntegral = constrain(
    steeringIntegral,
    -2.0f,
    2.0f);

  float derivative =
    (error - previousLineError) / dt;

  float correction =
    STEERING_KP * error
    + STEERING_KI * steeringIntegral
    + STEERING_KD * derivative;

  int servoAngle =
    SERVO_CENTER
    + SERVO_DIRECTION * (int)correction;

  servoAngle = constrain(
    servoAngle,
    SERVO_RIGHT_LIMIT,
    SERVO_LEFT_LIMIT);

  servoCommandAngle = servoAngle;
  steeringServo.write(servoCommandAngle);


  previousLineError = error;
}


// ============================================================
// 11. CURVE PWM REDUCTION
// ============================================================

float calculateCurvePWM(float error) {
  float curveFactor = fabsf(error) / MAX_ERROR;
  curveFactor = constrain(curveFactor, 0.0f, 1.0f);

  float pwm = BASE_MOTOR_PWM * (1.0f - 0.40f * curveFactor);

  return constrain(pwm, (float)MIN_CURVE_PWM, (float)PWM_LIMIT);
}


// ============================================================
// 12. LINE FOLLOWING CONTROL
// ============================================================

void runLineFollowing(float dt) {

  float position = 0.0f;

  lineDetected = readLinePosition(position);

  if (lineDetected) {

    lineError = position;

    lastLineSeenMs = millis();

    updateSteering(lineError, dt);

    targetMotorPWM =
      calculateCurvePWM(lineError);

  } else {

    // Line lost.
    targetMotorPWM = 0.0f;

    if (millis() - lastLineSeenMs > LINE_LOST_TIMEOUT_MS) {

      robotState = STOPPING;

      Serial.println("Line lost - stopping");
    }
  }
}


// ============================================================
// 13. EMERGENCY STOP
// ============================================================

void triggerEmergencyStop() {

  emergencyLatched = true;

  robotState = FAULT;

  targetMotorPWM = 0.0f;
  rampedMotorPWM = 0.0f;

  motorDisable();

  Serial.println();
  Serial.println("!!! EMERGENCY STOP !!!");
  Serial.println("Motor driver disabled.");
}


// ============================================================
// 14. RESET
// ============================================================

void resetRobot() {

  targetMotorPWM = 0.0f;
  rampedMotorPWM = 0.0f;

  steeringIntegral = 0.0f;

  lineError = 0.0f;
  previousLineError = 0.0f;

  motorDisable();

  steeringServo.write(SERVO_CENTER);

  if (digitalRead(ESTOP_PIN) == LOW) {

    emergencyLatched = false;
    robotState = IDLE;

    Serial.println("System reset - IDLE");

  } else {

    robotState = FAULT;

    Serial.println(
      "Reset rejected: check emergency-stop circuit");
  }
}


// ============================================================
// 15. STATE MACHINE
// ============================================================

void updateStateMachine() {

  // Normally closed emergency contact:
  // LOW = healthy
  // HIGH = emergency circuit open.
  if (digitalRead(ESTOP_PIN) == HIGH) {

    if (!emergencyLatched) {
      triggerEmergencyStop();
    }
  }

  // Start
  if (startButton.pressed()) {

    if (robotState == IDLE && !emergencyLatched) {

      robotState = RUNNING;

      targetMotorPWM = BASE_MOTOR_PWM;
      rampedMotorPWM = 0.0f;

      steeringIntegral = 0.0f;

      previousLineError = 0.0f;

      lastLineSeenMs = millis();

      Serial.println("Robot started");
    }
  }

  // Controlled stop
  if (stopButton.pressed()) {

    if (robotState == RUNNING) {

      robotState = STOPPING;

      targetMotorPWM = 0.0f;

      Serial.println("Controlled stop requested");
    }
  }

  // Reset
  if (resetButton.pressed()) {

    resetRobot();
  }
}


// ============================================================
// 16. DIAGNOSTICS
// ============================================================

void printDiagnostics() {

  if (millis() - lastDebugMs < 200) {
    return;
  }

  lastDebugMs = millis();

  Serial.print("Sensors (L C R): ");

  for (int i = 0; i < NUM_SENSORS; i++) {

    Serial.print(digitalRead(SENSOR_PINS[i]));
    Serial.print(" ");
  }

  Serial.print("| Target PWM: ");
  Serial.print(targetMotorPWM, 1);

  Serial.print(" | Ramp PWM: ");
  Serial.print(rampedMotorPWM, 1);

  Serial.print(" | Applied PWM: ");
  Serial.print(currentMotorPWM, 1);

  Serial.print(" | Error: ");
  Serial.print(lineError, 2);

  Serial.print(" | Servo angle: ");
  Serial.print(servoCommandAngle);

  Serial.print(" | State: ");

  switch (robotState) {

    case IDLE:
      Serial.println("IDLE");
      break;

    case RUNNING:
      Serial.println("RUNNING");
      break;

    case STOPPING:
      Serial.println("STOPPING");
      break;

    case FAULT:
      Serial.println("FAULT");
      break;
  }
}


// ============================================================
// 17. SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(500);

  Serial.println();
  Serial.println("====================================");
  Serial.println("ESP32 ACKERMANN LINE FOLLOWER");
  Serial.println("3x TCRT5000 + IBT-2 (OPEN-LOOP PWM)");
  Serial.println("====================================");

  // Servo.
  steeringServo.setPeriodHertz(50);

  steeringServo.attach(
    SERVO_PIN,
    500,
    2500);

  steeringServo.write(SERVO_CENTER);

  delay(500);

  // TCRT5000 inputs.
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  // Buttons.
  startButton.begin(START_PIN);
  stopButton.begin(STOP_PIN);
  resetButton.begin(RESET_PIN);

  // Emergency stop.
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  // AS5600 is not initialized or read in this version.

  // IBT-2.
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);

  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);

  bool pwmR = ledcAttach(
    RPWM_PIN,
    MOTOR_PWM_FREQUENCY,
    MOTOR_PWM_RESOLUTION);

  bool pwmL = ledcAttach(
    LPWM_PIN,
    MOTOR_PWM_FREQUENCY,
    MOTOR_PWM_RESOLUTION);

  if (!pwmR || !pwmL) {

    Serial.println("ERROR: PWM initialization failed.");

    while (true) {
      digitalWrite(R_EN_PIN, LOW);
      digitalWrite(L_EN_PIN, LOW);
      delay(100);
    }
  }

  motorDisable();

  // Check emergency circuit.
  if (digitalRead(ESTOP_PIN) == HIGH) {

    emergencyLatched = true;
    robotState = FAULT;

    Serial.println("Emergency circuit open.");
  }

  lastControlMs = millis();

  lastLineSeenMs = millis();

  Serial.println("Initialization complete.");

  if (robotState == IDLE) {
    Serial.println("Ready. Press START.");
  } else {
    Serial.println("FAULT: resolve issue and press RESET.");
  }
}


// ============================================================
// 18. MAIN LOOP
// ============================================================

void loop() {

  updateStateMachine();

  // Emergency condition is checked again before motor control.
  if (digitalRead(ESTOP_PIN) == HIGH) {
    if (!emergencyLatched) {
      triggerEmergencyStop();
    }
  }

  uint32_t now = millis();

  if (now - lastControlMs >= CONTROL_INTERVAL_MS) {

    float dt = (now - lastControlMs) / 1000.0f;

    lastControlMs = now;

    if (robotState == RUNNING) {

      runLineFollowing(dt);
    }

    if (robotState == RUNNING || robotState == STOPPING) {

      updateMotorRamp(dt);
      applyMotorPWM(rampedMotorPWM);

      // Once the commanded PWM reaches zero, disable the driver.
      if (rampedMotorPWM <= 0.5f) {

        if (robotState == STOPPING) {

          motorDisable();

          targetMotorPWM = 0.0f;
          rampedMotorPWM = 0.0f;

          steeringServo.write(SERVO_CENTER);

          robotState = IDLE;

          Serial.println("Robot stopped - IDLE");
        }
      }
    }

    else {

      motorDisable();
    }
  }

  printDiagnostics();
}
