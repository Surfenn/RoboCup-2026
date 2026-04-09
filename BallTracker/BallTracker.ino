#include "IRRing.h"
#include "RobotCompass.h"
#include "RobotMotors.h"
#include <Arduino.h>
#include <math.h>

// Instantiate the subsystems
IRRing irRing;
RobotCompass compass;
RobotMotors robot;

float initialHeading = 0.0;
bool isCalibrated = false;

// ============================================================
//  ---- TEST MODE ----
//  Set to 0 for normal ball-tracking.
//  Set to 1 for ANGLE-ONLY mode (prints ball angle, NO movement).
//  Set to 2 for MOTOR TEST (drives N, E, S, W in sequence, no IR).
//  Set to 3 for CONSTANT EAST DRIVE (drives 90 deg constantly for tuning).
// ============================================================
const int TEST_MODE = 0;

// ============================================================
//  ---- SPEED & POWER TUNING ----
//  basePursueSpeed: The target PWM power for chasing the ball.
//    - Value Range: 0 to 255.
//    - Current Value: 45 (Safe indoor walk-speed).
//    - Competition: 150-200 (Aggressive play).
//    - Performance Note: At speeds > 150, you may need to reduce 
//      kP_Rotation (e.g. 0.4) to maintain smooth movement.
// ============================================================

int basePursueSpeed = 45;
const float kP_Rotation = 0.7f;
const float HEADING_DEADZONE = 7.0f;
const float ANGLE_SMOOTH_ALPHA = 0.35f;
const float MIN_IR_STRENGTH = 35.0f;
const float MAX_ANGLE_JUMP = 120.0f;

// ============================================================

// Persistent angle smoother state
float smoothX = 1.0f;
float smoothY = 0.0f;

void setup() {
  Serial.begin(115200);
  delay(1000);

  irRing.begin();
  robot.begin();

  if (!compass.begin()) {
    Serial.println("Ooops, no BNO055 detected!");
  } else {
    Serial.println("Compass settling...");
    delay(400);
    initialHeading = compass.getHeading();
    isCalibrated = true;
  }

  Serial.print("Goal Heading Locked at: ");
  Serial.println(initialHeading);

  if (TEST_MODE == 1) {
    Serial.println("=== ANGLE-ONLY MODE: Robot will NOT move. ===");
    Serial.println("Move the ball around and check angles in Serial Monitor.");
  } else if (TEST_MODE == 2) {
    Serial.println("=== MOTOR TEST MODE ===");
    Serial.println("Robot will drive in 4 directions at low speed.");
    Serial.println("Watch which PHYSICAL direction it moves for each label.");
    delay(2000);
  } else if (TEST_MODE == 3) {
    Serial.println("=== CONSTANT EAST DRIVE MODE ===");
    Serial.println("Robot will drive at angle 90 (East) constantly.");
    delay(1000);
  }
}

// ============================================================
// MOTOR TEST MODE (TEST_MODE == 2)
// Drives in each cardinal direction for 2 seconds.
// YOU observe which physical direction the robot moves and report back.
// ============================================================
void motorTestLoop() {
  struct TestStep {
    const char *label;
    float angle;
  };

  // These are the angles we INTEND to drive.
  // Report which PHYSICAL direction the robot actually moves.
  TestStep steps[] = {
      {"NORTH (Forward)", 0},
      {"EAST  (Right)", 90},
      {"SOUTH (Back)", 180},
      {"WEST  (Left)", 270},
  };

  for (int i = 0; i < 4; i++) {
    Serial.print(">>> Driving: ");
    Serial.print(steps[i].label);
    Serial.print(" (angle=");
    Serial.print(steps[i].angle);
    Serial.println(")");

    unsigned long start = millis();
    while (millis() - start < 2000) {
      robot.drive(steps[i].angle, 40, 0); // speed=40, no turning
      delay(15);
    }

    // Stop for 1.5 seconds between each direction
    robot.brake();
    Serial.println("--- STOPPED ---");
    delay(1500);
  }

  Serial.println("=== MOTOR TEST COMPLETE ===");
  Serial.println(
      "Please report which physical direction the robot moved for each label.");
  Serial.println("Example: 'NORTH command -> robot moved WEST'");

  // Halt forever after test
  robot.brake();
  while (true) {
    delay(1000);
  }
}

// ============================================================
// ANGLE-ONLY MODE (TEST_MODE == 1)
// Prints ball angle + MaxIdx but does NOT drive motors.
// ============================================================
void angleOnlyLoop() {
  irRing.update();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    if (irRing.isBallDetected()) {
      Serial.print("Ball:");
      Serial.print(irRing.getBallAngle(), 0);
      Serial.print(" Str:");
      Serial.print(irRing.getBallStrength(), 0);
      Serial.print(" MaxIdx:");
      Serial.println(irRing.getStrongestSensor());
    } else {
      Serial.println("No ball detected");
    }
  }
}

// ============================================================
// NORMAL BALL-TRACKING MODE (TEST_MODE == 0)
// ============================================================
void normalLoop() {
  irRing.update();

  // ---- 1. Compass Heading Hold ----
  float turnRate = 0;
  if (isCalibrated) {
    float currentHeading = compass.getHeading();
    float error = initialHeading - currentHeading;
    if (error > 180.0f)
      error -= 360.0f;
    if (error < -180.0f)
      error += 360.0f;

    if (fabsf(error) > HEADING_DEADZONE) {
      turnRate = kP_Rotation * error;
      turnRate = constrain(turnRate, -80, 80);
    }
  }

  // ---- 2. Ball Tracking ----
  float driveAngle = 0;
  float activeSpeed = 0;
  static int detectionStreak = 0;

  if (irRing.isBallDetected()) {
    float rawAngle = irRing.getBallAngle();
    float rawStrength = irRing.getBallStrength();

    if (rawStrength >= MIN_IR_STRENGTH) {
      float currentSmoothAngle = atan2f(smoothY, smoothX) * (180.0f / PI);
      if (currentSmoothAngle < 0)
        currentSmoothAngle += 360.0f;
      float diff = fabsf(rawAngle - currentSmoothAngle);
      if (diff > 180.0f)
        diff = 360.0f - diff;

      if (diff < MAX_ANGLE_JUMP || rawStrength > 150.0f) {
        float rawRad = rawAngle * (PI / 180.0f);
        smoothX = (1.0f - ANGLE_SMOOTH_ALPHA) * smoothX +
                  ANGLE_SMOOTH_ALPHA * cosf(rawRad);
        smoothY = (1.0f - ANGLE_SMOOTH_ALPHA) * smoothY +
                  ANGLE_SMOOTH_ALPHA * sinf(rawRad);
      }
    }

    detectionStreak++;

    float ballAngle = atan2f(smoothY, smoothX) * (180.0f / PI);
    if (ballAngle < 0)
      ballAngle += 360.0f;

    if (detectionStreak >= 2) {
      activeSpeed = basePursueSpeed;
    }

    // Adaptive Orbital Logic
    float angError = ballAngle;
    if (angError > 180.0f)
      angError -= 360.0f;

    float baseOffset = angError * 0.8f;
    baseOffset = constrain(baseOffset, -50.0f, 50.0f);

    float damp = 1.0f - ((rawStrength - 80.0f) / 170.0f);
    damp = constrain(damp, 0.0f, 1.0f);

    float finalOffset = baseOffset * damp;
    driveAngle = ballAngle + finalOffset;

    if (driveAngle < 0)
      driveAngle += 360.0f;
    if (driveAngle >= 360)
      driveAngle -= 360.0f;

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 50) {
      lastPrint = millis();
      Serial.print("Ball:");
      Serial.print(ballAngle, 0);
      Serial.print(" Drive:");
      Serial.print(driveAngle, 0);
      Serial.print(" Str:");
      Serial.print(rawStrength, 0);
      Serial.print(" MaxIdx:");
      Serial.print(irRing.getStrongestSensor());
      Serial.print(" Turn:");
      Serial.println(turnRate, 0);
    }

  } else {
    detectionStreak = 0;
    smoothX = smoothX * 0.80f + 1.0f * 0.20f;
    smoothY = smoothY * 0.80f;
    static unsigned long lastMissedPrint = 0;
    if (millis() - lastMissedPrint >= 200) {
      lastMissedPrint = millis();
      Serial.println("Ball -> Stale (Braking)");
    }
  }

  robot.drive(driveAngle, activeSpeed, turnRate);
  delay(15);
}

void loop() {
  if (TEST_MODE == 3) {
    robot.drive(90, 60, 0); // Constants East drive at speed 60
    delay(15);
  } else if (TEST_MODE == 2) {
    motorTestLoop();
  } else if (TEST_MODE == 1) {
    angleOnlyLoop();
  } else {
    normalLoop();
  }
}
