#include <Servo.h>

// --- Pin Definitions ---
const int IR_PIN = A1;
const int SERVO_PIN = 3;

// --- Mechanical & Calibration Constants ---
const float SERVO_FLAT_ANGLE = 99.0; // Servo angle (deg) where beam is horizontal (alpha = 0)
const float MAX_TILT_DEG = 35.0;     // Safety saturation limit (+/- deg from flat)
const int DIRECTION = 1;             // Set to -1 if servo rotation direction is inverted

// --- Controller Gains & Reference (from Task 3a) ---
const float Kp = 7.0;                // Proportional gain (rad/m)
const float setpoint = 0.30;         // Reference position r(t) = 0.10 m (10 cm)

// --- Timing & Filter Variables ---
const unsigned long SAMPLE_TIME_MS = 20; // 50 Hz control loop
unsigned long lastTime = 0;
float filteredDistance = 0.10;
const float ALPHA_DIST = 0.25;       // EMA filter weight for noisy IR sensor

Servo beamServo;

float readDistanceMeters() {
  int adc = analogRead(IR_PIN);
  if (adc < 40) adc = 40; // Prevent divide-by-zero
  // Sharp GP2Y0A21YK0F approximation in meters (adjust if using a different IR model)
  float dist_cm = 29.988 * pow(adc * (5.0 / 1023.0), -1.173);
  return constrain(dist_cm / 100.0, 0.02, 0.45);
}

void setup() {
  Serial.begin(19200);
  beamServo.attach(SERVO_PIN);
  beamServo.write((int)SERVO_FLAT_ANGLE);
  filteredDistance = readDistanceMeters();
  delay(1000); // Wait for beam to settle
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime >= SAMPLE_TIME_MS) {
    lastTime = now;

    // 1. Read and filter ball position 'a' (in meters)
    float rawDistance = readDistanceMeters();
    filteredDistance = (ALPHA_DIST * rawDistance) + ((1.0 - ALPHA_DIST) * filteredDistance);

    // 2. Compute error e(t) = r(t) - a(t)
    float error = setpoint - filteredDistance;

    // 3. Proportional Control Law: u(t) = Kp * e(t) [in radians]
    float theta_rad = Kp * error;

    // 4. Convert radians to degrees and apply servo limits
    float theta_deg = theta_rad * (180.0 / PI);
    float servoCommand = SERVO_FLAT_ANGLE + (DIRECTION * theta_deg);
    servoCommand = constrain(servoCommand, SERVO_FLAT_ANGLE - MAX_TILT_DEG, SERVO_FLAT_ANGLE + MAX_TILT_DEG);

    beamServo.write((int)servoCommand);

    // 5. Log data for Arduino Serial Plotter (Setpoint_cm, Position_cm, Servo_deg)
    Serial.print("Setpoint_cm:"); Serial.print(setpoint * 100.0); Serial.print(",");
    Serial.print("Position_cm:"); Serial.print(filteredDistance * 100.0); Serial.print(",");
    Serial.print("Servo_deg:");   Serial.println(servoCommand);
  }
}
