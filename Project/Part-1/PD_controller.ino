#include <Servo.h>

// --- Pin Definitions ---
const int IR_PIN = A1;
const int SERVO_PIN = 3;

// --- Mechanical & Calibration Constants ---
const float SERVO_FLAT_ANGLE = 99.0; // Servo angle (deg) where beam is horizontal (alpha = 0)
const float MAX_TILT_DEG = 35.0;     // Safety saturation limit (+/- deg from flat)
const int DIRECTION = 1;             // Set to -1 if servo rotation direction is inverted

// --- Controller Gains & Reference (from Task 3 Part A) ---
// Set 1: Initial Design (Kp = 3.0, Td = 0.817s -> Kd = 2.45)
const float Kp = 3.0;
const float Kd = 2.45;
// Set 2: Low-Overshoot Tuned (uncomment to test Kp = 1.0, Kd = 7.0 from page 8)
// const float Kp = 1.0;
// const float Kd = 7.0;

const float setpoint = 0.10;         // Reference position r(t) = 0.10 m (10 cm)

// --- Timing & Filter Variables ---
const unsigned long SAMPLE_TIME_MS = 20; // 50 Hz control loop (dt = 0.02 s)
unsigned long lastTime = 0;
float filteredDistance = 0.10;
float prevError = 0.0;
float filteredDerivative = 0.0;

const float ALPHA_DIST = 0.25;       // EMA filter for position reading
const float ALPHA_DERIV = 0.20;      // Low-pass filter for derivative term (prevents IR noise spikes)

Servo beamServo;

float readDistanceMeters() {
  int adc = analogRead(IR_PIN);
  if (adc < 40) adc = 40;
  float dist_cm = 29.988 * pow(adc * (5.0 / 1023.0), -1.173);
  return constrain(dist_cm / 100.0, 0.02, 0.45);
}

void setup() {
  Serial.begin(115200);
  beamServo.attach(SERVO_PIN);
  beamServo.write((int)SERVO_FLAT_ANGLE);
  filteredDistance = readDistanceMeters();
  prevError = setpoint - filteredDistance;
  delay(1000);
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime >= SAMPLE_TIME_MS) {
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // 1. Read and filter ball position 'a' (in meters)
    float rawDistance = readDistanceMeters();
    filteredDistance = (ALPHA_DIST * rawDistance) + ((1.0 - ALPHA_DIST) * filteredDistance);

    // 2. Compute error and filtered derivative
    float error = setpoint - filteredDistance;
    float rawDerivative = (error - prevError) / dt;
    filteredDerivative = (ALPHA_DERIV * rawDerivative) + ((1.0 - ALPHA_DERIV) * filteredDerivative);
    prevError = error;

    // 3. PD Control Law: u(t) = Kp*e(t) + Kd*(de/dt) [in radians]
    float theta_rad = (Kp * error) + (Kd * filteredDerivative);

    // 4. Convert radians to degrees and apply servo limits
    float theta_deg = theta_rad * (180.0 / PI);
    float servoCommand = SERVO_FLAT_ANGLE + (DIRECTION * theta_deg);
    servoCommand = constrain(servoCommand, SERVO_FLAT_ANGLE - MAX_TILT_DEG, SERVO_FLAT_ANGLE + MAX_TILT_DEG);

    beamServo.write((int)servoCommand);

    // 5. Log data for Serial Plotter
    Serial.print("Setpoint_cm:"); Serial.print(setpoint * 100.0); Serial.print(",");
    Serial.print("Position_cm:"); Serial.print(filteredDistance * 100.0); Serial.print(",");
    Serial.print("Servo_deg:");   Serial.println(servoCommand);
  }
}
