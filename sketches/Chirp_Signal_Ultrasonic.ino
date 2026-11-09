
#include <Servo.h>
#include <math.h>

// Pin Locations
const int trigPin = 9;
const int echoPin = A1;
const int servoPin = 3;

// Variables
Servo servo;
long duration;
float distance;

// Chirp Signal Parameters
const float f0 = 0.1;          // Starting frequency (Hz)
const float f1 = 5.0;          // Ending frequency (Hz)
const float T_chirp = 20.0;    // Chirp duration (seconds)

const float amplitude = 45;  // Servo swing (+/- degrees)
const float centerAngle = 90;

const float servoMin = 40.0;
const float servoMax = 180.0;

// Timing
unsigned long startMillis;
unsigned long lastMillis;

const unsigned long samplePeriod = 20;  // 50 Hz target sampling

// --------------------------------
// Ultrasonic Distance Measurement
// --------------------------------
float getDistance() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000UL);

  // No valid echo received
  if (duration == 0) {
    return -1.0;
  }

  float d = duration * 0.0343 / 2.0;

  // Reject measurements outside sensor range
  if (d < 2.0 || d > 50.0) {
    return -1.0;
  }

  return d;
}

// --------------------------------
// Setup
// --------------------------------
void setup() {

  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  servo.attach(servoPin);

  servo.write(centerAngle);

  Serial.println("time_ms,servo_deg,dist_cm");

  delay(50);

  startMillis = millis();
  lastMillis = startMillis;
}

// --------------------------------
// Main Loop
// --------------------------------
void loop() {

  unsigned long now = millis();

  if (now - lastMillis < samplePeriod) {
    return;
  }

  lastMillis = now;

  // Elapsed time in seconds
  float t = (now - startMillis) / 1000.0;

  // Chirp frequency rate
  float k = (f1 - f0) / T_chirp;

  // Stop after chirp duration
  if (t >= T_chirp) {

    servo.write(centerAngle);

    return;
  }

  // Calculate chirp phase
  float phi = 2.0 * PI *
              (f0 * t + 0.5 * k * t * t);

  // Calculate servo command
  float servoCmd = centerAngle +
                   amplitude * sin(phi);

  // Apply servo limits
  servoCmd = constrain(servoCmd, servoMin, servoMax);

  // Move servo
  servo.write(servoCmd);

  // Measure distance
  float dist = getDistance();

  // Print CSV data
  Serial.print(now - startMillis);
  Serial.print(",");
  Serial.print(servoCmd, 2);
  Serial.print(",");
  Serial.println(dist, 2);
}
