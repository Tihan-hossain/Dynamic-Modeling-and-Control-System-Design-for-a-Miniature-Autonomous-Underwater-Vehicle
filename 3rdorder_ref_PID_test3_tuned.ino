#include <ESP32Servo.h>
#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;
Servo esc;

int escPin = 3;

// Target depth (in meters)
float z_target = -1.0;

// Reference model states
float z_ref = 0.0;
float v_ref = 0.0;
float a_ref = 0.0;

// Reference model parameters
float omega = 0.4;   // Natural frequency (rad/s)
float zeta = 1.2;    // Damping ratio

// PID gains (tuned)
float Kp = 85.0;
float Ki = 15.0;
float Kd = 80.0;

// Feedforward gain
float Kff = 100.0;   // Optional (set 0 to disable)

// PID controller memory
float prevError = 0;
float integral = 0;

// Time tracking
unsigned long lastTime = 0;

// Filtered depth (optional)
bool enableFilter = true;
float alpha = 0.1;  // 0.1 = smooth filter, 1.0 = raw value
float filteredDepth = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin(17, 18);

  while (!sensor.init()) {
    Serial.println("Sensor init failed!");
    delay(5000);
  }

  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(1026);  // For seawater

  esc.setPeriodHertz(50);  // ESC PWM frequency
  esc.attach(escPin, 1100, 1900);
  Serial.println("Arming ESC...");
  esc.writeMicroseconds(1500);
  delay(10000);
  Serial.println("ESC Armed. Starting 3rd-order depth tracking to -1.0 m...");

  lastTime = millis();
  filteredDepth = 0.0;

  Serial.println("Time,RefDepth,Depth,Error,PWM");
}

void loop() {
  sensor.read();
  float rawDepth = -sensor.depth();  // Make depth negative

  // Apply optional low-pass filter
  if (enableFilter) {
    filteredDepth = alpha * rawDepth + (1.0 - alpha) * filteredDepth;
  } else {
    filteredDepth = rawDepth;
  }

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // === Third-Order Reference Model ===
  float da_ref = -omega * omega * (z_ref - z_target) - 2 * zeta * omega * v_ref - a_ref;
  a_ref += da_ref * dt;
  v_ref += a_ref * dt;
  z_ref += v_ref * dt;

  // === PID + Feedforward Control (always active) ===
  float error = z_ref - filteredDepth;

  integral += error * dt;

  // Optional anti-windup
  integral = constrain(integral, -50, 50);

  float derivative = (error - prevError) / dt;
  prevError = error;

  float output = Kp * error + Ki * integral + Kd * derivative + Kff * a_ref;
  int pwmOutput = constrain(1500 + output, 1100, 1900);
  esc.writeMicroseconds(pwmOutput);

  // === Logging ===
  Serial.print(millis() / 1000.0, 2); Serial.print(",");
  Serial.print(z_ref, 3); Serial.print(",");
  Serial.print(filteredDepth, 3); Serial.print(",");
  Serial.print(error, 3); Serial.print(",");
  Serial.println(pwmOutput);

  delay(100);  // 10 Hz loop
}
