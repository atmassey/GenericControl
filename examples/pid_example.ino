#include "GenericControl.h"

// Pin definitions
const int analogInPin = A0;  // Analog input pin (sensor reading)
const int analogOutPin = 9;  // Analog output pin (PWM output)

// Controller parameters
double kp = 2.0;        // Proportional gain
double ki = 0.1;        // Integral gain
double kd = 0.5;        // Derivative gain
double setpoint = 512;  // Target value (0-1023 for analog input)

// Time tracking
unsigned long lastTime = 0;
unsigned long sampleTime = 50;  // Sample time in milliseconds

// Create a PID controller instance
PIDController pidController(kp, ki, kd, setpoint);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Configure controller
  pidController.setOutputLimits(0, 255);  // PWM range 0-255
  pidController.setIntegralLimits(-100, 100);  // Anti-windup limits
  
  // Print header for serial plotter
  Serial.println("Setpoint,Input,Output");
}

void loop() {
  // Check if it's time for a new control calculation
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastTime >= sampleTime) {
    // Convert time to seconds for the controller
    double currentTimeSeconds = currentMillis / 1000.0;
    
    // Read the input
    int sensorValue = analogRead(analogInPin);
    
    // Change setpoint every 10 seconds for testing
    if ((currentMillis / 10000) % 2 == 0) {
      pidController.setSetpoint(300);
    } else {
      pidController.setSetpoint(700);
    }
    
    // Calculate control output
    double output = pidController.calculate(sensorValue, currentTimeSeconds);
    
    // Apply the control output to the system
    analogWrite(analogOutPin, (int)output);
    
    // Print values for serial plotter
    Serial.print(pidController.getSetpoint());
    Serial.print(",");
    Serial.print(sensorValue);
    Serial.print(",");
    Serial.println(output);
    
    // Save current time for next iteration
    lastTime = currentMillis;
  }
}