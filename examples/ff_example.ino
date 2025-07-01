#include <GenericControl.h>

// Pin definitions for Arduino
const int SENSOR_PIN = A0;      // Analog input pin for process variable
const int ACTUATOR_PIN = 9;     // PWM output pin for control signal

// System parameters
const double SETPOINT = 50.0;   // Target setpoint (0-100 scale)
const double SAMPLE_TIME = 0.1; // Sample time in seconds (100ms)

// PID controller parameters
double kp = 2.0;       // Proportional gain
double ki = 0.1;       // Integral gain
double kd = 0.5;       // Derivative gain

// Feed forward controller parameters
double ff_gain = 0.5;  // Feed forward gain
double ff_offset = 10.0; // Feed forward offset (baseline control signal)

// Controllers
PIDController pidController(kp, ki, kd, SETPOINT);
FeedForwardController ffController(ff_gain, ff_offset);

// Time tracking variables
unsigned long lastTime = 0;
double currentTime = 0.0;

// Process simulation variables (for testing without real hardware)
double processValue = 25.0;  // Initial process value
bool useSimulation = true;   // Set to false when using with real hardware

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Configure PID controller
  pidController.setOutputLimits(0.0, 100.0);      // Control output range
  pidController.setIntegralLimits(-20.0, 20.0);   // Anti-windup limits
  
  // Configure feed forward controller
  ffController.setRampRate(5.0);  // Limit rate of change to 5 units per second
  
  // Set pin modes
  pinMode(SENSOR_PIN, INPUT);
  pinMode(ACTUATOR_PIN, OUTPUT);
  
  // Print header for data logging
  Serial.println("Time,Setpoint,ProcessValue,PIDOutput,FFOutput,TotalOutput");
}

void loop() {
  // Get current time
  unsigned long now = millis();
  currentTime = now / 1000.0;  // Convert to seconds
  
  // Check if it's time to calculate new control output
  if (now - lastTime >= SAMPLE_TIME * 1000) {
    
    // Read process value from sensor or simulation
    if (!useSimulation) {
      // Read from actual sensor
      processValue = analogRead(SENSOR_PIN);
      // Scale to 0-100 range (assuming 0-1023 ADC range)
      processValue = map(processValue, 0, 1023, 0, 100);
    }
    
    // Calculate PID control output
    double pidOutput = pidController.calculate(processValue, currentTime);
    
    // Calculate feed forward output based on setpoint
    double ffOutput = ffController.calculate(pidController.getSetpoint(), currentTime);
    
    // Combine outputs (with optional limiting)
    double totalOutput = pidOutput + ffOutput;
    totalOutput = constrain(totalOutput, 0, 100);  // Ensure total is within bounds
    
    // Apply control output to actuator or simulation
    if (!useSimulation) {
      // Convert to PWM range (0-255) and write to output
      int pwmOutput = map(totalOutput, 0, 100, 0, 255);
      analogWrite(ACTUATOR_PIN, pwmOutput);
    } else {
      // Update simulated process (simple first-order system)
      double deltaTime = (now - lastTime) / 1000.0;
      double processGain = 1.0;
      double processTimeConstant = 5.0;
      
      // Calculate process derivative: dPV/dt = (Gain*MV - PV)/TimeConstant
      double derivative = (processGain * totalOutput - processValue) / processTimeConstant;
      processValue += derivative * deltaTime;
      
      // Add some noise for realism
      processValue += random(-1, 2) * 0.1;
    }
    
    // Log data to serial
    Serial.print(currentTime, 2);
    Serial.print(",");
    Serial.print(pidController.getSetpoint());
    Serial.print(",");
    Serial.print(processValue);
    Serial.print(",");
    Serial.print(pidOutput);
    Serial.print(",");
    Serial.print(ffOutput);
    Serial.print(",");
    Serial.println(totalOutput);
    
    // Change setpoint halfway through test for demonstration
    if (currentTime > 30.0 && currentTime < 30.1) {
      pidController.setSetpoint(80.0);
    } else if (currentTime > 60.0 && currentTime < 60.1) {
      pidController.setSetpoint(30.0);
    }
    
    // Save current time for next iteration
    lastTime = now;
  }
}