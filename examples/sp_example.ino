#include <GenericControl.h>

// Time settings
const unsigned long SAMPLE_TIME_MS = 100;  // 100ms sample time (10Hz)
const double SAMPLE_TIME_SEC = SAMPLE_TIME_MS / 1000.0;

// Process parameters - these represent a slow process with significant delay
const double PROCESS_GAIN = 1.0;       // Process gain
const double TIME_CONSTANT = 10.0;     // Process time constant (seconds)
const double DEAD_TIME = 5.0;          // Process dead time (seconds)

// Controller settings
const double KP = 0.8;                 // Proportional gain
const double KI = 0.05;                // Integral gain
const double KD = 0.2;                 // Derivative gain
const double SETPOINT = 50.0;          // Target setpoint

// Controller objects for comparison
SmithPredictor smithPredictor(KP, KI, KD, PROCESS_GAIN, TIME_CONSTANT, DEAD_TIME);
PIDController standardPID(KP, KI, KD, SETPOINT);

// Process simulation variables
double processValue = 0.0;            // Current process value for Smith Predictor
double pidProcessValue = 0.0;         // Current process value for standard PID

// Delay buffer for simulating process dead time
const int DELAY_BUFFER_SIZE = 100;    // Sized to handle maximum expected delay
double delayBuffer[DELAY_BUFFER_SIZE];
int delayIndex = 0;
double delaySum = 0.0;                // For moving average filter

// Time tracking
unsigned long lastMillis = 0;
unsigned long startMillis = 0;
unsigned long simulationStep = 0;
const int MAX_STEPS = 600;            // Run for 60 seconds (at 10Hz)

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Configure controllers
  smithPredictor.setSetpoint(SETPOINT);
  smithPredictor.setOutputLimits(0.0, 100.0);
  
  standardPID.setSetpoint(SETPOINT);
  standardPID.setOutputLimits(0.0, 100.0);
  standardPID.setIntegralLimits(-20.0, 20.0);
  
  // Initialize delay buffer
  for (int i = 0; i < DELAY_BUFFER_SIZE; i++) {
    delayBuffer[i] = 0.0;
  }
  
  // Print CSV header for plotting
  Serial.println("Step,Time,Setpoint,SmithOutput,PIDOutput,ProcessValue,PIDProcessValue,ModelOutput,DelayedModelOutput");
  
  startMillis = millis();
}

void loop() {
  // Get current time
  unsigned long currentMillis = millis();
  double currentTimeSeconds = (currentMillis - startMillis) / 1000.0;
  
  // Calculate at fixed intervals
  if ((currentMillis - lastMillis >= SAMPLE_TIME_MS) && (simulationStep < MAX_STEPS)) {
    // Apply setpoint changes to test controllers
    if (simulationStep == 100) {  // At t=10s
      smithPredictor.setSetpoint(75.0);
      standardPID.setSetpoint(75.0);
    } else if (simulationStep == 300) {  // At t=30s
      smithPredictor.setSetpoint(25.0);
      standardPID.setSetpoint(25.0);
    }
    
    // Calculate control outputs
    double smithOutput = smithPredictor.calculate(processValue, currentTimeSeconds);
    double pidOutput = standardPID.calculate(pidProcessValue, currentTimeSeconds);
    
    // Simulate process response - first without delay to get immediate output
    double processDerivative = (PROCESS_GAIN * smithOutput - delayBuffer[delayIndex]) / TIME_CONSTANT;
    double newValue = delayBuffer[delayIndex] + processDerivative * SAMPLE_TIME_SEC;
    
    // Add small noise for realism
    newValue += random(-10, 11) * 0.05;
    
    // Update delay buffer (FIFO)
    delaySum -= delayBuffer[delayIndex];
    delayBuffer[delayIndex] = newValue;
    delaySum += newValue;
    
    // Update index for circular buffer
    delayIndex = (delayIndex + 1) % DELAY_BUFFER_SIZE;
    
    // Get delayed output (simulates dead time in the process)
    // For simplicity, we're using a fixed point delay that's closest to our desired dead time
    int delaySteps = round(DEAD_TIME / SAMPLE_TIME_SEC);
    int delayedIndex = (delayIndex + DELAY_BUFFER_SIZE - delaySteps) % DELAY_BUFFER_SIZE;
    processValue = delayBuffer[delayedIndex];
    
    // Simulate standard PID process (same plant but with separate state)
    double pidProcessDerivative = (PROCESS_GAIN * pidOutput - pidProcessValue) / TIME_CONSTANT;
    pidProcessValue += pidProcessDerivative * SAMPLE_TIME_SEC;
    
    // Delay the PID process to simulate dead time
    // In real application, both controllers would see the same process, but for comparison we keep them separate
    if (simulationStep >= delaySteps) {
      // Add noise for realism
      pidProcessValue += random(-10, 11) * 0.05;
    }
    
    // Print results for plotting
    Serial.print(simulationStep);
    Serial.print(",");
    Serial.print(currentTimeSeconds, 2);
    Serial.print(",");
    Serial.print(smithPredictor.setSetpoint);
    Serial.print(",");
    Serial.print(smithOutput);
    Serial.print(",");
    Serial.print(pidOutput);
    Serial.print(",");
    Serial.print(processValue);
    Serial.print(",");
    Serial.print(pidProcessValue);
    Serial.print(",");
    Serial.print(smithPredictor.getModelOutput());
    Serial.print(",");
    Serial.println(smithPredictor.getDelayedModelOutput());
    
    // Save timing information
    lastMillis = currentMillis;
    simulationStep++;
  }
  
  // Add small delay to avoid flooding the serial connection
  delay(10);
  
  // If simulation complete, pause
  if (simulationStep >= MAX_STEPS) {
    delay(1000);
  }
}