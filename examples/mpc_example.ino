#include <GenericControl.h>

// Time settings
const unsigned long SAMPLE_TIME_MS = 50;  // 50ms sample time
const double SAMPLE_TIME_SEC = SAMPLE_TIME_MS / 1000.0;

// MPC controller parameters
const int PREDICTION_HORIZON = 20;    // Number of steps to predict ahead
const int CONTROL_HORIZON = 5;        // Number of control moves to optimize
const double PROCESS_GAIN = 1.5;      // Process model gain
const double TIME_CONSTANT = 8.0;     // Process model time constant
const double DEAD_TIME = 0.0;         // Process model dead time

// Controller constraints
const double OUTPUT_MIN = 0.0;       // Minimum control output
const double OUTPUT_MAX = 100.0;     // Maximum control output
const double RATE_MIN = -10.0;       // Maximum decrease per second
const double RATE_MAX = 10.0;        // Maximum increase per second

// Controller weights
const double SETPOINT_WEIGHT = 1.0;  // Weight for setpoint tracking error
const double CONTROL_WEIGHT = 0.1;   // Weight for control effort
const double RATE_WEIGHT = 0.5;      // Weight for control rate changes

// Setpoint and disturbance
double setpoint = 50.0;              // Target value
double disturbance = 0.0;            // External disturbance

// Simulated process
double processValue = 25.0;         // Current process value

// Controllers for comparison
ModelPredictiveController mpcController(
    PREDICTION_HORIZON, CONTROL_HORIZON, 
    PROCESS_GAIN, TIME_CONSTANT, DEAD_TIME
);

PIDController pidController(2.0, 0.1, 1.0, setpoint); // PID for comparison

// Time tracking
unsigned long lastMillis = 0;
unsigned long startMillis = 0;
unsigned long simulationStep = 0;
const int MAX_STEPS = 1000;         // Maximum simulation steps

// Store future predictions for plotting
double predictedOutputs[30];       // Buffer for storing predictions
double plannedInputs[10];          // Buffer for storing planned inputs

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Configure MPC controller
  mpcController.setSetpoint(setpoint);
  mpcController.setConstraints(OUTPUT_MIN, OUTPUT_MAX, RATE_MIN, RATE_MAX);
  mpcController.setWeights(SETPOINT_WEIGHT, CONTROL_WEIGHT, RATE_WEIGHT);
  
  // Configure PID controller for comparison
  pidController.setOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  pidController.setIntegralLimits(-20.0, 20.0);
  
  // Print CSV header for plotting
  Serial.println("Step,Time,Setpoint,Process,MPC_Output,PID_Output,Disturbance,P1,P2,P3,P4,P5,P6,P7,P8,P9,P10");
  
  startMillis = millis();
}

void loop() {
  // Get current time
  unsigned long currentMillis = millis();
  double currentTimeSeconds = (currentMillis - startMillis) / 1000.0;
  
  // Calculate at fixed intervals
  if (currentMillis - lastMillis >= SAMPLE_TIME_MS && simulationStep < MAX_STEPS) {
    // Introduce setpoint changes
    if (simulationStep == 200) {
      setpoint = 75.0;
      mpcController.setSetpoint(setpoint);
      pidController.setSetpoint(setpoint);
    } else if (simulationStep == 500) {
      setpoint = 30.0;
      mpcController.setSetpoint(setpoint);
      pidController.setSetpoint(setpoint);
    }
    
    // Introduce disturbances
    if (simulationStep == 300) {
      disturbance = 15.0;  // Step disturbance
    } else if (simulationStep == 700) {
      disturbance = -10.0; // Another disturbance
    }
    
    // Calculate MPC control output
    double mpcOutput = mpcController.calculate(processValue, currentTimeSeconds);
    
    // Calculate PID control output for comparison
    double pidOutput = pidController.calculate(processValue, currentTimeSeconds);
    
    // Get future predictions from MPC
    const double* futureOutputs = mpcController.getFutureOutputs();
    const double* futureInputs = mpcController.getFutureInputs();
    
    // Copy predictions to local buffer for plotting (first 10 steps)
    for (int i = 0; i < 10; i++) {
      if (i < PREDICTION_HORIZON) {
        predictedOutputs[i] = futureOutputs[i];
      } else {
        predictedOutputs[i] = predictedOutputs[PREDICTION_HORIZON - 1];
      }
    }
    
    // Update simulated process with MPC output (first-order system with disturbance)
    double processDerivative = (PROCESS_GAIN * mpcOutput - processValue + disturbance) / TIME_CONSTANT;
    processValue += processDerivative * SAMPLE_TIME_SEC;
    
    // Add small random noise for realism
    processValue += random(-10, 11) * 0.05;
    
    // Output data for plotting
    Serial.print(simulationStep);
    Serial.print(",");
    Serial.print(currentTimeSeconds, 2);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(processValue);
    Serial.print(",");
    Serial.print(mpcOutput);
    Serial.print(",");
    Serial.print(pidOutput);
    Serial.print(",");
    Serial.print(disturbance);
    
    // Print first 10 predictions for visualization
    for (int i = 0; i < 10; i++) {
      Serial.print(",");
      Serial.print(predictedOutputs[i]);
    }
    Serial.println();
    
    // Save timing information
    lastMillis = currentMillis;
    simulationStep++;
  }
  
  // Small delay to avoid flooding the serial port
  delay(10);
  
  // If simulation is complete, pause
  if (simulationStep >= MAX_STEPS) {
    delay(1000);  // 1-second pause at the end
  }
}