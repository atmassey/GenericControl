# GenericControl

A comprehensive C++ control systems library providing multiple advanced control algorithms for Arduino and embedded systems.

## Overview

The GenericControl library implements several popular control algorithms commonly used in industrial automation, robotics, and process control. The library is designed to be modular, efficient, and easy to integrate into existing projects.

## Features

- **PID Controller**: Classic three-term controller with anti-windup and direction control
- **Feed Forward Controller**: Predictive control with rate limiting and filtering
- **Smith Predictor**: Advanced controller for processes with significant time delays
- **Model Predictive Controller (MPC)**: Optimal control with constraints and prediction horizons
- Cross-platform compatibility (Arduino, PC, embedded systems)
- Comprehensive examples and documentation

## Installation

### Arduino IDE
1. Download the library as a ZIP file
2. In Arduino IDE: Sketch → Include Library → Add .ZIP Library
3. Select the downloaded ZIP file
4. Include in your sketch: `#include <GenericControl.h>`

### PlatformIO
Add to your `platformio.ini`:
```ini
lib_deps = 
    https://github.com/yourusername/GenericControl.git
```

### Manual Installation
1. Clone or download the repository
2. Copy the src folder contents to your project
3. Include the header: `#include "GenericControl.h"`

## Library Structure

```
GenericControl/
├── src/
│   ├── GenericControl.h      # Main header file
│   └── GenericControl.cpp    # Implementation
├── examples/
│   ├── pid_example.ino       # Basic PID controller example
│   ├── ff_example.ino        # Feed forward controller example
│   ├── sp_example.ino        # Smith predictor example
│   └── mpc_example.ino       # Model predictive controller example
├── keywords.txt              # Arduino IDE syntax highlighting
└── README.md                # This file
```

## Controllers

### PID Controller

The PID (Proportional-Integral-Derivative) controller is the most widely used controller in industrial applications.

#### Features
- Configurable gains (Kp, Ki, Kd)
- Output limiting with anti-windup
- Integral term limiting
- Direction control (direct/reverse)
- Built-in derivative kick prevention

#### Basic Usage
```cpp
#include <GenericControl.h>

// Create PID controller with gains: Kp=2.0, Ki=0.1, Kd=0.5
PIDController pid(2.0, 0.1, 0.5, 50.0); // setpoint = 50.0

void setup() {
    pid.setOutputLimits(0, 100);        // Output range 0-100%
    pid.setIntegralLimits(-20, 20);     // Anti-windup limits
}

void loop() {
    double sensorValue = analogRead(A0);
    double currentTime = millis() / 1000.0;
    
    double output = pid.calculate(sensorValue, currentTime);
    
    // Apply output to actuator
    analogWrite(9, map(output, 0, 100, 0, 255));
    
    delay(100); // 100ms sample time
}
```

#### API Reference

**Constructor:**
```cpp
PIDController(double kp, double ki, double kd, double setpoint = 0.0, bool direction = true);
```

**Methods:**
- `void setGains(double kp, double ki, double kd)` - Update PID gains
- `void setSetpoint(double setpoint)` - Set target value
- `void setOutputLimits(double min, double max)` - Set output constraints
- `void setIntegralLimits(double min, double max)` - Anti-windup protection
- `double calculate(double currentValue, double currentTime)` - Main control calculation
- `void reset()` - Reset controller state
- `double getProportionalTerm()` - Get current P term
- `double getIntegralTerm()` - Get current I term  
- `double getDerivativeTerm()` - Get current D term

### Feed Forward Controller

Feed forward control provides predictive control action based on the setpoint, improving response time and reducing steady-state error.

#### Features
- Configurable gain and offset
- Rate limiting for smooth transitions
- Low-pass filtering
- Direction control

#### Basic Usage
```cpp
#include <GenericControl.h>

FeedForwardController ff(0.5, 10.0);  // gain=0.5, offset=10.0

void setup() {
    ff.setRampRate(5.0);  // Max change rate: 5 units/second
}

void loop() {
    double setpoint = analogRead(A0);
    double currentTime = millis() / 1000.0;
    
    double ffOutput = ff.calculate(setpoint, currentTime);
    
    // Use with PID for combined control
    double totalOutput = pidOutput + ffOutput;
    
    delay(100);
}
```

#### API Reference

**Constructor:**
```cpp
FeedForwardController(double gain = 0.0, double offset = 0.0, bool direction = true, float rampRate = 0.0);
```

**Methods:**
- `void setGain(double gain)` - Set feedforward gain
- `void setOffset(double offset)` - Set baseline offset
- `void setRampRate(float rampRate)` - Set rate limiting
- `double calculate(double currentValue, double currentTime)` - Calculate output
- `float lowPassFilter(float currentValue, float alpha)` - Apply filtering

### Smith Predictor

The Smith Predictor is designed for controlling processes with significant time delays (dead time). It uses an internal process model to predict the response without delay.

#### Features
- Internal process model (first-order + dead time)
- Integrated PID controller
- Dead time compensation
- Process model parameter tuning

#### Basic Usage
```cpp
#include <GenericControl.h>

// Create Smith Predictor: PID gains, process model parameters
SmithPredictor smith(0.8, 0.05, 0.2,     // Kp, Ki, Kd
                    1.0, 10.0, 5.0);      // gain, time constant, dead time

void setup() {
    smith.setSetpoint(50.0);
    smith.setOutputLimits(0, 100);
}

void loop() {
    double processOutput = readSensor();
    double currentTime = millis() / 1000.0;
    
    double controlSignal = smith.calculate(processOutput, currentTime);
    
    applyControl(controlSignal);
    delay(100);
}
```

#### API Reference

**Constructor:**
```cpp
SmithPredictor(double kp, double ki, double kd, double processGain, double processTimeConstant, double deadTime);
```

**Methods:**
- `void setControllerGains(double kp, double ki, double kd)` - Update PID gains
- `void setProcessModel(double gain, double timeConstant, double deadTime)` - Update model
- `void setSetpoint(double setpoint)` - Set target value
- `double calculate(double processOutput, double currentTime)` - Main calculation
- `double getModelOutput()` - Get predicted model output
- `double getDelayedModelOutput()` - Get delayed model prediction

### Model Predictive Controller (MPC)

MPC is an advanced control strategy that optimizes control moves over a prediction horizon while respecting constraints.

#### Features
- Configurable prediction and control horizons
- Output and rate constraints
- Tunable cost function weights
- Future trajectory optimization

#### Basic Usage
```cpp
#include <GenericControl.h>

// Create MPC: prediction horizon, control horizon, process model
ModelPredictiveController mpc(20, 5,        // horizons
                             1.5, 8.0);     // process gain, time constant

void setup() {
    mpc.setSetpoint(50.0);
    mpc.setConstraints(0, 100, -10, 10);     // output and rate limits
    mpc.setWeights(1.0, 0.1, 0.5);          // tracking, effort, rate weights
}

void loop() {
    double processValue = readSensor();
    double currentTime = millis() / 1000.0;
    
    double controlOutput = mpc.calculate(processValue, currentTime);
    
    applyControl(controlOutput);
    delay(50);  // Higher update rate for MPC
}
```

#### API Reference

**Constructor:**
```cpp
ModelPredictiveController(int predictionHorizon, int controlHorizon, double modelGain, double modelTimeConstant, double modelDeadTime = 0.0);
```

**Methods:**
- `void setHorizons(int predictionHorizon, int controlHorizon)` - Update horizons
- `void setModelParameters(double gain, double timeConstant, double deadTime)` - Update model
- `void setConstraints(double outputMin, double outputMax, double outputRateMin, double outputRateMax)` - Set limits
- `void setWeights(double setpointWeight, double controlWeight, double controlRateWeight)` - Tune cost function
- `double calculate(double currentValue, double currentTime)` - Main calculation
- `const double* getFutureOutputs()` - Get predicted trajectory
- `const double* getFutureInputs()` - Get planned control moves

## Examples

### Temperature Control with PID + Feed Forward

```cpp
#include <GenericControl.h>

PIDController pid(2.0, 0.1, 0.5, 70.0);  // Target: 70°C
FeedForwardController ff(0.3, 15.0);      // Baseline heating

void setup() {
    pid.setOutputLimits(0, 100);
    ff.setRampRate(10.0);
}

void loop() {
    double temperature = readTemperature();
    double setpoint = pid.getSetpoint();
    double time = millis() / 1000.0;
    
    double pidOutput = pid.calculate(temperature, time);
    double ffOutput = ff.calculate(setpoint, time);
    double totalHeating = constrain(pidOutput + ffOutput, 0, 100);
    
    setHeaterPower(totalHeating);
    delay(1000);  // 1-second control loop
}
```

### Process Control with Smith Predictor

```cpp
#include <GenericControl.h>

// For a process with 30-second delay
SmithPredictor controller(1.2, 0.02, 0.5,  // Conservative PID gains
                         2.0, 45.0, 30.0);  // Process: gain=2, τ=45s, delay=30s

void setup() {
    controller.setSetpoint(75.0);
    controller.setOutputLimits(0, 100);
}

void loop() {
    double flowRate = readFlowSensor();
    double time = millis() / 1000.0;
    
    double valvePosition = controller.calculate(flowRate, time);
    setValve(valvePosition);
    
    delay(5000);  // 5-second control loop for slow process
}
```
