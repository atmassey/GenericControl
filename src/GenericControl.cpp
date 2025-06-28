#include "GenericControl.h"
#include <algorithm>
#include <cmath>

PIDController::PIDController(double kp, double ki, double kd, double setpoint, bool direction) 
    : m_kp(kp)
    , m_ki(ki)
    , m_kd(kd)
    , m_setpoint(setpoint)
    , m_lastError(0.0)
    , m_integralSum(0.0)
    , m_outputMin(-1.0)
    , m_outputMax(1.0)
    , m_integralMin(-1.0)
    , m_integralMax(1.0)
    , m_lastTime(0.0)
    , m_firstCall(true)
    , m_direction(direction)
{
}

void PIDController::reset()
{
    m_lastError = 0.0;
    m_integralSum = 0.0;
    m_firstCall = true;
}

void PIDController::setGains(double kp, double ki, double kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

// Set the direction of the controller
void PIDController::setDirection(bool direction)
{
    if (direction) {
        // Set gains for direct control
        m_kp = std::abs(m_kp);
        m_ki = std::abs(m_ki);
        m_kd = std::abs(m_kd);
    } else {
        // Set gains for reverse control
        m_kp = -std::abs(m_kp);
        m_ki = -std::abs(m_ki);
        m_kd = -std::abs(m_kd);
    }
}

void PIDController::setSetpoint(double setpoint)
{
    m_setpoint = setpoint;
}

void PIDController::setOutputLimits(double min, double max)
{
    if (min < max) {
        m_outputMin = min;
        m_outputMax = max;
    }
}

void PIDController::setIntegralLimits(double min, double max)
{
    if (min < max) {
        m_integralMin = min;
        m_integralMax = max;
    }
}

double PIDController::calculate(double currentValue, double currentTime)
{
    // Handle first call (no derivative term on first call)
    if (m_firstCall) {
        m_firstCall = false;
        m_lastTime = currentTime;
        m_lastError = m_setpoint - currentValue;
        return m_kp * m_lastError;
    }
    
    // Calculate time delta
    double deltaTime = currentTime - m_lastTime;
    if (deltaTime <= 0.0) {
        // Invalid time delta, just return previous calculation
        return m_kp * m_lastError;
    }
    
    // Calculate error
    double error = m_setpoint - currentValue;
    
    // Proportional term
    double proportional = m_kp * error;
    
    // Integral term
    m_integralSum += error * deltaTime;
    // Apply anti-windup limits to integral sum
    m_integralSum = std::max(m_integralMin, std::min(m_integralSum, m_integralMax));
    double integral = m_ki * m_integralSum;
    
    // Derivative term (using derivative of measurement to avoid derivative kick)
    double derivative = 0.0;
    if (deltaTime > 0.0) {
        double errorDerivative = (error - m_lastError) / deltaTime;
        derivative = m_kd * errorDerivative;
    }
    
    // Calculate output
    double output = proportional + integral + derivative;
    
    // Apply output limits
    output = std::max(m_outputMin, std::min(output, m_outputMax));
    
    // Save for next iteration
    m_lastError = error;
    m_lastTime = currentTime;
    
    return output;
}

double PIDController::getProportionalTerm() const
{
    return m_kp * (m_setpoint - m_lastError);
}

double PIDController::getIntegralTerm() const
{
    return m_ki * m_integralSum;
}

double PIDController::getDerivativeTerm() const
{
    return m_kd * m_lastError; 
}

bool PIDController::getDirection() const
{
    return m_direction;
}

FeedForwardController::FeedForwardController(double gain, double offset, bool direction, float rampRate)
    : m_gain(gain)
    , m_offset(offset)
    , m_direction(direction)
    , m_rampRate(rampRate)
    , m_lastOutput(0.0f)
{
}

double FeedForwardController::calculate(double currentValue, double currentTime)
{
    // Calculate feedforward output
    double output = m_gain * currentValue + m_offset;

    // Apply ramp rate limiting
    if (m_rampRate > 0.0) {
        double deltaOutput = output - m_lastOutput;
        if (std::abs(deltaOutput) > m_rampRate * (currentTime - m_lastTime)) {
            output = m_lastOutput + std::copysign(m_rampRate * (currentTime - m_lastTime), deltaOutput);
        }
    }

    // Save last output and time for next calculation
    m_lastOutput = output;
    m_lastTime = currentTime;

    return output;
}

void FeedForwardController::setGain(double gain)
{
    m_gain = gain;
}

void FeedForwardController::setOffset(double offset)
{
    m_offset = offset;
}

void FeedForwardController::setDirection(bool direction)
{
    m_direction = direction;
}

void FeedForwardController::setRampRate(float rampRate)
{
    m_rampRate = rampRate;
}

float FeedForwardController::lowPassFilter(float currentValue, float alpha)
{
    // Apply low-pass filter to smooth the output
    m_lastOutput = (1.0f - alpha) * m_lastOutput + alpha * currentValue;
    return m_lastOutput;
}

double FeedForwardController::getGain() const
{
    return m_gain;
}

double FeedForwardController::getOffset() const
{
    return m_offset;
}

bool FeedForwardController::getDirection() const
{
    return m_direction;
}

float FeedForwardController::getRampRate() const
{
    return m_rampRate;
}

SmithPredictor::SmithPredictor(double kp, double ki, double kd,
                               double processGain, double processTimeConstant, 
                               double deadTime)
    : m_controller(kp, ki, kd)
    , m_processGain(processGain)
    , m_processTimeConstant(processTimeConstant)
    , m_deadTime(deadTime)
    , m_modelOutput(0.0)
    , m_delayedModelOutput(0.0)
    , m_lastProcessInput(0.0)
    , m_lastTime(0.0)
    , m_firstCall(true)
    , m_bufferIndex(0)
{
    // Initialize delay buffer
    for (int i = 0; i < MAX_DELAY_SAMPLES; i++) {
        m_delayBuffer[i].value = 0.0;
        m_delayBuffer[i].timestamp = 0.0;
    }
}

void SmithPredictor::setControllerGains(double kp, double ki, double kd)
{
    m_controller.setGains(kp, ki, kd);
}

void SmithPredictor::setProcessModel(double gain, double timeConstant, double deadTime)
{
    m_processGain = gain;
    m_processTimeConstant = timeConstant;
    m_deadTime = deadTime;
}

void SmithPredictor::setSetpoint(double setpoint)
{
    m_controller.setSetpoint(setpoint);
}

void SmithPredictor::setOutputLimits(double min, double max)
{
    m_controller.setOutputLimits(min, max);
}

void SmithPredictor::reset()
{
    m_controller.reset();
    m_modelOutput = 0.0;
    m_delayedModelOutput = 0.0;
    m_lastProcessInput = 0.0;
    m_firstCall = true;
    
    // Clear delay buffer
    for (int i = 0; i < MAX_DELAY_SAMPLES; i++) {
        m_delayBuffer[i].value = 0.0;
        m_delayBuffer[i].timestamp = 0.0;
    }
}

double SmithPredictor::updateProcessModel(double input, double deltaTime)
{
    // First-order process model: dy/dt = (K*u - y)/T
    if (m_processTimeConstant > 0.0) {
        double derivative = (m_processGain * input - m_modelOutput) / m_processTimeConstant;
        m_modelOutput += derivative * deltaTime;
    } else {
        // For integrating processes or zero time constant
        m_modelOutput = m_processGain * input;
    }
    
    return m_modelOutput;
}

double SmithPredictor::getDelayedOutput(double currentTime)
{
    // If no delay, return current model output
    if (m_deadTime <= 0.0) {
        return m_modelOutput;
    }
    
    // Find the sample that corresponds to the current time minus dead time
    double targetTime = currentTime - m_deadTime;
    
    // Search for the closest sample in the buffer
    double closestValue = m_modelOutput; // Default to current output
    double closestTimeDiff = m_deadTime; // Initialize with maximum possible difference
    
    for (int i = 0; i < MAX_DELAY_SAMPLES; i++) {
        double timeDiff = std::abs(m_delayBuffer[i].timestamp - targetTime);
        if (timeDiff < closestTimeDiff) {
            closestTimeDiff = timeDiff;
            closestValue = m_delayBuffer[i].value;
        }
    }
    
    return closestValue;
}

double SmithPredictor::calculate(double processOutput, double currentTime)
{
    // Handle first call
    if (m_firstCall) {
        m_firstCall = false;
        m_lastTime = currentTime;
        m_modelOutput = processOutput;
        m_delayedModelOutput = processOutput;
        m_lastProcessInput = 0.0;
        
        // Initialize buffer with current output
        for (int i = 0; i < MAX_DELAY_SAMPLES; i++) {
            m_delayBuffer[i].value = processOutput;
            m_delayBuffer[i].timestamp = currentTime;
        }
        
        return 0.0;
    }
    
    // Calculate time difference
    double deltaTime = currentTime - m_lastTime;
    if (deltaTime <= 0.0) {
        return m_lastProcessInput; // Return last output if no time has passed
    }
    
    // Smith predictor structure:
    // 1. Calculate predicted process output without delay
    double predictedOutput = m_modelOutput;
    
    // 2. Calculate delayed model output
    m_delayedModelOutput = getDelayedOutput(currentTime);
    
    // 3. Calculate the error for the controller
    // This is: setpoint - (actual process output + (predicted output - delayed output))
    double controlError = processOutput + (predictedOutput - m_delayedModelOutput);
    
    // Use the modified error with the PID controller
    // Note: We're temporarily setting the controller's setpoint to 0 and using the negated error
    double originalSetpoint = m_controller.getSetpoint();
    m_controller.setSetpoint(0.0);
    double controlSignal = m_controller.calculate(-controlError, currentTime);
    m_controller.setSetpoint(originalSetpoint);
    
    // 4. Update process model with new control signal
    updateProcessModel(controlSignal, deltaTime);
    
    // 5. Store model output in delay buffer
    m_bufferIndex = (m_bufferIndex + 1) % MAX_DELAY_SAMPLES;
    m_delayBuffer[m_bufferIndex].value = m_modelOutput;
    m_delayBuffer[m_bufferIndex].timestamp = currentTime;
    
    // Save values for next iteration
    m_lastProcessInput = controlSignal;
    m_lastTime = currentTime;
    
    return controlSignal;
}

double SmithPredictor::getModelOutput() const
{
    return m_modelOutput;
}

double SmithPredictor::getDelayedModelOutput() const
{
    return m_delayedModelOutput;
}

ModelPredictiveController::ModelPredictiveController(int predictionHorizon, int controlHorizon,
                                                 double modelGain, double modelTimeConstant, 
                                                 double modelDeadTime)
    : m_predictionHorizon(predictionHorizon)
    , m_controlHorizon(controlHorizon)
    , m_modelGain(modelGain)
    , m_modelTimeConstant(modelTimeConstant)
    , m_modelDeadTime(modelDeadTime)
    , m_outputMin(-1.0)
    , m_outputMax(1.0)
    , m_outputRateMin(-0.1)
    , m_outputRateMax(0.1)
    , m_setpointWeight(1.0)
    , m_controlWeight(0.1)
    , m_controlRateWeight(0.1)
    , m_setpoint(0.0)
    , m_lastOutput(0.0)
    , m_lastTime(0.0)
    , m_firstCall(true)
    , m_modelOutput(0.0)
{
    // Allocate memory for prediction arrays
    m_futureOutputs = new double[m_predictionHorizon];
    m_futureInputs = new double[m_controlHorizon];
    
    // Initialize arrays
    for (int i = 0; i < m_predictionHorizon; i++) {
        m_futureOutputs[i] = 0.0;
    }
    
    for (int i = 0; i < m_controlHorizon; i++) {
        m_futureInputs[i] = 0.0;
    }
}

ModelPredictiveController::~ModelPredictiveController()
{
    // Free allocated memory
    delete[] m_futureOutputs;
    delete[] m_futureInputs;
}

void ModelPredictiveController::setHorizons(int predictionHorizon, int controlHorizon)
{
    // Validate horizons
    if (predictionHorizon <= 0 || controlHorizon <= 0 || controlHorizon > predictionHorizon) {
        return; // Invalid parameters
    }
    
    // Clean up old arrays
    delete[] m_futureOutputs;
    delete[] m_futureInputs;
    
    // Set new horizons
    m_predictionHorizon = predictionHorizon;
    m_controlHorizon = controlHorizon;
    
    // Allocate new arrays
    m_futureOutputs = new double[m_predictionHorizon];
    m_futureInputs = new double[m_controlHorizon];
    
    // Initialize arrays
    for (int i = 0; i < m_predictionHorizon; i++) {
        m_futureOutputs[i] = 0.0;
    }
    
    for (int i = 0; i < m_controlHorizon; i++) {
        m_futureInputs[i] = 0.0;
    }
}

void ModelPredictiveController::setModelParameters(double gain, double timeConstant, double deadTime)
{
    m_modelGain = gain;
    m_modelTimeConstant = timeConstant;
    m_modelDeadTime = deadTime;
}

void ModelPredictiveController::setConstraints(double outputMin, double outputMax, 
                                           double outputRateMin, double outputRateMax)
{
    if (outputMin < outputMax) {
        m_outputMin = outputMin;
        m_outputMax = outputMax;
    }
    
    m_outputRateMin = outputRateMin;
    m_outputRateMax = outputRateMax;
}

void ModelPredictiveController::setWeights(double setpointWeight, double controlWeight, double controlRateWeight)
{
    m_setpointWeight = setpointWeight;
    m_controlWeight = controlWeight;
    m_controlRateWeight = controlRateWeight;
}

void ModelPredictiveController::setSetpoint(double setpoint)
{
    m_setpoint = setpoint;
}

void ModelPredictiveController::reset()
{
    m_lastOutput = 0.0;
    m_firstCall = true;
    m_modelOutput = 0.0;
    
    // Reset prediction arrays
    for (int i = 0; i < m_predictionHorizon; i++) {
        m_futureOutputs[i] = 0.0;
    }
    
    for (int i = 0; i < m_controlHorizon; i++) {
        m_futureInputs[i] = 0.0;
    }
}

double ModelPredictiveController::simulateStep(double currentInput, double currentOutput, double stepSize)
{
    // First order model: dy/dt = (K*u - y)/T
    if (m_modelTimeConstant > 0.0) {
        double derivative = (m_modelGain * currentInput - currentOutput) / m_modelTimeConstant;
        return currentOutput + derivative * stepSize;
    } else {
        // For zero time constant (or very fast systems)
        return m_modelGain * currentInput;
    }
}

void ModelPredictiveController::predictFutureOutputs(double currentValue)
{
    // Start with current system state
    double predictedOutput = currentValue;
    
    // Simulate forward in time using the model and planned inputs
    for (int i = 0; i < m_predictionHorizon; i++) {
        // Determine control input for this step
        double controlInput;
        if (i < m_controlHorizon) {
            controlInput = m_futureInputs[i];
        } else {
            // After the control horizon, use the last planned input
            controlInput = m_futureInputs[m_controlHorizon - 1];
        }
        
        // Simulate one step forward using the model
        // (Simplified, assumes fixed step size of 1.0)
        predictedOutput = simulateStep(controlInput, predictedOutput, 1.0);
        
        // Store prediction
        m_futureOutputs[i] = predictedOutput;
    }
}

void ModelPredictiveController::optimizeControlInputs(double currentValue)
{
    
    // Start with the previous control sequence shifted by one
    for (int i = 0; i < m_controlHorizon - 1; i++) {
        m_futureInputs[i] = m_futureInputs[i + 1];
    }
    m_futureInputs[m_controlHorizon - 1] = m_futureInputs[m_controlHorizon - 2];
    
    // Simple optimization: Iteratively improve the control sequence
    const int MAX_ITERATIONS = 50;
    const double ALPHA = 0.1;  // Learning rate
    
    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        // Predict outputs with current control sequence
        predictFutureOutputs(currentValue);
        
        // Calculate gradients for each control move
        for (int i = 0; i < m_controlHorizon; i++) {
            double gradient = 0.0;
            
            // Cost component from setpoint tracking
            for (int j = i; j < m_predictionHorizon; j++) {
                // Estimate the effect of this control move on future outputs
                double error = m_futureOutputs[j] - m_setpoint;
                gradient += m_setpointWeight * error;
            }
            
            // Cost component from control effort
            gradient += m_controlWeight * m_futureInputs[i];
            
            // Cost component from control rate changes
            if (i > 0) {
                double rateChange = m_futureInputs[i] - m_futureInputs[i-1];
                gradient += m_controlRateWeight * rateChange;
            } else {
                double rateChange = m_futureInputs[i] - m_lastOutput;
                gradient += m_controlRateWeight * rateChange;
            }
            
            // Update control move using gradient
            m_futureInputs[i] -= ALPHA * gradient;
            
            // Apply constraints to the control move
            m_futureInputs[i] = std::max(m_outputMin, std::min(m_futureInputs[i], m_outputMax));
            
            // Apply rate constraints
            if (i > 0) {
                double rateChange = m_futureInputs[i] - m_futureInputs[i-1];
                if (rateChange > m_outputRateMax) {
                    m_futureInputs[i] = m_futureInputs[i-1] + m_outputRateMax;
                } else if (rateChange < m_outputRateMin) {
                    m_futureInputs[i] = m_futureInputs[i-1] + m_outputRateMin;
                }
            } else {
                double rateChange = m_futureInputs[i] - m_lastOutput;
                if (rateChange > m_outputRateMax) {
                    m_futureInputs[i] = m_lastOutput + m_outputRateMax;
                } else if (rateChange < m_outputRateMin) {
                    m_futureInputs[i] = m_lastOutput + m_outputRateMin;
                }
            }
        }
    }
}

double ModelPredictiveController::calculate(double currentValue, double currentTime)
{
    // Handle first call
    if (m_firstCall) {
        m_firstCall = false;
        m_lastTime = currentTime;
        m_lastOutput = 0.0;
        m_modelOutput = currentValue;
        
        // Initialize prediction arrays
        for (int i = 0; i < m_predictionHorizon; i++) {
            m_futureOutputs[i] = currentValue;
        }
        
        for (int i = 0; i < m_controlHorizon; i++) {
            m_futureInputs[i] = 0.0;
        }
        
        return m_lastOutput;
    }
    
    // Calculate time delta
    double deltaTime = currentTime - m_lastTime;
    if (deltaTime <= 0.0) {
        return m_lastOutput; // No time has passed
    }
    
    // Update model with actual process output
    m_modelOutput = currentValue;
    
    // Optimize control sequence
    optimizeControlInputs(currentValue);
    
    // Get the first control move from the optimized sequence
    double output = m_futureInputs[0];
    
    // Apply limits one more time to be sure
    output = std::max(m_outputMin, std::min(output, m_outputMax));
    
    // Rate limiting
    double outputRate = (output - m_lastOutput) / deltaTime;
    if (outputRate > m_outputRateMax / deltaTime) {
        output = m_lastOutput + m_outputRateMax;
    } else if (outputRate < m_outputRateMin / deltaTime) {
        output = m_lastOutput + m_outputRateMin;
    }
    
    // Save state for next call
    m_lastOutput = output;
    m_lastTime = currentTime;
    
    return output;
}

double ModelPredictiveController::getModelOutput() const
{
    return m_modelOutput;
}

const double* ModelPredictiveController::getFutureOutputs() const
{
    return m_futureOutputs;
}

const double* ModelPredictiveController::getFutureInputs() const
{
    return m_futureInputs;
}

int ModelPredictiveController::getPredictionHorizon() const
{
    return m_predictionHorizon;
}

int ModelPredictiveController::getControlHorizon() const
{
    return m_controlHorizon;
}