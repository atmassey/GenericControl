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