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