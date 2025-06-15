#include "GenericControl.h"
#include <algorithm>

PIDController::PIDController(double kp, double ki, double kd, double setpoint, bool direction= true) 
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
