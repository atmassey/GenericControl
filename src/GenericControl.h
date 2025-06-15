
class PIDController {
private:
    // Controller gains
    double m_kp;          // Proportional gain
    double m_ki;          // Integral gain
    double m_kd;          // Derivative gain
    
    // Controller state
    double m_setpoint;    // Desired value
    double m_lastError;   // Last error for derivative calculation
    double m_integralSum; // Sum for integral term
    
    // Limits
    double m_outputMin;  
    double m_outputMax;   
    double m_integralMin; 
    double m_integralMax; 
    
    // Timing
    double m_lastTime;    // used to calculate time difference
    bool m_firstCall;     // Flag for first call to calculate
    bool m_direction;     // Direction of control (true for direct, false for reverse)

public:
    PIDController(double kp = 0.0, double ki = 0.0, double kd = 0.0, double setpoint = 0.0, bool direction = true);
    
    void reset();
    
    void setGains(double kp, double ki, double kd);
    
    void setSetpoint(double setpoint);
    
    void setOutputLimits(double min, double max);

    void setIntegralLimits(double min, double max);

    void setDirection(bool direction);
    
    double calculate(double currentValue, double currentTime);


    
    // Getters for internal values
    double getProportionalTerm() const;
    double getIntegralTerm() const;
    double getDerivativeTerm() const;
    bool getDirection() const;
};