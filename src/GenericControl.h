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
    double getSetpoint() const { return m_setpoint; }
};

class FeedForwardController {
private:
    double m_gain;          // Gain for feedforward control
    double m_offset;        // Offset for feedforward control
    bool m_direction;       // Direction of control (true for direct, false for reverse)
    float m_rampRate;       // Ramp rate for feedforward control
    float m_lastOutput = 0.0; // Last output value for ramping
    double m_lastTime = 0.0; // Last time calculate was called
public:
    FeedForwardController(double gain = 0.0, double offset = 0.0, bool direction = true, float rampRate = 0.0);
    
    void setGain(double gain);
    
    void setOffset(double offset);
    
    void setDirection(bool direction);
    
    void setRampRate(float rampRate);
    
    double calculate(double currentValue, double currentTime);
    
    float lowPassFilter(float currentValue, float alpha = 0.1f);
    
    // Getters for internal values
    double getGain() const;
    double getOffset() const;
    bool getDirection() const;
    float getRampRate() const;
};

class SmithPredictor {
private:
    PIDController m_controller;      // Main PID controller
    double m_processGain;            // Process model gain
    double m_processTimeConstant;    // Process model time constant (for first-order model)
    double m_deadTime;               // Process dead time/delay
    
    // Process model state
    double m_modelOutput;            // Current model output
    double m_delayedModelOutput;     // Delayed model output
    double m_lastProcessInput;       // Last input to process model
    
    // Timing
    double m_lastTime;               // Last time calculate was called
    bool m_firstCall;                // Flag for first call to calculate
    
    // Circular buffer for implementing delay
    static const int MAX_DELAY_SAMPLES = 1000;
    struct DelayBuffer {
        double value;
        double timestamp;
    };
    DelayBuffer m_delayBuffer[MAX_DELAY_SAMPLES];
    int m_bufferIndex;
    
    // Helper methods
    double updateProcessModel(double input, double deltaTime);
    double getDelayedOutput(double currentTime);
    
public:
    SmithPredictor(double kp = 0.0, double ki = 0.0, double kd = 0.0,
                   double processGain = 1.0, double processTimeConstant = 1.0, 
                   double deadTime = 0.0);
    
    void setControllerGains(double kp, double ki, double kd);
    void setProcessModel(double gain, double timeConstant, double deadTime);
    void setSetpoint(double setpoint);
    void setOutputLimits(double min, double max);
    
    void reset();
    
    // Main control calculation with Smith Predictor structure
    double calculate(double processOutput, double currentTime);
    
    // Getter methods
    double getModelOutput() const;
    double getDelayedModelOutput() const;
};

class ModelPredictiveController {
private:
    // Prediction and control horizons
    int m_predictionHorizon;  // Number of steps to predict ahead (N)
    int m_controlHorizon;     // Number of control moves to optimize (M)
    
    // Process model parameters
    double m_modelGain;       // Process gain
    double m_modelTimeConstant; // Process time constant
    double m_modelDeadTime;   // Process dead time
    
    // Constraints
    double m_outputMin;       // Minimum control output
    double m_outputMax;       // Maximum control output
    double m_outputRateMin;   // Minimum rate of change
    double m_outputRateMax;   // Maximum rate of change
    
    // Optimization weights
    double m_setpointWeight;  // Weight for setpoint tracking
    double m_controlWeight;   // Weight for control effort
    double m_controlRateWeight; // Weight for control rate changes
    
    // Controller state
    double m_setpoint;        // Target value
    double m_lastOutput;      // Last control output
    double m_lastTime;        // Last time calculate was called
    bool m_firstCall;         // Flag for first call
    
    // Internal model state
    double m_modelOutput;     // Current model output
    double* m_futureOutputs;  // Predicted future outputs
    double* m_futureInputs;   // Planned future inputs
    
    // Helper methods
    void predictFutureOutputs(double currentValue);
    void optimizeControlInputs(double currentValue);
    double simulateStep(double currentInput, double currentOutput, double stepSize);
    
public:
    ModelPredictiveController(int predictionHorizon = 10, int controlHorizon = 3,
                             double modelGain = 1.0, double modelTimeConstant = 1.0, 
                             double modelDeadTime = 0.0);
    ~ModelPredictiveController();
    
    void setHorizons(int predictionHorizon, int controlHorizon);
    void setModelParameters(double gain, double timeConstant, double deadTime);
    void setConstraints(double outputMin, double outputMax, 
                       double outputRateMin, double outputRateMax);
    void setWeights(double setpointWeight, double controlWeight, double controlRateWeight);
    void setSetpoint(double setpoint);
    
    void reset();
    double calculate(double currentValue, double currentTime);
    
    // Getters for internal state
    double getModelOutput() const;
    const double* getFutureOutputs() const;
    const double* getFutureInputs() const;
    int getPredictionHorizon() const;
    int getControlHorizon() const;
};