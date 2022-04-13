#ifndef SC
#define SC

#include<Arduino.h>
#include<vector>
class stepperController
{
  public:
    stepperController();
    ~stepperController();
    void setupStepper(float stepAngle, float microStepping, int dirPin, int pulsePin);
    void setupLinearActuator(float screwPitch, float outputGearRatio, float maxVelocity, float maxAcceleration, float maxDecelration, float maxTravleDistance,int limitTopPin,int limitBottomPince);
    void setupController(int topForcePin, int bottomForcePin);
    void setVelocity(float desiredVelocity);
    void setForce(float desiredForce);
    void setTarget(float desiredTarget);
    void emergencyStop();
    void resumeControl();
    void enablePositionControl();
    void enableForceControl();
    void enableSpeedControl();
    void enableGravity(bool val, float gravAcceleration);
    void controlLoop();
    void PIDPositionControlLoop();
    void PIDForceControlLoop();

    float getTopForce();
    float getBottomForce();
    
    float currentPos;
    float currentSpeed;
    float currentForce;
    bool zeroed;
  private:
    void calcSystemParamps(); 
    void stepStepperMotor();
    void speedController();
    void stepperControllerLoop();
    
    float _stepAngle;
    float _microStepping;
    float _dirPin; 
    float _pulsePin; 
    
    float _screwPitch;
    float _outputGearRatio;
    float _maxVelocity;
    float _maxAcceleration;
    float _maxDeceleration;
    float _maxTravleDistance;
    float _travelPerStep;

    float _gravAcceleration;
    bool _enableGravity;
    bool _holdingController;
    
    int _limitTopPin;
    int _limitBottomPin;
  
    float _targetVelcoty;
    float _targetPosition;
    float _targetForce;

    bool _positionControl;
    bool _forceControl;
    bool _speedControl;
    
    bool _stepperIsSetup; 
    bool _actuatorIsSetup; 

    bool _stop;

    int _topForcePin;
    int _bottomForcePin;


    float posLoopFrequency = 100; 
    float prePosLoopStart = 0; 
    float prePosError = 0;
    float pI = 0;
    float pKP = 0.05;
    float pKI = 0.005;
    float pKD = 0.1;

    
    float froceLoopFrequency = 100;
    float preFroceLoopStart = 0; 
    float preForceError = 0;
    float fKP = 0.5;
    float fKI = 0.8;
    float fKD = 0.02;
    float fErrorWindowSize = 50; //ms
    int vectorListSize = (int)((froceLoopFrequency/1000.0)*fErrorWindowSize);
    std::vector<float> iErrorList;
    float errorMargin = 50;


    float preForceWeight = 0.5;
    float preTopF = 0;
    float preBottomF = 0;


    float speedLoopFrequency = 100; 
    float preSpeedSetTime = 0;

    float preStepTime = 0;
};


#endif
