#ifndef DCMC
#define DCMC

#include<Arduino.h>
#include<vector>
#include <ESP32Servo.h>

class dcMotorController
{
  public:
    dcMotorController();
    ~dcMotorController();
    void setupMotor(int motorPin1, int motorPin2, int pwmPin);
    void setupLinearActuator(float screwPitch, float outputGearRatio, float maxVelocity, float maxAcceleration, float maxDecelration,  float maxTravleDistance, float travleVelocity, int limitTopPin, int limitBottomPin);
    void setupController(int topForcePin, int bottomForcePin);
    void setVelocity(float desiredVelocity);
    void setForce(float desiredForce);
    void setTargetPos(float desiredPos);
    void setCurrentPos(float pos);
    void setCurrentVelcoity(float velocity);
    void emergencyStop();
    void resumeControl();
    void enablePositionControl();
    void enableForceControl();
    void enableSpeedControl();
    void enableGravity(bool val, float gravAcceleration);
    void controlLoop();
    
    void PlayModeController();
    void CalibrateArmWeight();
    
    void PIDPositionControlLoop();
    void PIDForceControlLoop();

    float currentPos;
    float currentForce;
    float armWeight = 0;
    
    float bottomForceReading = 0;
    float topForceReading = 0;
  
    bool zeroed;
    
    bool overridePWMState = false;
    int overridePWM = 0;

    float rawTopForce = 0;
    float rawBottomForce = 0;
  private:
    void updateMotorSpeed();
    void speedController();
    void speedControllerNew();
    void dcMotorControllerLoop();
    void maxVelocityController();
    
    void readTopForce();
    void readBottomForce();
    
    float _stepAngle;
    float _microStepping;
    float _motorPin1; 
    float _motorPin2; 
    float _PWMPin;
    
    float _screwPitch;
    float _outputGearRatio;
    float _maxVelocity;
    float _travleVelocity;
    float _maxAcceleration;
    float _maxDeceleration;
    float _maxTravleDistance;

    float _gravAcceleration;
    bool _enableGravity;
    bool _holdingController;
    
    int _limitTopPin;
    int _limitBottomPin;

    float _currentPos;
    float _currentVelocity;
    float _currentPWM;
    
    float _targetVelocity;
    float _targetPosition;
    float _targetForce;

    bool _positionControl;
    bool _forceControl;
    bool _speedControl;
    
    bool _motorIsSetup; 
    bool _actuatorIsSetup; 

    bool _stop;

    int _topForcePin;
    int _bottomForcePin;
};


#endif
