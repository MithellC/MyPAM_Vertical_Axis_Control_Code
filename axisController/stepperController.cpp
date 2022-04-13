#include"stepperController.h"

stepperController::stepperController()
{
  _stepperIsSetup = false;
  _actuatorIsSetup = false;

  _positionControl = true;
  _forceControl = false;
  _speedControl = false;
  
  _stop = false;

  _targetVelcoty = 0;
  _targetPosition = 0;
  _targetForce = 0;
  _gravAcceleration = 0;
  _enableGravity = false;
  _holdingController = false;
  
  _positionControl = 0;
  _forceControl = 0;
    
  currentSpeed = 0;
  currentPos = 0;
  zeroed = false;
};

stepperController::~stepperController(){};

void stepperController::setupStepper(float stepAngle, float microStepping, int dirPin, int pulsePin)
{
  _stepAngle = stepAngle;
  _microStepping = microStepping;
  _dirPin = dirPin; 
  _pulsePin = pulsePin; 
  
  pinMode(_dirPin, OUTPUT);
  pinMode(_pulsePin, OUTPUT);
  
  _stepperIsSetup = true;
  
  if(_actuatorIsSetup && _stepperIsSetup) calcSystemParamps();
}

void stepperController::setupLinearActuator(float screwPitch, float outputGearRatio, float maxVelocity, float maxAcceleration, float maxDecelration,  float maxTravleDistance, int limitTopPin, int limitBottomPin)
{
  _screwPitch = screwPitch;
  _outputGearRatio = outputGearRatio;
  _maxVelocity = maxVelocity;
  _maxAcceleration = maxAcceleration;
  _maxDeceleration = maxDecelration;
  _maxTravleDistance = maxTravleDistance;
  
  _limitTopPin = limitTopPin;
  _limitBottomPin = limitBottomPin;
  pinMode(_limitTopPin, INPUT);
  pinMode(_limitBottomPin, INPUT);
  
  _actuatorIsSetup = true;

  if(_actuatorIsSetup && _stepperIsSetup) calcSystemParamps();
}

void stepperController::setupController(int topForcePin, int bottomForcePin)
{
  _topForcePin = topForcePin;
  _bottomForcePin = bottomForcePin;
  
  pinMode(_topForcePin, OUTPUT);
  pinMode(_bottomForcePin, OUTPUT);
}

void stepperController::calcSystemParamps()
{
  _travelPerStep = (((float)_stepAngle/360) * (float)_outputGearRatio * (float)_screwPitch) / (float)_microStepping;
}


//Velocity is a vector and so uses it sign to define its desired direction
void stepperController::setVelocity(float desiredVelocity)
{
  if(desiredVelocity > _maxVelocity){_targetVelcoty =_maxVelocity; return;}
  _targetVelcoty = desiredVelocity;
}
void stepperController::setForce(float desiredForce){_targetForce = desiredForce;}
void stepperController::setTarget(float desiredTarget){ _targetForce = desiredTarget; }
void stepperController::emergencyStop(){ _stop = true; }
void stepperController::resumeControl(){ _stop = false; }

void stepperController::enablePositionControl(){ _positionControl = true; _forceControl = false; _speedControl = false; }
void stepperController::enableForceControl(){ _forceControl = true; _positionControl = false; _speedControl = false; }
void stepperController::enableSpeedControl(){ _forceControl = false; _positionControl = false; _speedControl = true; }
void stepperController::enableGravity(bool val, float gravAcceleration){ _enableGravity = val; _gravAcceleration = gravAcceleration;}

void stepperController::controlLoop()
{
  if (_stop){return;}
  else if (_positionControl){
    PIDPositionControlLoop();
  }
  else if (_forceControl){
    PIDForceControlLoop();
  }
  else if(_speedControl){}
  
  speedController();
  stepperControllerLoop();
}


void stepperController::PIDPositionControlLoop()
{
  float currentTime = micros();
  if(currentTime > prePosLoopStart + 1000000/posLoopFrequency){
    float error = _targetPosition - currentPos;
    float pP = error * pKP;
    pI += (error * pKI);
    if ((pI < 0 && error > 0) || (pI > 0 && error < 0) || error == 0) pI = 0;
    float pD = (prePosError - error) * pKD;

    if (error = 0 )setVelocity(error);
    else setVelocity(pP + pI + pD);
    
    
    prePosLoopStart = currentTime;
    prePosError = error; 
  }
}

void stepperController::PIDForceControlLoop()
{
  
  float currentTime = micros();
  if(currentTime > preFroceLoopStart + 1000000/froceLoopFrequency){
    float error = _targetForce - (getTopForce() - getBottomForce());

    if (error > -errorMargin && error < errorMargin){error = 0;}
    iErrorList.push_back(error);
    if (iErrorList.size() > vectorListSize) iErrorList.erase(iErrorList.begin());
    float fI = 0;
    for(int i = 0; i < iErrorList.size(); i++) {fI += (iErrorList[i] * fKI);}
    
    float fP = error * fKP;
    float fD = (preForceError - error) * fKD;
    float velocity = fP + fI + fD;
    
    setVelocity(velocity);
    
    preFroceLoopStart = currentTime;
    preForceError = error; 
  }
}


float stepperController::getTopForce()
{ 
  float raw = (float)analogRead(_topForcePin);
  raw = raw * (preForceWeight) + raw *(1 - preForceWeight);
  preTopF = raw;
  float reading = raw;
  if (reading < 50) reading = 0;
  return reading;
}


float stepperController::getBottomForce()
{
  float raw = (float)analogRead(_bottomForcePin);
  raw = raw * (preForceWeight) + raw *(1 - preForceWeight);
  preBottomF = raw;

  float reading = raw;
  if (reading < 250) reading = 0;
  if(reading + getTopForce() == 0 && (_targetForce != 0 || _enableGravity)){_holdingController = false;}
  else _holdingController = true;
  
  /*Serial.print("     RawB:  ");
  Serial.print(raw);
  Serial.print("      BottomF:  ");
  Serial.println(reading);*/
  
  return reading;
}


void stepperController::speedController()
{
  float currentTime = micros();


  if(!_holdingController && _enableGravity && ( currentTime > (preSpeedSetTime + 1000000/speedLoopFrequency) ) && (getTopForce() + getBottomForce() == 0) ){
    currentSpeed -= _gravAcceleration / speedLoopFrequency;
    if (currentSpeed < -_maxVelocity) currentSpeed = -_maxVelocity;
    preSpeedSetTime = currentTime;
  }
  
  else if (!_holdingController && !_enableGravity){
    currentSpeed = 0;
  }
  
  else if (currentTime > preSpeedSetTime + 1000000/speedLoopFrequency){
    preSpeedSetTime = currentTime;
    if (_targetVelcoty < currentSpeed){
      if (currentSpeed > 0){currentSpeed -= _maxDeceleration / speedLoopFrequency;}
      else{currentSpeed -= _maxAcceleration / speedLoopFrequency; }
    }
    else if (_targetVelcoty > currentSpeed){ 
      if (currentSpeed < 0){currentSpeed += _maxDeceleration / speedLoopFrequency; }
      else{currentSpeed += _maxAcceleration / speedLoopFrequency; }
    }
      
    if (currentSpeed >  -_maxDeceleration / speedLoopFrequency && currentSpeed <  _maxDeceleration / speedLoopFrequency && _targetVelcoty == 0) currentSpeed = 0;
    else if (currentSpeed > _maxVelocity) currentSpeed = _maxVelocity;
    else if (currentSpeed < -_maxVelocity) currentSpeed = -_maxVelocity;
  }
}


void stepperController::stepperControllerLoop()
{
  //Serial.println("Pos: ");
  //Serial.println(currentPos);
  if (currentSpeed != 0){
    float cSpeed = currentSpeed;
    if (cSpeed < 0) cSpeed = -cSpeed;
    float timeInc = _travelPerStep/cSpeed * 1000000;
    float currentTime = micros();
    if(currentTime > timeInc + preStepTime){
      preStepTime = currentTime;
      stepStepperMotor();
    }
  }
}

void stepperController::stepStepperMotor(){
  if (currentSpeed < 0){
    if (digitalRead(_limitTopPin)){
      if (currentSpeed < 0) currentSpeed = 0;
      return;
    }
    digitalWrite(_dirPin, LOW);
    currentPos += _travelPerStep;
  }
  
  else if (currentSpeed > 0){
    if (digitalRead(_limitBottomPin)){
      if (currentSpeed > 0) currentSpeed = 0;
      currentPos = 0;
      zeroed = true;
      return;
      }
    digitalWrite(_dirPin, HIGH);
    currentPos -= _travelPerStep;
  }
  
  digitalWrite(_pulsePin, HIGH);
  delayMicroseconds(30);
  digitalWrite(_pulsePin, LOW);
}
