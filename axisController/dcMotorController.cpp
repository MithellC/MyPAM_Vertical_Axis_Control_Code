#include"dcMotorController.h"

dcMotorController::dcMotorController()
{
  _stepperIsSetup = false;
  _actuatorIsSetup = false;

  _positionControl = true;
  _forceControl = false;
  _speedControl = false;
  
  _stop = false;

  _targetVelocity = 0;
  _targetPosition = 0;
  _targetForce = 0;
  _gravAcceleration = 0;
  _enableGravity = false;
  _holdingController = false;
  
  _positionControl = 0;
  _forceControl = 0;
    
  _currentPWM = 0;
  currentPos = 0;
  zeroed = false;
};

dcMotorController::~dcMotorController(){};

ESP32PWM PWM;
void dcMotorController::setupMotor(int motorPin1, int motorPin2, int pwmPin)
{
  _motorPin1 = motorPin1; 
  _motorPin2 = motorPin2; 
  _PWMPin = pwmPin;
  
  ESP32PWM::allocateTimer(0);
  PWM.attachPin(_PWMPin, 15000, 10); 
  PWM.writeScaled(0);
  
  pinMode(_motorPin1, OUTPUT);
  pinMode(_motorPin2, OUTPUT);
  pinMode(_PWMPin, OUTPUT);
      
  _stepperIsSetup = true;
  
  if(_actuatorIsSetup && _stepperIsSetup) calcSystemParamps();
}

void dcMotorController::setupLinearActuator(float screwPitch, float outputGearRatio, float maxVelocity, float maxAcceleration, float maxDecelration,  float maxTravleDistance, int limitTopPin, int limitBottomPin)
{
  _screwPitch = screwPitch;
  _outputGearRatio = outputGearRatio;
  _maxVelocity = (int)(maxVelocity * 0.98);
  _maxAcceleration = maxAcceleration;
  _maxDeceleration = maxDecelration;
  _maxTravleDistance = maxTravleDistance;

  _limitTopPin = limitTopPin;
  _limitBottomPin = limitBottomPin;
  pinMode(_limitTopPin, INPUT_PULLDOWN);
  pinMode(_limitBottomPin, INPUT_PULLDOWN);

  _actuatorIsSetup = true;

  if(_actuatorIsSetup && _stepperIsSetup) calcSystemParamps();
}

void dcMotorController::setupController(int topForcePin, int bottomForcePin)
{
  _topForcePin = topForcePin;
  _bottomForcePin = bottomForcePin;
  
  pinMode(_topForcePin, INPUT);
  pinMode(_bottomForcePin, INPUT);
}

void dcMotorController::calcSystemParamps()
{
  _travelPerStep = (((float)_stepAngle/360) * (float)_outputGearRatio * (float)_screwPitch) / (float)_microStepping;
}


//Velocity is a vector and so uses it sign to define its desired direction
void dcMotorController::setVelocity(float desiredVelocity)
{
  if(desiredVelocity > _maxVelocity){_targetVelocity =_maxVelocity; return;}
  else if(desiredVelocity < -_maxVelocity){_targetVelocity = -_maxVelocity; return;}
  _targetVelocity = desiredVelocity;
}

void dcMotorController::setCurrentPos(float pos){ _currentPos = pos; }
void dcMotorController::setCurrentVelcoity(float velocity){ _currentVelocity = velocity; }

void dcMotorController::setForce(float desiredForce){_targetForce = desiredForce;}
void dcMotorController::setTargetPos(float desiredPos){ _targetPosition = desiredPos; }

void dcMotorController::emergencyStop(){ _stop = true; }
void dcMotorController::resumeControl(){ _stop = false; }

void dcMotorController::enablePositionControl(){ _positionControl = true; _forceControl = false; _speedControl = false; }
void dcMotorController::enableForceControl(){ _forceControl = true; _positionControl = false; _speedControl = false; }
void dcMotorController::enableSpeedControl(){ _forceControl = false; _positionControl = false; _speedControl = true; }
void dcMotorController::enableGravity(bool val, float gravAcceleration){ _enableGravity = val; _gravAcceleration = gravAcceleration;}

float sensorReadingLoopTime = 0;
 
void dcMotorController::controlLoop()
{
  if (millis() > sensorReadingLoopTime + 10)
  {
    sensorReadingLoopTime = millis();
    readTopForce();
    readBottomForce();
  }

  if (_stop){_currentPWM = 0;}
  
  else if (_positionControl){
    PIDPositionControlLoop();
  }
  else if (_forceControl){
    PIDForceControlLoop();
  }
  else if(_speedControl) speedController();
  
  dcMotorControllerLoop();
}

float positionLoopFrequency = 100;
float prePositionLoopStart = 0; 
float prePositionError = 0;

float pKP = 4.5;
float pKI = 0.6;
//float pKI = 0.006;
float pKD = 0.1;

//float pErrorWindowSize = 20000; //ms
float pErrorWindowSize = 50; //ms
int positionVectorListSize = (int)((positionLoopFrequency/1000.0) * pErrorWindowSize);
std::vector<float> positionIErrorList;


float posErrorMargin = 0.2;
void dcMotorController::PIDPositionControlLoop()
{
  float currentTime = micros();
  if(currentTime > prePositionLoopStart + 1000000/positionLoopFrequency){
    float error = _targetPosition - (_currentPos);
    if (error > -posErrorMargin && error < posErrorMargin){error = 0;}
    positionIErrorList.push_back(error);
    if (positionIErrorList.size() > positionVectorListSize) positionIErrorList.erase(positionIErrorList.begin());
    float pI = 0;
    for(int i = 0; i < positionIErrorList.size(); i++) {pI += (positionIErrorList[i] * pKI);}
    
    float pP = error * pKP;
    float pD = (prePositionError - error) * pKD;
    _currentPWM = pP + pI + pD;

    if (_currentPWM < -253) _currentPWM = -253;
    else if (_currentPWM > 253) _currentPWM = 253;
    
    prePositionLoopStart = currentTime;
    prePositionError = error; 
  }
}

float froceLoopFrequency = 100;
float preFroceLoopStart = 0; 
float preForceError = 0;

float fKP = 0.5;
float fKI = 0.0;
float fKD = 0.0;

//float fKP = 0.5;
//float fKI = 0.002;
//float fKD = 0;
//float fKI = 0;

float fErrorWindowSize = 50; //ms
int vectorListSize = (int)((froceLoopFrequency/1000.0)*fErrorWindowSize);
std::vector<float> iErrorList;

float errorMargin = 50;


float preFError = 0;
float preFWeight = 0.6;

void dcMotorController::PIDForceControlLoop()
{
  float currentTime = micros();
  if(currentTime > preFroceLoopStart + 1000000/froceLoopFrequency){
    float topForce =  topForceReading;
    float bottomForce = bottomForceReading;
    float error = _targetForce - (bottomForce - topForce);
    error = error*(1 - preFWeight) + preFError*preFWeight;
    preFError = error;
    
    /*Serial.print("BottomForce: ");
    Serial.print(-bottomForce);
    Serial.print("     TopForce: ");
    Serial.print(topForce);
    Serial.print("     error: ");
    Serial.println(error);*/
    
    if (error > -errorMargin && error < errorMargin){error = 0;}
    
    iErrorList.push_back(error);
    if (iErrorList.size() > vectorListSize) iErrorList.erase(iErrorList.begin());
    float fI = 0;
    for(int i = 0; i < iErrorList.size(); i++) {fI += (iErrorList[i] * fKI);}
    
    float fP = error * fKP;
    float fD = (preForceError - error) * fKD;
    _currentPWM = fP + fI + fD;
    
    //Account for known disturbances
    //1. Moving up requires a higher pwm
    if (_currentPWM > 0) _currentPWM *=1.2; 
    
    if (_currentPWM < -253) _currentPWM = -253;
    else if (_currentPWM > 253) _currentPWM = 253;

    //if (_currentPWM > _maxVelocity) _currentPWM = _maxVelocity;
    //else if (_currentPWM < -_maxVelocity) _currentPWM = -_maxVelocity;
    
    if (!_holdingController && _targetForce !=0 ) _currentPWM = 0;
    
    preFroceLoopStart = currentTime;
    preForceError = error; 

    /*
    Serial.print("targetForce: ");
    Serial.print(_targetForce);
    Serial.print("     currentForce: ");
    Serial.println(error);
    Serial.print("     _currentVelocity: ");
    Serial.print(_currentVelocity);
    //Serial.print("     _currentPWM: ");
    */
    //Serial.println(_currentPWM);
 
    
    
  }
}


float preForceWeight = 0.8;
float preTopF = 0;
void dcMotorController::readTopForce()
{ 
  float raw = (float)analogRead(_topForcePin);
  raw = 4095 - raw;
  if(raw < 0) raw =0;
  float filtered = preTopF * (preForceWeight) + raw *(1 - preForceWeight);
  preTopF = filtered;
  float reading = filtered;
  if (reading < 20) reading = 0;
  
  topForceReading = reading;
}

float preBottomF = 0;
void dcMotorController::readBottomForce()
{
  float raw = (float)analogRead(_bottomForcePin);
  raw = 4095 - raw;
  if(raw < 0) raw =0;
  float filtered = preBottomF * (preForceWeight) + raw *(1 - preForceWeight);
  preBottomF = filtered;
  
  float reading = filtered;
  //if (reading < 220) reading = 0;
  if (reading < 40) reading = 0;
  if(reading + topForceReading == 0 && (_targetForce != 0 || _enableGravity)){_holdingController = false;}
  else _holdingController = true;
  bottomForceReading = reading;
}


float speedLoopFrequency = 100; 
float preSpeedSetTime = 0;

    
float sKP = 0.07;
float sKI = 0.0005;
float sKD = 0;

float sErrorWindowSize = 10; //ms
int velcoityVectorListSize = (int)((speedLoopFrequency/1000.0)*sErrorWindowSize);
std::vector<float> SIErrorList;

float I = 0;
void dcMotorController::speedController()
{
  float currentTime = micros();
  if(!_holdingController && _enableGravity && ( currentTime > (preSpeedSetTime + 1000000/speedLoopFrequency) ) && (topForceReading + bottomForceReading == 0) ){
    _currentPWM -= _gravAcceleration / speedLoopFrequency;
    if (_currentPWM < -_maxVelocity) _currentPWM = -_maxVelocity;
    preSpeedSetTime = currentTime;
  }
  
  else if (!_holdingController && !_enableGravity && !_speedControl && !_positionControl){
    _currentPWM = 0;
  }
  
  else if (currentTime > preSpeedSetTime + 1000000/speedLoopFrequency){
    float maxInc = _maxAcceleration / speedLoopFrequency;
    float maxDec = _maxDeceleration / speedLoopFrequency;

    float error = _targetVelocity - _currentVelocity;
    float sP = error * sKP;

    SIErrorList.push_back(error);
    if (SIErrorList.size() > velcoityVectorListSize) SIErrorList.erase(SIErrorList.begin());
    float sI = 0;
    for(int i = 0; i < SIErrorList.size(); i++) {sI += (SIErrorList[i] * sKI);}

    
    float accel = sP + sI;
    
    if (accel > 0 && accel > maxInc) accel = maxInc;
    else if (accel < 0 && accel < -maxInc) accel = -maxInc;

    _currentPWM += accel;
    
    if (_currentPWM < -253) _currentPWM = -253;
    else if (_currentPWM > 253) _currentPWM = 253;
    
    /*Serial.print("_currentVelocity: ");
    Serial.print(_currentVelocity);
    Serial.print("     _targetVelocity: ");
    Serial.print(_targetVelocity);
    Serial.print("     _currentPWM: ");
    Serial.println(_currentPWM);*/

    preSpeedSetTime = currentTime;
  }
}


float preStepTime = 0;
void dcMotorController::dcMotorControllerLoop()
{
  //maxVelocityController();
  if(overridePWMState) _currentPWM = overridePWM;
  if (_currentPWM != 0){
    float cPWM = _currentPWM;
    if (cPWM < 0) cPWM = -cPWM;
    float timeInc = 10;
    float currentTime = micros();
    
    if(currentTime > timeInc + preStepTime){
      preStepTime = currentTime;
      updateMotorSpeed();
    }
  }
  
  else{
    digitalWrite(_motorPin1, LOW);
    digitalWrite(_motorPin2, LOW);
    PWM.writeScaled(0);
  }
}


void dcMotorController::updateMotorSpeed(){
  //Serial.print("_currentVelocity: ");
  //Serial.println(_currentVelocity);
  
  if (_currentPWM > 0){
    if (digitalRead(_limitTopPin)){
      if (_currentPWM > 0) _currentPWM = 0;
      //Serial.println("Top Lim");
      digitalWrite(_motorPin1, LOW);
      digitalWrite(_motorPin2, LOW);
      PWM.writeScaled(0);
      return;
    }
    digitalWrite(_motorPin1, HIGH);
    digitalWrite(_motorPin2, LOW);
    PWM.writeScaled(_currentPWM/255.0);
  }
  
  else if (_currentPWM < 0){
    if (digitalRead(_limitBottomPin)){
      if (_currentPWM < 0) _currentPWM = 0;
      currentPos = 0; zeroed = true;
      digitalWrite(_motorPin1, LOW);
      digitalWrite(_motorPin2, LOW);
      PWM.writeScaled(0);
      return;
    }
    digitalWrite(_motorPin1, LOW);
    digitalWrite(_motorPin2, HIGH);
    PWM.writeScaled(-_currentPWM/255.0);
  }
}

//Function used to check if current velocity is over max velocity 

float prePWM = 0;
float preSpeedPWM = 0;

bool maxVelMode = false;
/*void dcMotorController::maxVelocityController(){
  prePWM = _currentPWM; 
  if(maxVelMode) _currentPWM = preSpeedPWM;
  
  if(_currentVelocity > _maxVelocity){
    maxVelMode = true;
     _speedControl = true;
    setVelocity(_maxVelocity);
    speedController();
  }
  else if(_currentVelocity < -_maxVelocity){
    maxVelMode = true;
    _speedControl = true;
    setVelocity(-_maxVelocity);
    speedController();
  }
  else {
    maxVelMode = false;
    return;
  }
  
  preSpeedPWM = _currentPWM;
  
  if(_currentPWM < 0){
    if (_currentPWM < prePWM) _currentPWM = prePWM;
  }
  else if(_currentPWM > 0){
    if (_currentPWM > prePWM) _currentPWM = prePWM;
  }
  _speedControl = false;
  
}*/

bool first = true;
void dcMotorController::maxVelocityController(){
  if(!_speedControl){
    prePWM = _currentPWM; 
    if(maxVelMode) _currentPWM = preSpeedPWM;
    
    if(_currentVelocity <= 0) {
      setVelocity(-_maxVelocity);
      if (preSpeedPWM > 0 ) preSpeedPWM = prePWM;
    }
    else if(_currentVelocity > 0){
      setVelocity(_maxVelocity);
      if (preSpeedPWM < 0 ) preSpeedPWM = prePWM;
    }
   
    _speedControl = true;
    speedController();
    _speedControl = false;

    //Serial.println("Current PWM: " +String(_currentPWM)+ "    Pre PWM: " +String(prePWM) + "    Current Vel: " + _currentVelocity);
    if(_currentPWM < 0){
      if (_currentPWM < prePWM) _currentPWM = prePWM;
    }
    else if(_currentPWM > 0){
      if (_currentPWM > prePWM) _currentPWM = prePWM;
    }
    else maxVelMode = true;
    preSpeedPWM = _currentPWM;
  }
  
}

/*
else if (currentTime > preSpeedSetTime + 1000000/speedLoopFrequency){

    float maxInc = _maxAcceleration / speedLoopFrequency;
    float maxDec = _maxDeceleration / speedLoopFrequency;
    
    preSpeedSetTime = currentTime;
    if (_targetVelocity < _currentVelocity){
      if (_currentPWM > 0){_currentPWM -= maxDec;}
      else{_currentPWM -= maxInc; }
      
      if (_currentPWM < _targetVelocity ) _currentPWM = _targetVelocity;
    }
    else if (_targetVelocity > _currentVelocity){ 
      if (_currentPWM < 0){_currentPWM += maxDec; }
      else{_currentPWM += maxInc; }
      
      if (_currentPWM > _targetVelocity ) _currentPWM = _targetVelocity;
    }
      
    if (_currentPWM >  -_maxDeceleration / speedLoopFrequency && _currentPWM <  _maxDeceleration / speedLoopFrequency && _targetVelocity == 0) _currentPWM = 0;
    else if (_currentPWM > _maxVelocity) _currentPWM = _maxVelocity;
    else if (_currentPWM < -_maxVelocity) _currentPWM = -_maxVelocity;

    Serial.print("_currentVelocity: ");
    Serial.print(_currentVelocity);
    Serial.print("     _targetPWM: ");
    Serial.print(_targetVelocity);
    Serial.print("     _currentPWM: ");
    Serial.println(_currentPWM);
  }
*/
