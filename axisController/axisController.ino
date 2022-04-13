#include <FastLED.h>

#include "stepperController.h"
#include "dcMotorController.h"
#include "encoder.h"
#include "Comms.h"
#include "LEDController.h"

int forceTopPin = 34;
int forceBottomPin = 35;
int limitBottomPin = 25;
int limitTopPin = 26;

int voltRefPin= 13;

int dir = 16;
int stepp = 15;

int in1 = 16;
int in2 = 15;
int pwmPin = 4;

float screwPitch = 4;

bool homed = false;

float armWeight = 0;
stepperController stepper;
dcMotorController dcMotor;

encoder enc;
Comms comms;
LEDController ledController(18, 120, 24, 9, 10);

void setup() {
  pinMode(voltRefPin,OUTPUT);
  digitalWrite(voltRefPin,HIGH);
  
  Serial.begin(512000);
  
  comms.inital();
  
  dcMotorSetup();
  homeDCAxis();

  armWeight =  dcMotor.bottomForceReading - dcMotor.topForceReading;
  dcMotor.enableForceControl();
  dcMotor.setForce(0);
  
  //dcMotor.enableGravity(true, -300);
  //dcMotor.enablePositionControl();
  //dcMotor.setTargetPos(65);
}


void dcMotorSetup(){
  dcMotor.setupMotor(in1, in2, pwmPin);
  dcMotor.setupController(forceTopPin, forceBottomPin);
  
  //dcMotor.setupLinearActuator(screwPitch, 3, 255, 2000, 4000, 100, limitTopPin, limitBottomPin);
  dcMotor.setupLinearActuator(screwPitch, 3, 80, 2000, 4000, 100, limitTopPin, limitBottomPin);
}


void loop() 
{
  //demoAid();
  //demoConstMove();
  
  commandsCheck();
  DCMotorCrtitcalLoop();
}


void commandsCheck()
{
  if(comms.speedRun) sRun();
  
  if(comms.calibarteFlag)homeDCAxis();

  if (comms.VRControl && comms.velocityControl && homed) dcMotor.enableSpeedControl();
  else if (comms.VRControl && comms.positionControl && homed) dcMotor.enablePositionControl();
  else if (comms.VRControl && comms.forceControl && homed) dcMotor.enableForceControl();
  else if (!comms.VRControl && homed) dcMotor.enableForceControl();
}


void homeDCAxis()
{
  homed = false;
  dcMotor.enableSpeedControl();

  ledController.SetCalibrationState(0);
  moveToBottomLimit();
  
  ledController.SetCalibrationState(1);
  moveToTopLimit();
  
  homed = true;
  enc.encReady = homed;
  comms.calibarteFlag = false;
  ledController.SetCalibrationState(2);
}


void moveToBottomLimit(){
  dcMotor.setVelocity(-40); 
  while (!digitalRead(limitBottomPin)) {DCMotorCrtitcalLoop();}
  debounce();
  
  dcMotor.setVelocity(40); 
  while (digitalRead(limitBottomPin)) {DCMotorCrtitcalLoop();}
  debounce();
  
  dcMotor.setVelocity(-10);
  while (!digitalRead(limitBottomPin)) {DCMotorCrtitcalLoop();}
  debounce();
}


void moveToTopLimit(){
  dcMotor.setVelocity(40); 
  while (!digitalRead(limitTopPin)) {DCMotorCrtitcalLoop();}

  dcMotor.setVelocity(-40); 
  while (digitalRead(limitTopPin)) {DCMotorCrtitcalLoop();}
  debounce();

  dcMotor.setVelocity(10);
  while (!digitalRead(limitTopPin)) {DCMotorCrtitcalLoop();}
  debounce();

  comms.maxTravle = enc.currentPos;
  ledController.SetMaxTravle(comms.maxTravle);
}


void debounce()
{
  dcMotor.setVelocity(0); 
  float startTime = millis();
  while (millis() < startTime + 100) dcMotor.controlLoop();
}


void DCMotorCrtitcalLoop()
{
  enc.encoderLoop();
  //enc.debugInfo();
  
  dcMotor.setCurrentPos(enc.currentPos);
  dcMotor.setCurrentVelcoity(enc.currentVel);

  comms.commsLoop(enc.currentPos, enc.currentVel, dcMotor.topForceReading - dcMotor.bottomForceReading);

  ledController.SetPos(enc.currentPos);
  ledController.LEDLoop();
  ledController.SetTraget(comms.targetPos);

  dcMotor.setTargetPos(comms.targetPos);
  if (homed) dcMotor.setVelocity(comms.targetVel);

  if (comms.stopMovment)dcMotor.emergencyStop();
  else if (!comms.stopMovment)dcMotor.resumeControl();
  
  dcMotor.controlLoop();
}


bool moveUp = true;
void sRun(){
  dcMotor.overridePWMState = true;
  
  float startTime = millis();
  if(moveUp){
    dcMotor.overridePWM = 254;
    while (!digitalRead(limitTopPin)){
       //commandsCheck();     
       DCMotorCrtitcalLoop();
    }
    moveUp = false;
  }
  
  else{
    dcMotor.overridePWM = -254;
    while (!digitalRead(limitBottomPin)){
       //commandsCheck();     
       DCMotorCrtitcalLoop();
    }
    moveUp = true;
  }
  float endTime = millis();

  dcMotor.overridePWMState = false;
  comms.speedRun = false;
  comms.runTime = endTime - startTime;
}










































void demoConstMove(){
  dcMotor.enableSpeedControl();
  dcMotor.setVelocity(-120); 
  while (!digitalRead(limitBottomPin)) {DCMotorCrtitcalLoop();}
  dcMotor.setVelocity(120); 
  while (!digitalRead(limitTopPin)) {DCMotorCrtitcalLoop();}
}


void demoAid()
{
  if (digitalRead(limitBottomPin)){
    dcMotor.enableForceControl();
    dcMotor.setForce(armWeight - 60);
  }
  else if (digitalRead(limitTopPin)){
    dcMotor.enableForceControl();
    dcMotor.setForce(armWeight - 200);
  }
}




////Stepper motor code

void stepperAidMode()
{
  if (digitalRead(limitBottomPin)){
    stepper.enableForceControl();
    stepper.setForce(-1400);
  }
  else if (digitalRead(limitTopPin)){
    stepper.enableForceControl();
    stepper.setForce(-1100);
  }
}

float maxSpringForce = 1300;
float springDist = 80;
void stepperSpringMode()
{
  if (stepper.zeroed){
    stepper.setForce(map(stepper.currentPos, 0, springDist, 0, 1300));
  }
}


void stepperSetup()
{
  stepper.setupStepper(1.8, 2.0, dir, stepp);
  //stepper.setupLinearActuator(screwPitch, 3, 150, 1300, 3000, 200, limitTopPin, limitBottomPin);
  stepper.setupLinearActuator(screwPitch, 3, 280, 1300, 3000, 200, limitTopPin, limitBottomPin);
  //stepper.setupLinearActuator(screwPitch, 3, 300, 800, 3000, 200, limitTopPin, limitBottomPin);
  stepper.setupController(forceTopPin, forceBottomPin);
  stepper.enableForceControl();
  stepper.setForce(0);
  //stepper.enableGravity(true, -600);
  //stepper.enableSpeedControl();
  //stepper.setVelocity(100);
}
