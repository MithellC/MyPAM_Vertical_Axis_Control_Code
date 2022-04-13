#ifndef ENC
#define ENC

#include <Wire.h>
#include<Arduino.h>

class encoder 
{
 public:
  encoder();
  ~encoder();

  void encoderLoop();
  void debugInfo();
  
  float currentPos = 0; 
  float currentVel = 0;
  
  bool encReady = false; 

 private:   
  float lowPassFilter(float rawVelcoity);
  float preWeightFilter(float rawVelcoity);
  float movingAverageFilter(float rawVelcoitys[]);
  
  int encoderVal = 0;
  double absAngle = 0;
  int preQuadrent = 0;
  int currentQuadrent = 0;
  int numberOfTurns = 0;
  double totalAngle = 0;
  double angleOffset = 0;
  
  double prePos = 0;
  
  double currentTime = 0; 
  double preTime = 0; 

  int limitBottomPin = 25;
  int limitTopPin = 26;
};

#endif
