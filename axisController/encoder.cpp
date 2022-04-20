#include "encoder.h"

const int numOfMADataPoint = 100;
float MADataPoints [numOfMADataPoint];


encoder::encoder()
{
  Wire.begin();
  Wire.setClock(1000000);

  for(int i = 0; i < numOfMADataPoint; i++)MADataPoints[i] = 0.0;

}

encoder::~encoder(){};


void encoder::encoderLoop()
{
  Wire.beginTransmission(0x36);
  Wire.write(0x0E);
  Wire.endTransmission();
  Wire.requestFrom(0x36,2,true);
  encoderVal = (Wire.read() << 8 | Wire.read());
  absAngle = 360.0 - (encoderVal * (360.0/4096.0));


  if (absAngle >= 0 && absAngle < 90) currentQuadrent = 0;
  else if (absAngle >= 90 && absAngle < 180) currentQuadrent = 1;
  else if (absAngle >= 180 && absAngle < 270) currentQuadrent = 2;
  else if(absAngle >= 270 && absAngle <= 360) currentQuadrent = 3;
  
  //if (absAngle == 360) currentQuadrent = 3;
  //else currentQuadrent = (int)(absAngle/90.0);
  
  if (preQuadrent == 3 && currentQuadrent == 0) numberOfTurns++;
  else if (preQuadrent == 0 && currentQuadrent == 3) numberOfTurns--;
  preQuadrent = currentQuadrent;

  if (digitalRead(limitBottomPin)){ 
    angleOffset = absAngle; 
    numberOfTurns = 0;
    totalAngle = (absAngle - angleOffset) + 360.0 * numberOfTurns;
  }
  else totalAngle = (absAngle + 360.0 * numberOfTurns) - angleOffset;
  
  //Serial.println("Angle: " + (String)absAngle + "    num of turns: " + (String)numberOfTurns + "?");
  //currentPos = totalAngle / 360.0 * 8.0;
  
  currentPos = totalAngle / 360.0 * 4.0;
  
  currentTime = micros();
  currentRawVecloity = ((currentPos - prePos) / ((currentTime - preTime) / 1000000));
  if (currentRawVecloity > 400 || currentRawVecloity < - 400) currentRawVecloity = preRawVecloity;
  
  for(int i = numOfMADataPoint-1; i > 0; i--){MADataPoints[i] = MADataPoints[i-1];}
  
  MADataPoints[0] = currentRawVecloity;

  //currentVel = lowPassFilter(currentRawVecloity);
  currentVel = preWeightFilter(currentRawVecloity);
  //currentVel = movingAverageFilter(MADataPoints);
  //currentVel = currentRawVecloity;
  
  preRawVecloity = currentRawVecloity;
  
  preTime = currentTime;
  prePos = currentPos;
}


void encoder::debugInfo()
{
  /*Serial.print("abs Angle: ");
  Serial.print(absAngle);
  Serial.print("       total angle: ");
  Serial.print(totalAngle);
  Serial.print("       Pos: ");
  Serial.print(currentPos);
  Serial.print("    Velocity: ");
  Serial.println(currentVel);*/
}


float encoder::lowPassFilter(float rawVelcoity)
{
  return ( 0.969*(currentVel) + 0.0155*(preRawVecloity) + + 0.0155*(rawVelcoity) );
}


float weight = 0.985;
float encoder::preWeightFilter(float rawVelcoity)
{
  return ( (rawVelcoity)*(1 - weight) + currentVel*weight );
}


float encoder::movingAverageFilter(float rawVelcoitys[])
{
  float sum = 0;
  for(int i = 0; i < sizeof(rawVelcoitys); i++) sum += rawVelcoitys[i];
  
  return ( sum/sizeof(rawVelcoitys) ) ;
}
