#include "LEDController.h"

#define LED_PIN 18
#define NUM_LEDS 24

CRGB leds[NUM_LEDS];
EventTimer _ledTimer;

LEDController::LEDController(int ledPin, float maxTravle,int numOfLeds, int numTravleLEDs, int posOffset){
  pinMode(LED_PIN, OUTPUT);
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  
  _maxTravelDis = maxTravle;
  _numTravleLEDs = numTravleLEDs;
  _numOfLeds = numOfLeds;
  _posOffset = posOffset;
  _ledTimer.eventFrequency = 50.0;
}

LEDController::~LEDController(){}

void LEDController::LEDLoop(){
  if (ledState != _preLedState){
    _preLedState = ledState;
    UpdateLEDEffect();
    AnimationHandler();
  }
  else if(checkTimerState()){
    UpdateLEDEffect();
    AnimationHandler();
  }
}


void LEDController::UpdateLEDEffect(){
  switch (ledState){
    case errorState:
      ErrorState();
      break;
      
    case calibrating:
      CalibrationState();
      break;
      
    case playMode:
      PlayModeState();
      break;
      
    case funMode:
      RainbowMode();
      break;
  }
  
}


float errorBrightness = 0;
bool brightDir = false;

void LEDController::ErrorState()
{
  if (brightDir){
    errorBrightness += 0.25;
    if (errorBrightness >= 255){
      errorBrightness = 255;
      brightDir = !brightDir;
    }
  }
  
  else{
    errorBrightness -= 0.25;
    if (errorBrightness <= 0){
      errorBrightness = 0;
      brightDir = !brightDir;
    }
  }

  fill_solid(leds, _numOfLeds, CHSV(255,255,255));
  FastLED.setBrightness(errorBrightness);
  FastLED.show();
}

int preState = 0; 
float fadeVal = 0;
int fadeDir = true;

void LEDController::SetCalibrationState(int state){
  _calibrationState = state;
  ledState = calibrating;
}

void LEDController::CalibrationState()
{
  fill_solid(leds, _numOfLeds, CHSV(80,255,255));
  if (preState != _calibrationState){
    preState = _calibrationState;
    fadeVal = 0;
    fadeDir = true;
  }

  switch(_calibrationState){
    case 0:
      InOutFade(PosToLED(0), 220);
      break;
    case 1:
      InOutFade(PosToLED(_maxTravelDis), 220);
      break;
    case 2:
      ledState = playMode;
      _animationState = calibrated;
      break;
  }
  
  FastLED.setBrightness(255);
  FastLED.show();
}



void LEDController::InOutFade(int ledPos, int colour){
  if (fadeDir) {
    fadeVal += 0.05;
    if (fadeVal > 1){
      fadeVal = 1; 
      fadeDir = !fadeDir;
    }
  }
  
  else {
    fadeVal -= 0.05;
    if (fadeVal < 0){
      fadeVal = 0; 
      fadeDir = !fadeDir;
    }
  }
  leds[(int)(ledPos/100)] = CHSV(colour *(fadeVal) + rgb2hsv_approximate(leds[(int)(ledPos/100)]).hue * (1 - fadeVal) , 255, 255);
  leds[23 - (int)(ledPos/100)] = CHSV(colour *(fadeVal) + rgb2hsv_approximate(leds[(int)(ledPos/100)]).hue * (1 - fadeVal) , 255, 255);
}


void LEDController::SetPos(int pos){
  if (pos > _maxTravelDis + _posOffset) _currentPos = _maxTravelDis + _posOffset;
  else _currentPos = pos + _posOffset;
}

void LEDController::SetMaxTravle(int dis){
  _maxTravelDis = dis;
}

int green = 80;
int minBlue = 120;
void LEDController::PlayModeState()
{
  fill_solid(leds, _numOfLeds, CHSV(green,255,255));
  
  int tragetLED = PosToLED(_targetPos);
  if (tragetLED < 0) tragetLED = 0;
  DispPlayLEDS(tragetLED, 220);
  
  int currentLED = PosToLED(_currentPos);
  if (currentLED < 0) currentLED = 0;
  DispPlayLEDS(currentLED, 170);

  FastLED.setBrightness(255);
  FastLED.show();

}

int LEDController::PosToLED(int pos){
  return map(pos*10, _posOffset, (_maxTravelDis + _posOffset) * 10, 0, ((_numTravleLEDs - 1)*100));;
}


void LEDController::DispPlayLEDS(int ledVal, int colour){
  float weight = ((float)(ledVal%100) / 100.0);
  int ledIndex = ledVal/100;
  leds[ledIndex] = CHSV(colour, 255, 255);
  leds[23 - ledIndex] = CHSV(colour, 255, 255);
  
  if (ledVal > 100){
    leds[ledIndex - 1] = CHSV(colour *(1-weight) + rgb2hsv_approximate(leds[ledIndex - 1]).hue *weight , 255, 255);
    leds[24 - ledIndex] = CHSV(colour *(1-weight) + rgb2hsv_approximate(leds[24 - ledIndex]).hue *weight , 255, 255);
  }
  if (ledVal < (_numTravleLEDs) * 100){
    leds[ledIndex + 1] = CHSV(colour * weight + rgb2hsv_approximate(leds[ledIndex + 1]).hue * (1-weight) , 255, 255);
    leds[22 - ledIndex] = CHSV(colour * weight + rgb2hsv_approximate(leds[22 - ledIndex]).hue * (1-weight) , 255, 255);
  }
}


float rainbowHue = 0;
void LEDController::RainbowMode()
{
  rainbowHue += 0.025;
  if (rainbowHue >= 255) rainbowHue = 0;
 
  fill_solid(leds, _numOfLeds, CHSV(rainbowHue,255,255));
  FastLED.setBrightness(255);
  FastLED.show();
}


bool LEDController::checkTimerState(){
  float currentTime = micros();
  if (currentTime >= (_ledTimer.previousTime + (1/_ledTimer.eventFrequency)*1000000 )){
    _ledTimer.previousTime = currentTime;
    return true;
  }
  return false;
}


void LEDController::ReachedTarget(){
  _animationState = targetReached;
}


void LEDController::AnimationHandler(){
  switch(_animationState){
    case targetReached:
      TargetReachedAnimation();
      break;
      
    case calibrated:
      CalibratedAnimation();
      break;
      
    case none:
      _animationPos = 0;
     break;
  }
}


void LEDController::TargetReachedAnimation(){
  _animationPos += 0.5;
  int ledColour = 0;
  for (int i = (int)_animationPos; i >= 0; i--){
    if (i < 11 && ledColour < 210) {
      leds[i] = CHSV(ledColour, 255, 255);
      leds[23 - i] = CHSV(ledColour, 255, 255);
    }
    ledColour += 210/12; 
  }
  if (_animationPos > 24) _animationState = none;
}

void LEDController::CalibratedAnimation(){
  _animationPos += 5;
  if(_animationPos > _maxTravelDis * 2){
    if (_flipAnimation){
       _animationState = none;
       return;
    }
    _flipAnimation = true;
    _animationPos = 0;
  }

  if(!_flipAnimation){
    int ledPos = PosToLED(_animationPos);
    int minLED = ledPos/100 - PosToLED(_maxTravelDis)/100;
    for(int i = PosToLED(0); i < ledPos/100; i++){
      if (i < PosToLED(_maxTravelDis)/100 && i >  minLED){
        leds[i] = CHSV(220, 255, 255);
        leds[23 - i] = CHSV(220, 255, 255);
      }
    }
  }

  else{
     int ledPos = PosToLED(_maxTravelDis - _animationPos);
     int maxLED = PosToLED(_maxTravelDis)/100 + ledPos/100;
     for(int i = PosToLED(_maxTravelDis)/100; i >= ledPos/100; i--){
      if (i <= maxLED && i > PosToLED(0)){
        leds[i] = CHSV(220, 255, 255);
        leds[23 - i] = CHSV(220, 255, 255);
      }
    }
  }
}
