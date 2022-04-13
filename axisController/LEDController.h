#ifndef LED
#define LED

#include<FastLED.h>

enum LedState{
    errorState,
    calibrating,
    playMode,
    funMode,
    animation
};

enum Animations{
  targetReached,
  calibrated,
  none
};

struct EventTimer{
  int previousTime = 0;
  float eventFrequency = 10; //Hz
};

class LEDController
{
  public: 
    LEDController(int ledPin, float maxTravle,int numOfLeds, int numTravleLEDs, int posOffset);
    ~LEDController();
  
    void LEDLoop();

    void SetPos(int pos);
    void SetMaxTravle(int dis);

    void SetCalibrationState(int state);
    void ReachedTarget();
    
    LedState ledState = playMode;

  private: 
    void UpdateLEDEffect();
    void ErrorState();

    int _calibrationState = 0;
    void CalibrationState();
    void InOutFade(int ledPos, int colour);
    
    void PlayModeState();
    void RainbowMode();

    bool _flipAnimation = false;
    int _animationPos = 0;
    void AnimationHandler();
    void TargetReachedAnimation();
    void CalibratedAnimation();
    
    bool checkTimerState();
    void DispPlayLEDS(int ledVal, int colour);
    int PosToLED(int pos);

    LedState _preLedState = playMode;
    Animations _animationState = none;
    
    int _numOfLeds = 24;
    int _numTravleLEDs = 10;
    int _maxTravelDis = 140;
    int _currentPos = 0;
    int _targetPos = 100;
    int _posOffset = 0;
};

#endif
