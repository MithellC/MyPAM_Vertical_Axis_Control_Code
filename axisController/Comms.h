#ifndef COM
#define COM

#include "dcMotorController.h"

#include <WiFi.h>
#include "AsyncUDP.h"


class Comms{
  public: 
    Comms();
    ~Comms();
    
    void inital();
    void commsLoop(float currentPos, float currrentVelocity, float currentForce);

    float targetPos = 0;
    float targetVel = 0;
    float targetForce = 0;
    float maxTravle = 0;
    
    bool positionControl = true;
    bool forceControl = false;
    bool velocityControl = false;
    
    bool VRControl = false;
    
    bool stopMovment = false;
    bool calibarteFlag = false;

    float runTime = 99;
    bool speedRun = false;
    
  private: 
   AsyncUDP udp;

   void sendUDPComms(String msg, int port);
   void sendSerialData(String msg);
   void commuicationHandler(String msg);
   dcMotorController _dcMotor;

   bool gotMSG = false;
   String serialMsg; 
  
};

#endif
