#include "Comms.h"

Comms::Comms(){}
Comms::~Comms(){}

void Comms::inital(){
  //WiFi.begin("NETGEAR82", "mitch238");
  WiFi.softAP("Vertical_Axis", "123456789");
  
  if(udp.listen(55667)) {
    udp.onPacket([&](AsyncUDPPacket packet) {
        String msg = "";
        for (int i = 0; i < packet.length(); i++) msg+= (char)packet.data()[i];
        commuicationHandler(msg);
    });
  }
}


float preSentTime = 0;
void Comms::commsLoop(float currentPos, float currrentVelocity, float currentForce)
{
  if (millis() > preSentTime + 10)
  {
    //String msg = "pos:" + String(currentPos) + "|velcoity:" + String(currrentVelocity) + "|force:" + String(currentForce) + "|calibrating:" + String(calibarteFlag)+ "|stopped:" + String(stopMovment);
    //String msg = "p:" + String(currentPos) + "|v:" + String(currrentVelocity) + "|f:" + String(currentForce) + "|c:" + String(calibarteFlag)+ "|s:" + String(stopMovment);

    String caliStr = "0";
    if(calibarteFlag) caliStr = "1";
    
    String stopStr = "0";
    if(stopMovment) stopStr = "1";

    String speedRunB = "0";
    if (speedRun) speedRunB = "1";
    
    //String msg = "p:" + String(currentPos) + "|v:" + String(currrentVelocity) + "|f:" + String(currentForce) + "|c:" + caliStr+ "|s:" + stopStr + "|mt:" + maxTravle 
    String msg = "p:" + String(currentPos) + "|v:" + String(currrentVelocity) + "|f:" + String(currentForce) + "|c:" + caliStr+ "|s:" + stopStr + "|mt:" + maxTravle + "|rt:" + String(runTime) + "|sp:" + speedRunB;

    if (gotMSG) gotMSG = false;
    sendUDPComms(msg, 55776);
    sendSerialData(msg);
    preSentTime = millis();
  }
  
  if (Serial.available() > 0){
    serialMsg = "";
    while((Serial.available() > 0)){
      char currentChar = Serial.read();
      if (currentChar == '?'){
        commuicationHandler(serialMsg);
        break;
      }
      else serialMsg += currentChar;
    }
    //Serial.flush();
  }
}

void Comms::sendUDPComms(String msg, int port){
  udp.broadcastTo(msg.c_str(), port);
}

void Comms::sendSerialData(String msg){
  Serial.println("@" + msg + "?");
}

String Command = "";
String Data = "";
bool gotCmd = false;
  
void Comms::commuicationHandler(String msg)
{
  Command = "";
  Data = "";
  gotCmd = false;
  
  for (int i = 0; i < msg.length(); i++){
    if(msg[i] == ':') gotCmd = true;
    else if (gotCmd) Data += msg[i];
    else Command += msg[i];
  }
  
  if (Command == "setPos") targetPos = Data.toFloat();
  else if (Command == "setForce") targetForce = Data.toFloat();
  else if (Command == "setVel") targetVel = Data.toFloat();
  else if (Command == "vrControl") VRControl = true;
  else if (Command == "patientControl") VRControl = false;
  else if (Command == "forceControll") { forceControl = true; positionControl = false; velocityControl = false;}
  else if (Command == "positionControll") { forceControl = false; positionControl = true; velocityControl = false;}
  else if (Command == "velocityControl") { forceControl = false; positionControl = false; velocityControl = true;}
  
  else if (Command == "stop") { stopMovment = true;}
  else if (Command == "start") { stopMovment = false;}
  else if (Command == "calibrate") { calibarteFlag = true;}

  else if (Command == "sRun") { speedRun = true;}
}
