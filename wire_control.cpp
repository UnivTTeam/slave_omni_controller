#include<Wire.h>

#include "wire_control.h"
#include "device.h"
#include "params.h"

float readFloatValue(){
  uint8_t bytes[4];
  for(int i=0; i<4; i++){
      bytes[i] = Wire.read();
  }

  float value;
  memcpy(&value, bytes, sizeof(float));
  return value;
}

void receiveEvent(int cnt)
{
  using namespace TargetValue;
  if(Wire.available()>1){
    vel_x = readFloatValue();
    vel_y = readFloatValue();
    angular_vel = readFloatValue();
  }
}

void requestEvent(){
  using namespace CommandValue;
  //ログを送りつけたい
  //じつは直でwifiに流した方がいい？
  Wire.write(uint8_t(slave_status));
}
