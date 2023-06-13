#include<Wire.h>

#include "wire_control.h"
#include "device.h"
#include "params.h"

void receiveEvent(int cnt)
{
  using namespace TargetValue;
  if(Wire.available()>1){
    vel_x = Wire.read() * Params::MAX_PARA_VEL;
    vel_y = Wire.read() * Params::MAX_PARA_VEL;
    angular_vel = Wire.read() * Params::MAX_ROT_VEL;
    master_status = Wire.read();
  }
}

void requestEvent(){
  using namespace CommandValue;
  //ログを送りつけたい
  //じつは直でwifiに流した方がいい？
  Wire.write(uint8_t(slave_status));
}
