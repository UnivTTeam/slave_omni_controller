#include<Wire.h>

#include "wire_control.h"
#include "device.h"
#include "params.h"
#include "common.h"

void receiveEvent(int cnt)
{
  using namespace TargetValue;
  if(Wire.available()>1){
    vel_x = decode_uint8(Wire.read(), Params::MAX_PARA_VEL);
    vel_y = decode_uint8(Wire.read(), Params::MAX_PARA_VEL);
    angular_vel = decode_uint8(Wire.read(), Params::MAX_ROT_VEL);
    master_status = Wire.read();
  }
}

void requestEvent(){
  using namespace CommandValue;
  //ログを送りつけたい
  //じつは直でwifiに流した方がいい？
  Wire.write(uint8_t(slave_status));
}
