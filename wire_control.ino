
void receiveEvent(int cnt)
{
  if(Wire.available()>1){
    vel_x = Wire.read();
    vel_y = Wire.read();
    angular_vel = Wire.read();
    master_status = Wire.read();
  }
}

void requestEvent(){
  //ログを送りつけたい
  //じつは直でwifiに流した方がいい？
  Wire.write(uint8_t(slave_status));
}
