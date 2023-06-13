void calc_pwm(){
  //目標角速度と現在の角速度をpidして現在のPWM値+制御出力の計算
  //pwm範囲は  -250~250
   errLF = target_angular_LF - angular_LF;
   LF_pwm = pid_controller(LF_pwm,wheel_kp,wheel_ki,wheel_kd,wheel_p_max,errLF,errLF_past);
   LF_pwm = min(max(LF_pwm,-250),250);
   errLF_past = errLF;
   Serial.print("LF_pwm = ");
   Serial.println(LF_pwm);

   errLB = target_angular_LB - angular_LB;
   LB_pwm = pid_controller(LB_pwm,wheel_kp,wheel_ki,wheel_kd,wheel_p_max,errLB,errLB_past);
   LB_pwm = min(max(LB_pwm,-250),250);
   errLB_past = errLB;
   Serial.print("LB_pwm = ");
   Serial.println(LB_pwm);

   errRB = target_angular_RB - angular_RB;
   RB_pwm = pid_controller(RB_pwm,wheel_kp,wheel_ki,wheel_kd,wheel_p_max,errRB,errRB_past);
   RB_pwm = min(max(RB_pwm,-250),250);
   errRB_past = errRB;
   Serial.print("RB_pwm = ");
   Serial.println(RB_pwm);

   errRF = target_angular_RF - angular_RF;
   RF_pwm = pid_controller(RF_pwm,wheel_kp,wheel_ki,wheel_kd,wheel_p_max,errRF,errRF_past);
   RF_pwm = min(max(RF_pwm,-250),250);
   errRF_past = errRF;
   Serial.print("RF_pwm = ");
   Serial.println(RF_pwm);
}


void omni_vel_allocator(){

  //角速度指令を優先、余力で並進を行う
  int rest_for_trans = 250 - abs(angular_vel-128);
  //速度を犠牲にして移動方向を優先、正規化する
  float normalize_x = float(vel_x -128)/128;
  float normalize_y = float(vel_y -128)/128;
  float norm_size = sqrt(pow(normalize_x,2)+pow(normalize_y,2));
  if(norm_size < 1 && norm_size != 0){
    target_angular_LF = rest_for_trans *(- normalize_x/sqrt(2)+normalize_y/sqrt(2))+(angular_vel-128);
    target_angular_LB = rest_for_trans *(- normalize_x/sqrt(2)-normalize_y/sqrt(2))+(angular_vel-128);
    target_angular_RB = rest_for_trans *(  normalize_x/sqrt(2)-normalize_y/sqrt(2))+(angular_vel-128);
    target_angular_RF = rest_for_trans *(  normalize_x/sqrt(2)+normalize_y/sqrt(2))+(angular_vel-128);
  }
  else if(norm_size != 0){
    //ー250~0~250までで各輪の角速度指令を計算
    target_angular_LF = rest_for_trans *(- normalize_x/sqrt(2)/norm_size+normalize_y/sqrt(2)/norm_size)+(angular_vel-128);
    target_angular_LB = rest_for_trans *(- normalize_x/sqrt(2)/norm_size-normalize_y/sqrt(2)/norm_size)+(angular_vel-128);
    target_angular_RB = rest_for_trans *(  normalize_x/sqrt(2)/norm_size-normalize_y/sqrt(2)/norm_size)+(angular_vel-128);
    target_angular_RF = rest_for_trans *(  normalize_x/sqrt(2)/norm_size+normalize_y/sqrt(2)/norm_size)+(angular_vel-128); 
  }
  else{
    target_angular_LF = angular_vel-128;
    target_angular_LB = angular_vel-128;
    target_angular_RB = angular_vel-128;
    target_angular_RF = angular_vel-128;
  }

  Serial.print("target_angular_LF:");
  Serial.println(target_angular_LF);
  Serial.print("target_angular_LB:");
  Serial.println(target_angular_LB);
  Serial.print("target_angular_RB:");
  Serial.println(target_angular_RB);
  Serial.print("target_angular_RF:");
  Serial.println(target_angular_RF);

  //最大角速度を用いた角速度指令に落とし込む
  target_angular_LF = target_angular_LF /250 * max_angular_vel;
  target_angular_LB = target_angular_LB /250 * max_angular_vel;
  target_angular_RB = target_angular_RB /250 * max_angular_vel;
  target_angular_RF = target_angular_RF /250 * max_angular_vel;
}
