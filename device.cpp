#include <Wire.h>
#include <Arduino.h>

#include "wire_control.h"
#include "device.h"
#include "params.h"
#include "odometry.h"

#define IDC 23
#define OMNI_ID 0x34

int clipPwm(int pwm, bool reverse)
{
    using Params::MAX_PWM;
    pwm = max(pwm, -MAX_PWM);
    pwm = min(pwm, MAX_PWM);
    if(reverse){
      return -pwm;
    }else{
      return pwm;
    }
}

//モーター系
const int LF_A = 15;
const int LF_B = 2;
const int LB_A = 4;
const int LB_B = 16;
const int RB_A = 17;
const int RB_B = 5;
const int RF_A = 18;
const int RF_B = 19;

namespace CommandValue{
volatile int LF_pwm = 0;
volatile int LB_pwm = 0;
volatile int RB_pwm = 0;
volatile int RF_pwm = 0;
}

int pwm[8]={0,0,0,0,0,0,0,0};

namespace SensorRawValue{
//エンコーダ系
volatile int m_nOldRot_LF = 0;
volatile int m_nValue_LF  = 0;
volatile int m_nOldRot_LB = 0;
volatile int m_nValue_LB = 0;
volatile int m_nOldRot_RB = 0;
volatile int m_nValue_RB  = 0;
volatile int m_nOldRot_RF = 0;
volatile int m_nValue_RF  = 0;
}
namespace SensorValue{
volatile float angular_LF = 0.0f;
volatile float angular_LB = 0.0f;
volatile float angular_RB = 0.0f;
volatile float angular_RF = 0.0f;
}

//回転数制御用
float errLF = 0;
float errLF_past = 0;
float errLB = 0;
float errLB_past = 0;
float errRB = 0;
float errRB_past = 0;
float errRF = 0;
float errRF_past = 0;

float wheel_kp = 1.0;
float wheel_ki = 0.0;
float wheel_kd = 0.0;
float wheel_p_max = 5;

//状態
namespace TargetValue{
volatile int master_status = 0;//親機の状態
}
namespace CommandValue{
volatile int slave_status = 0;//本機の状態
}

//オドメトリ系
float pos_x = 0;//m
float pos_y = 0;//m
float yaw = 0;//rad

namespace TargetValue{
volatile float vel_x = 0.0f;
volatile float vel_y = 0.0f;
volatile float angular_vel = 0.0f;
}

//制御間隔(micro sec)
float dt = Params::control_interval_us;//20ms
unsigned long tmp_time = 0;
float stime = 0;
float stime_offset = 0;

void setupDevice() {
  Wire.begin(uint8_t(OMNI_ID));
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  Serial.begin(115200);
  // 使用するタイマーのチャネルと周波数を設定
  ledcSetup(0, 24000, 8);//24khz,8bitで設定
  ledcSetup(1, 24000, 8);
  ledcSetup(2, 24000, 8);//24khz,8bitで設定
  ledcSetup(3, 24000, 8);
  ledcSetup(4, 24000, 8);//24khz,8bitで設定
  ledcSetup(5, 24000, 8);
  ledcSetup(6, 24000, 8);//24khz,8bitで設定
  ledcSetup(7, 24000, 8);
  
  // 各Pinをチャネルへ接続
  ledcAttachPin(LF_A, 0);
  ledcAttachPin(LF_B, 1);
  ledcAttachPin(LB_A, 2);
  ledcAttachPin(LB_B, 3);
  ledcAttachPin(RB_A, 4);
  ledcAttachPin(RB_B, 5);
  ledcAttachPin(RF_A, 6);
  ledcAttachPin(RF_B, 7);

  
  pinMode(IDC,OUTPUT);
  
  pinMode(ENC_LF, INPUT_PULLUP);
  pinMode(ENC_LF2, INPUT_PULLUP);
  pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_LB2, INPUT_PULLUP);
  pinMode(ENC_RB, INPUT_PULLUP);
  pinMode(ENC_RB2, INPUT_PULLUP);
  pinMode(ENC_RF, INPUT_PULLUP);
  pinMode(ENC_RF2, INPUT_PULLUP);
  
  attachInterrupt(ENC_LF,rotRotEnc_LF,CHANGE);
  attachInterrupt(ENC_LB,rotRotEnc_LB,CHANGE);
  attachInterrupt(ENC_RB,rotRotEnc_RB,CHANGE);
  attachInterrupt(ENC_RF,rotRotEnc_RF,CHANGE);
} 

void send_pwm(){
  using namespace CommandValue;

  LF_pwm = clipPwm(LF_pwm, Params::reverse_wheel[0]);
  if(LF_pwm >=0){
    pwm[0]=LF_pwm;
    pwm[1]=0;
  }
  else if(LF_pwm <0){
    pwm[0]=0;
    pwm[1]=-LF_pwm;
  }
  
  LB_pwm = clipPwm(LB_pwm, Params::reverse_wheel[1]);
  if(LB_pwm >=0){
    pwm[2]=LB_pwm;
    pwm[3]=0;
  }
  else if(LB_pwm <0){
    pwm[2]=0;
    pwm[3]=-LB_pwm;
  }
  
  RB_pwm = clipPwm(RB_pwm, Params::reverse_wheel[2]);
  if(RB_pwm >=0){
    pwm[4]=RB_pwm;
    pwm[5]=0;
  }
  else if(RB_pwm <0){
    pwm[4]=0;
    pwm[5]=-RB_pwm;
  }
  
  RF_pwm = clipPwm(RF_pwm, Params::reverse_wheel[3]);
  if(RF_pwm >=0){
    pwm[6]=RF_pwm;
    pwm[7]=0;
  }
  else if(RF_pwm <0){
    pwm[6]=0;
    pwm[7]=-RF_pwm;
  }

  for(int i = 0; i < 8; i++){
    ledcWrite(i, pwm[i]);
  }
}




float dist(float x1,float y1,float x2,float y2){
  return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
}
