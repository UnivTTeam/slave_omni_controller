#include <Wire.h>
#include <Arduino.h>

#include "wire_control.h"
#include "device.h"
#include "params.h"
#include "encoder.h"

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

namespace GetPwmInternalValue {
// この変数は getPWMからしか使わない
std::array<int, 4> pwm_raw = {0, 0, 0, 0};
}

int getPwm(int i){
  using Params::MAX_PWM_DIFF;
  int last_pwm = GetPwmInternalValue::pwm_raw[i];
  int max_pwm = last_pwm + MAX_PWM_DIFF;
  int min_pwm = last_pwm - MAX_PWM_DIFF;

  int pwm = clipPwm(CommandValue::wheel_pwm[i], Params::reverse_wheel_motor[i]);

  if(!TargetValue::emergency){
    if(pwm > max_pwm){
      pwm = max_pwm;
    }else if(pwm < min_pwm){
      pwm = min_pwm;
    }
  }else{
    pwm = 0;
  }

  GetPwmInternalValue::pwm_raw[i] = pwm;
  return pwm;
}

//モーター系
constexpr int LF_A = 17;
constexpr int LF_B = 5;
constexpr int LB_A = 18;
constexpr int LB_B = 19;
constexpr int RB_A = 4;
constexpr int RB_B = 16;
constexpr int RF_A = 15;
constexpr int RF_B = 2;

// センサ系統
namespace SensorValue{
volatile float x = 0.0f;
volatile float y = 0.0f;
volatile float theta = 0.0f;
}

//状態
namespace TargetValue{
volatile float vel_x = 0.0f;
volatile float vel_y = 0.0f;
volatile float angular_vel = 0.0f;
volatile bool emergency = true;
}

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
  
#ifdef USE_ENCODER_FULL_RESOLUTION
  attachInterrupt(ENC_LF,rotRotEnc_LF,CHANGE);
  attachInterrupt(ENC_LB,rotRotEnc_LB,CHANGE);
  attachInterrupt(ENC_RB,rotRotEnc_RB,CHANGE);
  attachInterrupt(ENC_RF,rotRotEnc_RF,CHANGE);

  attachInterrupt(ENC_LF2,rotRotEnc_LF,CHANGE);
  attachInterrupt(ENC_LB2,rotRotEnc_LB,CHANGE);
  attachInterrupt(ENC_RB2,rotRotEnc_RB,CHANGE);
  attachInterrupt(ENC_RF2,rotRotEnc_RF,CHANGE);
#else
  attachInterrupt(ENC_LF,rotRotEnc_LF,RISING);
  attachInterrupt(ENC_LB,rotRotEnc_LB,RISING);
  attachInterrupt(ENC_RB,rotRotEnc_RB,RISING);
  attachInterrupt(ENC_RF,rotRotEnc_RF,RISING);
#endif
} 

void send_pwm(){
  int j = 0;

  for(int i=0; i<4; i++){
    int pwm = getPwm(i);

    if(pwm >= 0){
      ledcWrite(2*i, pwm);
      ledcWrite(2*i+1, 0);
    }else{
      ledcWrite(2*i, 0);
      pwm = -pwm;
      ledcWrite(2*i+1, pwm);
    }
  }
}
