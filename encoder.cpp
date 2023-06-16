#include <Wire.h>

#include "device.h"
#include "params.h"

using Params::enc_cycle, Params::gear_d, Params::control_interval_sec, Params::reverse_wheel_enc;

#define enc_lowpass 0.8  //for enc to wheelangularvel lowpass

struct EncoderDevice {
  explicit EncoderDevice(bool reverse, int aPinID, int bPinID) 
  : rawDiff(0), aPinID(aPinID), bPinID(bPinID), theta(0.0f), omega(0.0f)
  {
    state = getState();

    scale = diff / enc_cycle / gear_d * 2 * M_PI / 4;
    if(reverse){
        scale = -scale;
    }

    last_interrupt_time = micros() - 1;
    last_interrupt_direction = 0.0f;
  }

  int getState(){
    int aPin = digitalRead(aPinID);
    int bPin = digitalRead(bPinID);
    int ret = 2*aPin + (aPin^bPin);
    return ret;
  }

  void update()
  {
    float dTheta = scale * rawDiff;
    rawDiff = 0;

    theta += dTheta;
    
    float omega0 = dTheta / control_interval_sec;
    float omega_lp = enc_lowpass * omega + (1-enc_lowpass) * omega0;
    float omega_t = last_interrupt_direction / last_interrupt_time;

    omega = 0.5f * omega0 + 0.5f * omega_t;
  }

  void pinInterrupt()
  {
    int new_state = getState();
    int t = micros();

    int diff = new_state - state;
    if(diff == 3){
        diff = -1;
    }else if(diff == -3){
        diff = 1;
    }
    rawDiff += diff;

    last_interrupt_time = t;
    last_interrupt_direction = static_cast<float>(diff);
  }

  float getTheta(){ return theta; };
  float getOmega(){ return omega; };
private:
  int rawDiff;
  int aPinID;
  int bPinID;
  int state;

  int last_interrupt_time;
  float last_interrupt_direction;

  float theta;
  float omega;
  float scale;
};

std::array<EncoderDevice, 4> wheel_encs = {
  EncoderDevice(reverse_wheel_enc[0], ENC_LF, ENC_LF2),
  EncoderDevice(reverse_wheel_enc[1],ENC_LB, ENC_LB2),
  EncoderDevice(reverse_wheel_enc[2],ENC_RB, ENC_RB2),
  EncoderDevice(reverse_wheel_enc[3],ENC_RF, ENC_RF2),
};

void calc_wheel_ang_vel(){
  for(int i=0; i<4; i++){ 
    wheel_encs[i].update();
    SensorValue::wheel_theta[i] = wheel_encs[i].getTheta();
    SensorValue::wheel_omega[i] = wheel_encs[i].getOmega();
  }
}

void rotRotEnc_LF(void)//エンコーダの計測関数
{
  wheel_encs[0].pinInterrupt();
}

void rotRotEnc_LB(void)//エンコーダの計測関数
{
  wheel_encs[1].pinInterrupt();
}

void rotRotEnc_RB(void)//エンコーダの計測関数
{
  wheel_encs[2].pinInterrupt();
}
void rotRotEnc_RF(void)//エンコーダの計測関数
{
  wheel_encs[3].pinInterrupt();
}
