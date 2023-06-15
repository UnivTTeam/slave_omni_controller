#include <Wire.h>

#include "device.h"
#include "params.h"

using namespace SensorRawValue;
using namespace SensorValue;
using Params::enc_cycle, Params::gear_d, Params::control_interval_sec, Params::reverse_wheel_enc;

#define enc_lowpass 0.8  //for enc to wheelangularvel lowpass

struct EncoderDevice {
  explicit EncoderDevice(int aPinID, int bPinID) 
  : m_nOldRot(0), m_nValue(0), aPinID(aPinID), bPinID(bPinID)
  {
  }

  float getDiff()
  {
    float ret = m_nValue;
    m_nValue = 0.0f;
    return ret;
  }

  void update()
  {
    int aPin = digitalRead(aPinID);
    int bPin = digitalRead(bPinID);
    if(!aPin){  // ロータリーエンコーダー回転開始
      m_nOldRot = bPin?1:-1;
    } else { 
      if(bPin){
        if(m_nOldRot == -1){
          m_nValue--;
        }
      } else {
        if(m_nOldRot == 1){
          m_nValue++;
        }
      }
      m_nOldRot = 0;
    }    
  }

private:
  int m_nOldRot;
  int m_nValue;
  int aPinID;
  int bPinID;
};

std::array<EncoderDevice, 4> wheel_encs = {
  EncoderDevice(ENC_LF, ENC_LF2),
  EncoderDevice(ENC_LB, ENC_LB2),
  EncoderDevice(ENC_RB, ENC_RB2),
  EncoderDevice(ENC_RF, ENC_RF2),
};

float rawValueToTheta(int i, float diff)
{
  float theta = diff / enc_cycle / gear_d * 2 * M_PI;
  if(reverse_wheel_enc[i]){
    return -theta;
  }else{
    return theta;
  }
}

void calc_wheel_ang_vel(){
  for(int i=0; i<4; i++){
    float enc_diff = wheel_encs[i].getDiff();
    float dTheta = rawValueToTheta(0, enc_diff);
    wheel_theta[i] += dTheta;

    float omega = dTheta / control_interval_sec;
    wheel_omega[i] = enc_lowpass * wheel_omega[0] + (1-enc_lowpass) * omega;
  }
}

void rotRotEnc_LF(void)//エンコーダの計測関数
{
  wheel_encs[0].update();
}

void rotRotEnc_LB(void)//エンコーダの計測関数
{
  wheel_encs[1].update();
}

void rotRotEnc_RB(void)//エンコーダの計測関数
{
  wheel_encs[2].update();
}
void rotRotEnc_RF(void)//エンコーダの計測関数
{
  wheel_encs[3].update();
}
