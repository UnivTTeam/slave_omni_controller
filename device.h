#pragma once

// センサ系統
namespace SensorRawValue{
extern volatile int m_nOldRot_LF;
extern volatile int m_nValue_LF;
extern volatile int m_nOldRot_LB;
extern volatile int m_nValue_LB;
extern volatile int m_nOldRot_RB;
extern volatile int m_nValue_RB;
extern volatile int m_nOldRot_RF;
extern volatile int m_nValue_RF;
}
namespace SensorValue{
extern volatile float angular_LF;
extern volatile float angular_LB;
extern volatile float angular_RB;
extern volatile float angular_RF;
}

// 子機のプログラムでは読み取る変数
namespace TargetValue{
extern volatile float vel_x;
extern volatile float vel_y;
extern volatile float angular_vel;
extern volatile int master_status;
}

// 子機のプログラムでは書き込む変数
namespace CommandValue{
extern volatile int LF_pwm;
extern volatile int LB_pwm;
extern volatile int RB_pwm;
extern volatile int RF_pwm;
extern volatile int slave_status;
}

void setupDevice();
void send_pwm();
