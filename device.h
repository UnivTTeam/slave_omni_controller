#pragma once

#include <array>

// センサ系統
namespace SensorValue{
inline std::array<float, 4> wheel_theta{0.0f, 0.0f, 0.0f, 0.0f};
inline std::array<float, 4> wheel_omega{0.0f, 0.0f, 0.0f, 0.0f};

extern volatile float x;
extern volatile float y;
extern volatile float theta;
}

// 子機のプログラムでは読み取る変数
namespace TargetValue{
extern volatile float vel_x;
extern volatile float vel_y;
extern volatile float angular_vel;
extern volatile bool emergency;
}

// 子機のプログラムでは書き込む変数
namespace CommandValue{
inline std::array<float, 4> wheel_pwm{0.0f, 0.0f, 0.0f, 0.0f};
}

void setupDevice();
void send_pwm();
