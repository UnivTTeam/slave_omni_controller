#pragma once

#include "libwheels/transform2d/static_transform.hpp"

// 制御パラメタを入れるファイル
namespace Params{

// 制御周期: 20ms
inline const float control_interval_us = 20000.0f;
inline const float control_interval_sec = control_interval_us / 1000000.f;

//機体パラメータ
inline const float wheel_R = 0.06f;//m,タイヤの半径
inline const float gear_d = 30.0f;//減速比
inline const float enc_cycle = 16.0f;//一周あたりの分解能
inline const int MAX_PWM = 250;
inline const std::array<float, 4> max_wheel_anuglar_vel{0.0f, 0.0f, 0.0f, 0.0f};

// マシン座標上のタイヤ位置(x, y, theta)
inline const Transform::StaticTransform<float> lf_frame( 250.0f,  250.0f, 0.75 * M_PI);
inline const Transform::StaticTransform<float> lb_frame(-250.0f,  250.0f,-0.75 * M_PI);
inline const Transform::StaticTransform<float> rb_frame(-250.0f, -250.0f,-0.25 * M_PI);
inline const Transform::StaticTransform<float> rf_frame( 250.0f, -250.0f, 0.25 * M_PI);

// デバイスパラメタ
inline const std::array<bool, 4> reverse_wheel{false, false, false, false};

// 制御パラメタ
inline const float WheeFeedbackKp = 4.0f;

// 通信パラメタ：親機と子機で同期する
inline const float MAX_PARA_VEL = 5.0f;
inline const float MAX_ROT_VEL = 5.0f;
}

#define ENC_LF 13
#define ENC_LF2 12
#define ENC_LB 14
#define ENC_LB2 27
#define ENC_RB 26
#define ENC_RB2 25
#define ENC_RF 33
#define ENC_RF2 32

