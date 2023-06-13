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
inline const float max_angular_vel = 6.0f*M_PI ;//車輪の最大角速度
inline const float max_wheel_vel = max_angular_vel * wheel_R;

// マシン座標上のタイヤ位置(x, y, theta)
inline const Transform::StaticTransform<float> lf_frame(0.0f, 0.0f, M_PI / 2);
inline const Transform::StaticTransform<float> lb_frame(0.0f, 0.0f, M_PI / 2);
inline const Transform::StaticTransform<float> rb_frame(0.0f, 0.0f, M_PI / 2);
inline const Transform::StaticTransform<float> rf_frame(0.0f, 0.0f, M_PI / 2);
}

#define ENC_LF 13
#define ENC_LF2 12
#define ENC_LB 14
#define ENC_LB2 27
#define ENC_RB 26
#define ENC_RB2 25
#define ENC_RF 33
#define ENC_RF2 32

