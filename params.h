#pragma once

#include "libwheels/transform2d/static_transform.hpp"

// 制御パラメタを入れるファイル
namespace Params{

// 制御周期: 20ms
inline const float control_interval_us = 20000.0f;
inline const float control_interval_sec = control_interval_us / 1000000.f;

//機体パラメータ
inline const float wheel_R = 60.0f; //mm,タイヤの半径
inline const float gear_d = 30.0f; //減速比
inline const float enc_cycle = 16.0f;//一周あたりの分解能
inline const int MAX_PWM = 250;
inline const std::array<float, 4> max_wheel_anuglar_vel{
  6.5f * 2.0f * M_PI,
  6.5f * 2.0f * M_PI,
  6.5f * 2.0f * M_PI,
  6.5f * 2.0f * M_PI,
};

// drive params
inline const std::array<float, 4> pwm_per_omega = {4.393683486709225f, 4.3728115096745634f, 4.563073569555043f, 5.355068242836175f};
inline const std::array<float, 4> pwm0 = {37.37094249490049f, 38.53972376000872f, 34.30724861253726f, 9.37891629552986f};
inline const float omega_thresh = 30.0f / wheel_R;

// マシン座標上のタイヤ位置(x, y, theta)
inline const Transform::StaticTransform<float> lf_frame( 299.81f,  289.71f, 0.75 * M_PI);
inline const Transform::StaticTransform<float> lb_frame(-299.81f,  289.71f,-0.75 * M_PI);
inline const Transform::StaticTransform<float> rb_frame(-299.81f, -289.71f,-0.25 * M_PI);
inline const Transform::StaticTransform<float> rf_frame( 299.81f, -289.71f, 0.25 * M_PI);

// デバイスパラメタ
inline const std::array<bool, 4> reverse_wheel_motor{false, false, false, false};
inline const std::array<bool, 4> reverse_wheel_enc{true, false, false, false};

// 制御パラメタ
inline const float WheeFeedbackKp = 4.0f;

}

#define ENC_LF 13
#define ENC_LF2 12
#define ENC_LB 14
#define ENC_LB2 27
#define ENC_RB 26
#define ENC_RB2 25
#define ENC_RF 33
#define ENC_RF2 32

