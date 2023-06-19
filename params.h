#pragma once

#include "libwheels/transform2d/static_transform.hpp"

// 制御パラメタを入れるファイル
namespace Params{

// 制御周期: 20ms
inline constexpr float CONTROL_INTERVAL_MS = 20.0f;
inline float control_interval_sec = CONTROL_INTERVAL_MS / 1000.f;
inline float current_time = 0.0f;

//機体パラメータ
inline constexpr float wheel_R = 51.0f; //mm,タイヤの半径
inline constexpr float gear_d = 30.0f; //減速比
inline constexpr float enc_cycle = 16.0f;//一周あたりの分解能
inline constexpr int MAX_PWM = 250;
inline constexpr int MAX_PWM_DIFF = 60;
inline constexpr std::array<float, 4> max_wheel_anuglar_vel{
  6.5f * 2.0f * M_PI,
  6.5f * 2.0f * M_PI,
  6.5f * 2.0f * M_PI,
  6.5f * 2.0f * M_PI,
};

// drive params
inline const std::array<float, 4> pwm_per_omega = {4.377292f, 4.372072f, 4.487340f, 5.352004f};
inline constexpr float pwm0_offset = 30.0f;
inline const std::array<float, 4> pwm0 = {39.426388f+pwm0_offset, 32.658947f+pwm0_offset, 32.848671f+pwm0_offset, 7.160605f+pwm0_offset};

inline constexpr float WHEEL_FREE_THRESH_OMEGA = 50.0f / wheel_R;
inline constexpr float WHEEL_FAST_THRESH_OMEGA = 200.0f / wheel_R;

// マシン座標上のタイヤ位置(x, y, theta)
inline const Transform::StaticTransform<float> lf_frame( 299.81f,  289.71f, 0.75 * M_PI);
inline const Transform::StaticTransform<float> lb_frame(-299.81f,  289.71f,-0.75 * M_PI);
inline const Transform::StaticTransform<float> rb_frame(-299.81f, -289.71f,-0.25 * M_PI);
inline const Transform::StaticTransform<float> rf_frame( 299.81f, -289.71f, 0.25 * M_PI);

// デバイスパラメタ
inline constexpr std::array<bool, 4> reverse_wheel_motor{true, false, false, false};
inline constexpr std::array<bool, 4> reverse_wheel_enc{false, false, false, true};

#define USE_ENCODER_FULL_RESOLUTION

// 制御パラメタ
inline float WheelFeedbackKp = 8.0f;
inline float WheelFeedbackKd = 8.0f;

inline constexpr bool ENC_MAYBE_DEAD = true;

inline constexpr float MASTER_TIMEOUT_SEC = 0.2f;
inline float last_communication_time = -Params::MASTER_TIMEOUT_SEC;
}

#define ENC_LF 32
#define ENC_LF2 33
#define ENC_LB 25
#define ENC_LB2 26
#define ENC_RB 27
#define ENC_RB2 14
#define ENC_RF 12
#define ENC_RF2 13
