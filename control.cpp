#include <Arduino.h>

#include "control.h"
#include "params.h"
#include "device.h"
#include "libwheels/transform2d/transform2d.hpp"

const std::array<Transform::StaticTransform<float>, 4> 
    wheel_frames{Params::lf_frame, Params::lb_frame, Params::rb_frame, Params::rf_frame};

int wheel_controller(
      int i,
      float dest_angular_vel,
      float angular_vel)
{
  const Transform::StaticTransform<float>& wheel_frame = wheel_frames[i];
  const float max_wheel_anuglar_vel = Params::max_wheel_anuglar_vel[i];

  float angular_vel_error = angular_vel - dest_angular_vel;

  float omega_command = dest_angular_vel + (-Params::WheeFeedbackKp) * angular_vel_error;
  float pwm = Params::pwm_per_omega[i] * omega_command;
  if(omega_command > Params::omega_thresh){
    pwm += Params::pwm0[i];
  } else if(omega_command < -Params::omega_thresh){
    pwm -= Params::pwm0[i];
  } else{
    pwm = 0.0f;
  }
  return pwm;
}

std::array<float, 4> VOConv(float vx, float vy, float omega) {
  Transform::MultidiffTransform<float, 1> motion(
    Transform::StaticTransform<float>(0.0f, 0.0f, 0.0f),
    Transform::DynamicTransform<float>(vx, vy, omega)
  );
  std::array<float, 4> wheel_anuglar_vel;
  for(int i=0; i<4; i++){
    Transform::MultidiffTransform<float, 1> wheel_motion_from_field = motion + wheel_frames[i];
    Transform::MultidiffTransform<float, 1> wheel_dest = (-wheel_frames[i]) + wheel_motion_from_field;
    
    wheel_anuglar_vel[i] = wheel_dest.dynamic_frame[0].pos.x / Params::wheel_R;
  }
  return wheel_anuglar_vel;
}

std::array<float, 4> controller_impl(const std::array<float, 4>& angular_wheels) {
  using namespace TargetValue;
  std::array<float, 4> omega_rot = VOConv(0.0f, 0.0f, angular_vel);
  std::array<float, 4> omega_para = VOConv(vel_x, vel_y, 0.0f);
  float ratio = 1.0f;
  for(int i=0; i<4; i++){
    float limit = Params::max_wheel_anuglar_vel[i];
    if(std::abs(omega_rot[i]) >= limit){
      ratio = 0.0f;
    }else{
      float upper_bound = limit - omega_rot[i];
      float lower_bound = -limit - omega_rot[i];
      if(omega_para[i] > upper_bound){
        ratio = std::min(ratio, abs(omega_para[i] / upper_bound));
      }else if(omega_para[i] < lower_bound){
        ratio = std::min(ratio, abs(omega_para[i] / lower_bound));
      }
    }
  }
  
  std::array<float, 4> omega_dest = VOConv(ratio * vel_x, ratio * vel_y, angular_vel);
  std::array<float, 4> pwms;
  for(int i=0; i<4; i++){
    pwms[i] = wheel_controller(i, omega_dest[i], angular_wheels[i]);
  }

  return pwms;
}

void control() {
  using namespace SensorValue;

  std::array<float, 4> angulars{angular_LF, angular_LB, angular_RB, angular_RF};
  std::array<float, 4> pwms = controller_impl(angulars);
  
  /*
  // デバッグ用
  float t = micros() / (1000.0f * 1000.0f);
  Serial.printf("%f ", t);
  for(int i=0; i<4; i++){
    pwms[i] = 200.0f;
  }
  static std::array<float, 4> thetas{0.0f, 0.0f, 0.0f, 0.0f};
  for(int i=0; i<4; i++){
    thetas[i] += angulars[i] * Params::control_interval_sec;
    Serial.printf("%f ", thetas[i]);
  }
  Serial.printf("\n");
  */
  Serial.printf("%f %f %f %f %f %f %f\n", 
    TargetValue::vel_x, TargetValue::vel_y, TargetValue::angular_vel,
    pwms[0], pwms[1], pwms[2], pwms[3]);

  // float t = micros() / (1000.0f * 1000.0f);
  // Serial.printf("%f %f %f %f %f\n", t, angular_LF, angular_LB, angular_RB, angular_RF);

  CommandValue::LF_pwm = pwms[0];
  CommandValue::LB_pwm = pwms[1];
  CommandValue::RB_pwm = pwms[2];
  CommandValue::RF_pwm = pwms[3];
}
