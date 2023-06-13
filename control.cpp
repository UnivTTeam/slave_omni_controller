#include "control.h"
#include "params.h"
#include "device.h"
#include "libwheels/transform2d/transform2d.hpp"

const std::array<Transform::StaticTransform<float>, 4> 
    wheel_frames{Params::lf_frame, Params::lb_frame, Params::rb_frame, Params::rf_frame};

void setupController() {
}

int wheel_controller(
      float dest_vel,
      const Transform::StaticTransform<float>& wheel_frame,
      float angular_vel)
{
  float dest_angular_vel = dest_vel / Params::wheel_Reff;
  float angular_vel_error = angular_vel - dest_angular_vel;

  float omega = dest_angular_vel + (-Params::WheeFeedbackKp) * angular_vel_error;
  float pwm = Params::MAX_PWM * (omega / Params::max_angular_vel);
  return pwm;
}

std::array<float, 4> VOConv(float vx, float vy, float omega) {
  Transform::MultidiffTransform<float, 1> robot_dest(
    Transform::StaticTransform<float>(0.0f, 0.0f, 0.0f),
    Transform::DynamicTransform<float>(vx, vy, omega)
  );
  std::array<float, 4> ret;
  for(int i=0; i<4; i++){
    Transform::MultidiffTransform<float, 1> wheel_dest = robot_dest + wheel_frames[i];
    ret[i] = wheel_dest.static_frame.pos.x;
  }
  return ret;
}

std::array<float, 4> controller_impl(const std::array<float, 4>& angular_wheels) {
  using namespace TargetValue;
  std::array<float, 4> v_rot = VOConv(0.0f, 0.0f, angular_vel);
  std::array<float, 4> v_para = VOConv(vel_x, vel_y, 0.0f);
  float ratio = 1.0f;
  for(int i=0; i<4; i++){
    if(std::abs(v_rot[i]) >= Params::max_wheel_vel){
      ratio = 0.0f;
    }else{
      float upper_bound = Params::max_wheel_vel - v_rot[i];
      float lower_bound = -Params::max_wheel_vel - v_rot[i];
      if(v_para[i] > upper_bound){
        ratio = std::min(ratio, v_para[i] / upper_bound);
      }else if(v_para[i] < lower_bound){
        ratio = std::min(ratio, v_para[i] / lower_bound);
      }
    }
  }
  
  std::array<float, 4> v = VOConv(ratio * vel_x, ratio * vel_y, angular_vel);
  std::array<float, 4> pwms;
  for(int i=0; i<4; i++){
    pwms[i] = wheel_controller(v[i], wheel_frames[i], angular_wheels[i]);
  }
  return pwms;
}

void control() {
  using namespace SensorValue;

  std::array<float, 4> pwms = controller_impl(std::array<float, 4>{angular_LF, angular_LB, angular_RB, angular_RF});

  CommandValue::LF_pwm = pwms[0];
  CommandValue::LB_pwm = pwms[1];
  CommandValue::RB_pwm = pwms[2];
  CommandValue::RF_pwm = pwms[3];
}
