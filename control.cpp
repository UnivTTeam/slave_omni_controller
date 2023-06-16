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

float limitVelocity() {
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
  return ratio;
}

void drive(float vel_x, float vel_y, float angular_vel) {
  std::array<float, 4> omega_dest = VOConv(vel_x, vel_y, angular_vel);

  for(int i=0; i<4; i++){
    CommandValue::wheel_pwm[i] = wheel_controller(i, omega_dest[i], SensorValue::wheel_omega[i]);
  }
}

enum class Mode {
  Normal = 0,
  DeviceCheck = 1,
  WheelParams = 2,
  DriveCheck = 3,
};

Mode mode = Mode::Normal;
constexpr float device_check_pwm = 100.0f;
constexpr float wheel_params_pwm = 100.0f;
constexpr float drive_check_vel = 100.0f;
constexpr float drive_check_omega = 0.5f;

void control() {
  using namespace SensorValue;
  using CommandValue::wheel_pwm;

  wheel_pwm = std::array<float, 4>{{0.0f, 0.0f, 0.0f, 0.0f}};

  if(mode == Mode::Normal){
    using namespace TargetValue;
    float ratio = limitVelocity();
    drive(ratio * vel_x, ratio * vel_y, angular_vel);

    // map log
    float t = micros() / (1000.0f * 1000.0f);
    Serial.printf("t: %f vx: %f vy: %f om: %f pwm: %f %f %f %f wheel_omega: %f %f %f %f\n", 
      t, TargetValue::vel_x, TargetValue::vel_y, TargetValue::angular_vel,
      wheel_pwm[0], wheel_pwm[1], wheel_pwm[2], wheel_pwm[3],
      wheel_omega[0], wheel_omega[1], wheel_omega[2], wheel_omega[3]);
  }else if(mode == Mode::DeviceCheck){
    int t = millis() / 1000;
    t %= 24;
    if(t%2 == 0){
      if(t<8){
        int i = t/2;
        wheel_pwm[i] = device_check_pwm;
      } else if(t<10){
        for(int i=0; i<4; i++){
          wheel_pwm[i] = device_check_pwm;
        }
      } else if(t<18){
        int i = t/2 - 5;
        wheel_pwm[i] = -device_check_pwm;
      } else if(t<20){
        for(int i=0; i<4; i++){
          wheel_pwm[i] = -device_check_pwm;
        }
      }
    }
    Serial.printf("%d ", t);
    for(int i=0; i<4; i++){
      Serial.printf(" %f %f ", wheel_theta[i], wheel_omega[i]);
    }
    Serial.printf("\n");
  }else if(mode == Mode::WheelParams){
    // wheel params
    float t = micros() / (1000.0f * 1000.0f);
    for(int i=0; i<4; i++){
      wheel_pwm[i] = 200.0f;
    }
    Serial.printf("%f %f %f %f %f\n", t,
        wheel_theta[0], wheel_theta[1], wheel_theta[2], wheel_theta[3]);
  }else if(mode == Mode::DriveCheck){
    Params::WheeFeedbackKp = -4.0f;
    int t = millis() / 1000;
    
    if(t%4 == 2){
      t--;
    }
    t /= 2;

    t %= 14;
    if(t%2 == 0){
      if(t==0){
        drive(drive_check_vel, 0.0f, 0.0f);
      } else if(t==2){
        drive(0.0f, drive_check_vel, 0.0f);
      } else if(t==4){
        drive(-drive_check_vel, 0.0f, 0.0f);
      } else if(t==6){
        drive(0.0f, -drive_check_vel, 0.0f);
      } else if(t==8){
        drive(0.0f, 0.0f, drive_check_omega);
      } else if(t==10){
        drive(0.0f, 0.0f, -drive_check_omega);
      } 
    }
    Serial.printf("t: %d vx: %f vy: %f om: %f pwm: %f %f %f %f wheel_omega: %f %f %f %f\n", 
      t, TargetValue::vel_x, TargetValue::vel_y, TargetValue::angular_vel,
      wheel_pwm[0], wheel_pwm[1], wheel_pwm[2], wheel_pwm[3],
      wheel_omega[0], wheel_omega[1], wheel_omega[2], wheel_omega[3]);
  }
}
