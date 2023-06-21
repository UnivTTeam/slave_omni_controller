#include <Arduino.h>
#include <vector>
#include <array>
#include <numeric>

#include "control.h"
#include "params.h"
#include "device.h"
#include "libwheels/transform2d/transform2d.hpp"
#include "wire_control.h"

using Params::current_time;

enum class Mode {
  Normal = 0,
  SensorCheck = 1,
  DeviceCheck = 2,
  WheelParams = 3,
  DriveCheck = 4,
  LoggerTest = 5,
  AutoWheelParams = 6,
};

Mode mode = Mode::Normal;
constexpr float device_check_pwm = 100.0f;
constexpr int wheel_param_target_id = 3;
constexpr float wheel_params_pwm = 80.0f;
constexpr float drive_check_vel = 100.0f;
constexpr float drive_check_omega = 0.5f;

const std::array<Transform::StaticTransform<float>, 4> 
    wheel_frames{Params::lf_frame, Params::lb_frame, Params::rb_frame, Params::rf_frame};

std::array<float, 4> last_wheel_anuglar_vel_error = {0.0f, 0.0f, 0.0f, 0.0f};
int wheel_controller(
      int i,
      float dest_angular_vel,
      float angular_vel)
{
  const Transform::StaticTransform<float>& wheel_frame = wheel_frames[i];
  const float max_wheel_anuglar_vel = Params::max_wheel_anuglar_vel[i];

  float angular_vel_error = angular_vel - dest_angular_vel;
  if(Params::ENC_MAYBE_DEAD){
    angular_vel_error = 0.0f;
  }

  float omega_command = dest_angular_vel 
    + (-Params::WheelFeedbackKp) * angular_vel_error
    + (-Params::WheelFeedbackKd) * (angular_vel_error - last_wheel_anuglar_vel_error[i]);
  float pwm = Params::pwm_per_omega[i] * omega_command;
  if(omega_command > Params::WHEEL_FREE_THRESH_OMEGA){
    float ratio = min(omega_command / Params::WHEEL_FAST_THRESH_OMEGA, 1.0f);
    pwm += ratio * Params::pwm0[i];
  } else if(omega_command < -Params::WHEEL_FREE_THRESH_OMEGA){
    float ratio = min(-omega_command / Params::WHEEL_FAST_THRESH_OMEGA, 1.0f);
    pwm -= ratio * Params::pwm0[i];
  } else{
    pwm = 0.0f;
  }
  last_wheel_anuglar_vel_error[i] = angular_vel_error;
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

namespace AutoWheelParam {
void auto_wheel_params();
}

void control() {
  using namespace SensorValue;
  using CommandValue::wheel_pwm;

  wheel_pwm = std::array<float, 4>{{0.0f, 0.0f, 0.0f, 0.0f}};

  if(mode == Mode::Normal){
    using namespace TargetValue;
    float ratio = limitVelocity();
    drive(ratio * vel_x, ratio * vel_y, angular_vel);

    // map log
    Serial.printf("t: %f vx: %f vy: %f om: %f pwm: %f %f %f %f wheel_omega: %f %f %f %f\n", 
      current_time, TargetValue::vel_x, TargetValue::vel_y, TargetValue::angular_vel,
      wheel_pwm[0], wheel_pwm[1], wheel_pwm[2], wheel_pwm[3],
      wheel_omega[0], wheel_omega[1], wheel_omega[2], wheel_omega[3]);
  }else if(mode == Mode::SensorCheck){
    Serial.printf("%f ", current_time);
    for(int i=0; i<4; i++){
      Serial.printf(" %f %f ", wheel_theta[i], wheel_omega[i]);
    }
    Serial.printf("\n");
  }else if(mode == Mode::DeviceCheck){
    TargetValue::emergency = false;
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
    TargetValue::emergency = false;
    // wheel params
    for(int i=0; i<4; i++){
      if(wheel_param_target_id == -1 || wheel_param_target_id == i){
        wheel_pwm[i] = wheel_params_pwm;
      }
    }
    Serial.printf("%f %f %f %f %f\n", current_time,
        wheel_theta[0], wheel_theta[1], wheel_theta[2], wheel_theta[3]);
  }else if(mode == Mode::DriveCheck){
    TargetValue::emergency = false;
    Params::WheelFeedbackKp = -4.0f;
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
    Serial.printf("t: %d emg:%s vx: %f vy: %f om: %f pwm: %f %f %f %f wheel_omega: %f %f %f %f\n", 
      t, (TargetValue::emergency ? "true" : "false"),
      TargetValue::vel_x, TargetValue::vel_y, TargetValue::angular_vel,
      wheel_pwm[0], wheel_pwm[1], wheel_pwm[2], wheel_pwm[3],
      wheel_omega[0], wheel_omega[1], wheel_omega[2], wheel_omega[3]);
  }else if(mode == Mode::LoggerTest){
    SensorValue::x += 0.1f;
    SensorValue::y += 1.0f;

    Serial.printf("%f %f %f %f %f %f \n",
      SensorValue::x, SensorValue::y, SensorValue::theta,
      TargetValue::vel_x, TargetValue::vel_y, TargetValue::angular_vel);
  }else if(mode == Mode::AutoWheelParams){
    TargetValue::emergency = false;
    AutoWheelParam::auto_wheel_params();
  }

  // ログ
  if(mode != Mode::AutoWheelParams){
    sendWiFi();    
  }
}

namespace AutoWheelParam {
template <class T>
T mean(const std::vector<T>& vec)
{
    return std::accumulate(vec.begin(), vec.end(), T(0)) / vec.size();
}

template <class T>
std::array<T, 2> fit(const std::vector<T>& x, const std::vector<T>& y)
{
    T mx = mean(x);
    T my = mean(y);

    T cov = 0;
    T Sx = 0;
    for(int i=0; i<x.size(); i++){
        T dx = x[i] - mx;
        T dy = y[i] - my;
        cov += dx * dy;
        Sx += dx * dx;
    }
    cov /= x.size();
    Sx /= x.size();
    
    T a = cov / Sx;
    T b = my - a * mx;
    return std::array<T, 2>{a, b};
}

constexpr float dT = 1.0f;
constexpr float duration = 1.0f;
const std::vector<float> pwms = {
  100.0f, 150.0f, 200.0f, -100.0f, -150.0f, -200.0f
};
void setPwm(float pwm){
  for(int i=0; i<4; i++){
    CommandValue::wheel_pwm[i] = pwm;
  }
}

bool start = false;
float t0 = 0.0f;
std::array<float, 4> theta0{0.0f, 0.0f, 0.0f, 0.0f};
std::vector<std::array<float, 4>> omegas;
int step = 0;
float last_pwm = 0.0f;
void auto_wheel_params()
{
  if(!start){
    Serial.printf("auto wheel waiting...\n");
    if(TargetValue::vel_x != 0.0f || TargetValue::vel_y != 0.0f){
      start = true;
      t0 = current_time;
      Serial.printf("auto wheel start\n");
    }
  }

  if(start){ 
    for(int i=0; i<pwms.size(); i++){
      if(step == 3*i){
        float r = (current_time - t0) / dT;
        if(r < 1.0f){
          float pwm = last_pwm + r * (pwms[i] - last_pwm);
          setPwm(pwm);
        }else{
          step++;
          t0 = current_time;
          last_pwm = pwms[i];
        }
      }
      if(step == 3*i+1){
        if(current_time - t0 < dT){
          setPwm(pwms[i]);
        }else{
          step++;
          t0 = current_time;
          theta0 = SensorValue::wheel_theta;
        }
      }
      if(step == 3*i+2){
        float t = current_time - t0;
        if(t < duration){
          setPwm(pwms[i]);
        }else{
          step++;
          t0 = current_time;
          std::array<float, 4> omega;
          for(int i=0; i<4; i++){
            omega[i] = (SensorValue::wheel_theta[i] - theta0[i]) / t;
          }
          omegas.push_back(omega);
        }
      }
    }
    if(step == 3 * pwms.size()){
      float r = (current_time - t0) / dT;
      if(r < 1.0f){
        float pwm = (1.0f - r) * last_pwm;
        setPwm(pwm);
      }else{
        setPwm(0.0f);
      }
    }
    Serial.printf("step: %d\n", step);
  }

  if(pwms.size() == omegas.size()){
    std::array<float, 4> a;
    std::array<float, 4> b;
    for(int i=0; i<4; i++){
      std::vector<float> X;
      std::vector<float> Y;
      for(int j=0; j<pwms.size(); j++){
        float x = pwms[j];
        float y = omegas[j][i];
        if(x < 0.0f){
          x = -x;
          y = -y;
        }
        X.push_back(x);
        Y.push_back(y);
      }
      auto ret = fit(Y, X);
      a[i] = ret[0];
      b[i] = ret[1];
    }
    Serial.printf("inline const std::array<float, 4> pwm_per_omega = {%ff, %ff, %ff, %ff};\n", a[0], a[1], a[2], a[3]);
    Serial.printf("inline const std::array<float, 4> pwm0 = {%ff, %ff, %ff, %ff};\n\n", b[0], b[1], b[2], b[3]);
  }
}
} // namespace AutoWheelParam
