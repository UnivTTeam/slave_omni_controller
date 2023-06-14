#include "device.h"
#include "odometry.h"
#include "params.h"
#include "control.h"

void setup() {
  setupDevice();
} 

float last_control_time = 0;
void loop() {
  // メインループ
  last_control_time = micros() - Params::control_interval_us;

  // 一定周期ごとに制御を行う
  if (micros() - last_control_time >= Params::control_interval_us) {
    // 時刻情報をアップデート
    last_control_time = micros();

    // エンコーダーの積算値から各輪の角速度を計算
    calc_wheel_ang_vel();

    // 指令xy速度と角速度に応じて各輪を制御
    // omni_vel_allocator(); -> calc_pwm(); に相当
    control();
  
    //pwmを各モーターに指令
    send_pwm();
  }
}
