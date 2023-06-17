#include "device.h"
#include "encoder.h"
#include "params.h"
#include "control.h"
#include "wire_control.h"

void setup() {
  setupDevice();
  setupWiFi();
}

constexpr float CONTROL_INTERVAL_US = Params::CONTROL_INTERVAL_MS * 1000.0f;

void loop() {
  // 現在時刻
  static float last_control_time_us = micros() - CONTROL_INTERVAL_US;
  float current_time_us = micros();
  float interval_us = current_time_us - last_control_time_us;

  if (interval_us >= CONTROL_INTERVAL_US) {
    // 時刻情報アップデート
    last_control_time_us = current_time_us;
    Params::control_interval_sec = interval_us / (1000.0f * 1000.0f);

    // デバイス情報アップデート
    calc_wheel_ang_vel();

    // 制御
    control();
  
    //pwmを各モーターに指令
    send_pwm();
  }
}
