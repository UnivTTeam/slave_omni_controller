#include "device.h"
#include "encoder.h"
#include "params.h"
#include "control.h"

//#include <WiFi.h>
//
//
//const char* SSID = "toriyama_dell";           // WiFi SSID
//const char* PASSWORD = "chameleon";   // WiFi Password
//
//static WiFiUDP wifiUdp; 
//static const char *kRemoteIpadr = "192.168.137.1";
//static const int kRmoteUdpPort = 9000;
//
//unsigned long previousMillis = 0;
//const char SENSOR_JSON[] PROGMEM = R"=====({"pos_x":%.4f,"pos_y":%.1f,"yaw":%.1f,"imu_yaw":%.1f,"vel":%.1f,"angular_vel":%.1f,"stime":%.1f})=====";
//
//float posx = 13.2345;

//static void WiFi_setup()
//{
//  static const int kLocalPort = 7000;
//
//  WiFi.begin(SSID, PASSWORD);
//  while( WiFi.status() != WL_CONNECTED) {
//    delay(500);  
//  }  
//  wifiUdp.begin(kLocalPort);
//}

void setup() {
  setupDevice();
//  WiFi_setup();
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

//    wifi_log();
  }
}

//void wifi_log(){
//  char payload[150];
//  snprintf_P(payload, sizeof(payload), SENSOR_JSON, TargetValue::vel_x, TargetValue::vel_y, TargetValue::angular_vel, posx, posx, posx,posx);
//  wifiUdp.beginPacket(kRemoteIpadr, kRmoteUdpPort);
//  std::string str = payload;
//  uint8_t arr[sizeof(payload)];
//  std::copy(str.begin(), str.end(), std::begin(arr));
//  wifiUdp.write(arr, sizeof(arr));
//  wifiUdp.endPacket();
//
//
//  if ((WiFi.status() != WL_CONNECTED) && (millis() - previousMillis >=100)) {
//   WiFi.disconnect();
//   WiFi.reconnect();
//   previousMillis = millis();
//  }
//}
