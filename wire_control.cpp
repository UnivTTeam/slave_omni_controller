#include <Wire.h>
#include <WiFi.h>
#include <Arduino.h>

#include "wire_control.h"
#include "device.h"
#include "params.h"

bool enable_wifi = false;

const char* SSID = "tk13a60d";           // WiFi SSID
const char* PASSWORD = "hogepyon203";    // WiFi Password
static const char *kRemoteIpadr = "192.168.255.36";
/*
const char* SSID = "toriyama_dell";           // WiFi SSID
const char* PASSWORD = "chameleon";   // WiFi Password
static const char *kRemoteIpadr = "192.168.137.1";
*/

static WiFiUDP wifiUdp; 
static const int kRmoteUdpPort = 9000;

const char SENSOR_JSON[] PROGMEM = 
  R"=====({"t":%.2f,"pos_x":%.2f,"pos_y":%.2f,"theta":%.2f,"vx":%.2f,"vy":%.2f,"omega":%.2f,"pwm0":%.1f,"pwm1":%.1f,"pwm2":%.1f,"pwm3":%.1f})=====";

void setupWiFi()
{
  static const int kLocalPort = 7000;
  WiFi.begin(SSID, PASSWORD);
  wifiUdp.begin(kLocalPort);
}

void sendWiFi()
{
  // 送信するデータを作成
  char payload[1000];
  float t = micros() / (1000.0f * 1000.0f);
  snprintf_P(payload, sizeof(payload), SENSOR_JSON, 
    t,
    SensorValue::x, SensorValue::y, SensorValue::theta,
    TargetValue::vel_x, TargetValue::vel_y, TargetValue::angular_vel,
    CommandValue::wheel_pwm[0], CommandValue::wheel_pwm[1], CommandValue::wheel_pwm[2], CommandValue::wheel_pwm[3]);
  std::string msg_str = payload;
  uint8_t binary_array[sizeof(payload)];
  std::copy(msg_str.begin(), msg_str.end(), std::begin(binary_array));
  
  wifiUdp.beginPacket(kRemoteIpadr, kRmoteUdpPort);
  wifiUdp.write(binary_array, sizeof(binary_array));
  wifiUdp.endPacket();

  static int last_connect_time_msec = millis() - 200;
  if ((WiFi.status() != WL_CONNECTED) && (millis() - last_connect_time_msec >=100)) {
    WiFi.disconnect();
    WiFi.reconnect();
    last_connect_time_msec = millis();
  }
}

bool readBool(){
  uint8_t value = Wire.read();
  uint8_t count = 0;
  for(uint8_t i=1; i; i*=2){
    if(value & i){ count++; }
  }
  return (count > 3);
}

float readFloatValue(){
  uint8_t bytes[4];
  for(int i=0; i<4; i++){
      bytes[i] = Wire.read();
  }

  float value;
  memcpy(&value, bytes, sizeof(float));
  return value;
}

void receiveEvent(int cnt)
{
  using namespace TargetValue;
  if(Wire.available()>1){
    emergency = readBool();
    
    SensorValue::x = readFloatValue();
    SensorValue::y = readFloatValue();
    SensorValue::theta = readFloatValue();

    vel_x = readFloatValue();
    vel_y = readFloatValue();
    angular_vel = readFloatValue();
  }
}

void requestEvent(){
}
