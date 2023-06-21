#include <Wire.h>
#include <WiFi.h>
#include <Arduino.h>

#include "wire_control.h"
#include "device.h"
#include "params.h"

const char* SSID = "SEKKEN-DYNABOOK 9513";           // WiFi SSID
const char* PASSWORD = "446c6*S5";    // WiFi Password
static const char *kRemoteIpadr = "192.168.137.1";

// const char* SSID = "toriyama_dell";           // WiFi SSID
// const char* PASSWORD = "chameleon";   // WiFi Password
// static const char *kRemoteIpadr = "192.168.137.1";


static WiFiUDP wifiUdp; 
static const int kRmoteUdpPort = 9000;

const char SENSOR_JSON[] PROGMEM = 
  R"=====({"t":%.2f,"s":%d,"x":%.2f,"y":%.2f,"th":%.5f,"vx":%.2f,"vy":%.2f,"om":%.5f,"p0":%.2f,"p1":%.2f,"p2":%.2f,"p3":%.2f,"om0":%.2f,"om1":%.2f,"om2":%.2f,"om3":%.2f})=====";

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
    t, TargetValue::master_step,
    SensorValue::x, SensorValue::y, SensorValue::theta,
    TargetValue::vel_x, TargetValue::vel_y, TargetValue::angular_vel,
    CommandValue::wheel_pwm[0], CommandValue::wheel_pwm[1], CommandValue::wheel_pwm[2], CommandValue::wheel_pwm[3],
    SensorValue::wheel_omega[0], SensorValue::wheel_omega[1], SensorValue::wheel_omega[2], SensorValue::wheel_omega[3]);
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
    Params::last_communication_time = Params::current_time;

    emergency = readBool();
    master_step = Wire.read();
    if(master_step==0xff){
      master_step = -1;
    }

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
