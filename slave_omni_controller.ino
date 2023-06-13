#include<Wire.h>
#define ENC_LF 13
#define ENC_LF2 12
#define ENC_LB 14
#define ENC_LB2 27
#define ENC_RB 26
#define ENC_RB2 25
#define ENC_RF 33
#define ENC_RF2 32

#define IDC 23
#define OMNI_ID 0x34

#define PI 3.141592653589793  

#define enc_lowpass 0.8  //for enc to wheelangularvel lowpass

//モーター系
const int LF_A = 15;
const int LF_B = 2;
const int LB_A = 4;
const int LB_B = 16;
const int RB_A = 17;
const int RB_B = 5;
const int RF_A = 18;
const int RF_B = 19;

int LF_pwm = 0;
int LB_pwm = 0;
int RB_pwm = 0;
int RF_pwm = 0;

int pwm[8]={0,0,0,0,0,0,0,0};

//エンコーダ系
volatile int m_nOldRot_LF = 0;
volatile int m_nValue_LF  = 0;
volatile int m_nOldRot_LB = 0;
volatile int m_nValue_LB = 0;
volatile int m_nOldRot_RB = 0;
volatile int m_nValue_RB  = 0;
volatile int m_nOldRot_RF = 0;
volatile int m_nValue_RF  = 0;


//回転数制御用
float errLF = 0;
float errLF_past = 0;
float errLB = 0;
float errLB_past = 0;
float errRB = 0;
float errRB_past = 0;
float errRF = 0;
float errRF_past = 0;

float target_angular_LF = 0;
float target_angular_LB = 0;
float target_angular_RB = 0;
float target_angular_RF = 0;

float angular_LF = 0;//rad/s
float angular_LB = 0;//rad/s
float angular_RB = 0;//rad/s
float angular_RF = 0;//rad/s

float wheel_kp = 1.0;
float wheel_ki = 0.0;
float wheel_kd = 0.0;
float wheel_p_max = 5;

//状態
int master_status = 0;//親機の状態
int slave_status = 0;//本機の状態

//オドメトリ系
float pos_x = 0;//m
float pos_y = 0;//m
float yaw = 0;//rad
float vel_x = 138;
float vel_y = 128;
float angular_vel = 128;

//機体パラメータ
float wheel_R = 0.06;//m,タイヤの半径

float gear_d = 30;//減速比
float enc_cycle = 16;//一周あたりの分解能

float max_angular_vel = 6*PI ;//車輪の最大角速度

//制御間隔(micro sec)
float dt = 20000;//20ms
unsigned long tmp_time = 0;
float stime = 0;
float stime_offset = 0;


void setup() {
  Wire.begin(uint8_t(OMNI_ID));
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  Serial.begin(115200);
  // 使用するタイマーのチャネルと周波数を設定
  ledcSetup(0, 24000, 8);//24khz,8bitで設定
  ledcSetup(1, 24000, 8);
  ledcSetup(2, 24000, 8);//24khz,8bitで設定
  ledcSetup(3, 24000, 8);
  ledcSetup(4, 24000, 8);//24khz,8bitで設定
  ledcSetup(5, 24000, 8);
  ledcSetup(6, 24000, 8);//24khz,8bitで設定
  ledcSetup(7, 24000, 8);
  
  // 各Pinをチャネルへ接続
  ledcAttachPin(LF_A, 0);
  ledcAttachPin(LF_B, 1);
  ledcAttachPin(LB_A, 2);
  ledcAttachPin(LB_B, 3);
  ledcAttachPin(RB_A, 4);
  ledcAttachPin(RB_B, 5);
  ledcAttachPin(RF_A, 6);
  ledcAttachPin(RF_B, 7);

  
  pinMode(IDC,OUTPUT);
  
  pinMode(ENC_LF, INPUT_PULLUP);
  pinMode(ENC_LF2, INPUT_PULLUP);
  pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_LB2, INPUT_PULLUP);
  pinMode(ENC_RB, INPUT_PULLUP);
  pinMode(ENC_RB2, INPUT_PULLUP);
  pinMode(ENC_RF, INPUT_PULLUP);
  pinMode(ENC_RF2, INPUT_PULLUP);
  
  attachInterrupt(ENC_LF,rotRotEnc_LF,CHANGE);
  attachInterrupt(ENC_LB,rotRotEnc_LB,CHANGE);
  attachInterrupt(ENC_RB,rotRotEnc_RB,CHANGE);
  attachInterrupt(ENC_RF,rotRotEnc_RF,CHANGE);
} 


void loop() {
  if(micros() - tmp_time >= dt){//メインの制御ループ
    
    //エンコーダーの積算値より左右の車輪の角速度計算
    calc_wheel_ang_vel();
    // 左右の車輪の角速度からオドメトリの計算
//    calc_odometry();
    //オドメトリから目標点を計算
//    calc_waypoint();      
    //目標速度と角速度から左右の車輪の目標角速度を求める
//    calc_target_wheel_ang_vel();

    //指令xy速度と角速度に応じて各輪の目標角速度を計算
    omni_vel_allocator();
    
    //現在の車輪角速度と目標角速度からPIDしてPWMを計算
    calc_pwm();
  
    //pwmを各モーターに指令
    send_pwm();
    
    tmp_time = micros();
  }
}



void send_pwm(){
  if(LF_pwm >=0){
    pwm[0]=LF_pwm;
    pwm[1]=0;
  }
  else if(LF_pwm <0){
    pwm[0]=0;
    pwm[1]=-LF_pwm;
  }
  
  if(LB_pwm >=0){
    pwm[2]=LB_pwm;
    pwm[3]=0;
  }
  else if(LB_pwm <0){
    pwm[2]=0;
    pwm[3]=-LB_pwm;
  }
  
  if(RB_pwm >=0){
    pwm[4]=RB_pwm;
    pwm[5]=0;
  }
  else if(RB_pwm <0){
    pwm[4]=0;
    pwm[5]=-RB_pwm;
  }
  
  if(RF_pwm >=0){
    pwm[6]=RF_pwm;
    pwm[7]=0;
  }
  else if(RF_pwm <0){
    pwm[6]=0;
    pwm[7]=-RF_pwm;
  }

  for(int i = 0; i < 8; i++){
    ledcWrite(i, pwm[i]);
  }
}




float dist(float x1,float y1,float x2,float y2){
  return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
}
