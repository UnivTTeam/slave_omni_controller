void calc_wheel_ang_vel(){
  float LF_enc_diff = m_nValue_LF;
  m_nValue_LF = 0;
  float LF_a = LF_enc_diff / enc_cycle / gear_d * 2 * PI / dt * 1000000;
  angular_LF = enc_lowpass * angular_LF + (1-enc_lowpass) * LF_a;
  
  float LB_enc_diff = m_nValue_LB;
  m_nValue_LB = 0;
  float LB_a = LB_enc_diff / enc_cycle / gear_d * 2 * PI / dt * 1000000;
  angular_LB = enc_lowpass * angular_LB + (1-enc_lowpass) * LB_a;
 
  float RB_enc_diff = m_nValue_RB;
  m_nValue_RB = 0;
  float RB_a = RB_enc_diff / enc_cycle / gear_d * 2 * PI / dt * 1000000;
  angular_RB = enc_lowpass * angular_RB + (1-enc_lowpass) * RB_a;

  float RF_enc_diff = m_nValue_RF;
  m_nValue_RF = 0;
  float RF_a = RF_enc_diff / enc_cycle / gear_d * 2 * PI / dt * 1000000;
  angular_RF = enc_lowpass * angular_RF + (1-enc_lowpass) * RF_a;
  
//  Serial.print("angular_LF");
//  Serial.println(angular_LF);
//  Serial.print("angular_LB");
//  Serial.println(angular_LB);
//  Serial.print("angular_RB");
//  Serial.println(angular_RB);
//  Serial.print("angular_RF");
//  Serial.println(angular_RF);
  //エンコーダのインクリメントから車輪の角速度を求める
}








void rotRotEnc_LF(void)//エンコーダの計測関数
{
  int aPin,bPin;
  aPin=digitalRead(ENC_LF);
  bPin=digitalRead(ENC_LF2);
  if(!aPin){  // ロータリーエンコーダー回転開始
    m_nOldRot_LF = bPin?1:-1;
  } else { 
    if(bPin){
      if(m_nOldRot_LF == -1){
        m_nValue_LF--;
      }
    } else {
      if(m_nOldRot_LF == 1){
        m_nValue_LF++;
      }
    }
    m_nOldRot_LF = 0;
  }
}

void rotRotEnc_LB(void)//エンコーダの計測関数
{
  int aPin,bPin;
  aPin=digitalRead(ENC_LB);
  bPin=digitalRead(ENC_LB2);
  if(!aPin){  // ロータリーエンコーダー回転開始
    m_nOldRot_LB = bPin?1:-1;
  } else { 
    if(bPin){
      if(m_nOldRot_LB == -1){
        m_nValue_LB--;
      }
    } else {
      if(m_nOldRot_LB == 1){
        m_nValue_LB++;
      }
    }
    m_nOldRot_LB = 0;
  }
}
void rotRotEnc_RB(void)//エンコーダの計測関数
{
  int aPin,bPin;
  aPin=digitalRead(ENC_RB);
  bPin=digitalRead(ENC_RB2);
  if(!aPin){  // ロータリーエンコーダー回転開始
    m_nOldRot_RB = bPin?1:-1;
  } else { 
    if(bPin){
      if(m_nOldRot_RB == -1){
        m_nValue_RB--;
      }
    } else {
      if(m_nOldRot_RB == 1){
        m_nValue_RB++;
      }
    }
    m_nOldRot_RB = 0;
  }
}
void rotRotEnc_RF(void)//エンコーダの計測関数
{
  int aPin,bPin;
  aPin=digitalRead(ENC_RF);
  bPin=digitalRead(ENC_RF2);
  if(!aPin){  // ロータリーエンコーダー回転開始
    m_nOldRot_RF = bPin?1:-1;
  } else { 
    if(bPin){
      if(m_nOldRot_RF == -1){
        m_nValue_RF--;
      }
    } else {
      if(m_nOldRot_RF == 1){
        m_nValue_RF++;
      }
    }
    m_nOldRot_RF = 0;
  }
}
