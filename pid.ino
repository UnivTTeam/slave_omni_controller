int pid_controller(int pwm_past,float kp,float ki,float kd,int p_max,float err,float err_past){
    //回転数のrefとobs(回転数の観測値)とpidゲインを入力してpwmを調整
    int pwm_diff = 0;
    int err_i = min(max(-3,int(ki *(err + err_past))),3);
    int err_p = min(max(-p_max,int(kp * err)),p_max);
    int err_d = min(max(-12,int(kd * (err-err_past))),12);
    pwm_diff = min(max(-15,int(err_p + err_i + err_d)),15);//回転数をpidする値を入れ、司令されているPWMの値に調整を加える
    int pwm_now = pwm_past + pwm_diff;
    return pwm_now;
}
