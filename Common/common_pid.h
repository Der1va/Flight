#ifndef COMMON_PID_H
#define COMMON_PID_H

typedef struct
{
    float Kp; //比例系数
    float Ki; //积分系数
    float Kd; //微分系数
    float error; //当前误差值
    float target; //目标值
    float integral; //积分值
    float measurement; //测量值
    float last_error; //上一次误差值
    float output; //PID输出值
} PID_Controller;

//单词pid计算
void Common_PID_Calculate(PID_Controller *pid);
//串级pid计算
void Common_PID_Calculate_chain(PID_Controller *out_pid, PID_Controller *in_pid);

#endif // COMMON_PID_H