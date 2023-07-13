#ifndef _CONNECT_H_
#define _CONNECT_H_
#include "main.h"
#define PI (float)3.14159
#define OK 1
#define NOT_OK 2
#define INF 0xffff

#define	MOTOR_PRC	1440	//二分频，每圈1560个脉冲
#define D_ANGLE (float)(360.0f/MOTOR_PRC)	//一个脉冲转过的角度，单位为°
#define CONTROL_CYCLE	10	//控制周期，10ms
#define DEADZONE 23

#define ERECT_DATA 10	//直立时adc读取到的值

#define POLE_M 0.0383f	//摆杆质量，kg
#define POLE_L 0.085f	//摆杆长度，m
#define R	0.115f	//转轴长度，m
#define POLE_I (float)(POLE_M*POLE_L*POLE_L/3)	//摆杆转动惯量
/* PID控制相关参数 */
typedef struct PID
{
    float kp, ki, kd;
    float target_val, cur_val;
    float err, err_k1;
    float i,d;
    float output, output_last;
} pid;

typedef struct M{
	float Speed;	//角速度，单位 °/s
	float beta;	//角加速度，单位rad/s^2
	float Tar_Speed;	//电机目标速度
	float ang;			//当前角度值
	float ang_last;
	int QEIPostion;	//本次QEI的计数值
	int QEIPostion_k1;
}motor;

typedef struct P{
	float angle;
	float angle_last;
	float alpha;	//角速度，单位rad/s
}pole;

extern motor M;
extern pid pid_v;
extern pid pid_a;
extern pid pid_p;
extern pole pole_ins;
extern uint16_t adc_buf[3];
extern int delay_counter;

void mission1(void);
void mission2(void);
void mission4(void);
void erect_loop(void);
/***** 其他控制函数 *****/
//将ADC采样转换为角度值
float adc2angle(void);
int if_in_deadzone(int val1,int val2);
/***** 电机控制函数 *****/
void motor_pwm_set(float pwm);
void motor_Init(motor *M);
void speed_cal(void);
/***** PID底层 *****/
void pid_init(pid *pid_controller, float p, float i, float d);
float pid_position(pid *p, float err, float outMax, float outMin, float i_Max,float);
void pid_reset(pid* pid_controller);
float first_order_filter(float new_value,float last_value,float a);
#endif
