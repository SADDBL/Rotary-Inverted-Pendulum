#ifndef _CONNECT_H_
#define _CONNECT_H_
#include "main.h"
#define PI (float)3.14159
#define OK 1
#define NOT_OK 2
#define INF 0xffff

#define	MOTOR_PRC	1440	//����Ƶ��ÿȦ1560������
#define D_ANGLE (float)(360.0f/MOTOR_PRC)	//һ������ת���ĽǶȣ���λΪ��
#define CONTROL_CYCLE	10	//�������ڣ�10ms
#define DEADZONE 20

#define ERECT_DATA 10	//ֱ��ʱadc��ȡ����ֵ

#define POLE_M 0.0383f	//�ڸ�������kg
#define POLE_L 0.085f	//�ڸ˳��ȣ�m
#define R	0.115f	//ת�᳤�ȣ�m
#define POLE_I (float)(POLE_M*POLE_L*POLE_L/3)	//�ڸ˶�֧���ת������
#define POLE_I_2_MEOTOR (float)(POLE_M*POLE_L*POLE_L)	//�ڸ˶Ե����ת������
#define ARM_M 0.1396f	//�ڱ�����
#define ARM_I (float)(ARM_M*R*R/3)	//�ڱ�ת������
#define SENSOR_M 0.0614f	//����������
#define SENSOR_I (float)(SENSOR_M*R*R)
	
#define n 1.333f
#define g 9.8f
/* PID������ز��� */
typedef struct PID
{
    float kp, ki, kd;
    float target_val, cur_val;
    float err, err_k1;
    float i,d;
    float output, output_last;
} pid;

typedef struct M{
	float Speed;	//���ٶȣ���λ ��/s
	float beta;	//�Ǽ��ٶȣ���λrad/s^2
	float Tar_Speed;	//���Ŀ���ٶ�
	float ang;			//��ǰ�Ƕ�ֵ
	float ang0;
//	float ang_last;
//	int QEIPostion;	//����QEI�ļ���ֵ
//	int QEIPostion_k1;
}motor;

typedef struct P{
	float angle;
	float angle_last;
	float alpha;	//���ٶȣ���λrad/s
}pole;

extern motor M;
extern pid pid_v;
extern pid pid_a;
extern pid pid_p;
extern pid pid_b;
extern pid pid_m;
extern pole pole_ins;
extern uint16_t adc_buf[3];
extern int delay_counter;

void mission1(void);
void mission2(void);
void mission4(void);
void mission5(void);
void mission6(void);
void erect_loop(void);
/***** �������ƺ��� *****/
//��ADC����ת��Ϊ�Ƕ�ֵ
int sgn(float val);
float adc2angle(void);
float beta_cal(void);
int if_in_deadzone(int val1,int val2);
/***** ������ƺ��� *****/
void motor_pwm_set(float pwm);
void motor_Init(motor *M);
void motor_beta_set(void);
void motor_speed_set(float v);
void m_yaw_cal(void);
/***** PID�ײ� *****/
void pid_init(pid *pid_controller, float p, float i, float d);
float pid_position(pid *p, float err, float outMax, float outMin, float i_Max,float);
void pid_reset(pid* pid_controller);
float first_order_filter(float new_value,float last_value,float a);
void pid_resetpara(pid *pid,float p,float i,float d);
#endif
