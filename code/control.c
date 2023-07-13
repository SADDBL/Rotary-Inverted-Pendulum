#include "control.h"
#include "connect.h"
#include "tim.h"
#include <stdio.h>
#include <math.h>

/***** 顶层控制函数 *****/
int state_val = 0;
int delay_counter = 0;
void mission1(void){
	switch (state_val){
		case 0:{
			state_val++;
			break;
		}
		case 1:{
			delay_counter++;
			motor_pwm_set(40);
			if(delay_counter==37){
				delay_counter=0;
			//	motor_pwm_set(0);
				state_val++;
			}
			break;
		}
		case 2:{
			delay_counter++;
			motor_pwm_set(-40);
			if(delay_counter==37){
				delay_counter=0;
				//motor_pwm_set(0);
				state_val++;
			}
			break;
		}
		case 3:{
			delay_counter++;
			if((fabs(pole_ins.angle)<40))
				motor_speed_set(1000);
			else
				motor_speed_set(0);
			if(pole_ins.angle>320){
				state_val++;
				motor_pwm_set(0);
			}
			break;
		}
		
		case 4:{
			delay_counter++;
			if((fabs(pole_ins.angle)>320))
				motor_speed_set(-1000);
			else
				motor_speed_set(0);
			if(pole_ins.angle<4){
				state_val=3;
				motor_speed_set(0);
			}
			break;
		}
	}
}


void mission2(void){
	float v=650;
	switch (state_val){
		case 0:{
			state_val++;
			break;
		}
		case 1:{
			motor_speed_set(v);
			if(fabs(M.Speed-v)<40){
				state_val++;
				motor_pwm_set(0);
			}
			break;
		}
		case 2:{
			motor_speed_set(0);
			if(pole_ins.alpha<10){
				state_val++;
			}
			break;
		}
		case 3:{
			motor_speed_set(-v*0.5);
			if(fabs(M.Speed+v*0.5)<40){
				state_val++;
				motor_pwm_set(0);
			}
			break;
		}
		case 4:{
			motor_speed_set(0);
			if(fabs(pole_ins.alpha)<10){
				state_val=6;
			}
			break;
		}
		case 5:{
			motor_speed_set(v);
			if(fabs(M.Speed-v)<40){
				state_val++;
				motor_pwm_set(0);
			}
			break;
		}
		case 6:{
			motor_speed_set(0);
			if(fabs(pole_ins.alpha)<10){
				state_val=7;
			}
			break;
		}
		case 7:{
			motor_pwm_set(0);
		}
	}
}

//void mission4(void){
//	float beta_tar;
//	switch(state_val){
//		case 0:{
//			delay_counter++;
//			//motor_beta_set();
//			
//			if(delay_counter==17){
//				beta_tar=-beta_cal();
//				motor_speed_set(M.Speed+beta_tar*180/PI*1.25*CONTROL_CYCLE/1000);
//				delay_counter=0;
//			}
//			if(fabs(jy901_ins.wz)>600) motor_pwm_set(0);
//			if(fabs(180-pole_ins.angle)<12&&fabs(pole_ins.alpha)<5){
//				delay_counter=0;
//				M.ang=0;
//				motor_pwm_set(0);
//				state_val++;
//			}
//			break;
//		}
//		case 1:{
//			if(fabs(180-pole_ins.angle)<30){
//				erect_loop();
//			}
//			else state_val=0;
//		}
//	}
//}

void mission4(void){
	switch (state_val){
		case 0:{
			state_val++;
			break;
		}
		case 1:{
			delay_counter++;
			motor_pwm_set(40);
			if(delay_counter==37){
				delay_counter=0;
			//	motor_pwm_set(0);
				state_val++;
			}
			break;
		}
		case 2:{
			delay_counter++;
			motor_pwm_set(-40);
			if(delay_counter==37){
				delay_counter=0;
				//motor_pwm_set(0);
				state_val++;
			}
			break;
		}
		case 3:{
			delay_counter++;
			if((fabs(pole_ins.angle)<70))
				motor_speed_set(1500);
			else
				motor_speed_set(0);
			if(pole_ins.angle>320){
				state_val++;
				motor_pwm_set(0);
			}
			if(fabs(180-pole_ins.angle)<12&&fabs(pole_ins.alpha)<5){
				delay_counter=0;
				M.ang=0;
				motor_pwm_set(0);
				state_val=5;
			}
			break;
		}
		
		case 4:{
			delay_counter++;
			if((fabs(pole_ins.angle)>290))
				motor_speed_set(-1500);
			else
				motor_speed_set(0);
			if(pole_ins.angle<4){
				state_val=3;
				motor_speed_set(0);
			}
			if(fabs(180-pole_ins.angle)<12&&fabs(pole_ins.alpha)<5){
				delay_counter=0;
				M.ang=0;
				motor_pwm_set(0);
				state_val=5;
			}
			break;
		}
		case 5:{
			if(fabs(180-pole_ins.angle)<30){
				erect_loop();
			}
			else state_val=0;
		}
	}
}

void mission5(void){
	switch(state_val){
		case 0:{
			erect_loop();
			pid_resetpara(&pid_p,0,0,0);
			pid_resetpara(&pid_v,0.05,0.001,0);
			state_val++;
			break;
		}
		case 1:{
			if(fabs(180-pole_ins.angle)<30){
				erect_loop();
			}
			else
				motor_pwm_set(0);
		}
	}
}

void mission6(void){
	switch(state_val){
		case 0:{
			erect_loop();
			pid_resetpara(&pid_p,0.05,0,0);
			M.ang = 0;
			//pid_resetpara(&pid_v,0.05,0.001,0);
			state_val++;
			break;
		}
		case 1:{
			if(fabs(180-pole_ins.angle)<30){
				erect_loop();
			}
			else
				motor_pwm_set(0);
		}
	}
}


/**
  * @brief  倒立摆直立环，使用摆的角度环和电机位置环并联，均为负反馈
  * @param  None
  * @retval None
  */
void erect_loop(void){
	int pwm;
	//角度环，输出PWM，使用PD控制
	pid_position(&pid_a,177.5-pole_ins.angle,100-DEADZONE,DEADZONE-100,100,1);
	//位置环，输出PWM，使用Pd控制
	pid_position(&pid_p,M.ang,100-DEADZONE,DEADZONE-100,50,0.4);
	//速度环，输出PWM，使用PI控制
	pid_position(&pid_v,M.Speed-50,100-DEADZONE,DEADZONE-100,500,1);
	pwm=-pid_a.output+pid_p.output+pid_v.output;
	if(pwm>0) pwm+=DEADZONE;
	else if(pwm<0) pwm-=DEADZONE;
	else pwm=0;
	//角度环，输出PWM，使用PD控制
//	pid_position(&pid_a,178-pole_ins.angle,100,-100,100);
//	//位置环，输出PWM，使用PI控制
//	pid_position(&pid_p,M.ang,100,-100,500);
//	pwm=pid_a.output+pid_p.output;
	if(pwm>100) pwm=100;
	else if(pwm<-100) pwm = -100;
	motor_pwm_set(pwm);
}

/***** 其他控制函数 *****/
int sgn(float val){
	if(val>0) return 1;
	else if(val<0) return -1;
	return 0;
}
//计算摆棒达到某角度需要的角加速度
float a;
float beta_cal(void){
	float ang,temp;
	if(pole_ins.angle>180) ang = pole_ins.angle-360;
	else if(pole_ins.angle<0) ang = pole_ins.angle;
	else ang = pole_ins.angle;
	ang = ang*PI/180;
	temp = 0.5f*(POLE_I*pole_ins.alpha*pole_ins.alpha-POLE_M*POLE_L*9.8*(1+cosf(ang)));
	a = n*g*sgn(temp*pole_ins.alpha*cosf(ang*PI/180));
	return a/R;
}

//将ADC采样转换为角度值
float adc2angle(void){
	float ang_cur;
	static int val_last = 0;
	int val_cur = (adc_buf[0]+adc_buf[0]+adc_buf[0])/3;
	//角度转换
	if(val_cur - val_last<-1000&&val_cur<2100) val_cur = 0;
	else if(val_cur - val_last>1000&&val_cur<2100) val_cur = 4096;
	pole_ins.angle = (float)(val_cur)*0.0842f-4.2326f;
	if(pole_ins.angle-pole_ins.angle_last>200) pole_ins.alpha = (pole_ins.angle-360-pole_ins.angle_last)*PI/180/CONTROL_CYCLE*1000;
	else if(pole_ins.angle-pole_ins.angle_last<-200) pole_ins.alpha = (pole_ins.angle+360-pole_ins.angle_last)*PI/180/CONTROL_CYCLE*1000;
	else pole_ins.alpha = (pole_ins.angle-pole_ins.angle_last)*PI/180/CONTROL_CYCLE*1000;;
	pole_ins.angle_last = pole_ins.angle;
	val_last = val_cur;
	return ang_cur;
}

//判断摆是否处于死区
int if_in_deadzone(int val1,int val2){
	if(val1==0&&val2>1900&&val2<2200) return NOT_OK;
	else if(val2==0&&val1>1900&&val1<2200) return NOT_OK;
	else if(val2==0&&val1==0) return NOT_OK;
	else if(val1>4000&&val1<4097&&val2>1900&&val2<2200) return NOT_OK;
	else if(val2==0) return NOT_OK;
	return OK;
}

/***** 电机控制函数 *****/
/**
  * @brief  电机结构体初始化
  * @param  *M 待初始化的结构体
  * @retval None
  */
void motor_Init(motor *M){
	M->Speed = 0;
	M->Tar_Speed = 0;
	M->ang = 0;
	M->beta =0;
	M->ang0 = jy901_ins.yaw;
}

/**
  * @brief  电机PWM设置，死区0.25
  * @param  pwm 电机PWM占空比，范围-100-100，取0时电机停止
  * @retval None
  */
void motor_pwm_set(float pwm){
	if(pwm<0){
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,(uint16_t)(-10*pwm));
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,0);
	}
	else{
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,(uint16_t)(10*pwm));
	}
}

/**
  * @brief  设置电机角加速度
  * @param  None
  * @retval None
  */
float beta_tar;
void motor_beta_set(){
	beta_tar = beta_cal();
	pid_position(&pid_b,beta_tar-M.beta,95,-95,1000,1);
	motor_pwm_set(pid_b.output);
}

/**
  * @brief  计算电机角速度，并更新角度路程
  * @param  None
  * @retval None
  */
//void speed_cal(void){
//	static float v_last;//ang_last ;
//	int err=0;
//	int cur = __HAL_TIM_GetCounter(&htim3);
//	M.QEIPostion = cur;
//	//处理溢出
//	err = cur - M.QEIPostion_k1;
//	if(M.QEIPostion<M.QEIPostion_k1&&err<-10000){
//		err +=0xffff;
//	}
//	else if(M.QEIPostion>M.QEIPostion_k1&&err>10000){
//		err -=0xffff;
//	}
//	//计算速度和滤波
//	M.Speed = err*1000*D_ANGLE/CONTROL_CYCLE;	//°/s，计算角速度
//	M.Speed = first_order_filter(M.Speed, v_last,(float)0.6);
//	//计算角加速度
//	M.beta = (M.Speed -v_last)/CONTROL_CYCLE*1000*PI/180;
//	v_last = M.Speed;
//	//更新角度
//	//M.ang += D_ANGLE * err;
//	//M.ang = first_order_filter(M.ang,M.ang_last,1.0f);
//	//M.ang_last = M.ang;
//	M.QEIPostion_k1 = cur;
//}

void yaw_set0(void){
	M.ang0 = jy901_ins.yaw;
}

void m_yaw_cal(void){
//	float err = jy901_ins.yaw-M.ang0;
//	if(err>300) M.ang+=err - 360;
//	else if(err<-300) M.ang+=err+360;
//	else M.ang+=err;
//	M.ang0 = jy901_ins.yaw;
	M.ang+=jy901_ins.wz*CONTROL_CYCLE/1000;
}

/**
  * @brief  电机速度设置
  * @param  vl 左轮目标速度值，+为逆时针，-为顺时针，范围-1.2-1.2
  * @retval None
  */
void motor_speed_set(float v){
	float pwm;
	//左轮
	pid_m.cur_val = M.Speed;
	pid_m.target_val = v;
	pid_position(&pid_m,pid_m.target_val - pid_m.cur_val,(float)100-DEADZONE,DEADZONE-(float)100,2000,1);
	pwm=pid_m.output;
	if(pwm>0) pwm+=DEADZONE;
	else if(pwm<0) pwm-=DEADZONE;
	else pwm=0;
	motor_pwm_set(pwm);
}

/***** PID底层 *****/
void pid_resetpara(pid *pid,float p,float i,float d){
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
}
void pid_init(pid *pid_controller, float p, float i, float d)
{
    pid_controller->kp = p;
    pid_controller->ki = i;
    pid_controller->kd = d;
    pid_controller->cur_val = 0;
    pid_controller->target_val = 0;
    pid_controller->err = 0;
    pid_controller->err_k1 = 0;
    pid_controller->output = 0;
    pid_controller->output_last = 0;
    pid_controller->i = 0;
	pid_controller->d = 0;
}
/**
 * @brief  位置式PID实现函数
 * @param  *p PID结构体
 * @param  err 误差值：err = current - target
 * @param  outMax 输出最大值
 * @param  outMin 输出最小值
 * @param  i_Max 积分上限
 * @retval p->output
 */
float pid_position(pid *p, float err, float outMax, float outMin, float i_Max,float a){
  if(a<1&&a>0)
		p->err = first_order_filter(err,p->err_k1,a);
	else if(a==1){
		p->err = err;
	}
	
    // 抗积分饱和
    if (p->output_last > outMax || p->output_last < outMin)
    {
        if (p->output_last * p->err < 0) // err使积分项绝对值减小
            p->i += p->err;
        else
            p->i = p->i;
    }
    else
        p->i += p->err;
    // 积分限幅
    if (p->i > i_Max)
        p->i = i_Max;
    else if (p->i < -i_Max)
        p->i = -i_Max;
		p->d = p->err - p->err_k1;
    p->output = p->kp * p->err + p->ki * p->i + p->kd * (p->d);
    p->err_k1 = p->err;
    p->output_last = p->output;

    // 输出限幅
    if (p->output > outMax)
        p->output = outMax;
    if (p->output < outMin)
        p->output = outMin;

    return p->output;
}

/**
  * @brief  PID结构体复位，在切换任务时调用
  * @param  *p PID结构体
  * @param  err 误差值：err = current - target
  * @retval None
  */
void pid_reset(pid* pid_controller){
	pid_controller->cur_val = 0;
	pid_controller->target_val = 0;
	pid_controller->err = 0;
	pid_controller->err_k1 = 0;
	pid_controller->output = 0;
	pid_controller->output_last = 0;
	pid_controller->i = 0;
}

/**
  * @brief  一阶滤波
  * @param  new_value 新值
	* @param  last_value 上一次的值
  * @param  a 滤波系数
  * @retval 滤波后的值
  */
float first_order_filter(float new_value,float last_value,float a){
	//a的取值决定了算法的灵敏度，a越大，新采集的值占的权重越大，算法越灵敏，但平顺性差
	//相反，a越小，新采集的值占的权重越小，灵敏度差，但平顺性好。
	float flitered = new_value*a + last_value*(1-a);
	return flitered;
}
