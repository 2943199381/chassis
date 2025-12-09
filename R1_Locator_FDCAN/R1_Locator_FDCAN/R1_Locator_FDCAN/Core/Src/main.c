/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "bsp_can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//#define jiaozhun

//r2 ֱ����б�߼н�Ϊ  68.2����
struct GyroResult gyroResult;
struct EncoderResult encoderRes;
volatile struct LocatorResult lcResult;
struct GyroResult timerGyroResult;
struct EncoderResult timerEncResult;
struct LocatorResult timerlcResult;
struct EncoderResult lastEncResult;
struct GyroResult lastGyroResult;
double QEI_pulse[2] = {0};
float encVel[2];
double vx_dis=0,vy_dis=0,vr_ang=0;
double total_x[5]={0},total_y[5]={0},total_r[5]={0};
int v_n=0;
float dis1=0,dis2=0,dis3=0;   //���������Ϣ
int if_relc=0;   //�Ƿ�Ҫ�����ض�λ
int send_times=0;
uint8_t restart_flag=0;
#ifdef jiaozhun
	double floating=0;
	double floating_total=0;
	int cnt=0;
	int cnt_p=0;  //��ת����
	int cnt_n=0;  //��ת����
	int pulse_out=0;
	int jiaozhun_flag=0;
	float ang_total=0;
	int start_flag_jiaozhun=1;
	float gyro_angle_jiaozhun=0;
	float last_angle_jiaozhun=0;
	float gyro_angle_jiaozhun_start=0;
	float del_angle_jiaozhun=0;
	float p_del_angle_jiaozhun_total=0;
	float n_del_angle_jiaozhun_total=0;
	int pulse1_jiaozhun=0;
	int pulse0_jiaozhun=0;
	int pulse1_jiaozhun_start=0;
	int pulse0_jiaozhun_start=0;
	int gyro_start_time_jiaozhun=0;
	int last_out_time=0;
	int now_out_time=0;
	int pulse0_jiaozhun_last=0;
	int pulse1_jiaozhun_last=0;
	#define floating_numbers 5000
#endif
#ifndef jiaozhun
//	#define p_rotation_e0 (11470.064453)
//	#define p_rotation_e1 (-4111.074707)
//	#define n_rotation_e0 (11170.486328)
//	#define n_rotation_e1 (-4682.280273)
//	#define p_rotation_e0 (11226.939453)
//	#define p_rotation_e1 (-4913.593750)
//	#define n_rotation_e0 (11356.878906)
//	#define n_rotation_e1 (-4422.673340)
//		#define p_rotation_e0 (10388.661133)
//		#define p_rotation_e1 (-4365.110352)
//		#define n_rotation_e0 (10123.064453)
//		#define n_rotation_e1 (-4333.879883)
//		#define p_rotation_e0 (-18005.494141)
//		#define p_rotation_e1 (5290.318359)
//		#define n_rotation_e0 (-18746.431641)
//		#define n_rotation_e1 (4932.718750)
		#define p_rotation_e0 (-886.069336)
		#define p_rotation_e1 (-881.045837)
		#define n_rotation_e0 (-1401.902344)
		#define n_rotation_e1 (-657.607117)
//		#define p_rotation_e0 (-617.054382)
//		#define p_rotation_e1 (-1177.658569)
//		#define n_rotation_e0 (-1306.006226)
//		#define n_rotation_e1 (-923.308594)
//				#define p_rotation_e0 (0)
//		#define p_rotation_e1 (0)
//		#define n_rotation_e0 (0)
//		#define n_rotation_e1 (0)

/*
ϵ������:
y     e0 e1    0.0139    0.0137
x			     		-0.0137    0.0138
*/
//	#define I_mul_x0 (-0.01441959481)
//	#define I_mul_x1 (0.01427929672)
//	#define I_mul_y0 (0.01435360245)
//	#define I_mul_y1 (0.01423833601)
//	#define I_mul_x0 (-0.01695887872)
//	#define I_mul_x1 (0.01696927525)
//	#define I_mul_y0 (0.01714115517)
//	#define I_mul_y1 (0.01699685242)
//	#define I_mul_x0 (-0.01349679714)
//	#define I_mul_x1 (0.01370873399)
//	#define I_mul_y0 (0.01317209955)
//	#define I_mul_y1 ( 0.01405904692)
//	#define I_mul_x0 (0.01382768001)
//	#define I_mul_x1 (0.01371409417)
//	#define I_mul_y0 (0.01368305978)
//	#define I_mul_y1 (-0.0138106111)
	#define I_mul_x0 (0.01383840444)
	#define I_mul_x1 (0.01379097491)
	#define I_mul_y0 ( 0.01380621156)
	#define I_mul_y1 (-0.01376404342)
#endif

#ifdef jiaozhun
	int p_rotation_e0_total=0;
	int p_rotation_e1_total=0;
	int n_rotation_e0_total=0;
	int n_rotation_e1_total=0;
	
	float p_rotation_e0=0;
	float p_rotation_e1=0;
	float n_rotation_e0=0;
	float n_rotation_e1=0;

	float I_mul_x0=0;
	float I_mul_x1=0;
	float I_mul_y0=0;
	float I_mul_y1=0;
#endif
//�ض������   uart2�������������
void USART_printf(char *fmt, ...)
{
	va_list ap;
	char str[128];

	va_start(ap,fmt);
	vsprintf(str,fmt,ap);
	va_end(ap);
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str),0xFFFF);
}

//
int if_laser_ready=0;
State state ;
Covariance cov ; 
double process_noise[3][3] ;
double measurement_noise[3][3];

//���ʱ�õ���һЩ����
int data_state=0;   	//��ȡ����״̬  0��ʾ����  1��ʾִ��
int cal_state=0;		//����״̬		0��ʾ����  1��ʾִ��
int now_state=0;		//Ŀǰ״̬     	1��ʾ��ȡ  0��ʾ����
int gyro_update=0;
int encoder_updata=0;
int gyro_start=0;
int if_gyro_err=0;
signed int gyro_vel0=0;
volatile double del_encoder0=0;
volatile double del_encoder1=0;
typedef   signed  int s32;
//double floating0=41.4023013;
//double floating0=40.41666666666;
//double floating0=14285.4411;
double floating0=41.3323674;;
//double convert[2]={1,1};
double convert[2]={20.0720416085,20.05672187};// 3.169196//20.068977071
int gyroloop=0;

//Pc����
uint8_t Pc_buffer[11];
int if_get_laser=0;

//�����״�����
float laser_x=0;
float laser_y=0;
float jiguang_x=0;
float jiguang_y=0;
float laser_data[3];

//���������ݻ�ȡ�Լ���̬�����һЩ�����ͱ���
/**********************************************************************************************************************************/
volatile uint8_t gyro_buffer[33];
uint8_t if_gyro_right=0;

//�����ǽǶ�ֵ��
double gyro_angle=0;
double gyro_angle_del=0;
float q_angle[4]={1,0,0,0};     //��ת��Ԫ��
float Gyro_v[3]={0,0,0};        //�ֱ��Ӧ��x y z��ת�Ľ��ٶ�
float Gyro_a[3]={0,0,0};        //�ֱ��Ӧ��x�� y�� z�᷽��ļ��ٶ�
float gyro_dealt_t=0;	        //���ڴ���ʱ��
float gyro_start_time=0;		//���ڼ�¼��ʼʱ�䣬�����Ư
float gyrotime=0;	        	//��¼������ʱ���
float gyro_lasttime=0;			//��¼�ϴλ�ȡ�����ǵ�ʱ���
int if_gyro_start_first=1;		//��ʼʱ���õ��ı�־λ
uint32_t gyro_vel[3]={0,0,0};   //���ڽ��ս��ٶ�����
uint32_t gyro_a[3]={0,0,0};     //���ڽ���������ٶ�����
float accex=0;					//���ڴ洢������ֵ 
float accey=0;
float accez=0;
int cnt1=0;
int cnt2=0;

//  提取原始角速度和加速度数据
void get_gyro_data()
{
	gyro_vel[0]=(gyro_buffer[5]<<24) | (gyro_buffer[4]<<16)|(gyro_buffer[3]<<8)|gyro_buffer[2];
	gyro_vel[1]=(gyro_buffer[9]<<24) | (gyro_buffer[8]<<16)|(gyro_buffer[7]<<8)|gyro_buffer[6];
	gyro_vel[2]=(gyro_buffer[13]<<24) | (gyro_buffer[12]<<16)|(gyro_buffer[11]<<8)|gyro_buffer[10];
	gyro_a[0]=(gyro_buffer[17]<<24) | (gyro_buffer[16]<<16)|(gyro_buffer[15]<<8)|gyro_buffer[14];
	gyro_a[1]=(gyro_buffer[21]<<24) | (gyro_buffer[20]<<16)|(gyro_buffer[19]<<8)|gyro_buffer[18];
	gyro_a[2]=(gyro_buffer[25]<<24) | (gyro_buffer[24]<<16)|(gyro_buffer[23]<<8)|gyro_buffer[22];
	for(int i=0;i<3;i++)
	{
		Gyro_v[i]=(*((float *)(&gyro_vel[i])))*0.01745329252;  //  将陀螺仪角速度由度/秒（°/s）转为弧度/秒（rad/s），乘以π/180=0.01745329252。
		Gyro_a[i]=*((float *)(&gyro_a[i]));  //  加速度数据直接读取为浮点数
	}
	//  坐标系方向调整
	Gyro_a[0]*=-1;
	Gyro_a[2]*=-1;
	Gyro_v[0]*=-1;
	Gyro_v[2]*=-1;
	 if(Gyro_v[2]>4.98||Gyro_v[2]<-4.98)
	{
		if_gyro_err=1;
		//#ifdef jiaozhun
		USART_printf("err\n");
		//#endif		
	}
}


//  快速平方根倒数算法，用于向量单位化
static float invSqrt(float x)
{
	float halfx=0.5f*x;
	float y=x;
	long  i=*(long*)&y;
	i=0x5f3759df-(i>>1);
	y=*(float*)&i;
	y=y*(1.5f-(halfx*y*y));
	return y;
}


//  加速度修正陀螺仪误差
void gyro_err_clear(float delat_t )
{
	//  单位化加速度向量
	//  加速度计测得的向量应当主要表示重力方向（若无剧烈加速度）。因此将它归一化，得到当前测量的“地球重力方向”


	float xishu=invSqrt(Gyro_a[0]*Gyro_a[0]+Gyro_a[1]*Gyro_a[1]+Gyro_a[2]*Gyro_a[2]);
	Gyro_a[0]*=xishu;
	Gyro_a[1]*=xishu;
	Gyro_a[2]*=xishu;
	//��ȡ��̬�����е���������
	float Vx=2*(q_angle[1]*q_angle[3]-q_angle[0]*q_angle[2]);
	float Vy=2*(q_angle[1]*q_angle[0]+q_angle[3]*q_angle[2]);
	float Vz=1-2*(q_angle[1]*q_angle[1]+q_angle[2]*q_angle[2]);
	//USART_printf(" q0=%f\n",q_angle[0]);
	//�����̬���
	float ex=Gyro_a[1]*Vz-Gyro_a[2]*Vy;
	float ey=Gyro_a[2]*Vx-Gyro_a[0]*Vz;
	float ez=Gyro_a[0]*Vy-Gyro_a[1]*Vx;
	//USART_printf(" %f\n",ez);
	//������
	float ki=0.001;
	float kp=0.01;

	accex+=ex*ki*delat_t;
	accey+=ey*ki*delat_t;
	accez+=ez*ki*delat_t;
	
	//���ٶ�����
	Gyro_v[0]+=kp*ex+accex;
	Gyro_v[1]+=kp*ey+accey;
	Gyro_v[2]+=kp*ez+accez;
	//USART_printf(" v2=%f\n",Gyro_v[2]);
	
}

//  更新四元数
void q_angle_update(float delat_t ) 
{
	q_angle[0]=q_angle[0]+0.5*delat_t*(-1*Gyro_v[0]*q_angle[1]-Gyro_v[1]*q_angle[2]-Gyro_v[2]*q_angle[3]);
	q_angle[1]=q_angle[1]+0.5*delat_t*( 1*Gyro_v[0]*q_angle[0]-Gyro_v[1]*q_angle[3]+Gyro_v[2]*q_angle[2]);
	q_angle[2]=q_angle[2]+0.5*delat_t*( 1*Gyro_v[0]*q_angle[3]+Gyro_v[1]*q_angle[0]-Gyro_v[2]*q_angle[1]);
	q_angle[3]=q_angle[3]+0.5*delat_t*(-1*Gyro_v[0]*q_angle[2]+Gyro_v[1]*q_angle[1]+Gyro_v[2]*q_angle[0]);
	
	//  单位话四元数
	float xishu=invSqrt(q_angle[0]*q_angle[0]+q_angle[1]*q_angle[1]+q_angle[2]*q_angle[2]+q_angle[3]*q_angle[3]);
	for(int i=0;i<=3;i++)
	{
		q_angle[i]*=xishu;
	}
}

//  欧拉角解算
float get_gyro_angle(int i)
{
	float g1=0,g2=0,g3=0,g4=0,g5=0;
  float angle[3]={0};
 
	g1=2*(q_angle[1]*q_angle[3]-q_angle[0]*q_angle[2]);
	g2=2*(q_angle[1]*q_angle[0]+q_angle[3]*q_angle[2]);
	g3=   q_angle[0]*q_angle[0]-q_angle[1]*q_angle[1]-q_angle[2]*q_angle[2]+q_angle[3]*q_angle[3];
	g4=2*(q_angle[1]*q_angle[2]+q_angle[3]*q_angle[0]);
	g5=   q_angle[0]*q_angle[0]+q_angle[1]*q_angle[1]-q_angle[2]*q_angle[2]-q_angle[3]*q_angle[3];
	//  将四元数钻转化为欧拉角
	angle[0]=-1*asinf(g1);
	angle[1]=atanf(g2/g3);
	angle[2]=atan2f(g4,g5);
	//USART_printf("g5=%f\n",g4/g5);
	return angle[i];
}
//���������ݸ��� �������Ƕ�ֵ��ʱ����ĸ���
void gyrodata_update()
{
	//USART_printf("get");
	get_gyro_data();
	gyrotime=HAL_GetTick();
	if(if_gyro_start_first)  //��ȡ��ʼʱ��
	{
		gyro_start_time=gyrotime;
		if_gyro_start_first=0;
	}
	gyro_dealt_t=(gyrotime-gyro_lasttime)*0.001;
	gyro_err_clear(gyro_dealt_t);
	q_angle_update(gyro_dealt_t);
	gyro_angle=get_gyro_angle(2);
	gyro_angle+=0.00075*(gyrotime-gyro_start_time)*0.001; //0.000832 
	//USART_printf("angz %f \n",Gyroz);
	//Gyroz=Gyroz+0.2056-(gyrotime-gyro_lasttime)*0.001*0.0031;
	if(gyro_angle >= pi)
	{
		gyro_angle -= 2*pi;
	}
	if(gyro_angle <= -1*pi)
	{
		gyro_angle+= 2*pi;
	}
	gyro_lasttime=gyrotime;
	if_gyro_right=0;	
	//y = -0.0434x + 0.0031  �Ƕ�������Ư	
}

/**********************************************************************************************************************************/

//�Զ�װ�ش�������ת���+1  ��ת���-1��
volatile int32_t ENCODER_overflow_cnt[2] = {-1,-1};   
TIM_TypeDef* ENCODER_TIM[2]={TIM1, TIM2};


//����������ֵ
int32_t Get_Encoder_Pulse_Count(int i)
{
	return (ENCODER_overflow_cnt[i] * (1 + (ENCODER_TIM[i]->ARR)) + ENCODER_TIM[i]->CNT);    //�õ���������
}	

float pos_x=0;
float pos_y=0;
float sum_del_encoder0=0;
float sum_del_encoder1=0;
float sum_del_angle=0;



void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart)
{
	//USART_printf("get\n");
	if(huart->Instance == USART1)   
	{
		//USART_printf("huart111\n");
		if(!if_gyro_right)
		{
			
			if(gyro_buffer[0]==0x80)
			{
       // USART_printf("get3\n");
				if_gyro_right=1;
			}
		}
		else
		{
			gyro_vel0=((((s32)gyro_buffer[0]) << 25) | (((s32)gyro_buffer[1]) << 18) | (((s32)gyro_buffer[2]) << 11) | (((s32)gyro_buffer[3]) << 4)) >> 4;
			gyro_angle_del=gyro_vel0-floating0;
			#if 0
			gyro_angle += gyro_angle_del/GYRO_SCALAR;  //У׼��Ưʱ��
			#endif
			#if 1
			if(gyro_angle_del >=0) gyro_angle_del *=convert[0];
			else gyro_angle_del *=convert[1];
			
			
    gyro_angle += gyro_angle_del/GYRO_SCALAR;
			
		//У׼����תϵ��  gyro_total*convert/GYRO_SCALAR=loop*2*pi;  22.431381  -62.79590  3.169256
			#if 1
    if (gyro_angle >= pi)
    {
        gyroloop++;
        gyro_angle -= 2*pi;
    }
    else if (gyro_angle <= -pi)
    {
        gyroloop--;
        gyro_angle += 2*pi;
    }
		#endif
		#endif
		
		gyroResult.rotation=gyro_angle;
		gyroResult.loop=gyroloop;
		gyroResult.timeStamp = GenerateTimeStamp();
		double pul0 = Get_Encoder_Pulse_Count(0);
		double pul1 = Get_Encoder_Pulse_Count(1);
		encoderRes.distance[0] += pul0-QEI_pulse[0];
		encoderRes.distance[1] += pul1-QEI_pulse[1];
		QEI_pulse[0] = pul0;
		QEI_pulse[1] = pul1;
		encoderRes.timeStamp = GenerateTimeStamp();
		cnt1++;
		if_gyro_right=0;
		}
		#if 0
		if(!if_gyro_right)
		{
			//USART_printf("get3");
			if(gyro_buffer[0]==0xBD)
			{
				if_gyro_right=1;
			}
		}
		else
		{
			//USART_printf("get2");
			if(!(gyro_buffer[0]==0xDB&&gyro_buffer[1]==0x0A))
			{
				//USART_printf("get2");
				if_gyro_right=0; 
				return;
			}
			else
			{
//				data_state=1;
//				if(cal_state==1)
//				{
//					//USART_printf("wait cal\n");
//					data_state=0;
//					return;
//				}
				
				//gyro_update=1;
				gyrodata_update();
				
				
				gyroResult.rotation=gyro_angle;
				gyroResult.timeStamp = GenerateTimeStamp();
				float pul0 = Get_Encoder_Pulse_Count(0);
				float pul1 = Get_Encoder_Pulse_Count(1);
				encoderRes.distance[0] += pul0-QEI_pulse[0];
				encoderRes.distance[1] += pul1-QEI_pulse[1];
				QEI_pulse[0] = pul0;
				QEI_pulse[1] = pul1;
				encoderRes.timeStamp = GenerateTimeStamp();
				cnt1++;
				
				//process_locator_data();
			}
			//data_state=0;
		}
		#endif
		#ifndef jiaozhun
		return;
		#endif
	}
	if(huart->Instance == USART2)
	{
		#ifdef jiaozhun
		if(Pc_buffer[0]=='1'&&Pc_buffer[1]=='s')
		{
			jiaozhun_flag=1;
			start_flag_jiaozhun=1;
			USART_printf("start floating jiaozhun!");
			
		}
		else if(Pc_buffer[0]=='2'&&Pc_buffer[1]=='s')
		{
			jiaozhun_flag=2;
			start_flag_jiaozhun=1;
			USART_printf("start encoder jiaozhun!");
			pulse_out=1;
			
		}
		else if(Pc_buffer[0]=='2'&&Pc_buffer[1]=='e')
		{
			jiaozhun_flag=2;
			pulse_out=2;
			
		}
		else if(Pc_buffer[0]=='3'&&Pc_buffer[1]=='s')
		{
			jiaozhun_flag=3;
			start_flag_jiaozhun=1;
			pulse_out=1;
			USART_printf("start locator jiaozhun!");
			
		}
		else if(Pc_buffer[0]=='3'&&Pc_buffer[1]=='e')
		{
			jiaozhun_flag=3;
			pulse_out=2;
			
		}
		else if(Pc_buffer[0]=='4'&&Pc_buffer[1]=='s')
		{
			jiaozhun_flag=4;
			start_flag_jiaozhun=1;
			pulse_out=1;
			USART_printf("start locator jiaozhun!");
			
		}
		else if(Pc_buffer[0]=='4'&&Pc_buffer[1]=='e')
		{
			jiaozhun_flag=4;
			pulse_out=2;
			
		}			
		else{
			USART_printf("er111r\n");
		}
		HAL_UART_Receive_IT(&huart2, (uint8_t*) Pc_buffer, 2);
		#else

				if(!if_get_laser)
		{
			if(Pc_buffer[0]==0xBE)
			{
				
				//printf("%f\n", lcResult.x);
				if_get_laser=1;
				//USART_printf("IFLOC %d\n",if_get_loc);
			}

			//printf("IFLOC %d\n",if_get_loc);
		}
		else
		{
			
			static uint32_t data_base;

			
			if(!(Pc_buffer[0]==0xBF&&Pc_buffer[1]==0XCF))
			{
					if_get_laser=0;
				//	printf("err");
			//	
			}
				
			else
			{

								//1.01608579
			  data_base=(uint32_t)Pc_buffer[5]<<24|(uint32_t)Pc_buffer[4]<<16|(uint32_t)Pc_buffer[3]<<8|(uint32_t)Pc_buffer[2];
				laser_x=(*((float *)(&data_base)));
				data_base=(uint32_t)Pc_buffer[9]<<24|(uint32_t)Pc_buffer[8]<<16|(uint32_t)Pc_buffer[7]<<8|(uint32_t)Pc_buffer[6];
				laser_y=*((float *)(&data_base));
        if_get_laser=0;
				laser_data[0]=laser_x;
				laser_data[1]=laser_y;
				laser_data[2]=lcResult.r;
				if_laser_ready=1;
			}
		}
		
		#endif
        
	}
}
//E0 TIM1 A8 A9
//E1 TIM2 A0 A1

void  HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//encoder  0   ������
 	if(htim->Instance==TIM1)
	{
		
		//USART_printf("in\n");
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1))
		{
			--ENCODER_overflow_cnt[0];
			//USART_printf("11  %d\n",ENCODER_overflow_cnt[0]);
		}
		else
		{
			++ENCODER_overflow_cnt[0];
			//USART_printf("22  %d\n",ENCODER_overflow_cnt[0]);
		}
//		float pul0 = Get_Encoder_Pulse_Count(0);
//		float pul1 = Get_Encoder_Pulse_Count(1);
//		encoderRes.distance[0] += pul0-QEI_pulse[0];
//		encoderRes.distance[1] += pul1-QEI_pulse[1];
//    QEI_pulse[0] = pul0;
//		QEI_pulse[1] = pul1;
//		encoderRes.timeStamp = GenerateTimeStamp();
	}
	//encoder 1  ������
	if(htim->Instance==TIM2)
	{
		//process_locator_data();
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
		{
			--ENCODER_overflow_cnt[1];
		}
		else
		{
			++ENCODER_overflow_cnt[1];
		}
//		float pul0 = Get_Encoder_Pulse_Count(0);
//		float pul1 = Get_Encoder_Pulse_Count(1);
//		encoderRes.distance[0] += pul0-QEI_pulse[0];
//		encoderRes.distance[1] += pul1-QEI_pulse[1];
//    QEI_pulse[0] = pul0;
//		QEI_pulse[1] = pul1;
//		encoderRes.timeStamp = GenerateTimeStamp();		
	}
}
/*************************************************************/

void LocatorUpdate()
{


//	cal_state=1;   //��������ʱ  cal_state=1 now_state=0;
//	if(data_state==1)
//	{
//		cal_state=0;
//		USART_printf("wait get\n");
//		return;
//	}
//	
//	
//	if(gyro_update==0) //cal_state[1]=0;
//	{         
//		cal_state=0;
//		//USART_printf("wait gyro\n");
//		return;
//	}
//	else gyro_update=0; 
	
	lcResult.timeStamp = GenerateTimeStamp();
	double dx,dy,dr,sinr,cosr;
	double dz;

//	gyroResult.rotation=gyro_angle;
//	gyroResult.timeStamp = GenerateTimeStamp();
//	float pul0 = Get_Encoder_Pulse_Count(0);
//	float pul1 = Get_Encoder_Pulse_Count(1);
//	data_state=0;  
//	encoderRes.distance[0] += pul0-QEI_pulse[0];
//	encoderRes.distance[1] += pul1-QEI_pulse[1];
//	QEI_pulse[0] = pul0;
//	QEI_pulse[1] = pul1;
//	encoderRes.timeStamp = GenerateTimeStamp();
				
	struct GyroResult readGyro = gyroResult;
	struct EncoderResult readEnc = encoderRes;
  struct LocatorResult readlc = lcResult;
	
	double deltaTime = TIMESTAMP2SECOND(readlc.timeStamp,timerlcResult.timeStamp);
	if(deltaTime>LOCATOR_MIN_TIMEGAP)
	{
		deltaTime*=5;
		total_x[v_n]=vx_dis;
		total_y[v_n]=vy_dis;
		total_r[v_n]=vr_ang;
		v_n=(v_n==4)?0:(v_n+1);
		for(int n=0;n<=4;n++)
		{
			lcResult.vx+=total_x[n];
			lcResult.vy+=total_y[n];
			lcResult.vAng+=total_r[n];
		}
		lcResult.vx=(total_x[0]+total_x[1]+total_x[2]+total_x[3]+total_x[4])/deltaTime;
		lcResult.vy=(total_y[0]+total_y[1]+total_y[2]+total_y[3]+total_y[4])/deltaTime;
		lcResult.vAng=(total_r[0]+total_r[1]+total_r[2]+total_r[3]+total_r[4])/deltaTime;
		vx_dis=0;
		vy_dis=0;
		vr_ang=0;
		timerlcResult = readlc;
	
	}	
	
	//dx dy dr
	dr=readGyro.rotation-lastGyroResult.rotation;

	//dr����һ��
	if (dr >= pi) dr -= 2*pi;
	else if (dr <= -pi) dr += 2*pi;
	//lcResult.r����һ��
  lcResult.r += dr;
	vr_ang+=dr;
	if (lcResult.r >= pi) lcResult.r -= 2*pi;
	else if (lcResult.r <= -pi) lcResult.r += 2*pi;
	//ʵ��Ϊ�������̷����dealt dis
	dx = readEnc.distance[0]-lastEncResult.distance[0];
	dy = readEnc.distance[1]-lastEncResult.distance[1];
	//����ת���Ƕȴ����ı���������
	if(dr>0)
	{
		 dx -= p_rotation_e0*dr;
		 dy -= p_rotation_e1*dr;
	}
	else
	{
		 dx -= n_rotation_e0*dr;
		 dy -= n_rotation_e1*dr;
	}
	//ת��Ϊ��������ϵ�µ�dealt xy
	dz = dx;//using as a buffer
	dx=I_mul_x0*dz+I_mul_x1*dy;
	dy=I_mul_y0*dz+I_mul_y1*dy;
	//ת��Ϊȫ������ϵ�µĶ�λ�������� dx dy
	dz = dx;//using as a buffer

	//USART_printf("dx%f  dy%f\n",dx,dy);

	sinr = sin(lcResult.r);
	cosr = cos(lcResult.r);
	dx = cosr*dz-sinr*dy;
	//r>0
	dy = sinr*dz+cosr*dy;
//	if(dx>0.1)
//	{
//	USART_printf("dx%f\n",dx);
//	}
	lcResult.x += dx;
	lcResult.y += dy;
	vx_dis+=dx;
	vy_dis+=dy;
  //�����ٶ�  (����ȥ�趨ֹͣ����)


	lastEncResult = readEnc;
	lastGyroResult = readGyro;
	//cal_state=0;
	if(restart_flag==1)
	{
		//LocatorSystemInitialize();
		lcResult.x=105;
		lcResult.y=105;
//		lcResult.x=0;
//		lcResult.y=0;
		lcResult.r=0;
		restart_flag=0;
		
	}

}



void LocatorSystemInitialize()
{
    gyroResult.timeStamp = 0; 
    gyroResult.rotation = 0;
	  encoderRes.distance[0]=0;
	  encoderRes.distance[1]=0;
	  lcResult.timeStamp=0;
    lcResult.x = 105;
    lcResult.y = 105;
//	lcResult.x=0;
//		lcResult.y=0;
    lcResult.r = 0;
	  lcResult.vx = 0;
    lcResult.vy = 0;
    lcResult.vAng = 0;
	  lastEncResult = encoderRes;
    lastGyroResult = gyroResult;
}

void send_lc(float x ,float y ,float r,float vx,float vy,float vr){
	static uint8_t data_byte[24];
	uint8_t* tmp1 = (uint8_t*)&x;
	data_byte[0] = tmp1[0]; 
	data_byte[1] = tmp1[1];
	data_byte[2] = tmp1[2];
	data_byte[3] = tmp1[3];
	uint8_t* tmp2 = (uint8_t*)&y;
	data_byte[4] = tmp2[0]; 
	data_byte[5] = tmp2[1];
	data_byte[6] = tmp2[2];
	data_byte[7] = tmp2[3];                                                                  
	uint8_t* tmp3 = (uint8_t*)&r;
	data_byte[8] = tmp3[0]; 
	data_byte[9] = tmp3[1];
	data_byte[10] = tmp3[2];
	data_byte[11] = tmp3[3];
	uint8_t* tmp4 = (uint8_t*)&vx;
	data_byte[12] = tmp4[0]; 
	data_byte[13] = tmp4[1];
	data_byte[14] = tmp4[2];
	data_byte[15] = tmp4[3];
	uint8_t* tmp5 = (uint8_t*)&vy;
	data_byte[16] = tmp5[0]; 
	data_byte[17] = tmp5[1];
	data_byte[18] = tmp5[2];
	data_byte[19] = tmp5[3];
	uint8_t* tmp6 = (uint8_t*)&vr;
	data_byte[20] = tmp6[0]; 
	data_byte[21] = tmp6[1];
	data_byte[22] = tmp6[2];
	data_byte[23] = tmp6[3];
	
	 //����FDCAN��Ϣ�ṹ
//    uint8_t TxData[12] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,0x09,0xA,0xB,0xC};
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 0xAA;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_24;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    // ��������
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data_byte) != HAL_OK)
    {
        // ������
        USART_printf("no sb!\n");
       // Error_Handler();
    }
		
}


//�����״��㷨��ʼ������
void laser_start(){
  FDCAN_TxHeaderTypeDef TxMessage;
    uint8_t txData[8];
    TxMessage.Identifier=0xAA;//���﷢�͸�����������id��Ϊ0xAA
    TxMessage.IdType = FDCAN_STANDARD_ID;
    TxMessage.TxFrameType = FDCAN_DATA_FRAME;
    TxMessage.DataLength = FDCAN_DLC_BYTES_8;
    TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxMessage.BitRateSwitch = FDCAN_BRS_ON;
    TxMessage.FDFormat = FDCAN_FD_CAN;
    TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxMessage.MessageMarker = 0;
    txData[0]=txData[1]=txData[2]=txData[3]=txData[4]=txData[5]=txData[6]=txData[7]=0;//��������ν
    if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&TxMessage,txData)!=HAL_OK)
    {
        printf("Send :Laser ERROR\n");
    }
}



void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
      if(huart == &huart1)
    {
        __HAL_UNLOCK(huart);
        HAL_UART_Receive_IT(&huart1, (uint8_t*) gyro_buffer, 1);
    }
        else if(huart == &huart2)
        {
        __HAL_UNLOCK(huart);
        HAL_UART_Receive_IT(&huart2, (uint8_t*) Pc_buffer, 2);
        }
}




//can1��������
uint32_t dis1_temp = 0;
uint32_t dis2_temp = 0;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan==&hfdcan1){
			FDCAN_RxHeaderTypeDef rx_header;
			uint8_t rx_data[8];
			static uint32_t data_base;
			if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {

            Error_Handler();
        }
			if(rx_header.Identifier==0xAE)
			{
				if(rx_data[0]=='r')
				{
					laser_start();
					restart_flag=1;
				}
			}
			if(rx_header.Identifier==0xBE)
			{
				if(rx_data[0]=='j')
				{
					if(dis1-5+23.06<2500  && dis1-5+23.06>200  &&  dis2-5-21.46>200 && dis2-5-21.46<2500)
					{
						lcResult.x=dis1-5+23.06;  //703
						lcResult.y=dis2-5-21.46;  //86
					}
					else
					{
						lcResult.x=lcResult.x;
						lcResult.y=lcResult.y;
					}
					
					jiguang_restart_position( lcResult.x,lcResult.y,lcResult.r,lcResult.vx,lcResult.vy,lcResult.vAng);
					
						//	jiguang_restart_position( lcResult.x,lcResult.y,lcResult.r,lcResult.vx,lcResult.vy,lcResult.vAng);
				//	jiguang_restart_position( lcResult.x,lcResult.y,lcResult.r,lcResult.vx,lcResult.vy,lcResult.vAng);
				}
			}
		
    }
}

void restart_position();
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if(hfdcan == &hfdcan2)
    {
			//USART_printf("LSMCKAJSAK");
        FDCAN_RxHeaderTypeDef rx_header;
				uint8_t rx_data[8];
				static uint64_t data_base;
				if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data)!=HAL_OK)
				{
					USART_printf("NO laser\n");
				}
				if(rx_header.Identifier==0x14)
				{
					dis1_temp=(uint32_t)((rx_data[3]<<24)|(rx_data[2]<<16)|(rx_data[1]<<8)|(rx_data[0]));
					//dis1=*((float*)&dis1);
					dis1 = (float)dis1_temp/100.0f;
					//dis1 =  2000;
					dis2_temp=(uint32_t)((rx_data[7]<<24)|(rx_data[6]<<16)|(rx_data[5]<<8)|(rx_data[4]));
			    //dis2=*((float*)&dis2);
					dis2 = (float)dis2_temp/100.0f;
					//dis2 = 1000;
				}
		
    
    }
}
void jiguang_restart_position(float x ,float y ,float r,float vx,float vy,float vr)
{
		static uint8_t data_byte[24];
	uint8_t* tmp1 = (uint8_t*)&x;
	data_byte[0] = tmp1[0]; 
	data_byte[1] = tmp1[1];
	data_byte[2] = tmp1[2];
	data_byte[3] = tmp1[3];
	uint8_t* tmp2 = (uint8_t*)&y;
	data_byte[4] = tmp2[0]; 
	data_byte[5] = tmp2[1];
	data_byte[6] = tmp2[2];
	data_byte[7] = tmp2[3];                                                                  
	uint8_t* tmp3 = (uint8_t*)&r;
	data_byte[8] = tmp3[0]; 
	data_byte[9] = tmp3[1];
	data_byte[10] = tmp3[2];
	data_byte[11] = tmp3[3];
	uint8_t* tmp4 = (uint8_t*)&vx;
	data_byte[12] = tmp4[0]; 
	data_byte[13] = tmp4[1];
	data_byte[14] = tmp4[2];
	data_byte[15] = tmp4[3];
	uint8_t* tmp5 = (uint8_t*)&vy;
	data_byte[16] = tmp5[0]; 
	data_byte[17] = tmp5[1];
	data_byte[18] = tmp5[2];
	data_byte[19] = tmp5[3];
	uint8_t* tmp6 = (uint8_t*)&vr;
	data_byte[20] = tmp6[0]; 
	data_byte[21] = tmp6[1];
	data_byte[22] = tmp6[2];
	data_byte[23] = tmp6[3];
	
	 //����FDCAN��Ϣ�ṹ
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 0xCE;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_24;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    // ��������
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data_byte) != HAL_OK)
    {
        // ������
        printf("no sb!\n");
       // Error_Handler();
    }
}
void restart_position()
{
	static uint8_t data_byte[1];
    data_byte[0] = (dis1_temp & 0XFF);//'r';

	 //创建FDCAN消息结构
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = 0xAE;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    // 发送数据
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, data_byte) != HAL_OK)
    {
        // 错误处理
        printf("no sb!\n");
       // Error_Handler();
    }
}


// ��һ���Ƕ� [-PI, PI]
double normalize_angle(double angle) {
    while (angle > pi) angle -= 2 * pi;
    while (angle < -pi) angle += 2 * pi;
    return angle;
}

// ״̬Ԥ��
void predict(State *state, Covariance *cov, double imu_x, double imu_y, double imu_theta, double Q[3][3]) {
    // ����״̬
    state->x = imu_x;
    state->y = imu_y;
    state->theta = normalize_angle(imu_theta);

    // ״̬ת�ƾ��� F
    double F[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    // ����Э������� P_k = F * P_{k-1} * F^T + Q
    double P_new[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                P_new[i][j] += F[i][k] * cov->P[k][j];
            }
            P_new[i][j] += Q[i][j];
        }
    }

    // ����Э����
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cov->P[i][j] = P_new[i][j];
        }
    }
}

// ״̬���£��ںϼ����״����ݣ�
void update(State *state, Covariance *cov, float z[3], double R[3][3]) {
    // �������� H
    double H[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    // �����в� y = z - H * x
    double y[3] = {
        z[0] - state->x,
        z[1] - state->y,
        z[2] - state->theta
    };
    y[2] = normalize_angle(y[2]);

    // ���� S = H * P * H^T + R
    double S[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                S[i][j] += H[i][k] * cov->P[k][j];
            }
            S[i][j] += R[i][j];
        }
    }

    // ���㿨�������� K = P * H^T * S^-1����Ϊ�Խ��ߴ���
    double K[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = cov->P[i][j] / S[j][j]; // ����������
        }
    }

    // ����״̬ x = x + K * y
    state->x += K[0][0] * y[0] + K[0][1] * y[1] + K[0][2] * y[2];
    state->y += K[1][0] * y[0] + K[1][1] * y[1] + K[1][2] * y[2];
    state->theta += K[2][0] * y[0] + K[2][1] * y[1] + K[2][2] * y[2];
    state->theta = normalize_angle(state->theta);

    // ����Э���� P = (I - K * H) * P
    double I_KH[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        I_KH[i][i] = 1.0;
        for (int j = 0; j < 3; j++) {
            I_KH[i][j] -= K[i][j] * H[j][j];
        }
    }

    double P_new[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                P_new[i][j] += I_KH[i][k] * cov->P[k][j];
            }
        }
    }

    // ����Э����
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cov->P[i][j] = P_new[i][j];
        }
    }
}
void Merge_init()
{
	State state = {0, 0, 0};
    Covariance cov = { 
        {{0.1, 0, 0}, 
		 {0, 0.1, 0}, 
		 {0, 0, 0.1}} 
    };

    // ���������Ͳ�������
    double process_noise[3][3] = {
        {0.01, 0, 0},
        {0, 0.01, 0},
        {0, 0, 0.01}
    };
    double measurement_noise[3][3] = {
        {2.974, 0, 0},
        {0, 4.974, 0},
        {0, 0, 0}
    };
}
void Merge_calculate()
{
	predict(&state, &cov, lcResult.x, lcResult.y, lcResult.r, process_noise);
	if(if_laser_ready)
	{
		update(&state, &cov, laser_data, measurement_noise);
		if_laser_ready=0;
//		lcResult.x=state.x;
//		lcResult.y=state.y;
		lcResult.x=laser_x;
		lcResult.y=laser_y;
		lcResult.r=state.theta;
	}

}
//	float x=0.01,y=0.01,r=0.01;
/*************************************************************/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FDCAN2_Init();
  /* USER CODE BEGIN 2 */
	//Merge_init();
	
  HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Receive_IT(&huart1, (uint8_t*) gyro_buffer, 1);
	HAL_UART_Receive_IT(&huart2, (uint8_t*) Pc_buffer, 2);
	USART_printf("ENTER MAIN\n");
	
	//ʱ���߶���  ��һ�λ�ȡʱ��
	int time0=HAL_GetTick();
	int time1=time0;
	int time2=time0;
	int time3=time0;
	//USART_printf("okkk");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		LocatorSystemInitialize();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//���»�ȡʱ��
		//gyro_cal();
		LocatorUpdate();
	//	Merge_calculate();
		//cnt2++;
		time0=HAL_GetTick();
		//cal_loc();
		
		int32_t pulse0=Get_Encoder_Pulse_Count(0); 
		int32_t pulse1=Get_Encoder_Pulse_Count(1); 
		//��ͬ����ʱ�����½��в���
		//restart_position();
		if(time0-time1>=400) 
		{
			//restart_position();
			//���ⷢ����Ϣʱ���ܴ����κδ�ӡ
			//USART_printf("dis1 %f \n",dis1);
			
//	USART_printf("position x%.6f y%.6f r%.6f \n",lcResult.x,lcResult.y,lcResult.r);
	//USART_printf("laser position x%.6f y%.6f \n",laser_x ,laser_y);
//	USART_printf(" JIGUANG position x%.6f y%.6f  \n",laser_x,laser_y);

//			USART_printf("ver vx%.6f vy%.6f vr%.6f \n",lcResult.vx,lcResult.vy,lcResult.vAng);
	//		USART_printf("%.2f %.2f %.2f\n",dis1,dis2, dis3);
//			USART_printf("pulse1 %d\n",pulse1);
			//USART_printf("floating=%f",gyro_angle);
			//gyro_angle;
	//		USART_printf("ang %f,cnt %d\n",gyro_angle,cnt1);
//		int32_t p0=Get_Encoder_Pulse_Count(0); 
//		int32_t p1=Get_Encoder_Pulse_Count(1);
			//USART_printf("p0=%d     p1=%d\n",pulse0,pulse1);
			
//			USART_printf("%d\n",pulse1);

//			USART_printf("%d\n",ENCODER_overflow_cnt[0]);
//			USART_printf("%d\n",__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1));
			//USART_printf("%d\n",ENCODER_TIM[0]->CNT);
//			if(flag)
//			{
//				USART_printf("angz %f    time  %d\n",Gyroz, time0);
//				USART_printf("angz %f    time  %d\n",Gyroz_a, time0);
//				USART_printf("angx %f    time  %d\n",Gyrox_a, time0);
//				USART_printf("angy %f    time  %d\n",Gyroy_a, time0);
//				USART_printf("buff10  %d\n",gyro_buffer[10]);
//				USART_printf("buff11  %d\n",gyro_buffer[11]);
//				USART_printf("buff12  %d\n",gyro_buffer[12]);
//				USART_printf("buff13  %d\n",gyro_buffer[13]);
//			}
			time1=time0;
		}
		if(time0-time2>=2)
		{
			#ifdef jiaozhun
				go_jiaozhun();
			#endif
			time2=time0;
		}
		if(time0-time3>=20)
		{
			//USART_printf("cnt1=%d, cnt2=%d\n",cnt1,cnt2);
//			cnt1=0;
//			cnt2=0;
//			x+=0.01;
//			y+=0.02;
//			r-=0.01;
			//#ifndef jiaozhun
			//
			//USART_printf("position  x%.6f y%.6f r%.6f \n",pos_x,pos_y,gyro_angle);
	 send_lc(lcResult.x,lcResult.y,lcResult.r,lcResult.vx,lcResult.vy,lcResult.vAng);	  //�������  ����3   ����ʱ����
			//send_lc(lcResult.x,lcResult.y,lcResult.r,lcResult.vx,lcResult.vy,lcResult.vAng);
	//USART_printf("x%f,  y%f  z%f\n",lcResult.x,lcResult.y,lcResult.r);
//			USART_printf("%lf,   %lf\n",pulse0,pulse1);
			//send_lc(1.1,1.1,1.1,1.1,1.1,1.1);
			//#endif	
			time3=time0;
  }
}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#ifdef jiaozhun
void go_jiaozhun(void)
{
	now_out_time=HAL_GetTick();
	//��ֹУ׼��Ư
		if(jiaozhun_flag==1)
		{
			
			if(start_flag_jiaozhun)
			{
				gyro_angle_jiaozhun_start=gyro_angle;
				start_flag_jiaozhun=0;
				gyro_start_time_jiaozhun=HAL_GetTick();
			}
			gyro_angle_jiaozhun=gyro_angle-gyro_angle_jiaozhun_start;
			cnt++;
			if(now_out_time-last_out_time>400){
					USART_printf("gyro_angle=%f\n",gyro_angle_jiaozhun);
				last_out_time=now_out_time;
			}
			

			
			//floating_total+=gyro_angle_jiaozhun;
			//����У׼��Ưʱ��
			if(cnt>floating_numbers&&jiaozhun_flag==1)
			{
				int gyro_end_time_jiaozhun=HAL_GetTick();
				floating=gyro_angle_jiaozhun/((gyro_end_time_jiaozhun-gyro_start_time_jiaozhun)*0.001);
				USART_printf("floating=%f\n",floating);
				USART_printf("time=%d\n",gyro_end_time_jiaozhun-gyro_start_time_jiaozhun);
				USART_printf("floating=%f\n",floating);
				USART_printf("floating=%f\n",floating);
				jiaozhun_flag=0;
				USART_printf("end floating jiaozhun!\n");
			}
		}
	//ֻƽ��   У׼  ����;���֮��Ļ����ϵ   �Լ���������ϵ�Ͷ�λ����ϵ�Ļ����ϵ  
		if(jiaozhun_flag==2)
		{
			//��ȡ��ʼʱ�̵���������ֵ
			if(start_flag_jiaozhun)
			{
				pulse1_jiaozhun_start=Get_Encoder_Pulse_Count(1);
				pulse0_jiaozhun_start=Get_Encoder_Pulse_Count(0);
				start_flag_jiaozhun=0;
			}
			pulse1_jiaozhun=Get_Encoder_Pulse_Count(1)-pulse1_jiaozhun_start;
			pulse0_jiaozhun=Get_Encoder_Pulse_Count(0)-pulse0_jiaozhun_start;
			if(pulse_out)	
			{
				if(now_out_time-last_out_time>400){
				USART_printf("pulse0=%d\n",pulse0_jiaozhun);
				USART_printf("pulse1=%d\n",pulse1_jiaozhun);
				last_out_time=now_out_time;
			}

			}
			if(pulse_out==2)
			{
				USART_printf("endpulse0=%d  endpulse1=%d\n",pulse0_jiaozhun,pulse1_jiaozhun);
				USART_printf("end edncoder jiaozhun!\n");
				pulse_out=0;
			}
		}
		//ֻ��ת  У׼  ��������ת������仯   ��Ϊ��ת�ͷ�ת������ �ֱ�ȥУ׼����ϵ��
		//˳�������ǽǶ�ֵ����ķ�����ת
		if(jiaozhun_flag==3)
		{
			if(start_flag_jiaozhun)
			{
				pulse1_jiaozhun_start=Get_Encoder_Pulse_Count(1);
				pulse0_jiaozhun_start=Get_Encoder_Pulse_Count(0);
				gyro_angle_jiaozhun_start=gyro_angle;
				last_angle_jiaozhun=gyro_angle_jiaozhun_start;
//				pulse0_jiaozhun_last=pulse0_jiaozhun_start;
//				pulse1_jiaozhun_last=pulse1_jiaozhun_start;
//				p_rotation_e0_total=0;
//				p_rotation_e1_total=0;
				p_del_angle_jiaozhun_total=0;
				start_flag_jiaozhun=0;
//				cnt_p=0;
				del_angle_jiaozhun=0;
			}
		

		
			pulse0_jiaozhun=Get_Encoder_Pulse_Count(0);
			pulse1_jiaozhun=Get_Encoder_Pulse_Count(1);
		
			del_angle_jiaozhun=gyro_angle-last_angle_jiaozhun;
//			int del_encoder0_jiaozhun=(pulse0_jiaozhun-pulse0_jiaozhun_last);
//			int del_encoder1_jiaozhun=(pulse1_jiaozhun-pulse1_jiaozhun_last);
			
			if(del_angle_jiaozhun>pi||del_angle_jiaozhun<-pi)
			{
				if(last_angle_jiaozhun<0)            // del_angle > pi  ���ŽǶȼ�С�ķ�����תһ��С��pi�ĽǶ�
				{
					last_angle_jiaozhun+=2*pi;
				}
				else                        //del_angle<-pi    ���ŽǶ����ӵķ�����תһ��С��pi�ĽǶ�
				{
					last_angle_jiaozhun-=2*pi;
				}
			}
			del_angle_jiaozhun=gyro_angle-last_angle_jiaozhun;     //�õ�һ�������ź͹涨�Ƕ�������ͬ���    С��pi�ĽǶ�

			if(del_angle_jiaozhun>0)
			{
//				 cnt_p++;
//				 p_rotation_e0_total+=(del_encoder0_jiaozhun/del_angle_jiaozhun);
//				 p_rotation_e1_total+=(del_encoder1_jiaozhun/del_angle_jiaozhun);
				  p_del_angle_jiaozhun_total+=del_angle_jiaozhun;
			}
			if(pulse_out)
			{
				if(now_out_time-last_out_time>400){
				USART_printf("pulse0=%d\n",pulse0_jiaozhun);
				USART_printf("pulse1=%d\n",pulse1_jiaozhun);
				USART_printf("gyro_angle=%f\n",gyro_angle);
					last_out_time=now_out_time;
				}
			}
			if(pulse_out==2)
			{
//				p_rotation_e0=p_rotation_e0_total/cnt_p;
//				p_rotation_e1=p_rotation_e1_total/cnt_p;
				p_rotation_e0=(pulse0_jiaozhun-pulse0_jiaozhun_start)/p_del_angle_jiaozhun_total;
				p_rotation_e1=(pulse1_jiaozhun-pulse1_jiaozhun_start)/p_del_angle_jiaozhun_total;
				USART_printf("p_rotation_e0=%f\n",p_rotation_e0);
				USART_printf("p_rotation_e1=%f\n",p_rotation_e1);
				USART_printf("p_total_ang=%f\n",p_del_angle_jiaozhun_total);
				USART_printf("go next locator jiaozhun!\n");
				pulse_out=0;
			}
			last_angle_jiaozhun=gyro_angle;
//			pulse0_jiaozhun_last=pulse0_jiaozhun;
//			pulse1_jiaozhun_last=pulse1_jiaozhun;
		}
		
		//˳�������ǽǶ�ֵ��С�ķ�����ת
		if(jiaozhun_flag==4)
		{
			if(start_flag_jiaozhun)
			{
				pulse1_jiaozhun_start=Get_Encoder_Pulse_Count(1);
				pulse0_jiaozhun_start=Get_Encoder_Pulse_Count(0);
				gyro_angle_jiaozhun_start=gyro_angle;							
				last_angle_jiaozhun=gyro_angle_jiaozhun_start;
//				pulse0_jiaozhun_last=pulse0_jiaozhun_start;
//				pulse1_jiaozhun_last=pulse1_jiaozhun_start;
//				n_rotation_e0_total=0;
//				n_rotation_e1_total=0;
				n_del_angle_jiaozhun_total=0;
				start_flag_jiaozhun=0;
				del_angle_jiaozhun=0;
//				cnt_n=0;
			}

		
			pulse0_jiaozhun=Get_Encoder_Pulse_Count(0);
			pulse1_jiaozhun=Get_Encoder_Pulse_Count(1);
		
			del_angle_jiaozhun=gyro_angle-last_angle_jiaozhun;
//			int del_encoder0_jiaozhun=(pulse0_jiaozhun-pulse0_jiaozhun_last);
//			int del_encoder1_jiaozhun=(pulse1_jiaozhun-pulse1_jiaozhun_last);
			
			if(del_angle_jiaozhun>pi||del_angle_jiaozhun<-pi)
			{
				if(last_angle_jiaozhun<0)            // del_angle > pi  ���ŽǶȼ�С�ķ�����תһ��С��pi�ĽǶ�
				{
					last_angle_jiaozhun+=2*pi;
				}
				else                        //del_angle<-pi    ���ŽǶ����ӵķ�����תһ��С��pi�ĽǶ�
				{
					last_angle_jiaozhun-=2*pi;
				}
			}
			del_angle_jiaozhun=gyro_angle-last_angle_jiaozhun;     //�õ�һ�������ź͹涨�Ƕ�������ͬ���    С��pi�ĽǶ�

			if(del_angle_jiaozhun<0)
			{
//				 cnt_n++;
//				 n_rotation_e0_total+=(del_encoder0_jiaozhun/del_angle_jiaozhun);
//				 n_rotation_e1_total+=(del_encoder1_jiaozhun/del_angle_jiaozhun);
				 n_del_angle_jiaozhun_total+=del_angle_jiaozhun;
			}
			if(pulse_out)
			{
				if(now_out_time-last_out_time>400){
				USART_printf("pulse0=%d\n",pulse0_jiaozhun);
				USART_printf("pulse1=%d\n",pulse1_jiaozhun);
				USART_printf("gyro_angle=%f\n",gyro_angle);
					last_out_time=now_out_time;
				}
			}
			if(pulse_out==2)
			{
				n_rotation_e0=(pulse0_jiaozhun-pulse0_jiaozhun_start)/n_del_angle_jiaozhun_total;
				n_rotation_e1=(pulse1_jiaozhun-pulse1_jiaozhun_start)/n_del_angle_jiaozhun_total;
				USART_printf("n_rotation_e0=%f\n",n_rotation_e0);
				USART_printf("n_rotation_e1=%f\n",n_rotation_e1);
				USART_printf("n_total_ang=%f\n",n_del_angle_jiaozhun_total);
				USART_printf("end locator jiaozhun!\n");
				pulse_out=0;
			}
			last_angle_jiaozhun=gyro_angle;
//			pulse0_jiaozhun_last=pulse0_jiaozhun;
//			pulse1_jiaozhun_last=pulse1_jiaozhun;
		}
}
#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
