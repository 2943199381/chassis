#include "chassis_route.h"
#include "chassis_path.h"
#include "chassis_pid.h"
#include "chassis_driver.h"
#include "locator_driver.h"
#include "math.h"
#include "bsp_can.h"
#include <stdio.h>

float KP_route=3.5;
float KI_route=0.085;
float KD_route=0.00020;

float sumerr_ang_spd;
vec2 sum_err_spd;
vec2 sum_err_spd_kj;

float abs_limit_route_ill=100;
float abs_limit_route_vel=2000;

int flag_if_route_finish_g;
int flag_if_change_target_g=0;
vec2 route_spd_local;

double pid_path_angle(double target_angle, double now_angle)
{
    /* 使用新的 PID 封装函数计算角速度，避免在本模块重复实现 PID 逻辑 */
    return (double)PID_Angle_Calculate(&chassis_yaw_pid, (float)target_angle, (float)now_angle);
}

vec2 getSpd_Route2Point(Point start_point,Point target_point,Dis_spd dis_spd)
{
	static vec2 last_point;
	static vec2 spd;
    double min=0.0001;
    Point now_point;
    /* 直接使用全局定位结果 lcResult 填充 Point */
    now_point.x = lcResult.x;
    now_point.y = lcResult.y;
    Speed_Distribute(start_point,target_point,dis_spd);
	spd.x = KP_route * (target_point.x - now_point.x);
	spd.y = KP_route * (target_point.y - now_point.y);
	sum_err_spd.x+=KI_route * (target_point.x - now_point.x);
	sum_err_spd.y+=KI_route * (target_point.y - now_point.y);
	sum_err_spd.x-=KD_route*(now_point.x-last_point.x);
	sum_err_spd.y-=KD_route*(now_point.y-last_point.y);
	if(sum_err_spd.x>abs_limit_route_ill) sum_err_spd.x=abs_limit_route_ill;
	if(sum_err_spd.x<-abs_limit_route_ill) sum_err_spd.x=-abs_limit_route_ill;
	if(sum_err_spd.y>abs_limit_route_ill) sum_err_spd.y=abs_limit_route_ill;
	if(sum_err_spd.y<-abs_limit_route_ill) sum_err_spd.y=-abs_limit_route_ill;
    
    static int index_x,index_y;
//x������ַ���
    if(fabs(sum_err_spd.x)>40)
    {
        index_x=0;
    }
    else{
        index_x=1;
    }
//y������ַ���
	if(fabs(sum_err_spd.y)>40)
	{
		index_y=0;
	}
	else{
		index_y=1;
	}
	//printf("kaojin_i_spd_x:%f kaojin_i_spd_y:%f",sum_err_spd.x,sum_err_spd.y);
//	spd.x+=index_x*sum_err_spd.x;
//	spd.y+=index_y*sum_err_spd.y;
    spd.x = KP_route * (target_point.x - now_point.x)+index_x*sum_err_spd.x;
	spd.y = KP_route * (target_point.y - now_point.y)+index_y*sum_err_spd.y;
    /* get_length 接受 Point_struct，因此做简单转换 */
    Point_struct tp = { target_point.x, target_point.y };
    Point_struct np = { now_point.x, now_point.y };
    double dis=get_length(tp,np);
	static vec2 spd_direction;
	spd_direction.x=target_point.x - now_point.x;
	spd_direction.y=target_point.y - now_point.y;
	spd_direction.x/=dis;
	spd_direction.y/=dis;
    //printf("dx:%.1f dy:%.1f\n",spd_direction.x,spd_direction.y);
    
    double abs_spd=(sqrt(pow(spd.x, 2)+pow(spd.y, 2)));
    if(abs_spd>abs_limit_route_vel) 
    {
        //printf("x:%.1f y:%.1f\n",spd.x,spd.y);
        
        spd.x=abs_limit_route_vel*spd_direction.x;
        spd.y=abs_limit_route_vel*spd_direction.y;
    }
	if(spd_direction.x<min&&spd_direction.x>-min)
	{
		spd.x=0;
	}
	
	if(spd_direction.y<min&&spd_direction.y>-min)
	{
		spd.y=0;
	}
	last_point.x=now_point.x;
	last_point.y=now_point.y;
	
    //USART_printf("x:%.1f y:%.1f\n",spd.x,spd.y);
    return spd;
}

double getAng_Route2Point(float path_pos, float length,float start_ang, float end_ang)
{
    float tpro;
    if( path_pos < 0.1*length)
    {
        tpro = 0;
    }
    else{
        tpro = 2*path_pos / length ; 
    }
    if(tpro > 1)
    {
      tpro = 1;
    }

    return start_ang+(end_ang - start_ang) * tpro;
}



void Speed_Distribute(Point start,Point target,Dis_spd dis_spd)
{
    Point P_now;
    P_now.x=lcResult.x;
    P_now.y=lcResult.y;
    vec2 P_spd_now;
    P_spd_now.x=route_spd_local.x;
    P_spd_now.y=route_spd_local.y;
    double P_vel_now=sqrt(pow(P_spd_now.x, 2)+pow(P_spd_now.y, 2));
    /* get_length 接受 Point_struct，因此先转换 */
    Point_struct ps_now = { P_now.x, P_now.y };
    Point_struct ps_start = { start.x, start.y };
    Point_struct ps_target = { target.x, target.y };
    double route_length=get_length(ps_now, ps_start);//����·������
    double route_length_to_target=get_length(ps_now, ps_target);//����Ŀ����·������
    double route_total_len=get_length(ps_start, ps_target);//��·������
    /* 使用当前速度计算停止距离 s = v^2 / (2a) ，并据此决定是否进入减速段
       这里用 dis_spd.up_stage/down_stage 与 maxspd/addspd/stopspd 来估算加减速度
       并基于能否在剩余距离内停下调整 abs_limit_route_vel。 */

    if (dis_spd.up_stage <= 0) dis_spd.up_stage = 1.0;
    if (dis_spd.down_stage <= 0) dis_spd.down_stage = 1.0;

    /* 估算平均减速度与加速度（优先使用用户提供的 dis_spd.decel/accel） */
    double a_dec;
    double a_acc;
        a_dec = dis_spd.decel;
        a_acc = dis_spd.accel;

    double stopping_distance = (P_vel_now * P_vel_now) / (2.0 * a_dec);

    /* 统一逻辑：无论目标是否发生变化，都先根据停止距离判断是否必须减速；否则按加速/匀速/减速段计算允许速度 */
    if (stopping_distance >= route_length_to_target)
    {
        abs_limit_route_vel = sqrt(fmax(0.0, dis_spd.stopspd * dis_spd.stopspd + 2.0 * a_dec * route_length_to_target));
    }
    else
    {
        if (route_length <= dis_spd.up_stage)
        {
            abs_limit_route_vel = sqrt(fmin(dis_spd.maxspd * dis_spd.maxspd,
                                            dis_spd.addspd * dis_spd.addspd + 2.0 * a_acc * route_length));
        }
        else if (route_length > dis_spd.up_stage && route_length < route_total_len - dis_spd.down_stage)
        {
            abs_limit_route_vel = dis_spd.maxspd;
        }
        else if (route_length >= route_total_len - dis_spd.down_stage && route_length < route_total_len)
        {
            double remain = route_total_len - route_length;
            abs_limit_route_vel = sqrt(fmax(0.0, dis_spd.stopspd * dis_spd.stopspd + 2.0 * a_dec * remain));
        }
        else
        {
            abs_limit_route_vel = dis_spd.stopspd;
        }
    }
}

int Route2Point(Point start_point,Point target_point,Dis_spd dis_spd,double target_angle)
{
    Point now_point;
    now_point.x=lcResult.x;
    now_point.y=lcResult.y;
    
    static Point first_point;
    static int flag=0;
    if(flag==0)
    {
        first_point=target_point;
        flag++;
    }
    else{
    }
    
    if(target_point.x!=first_point.x||target_point.y!=first_point.y)
    {
        flag_if_change_target_g=1;
    }
        
    Point_struct ps_now2 = { now_point.x, now_point.y };
    Point_struct ps_startp = { start_point.x, start_point.y };
    Point_struct ps_targetp = { target_point.x, target_point.y };
    double route_length=get_length(ps_now2, ps_startp);//����·������
    double route_total_len=get_length(ps_startp, ps_targetp);//��·������
    double distance=get_length(ps_now2, ps_targetp);
    vec2 route_spd=getSpd_Route2Point(start_point,target_point,dis_spd);
    double target_angle_route = getAng_Route2Point(route_length,route_total_len,lcResult.r,target_angle);
    double route_spd_angle=pid_path_angle(target_angle_route,lcResult.r);
    if(distance<10&&(target_angle-lcResult.r)<0.05&&(target_angle-lcResult.r)>-0.05&&lcResult.vx<50&&lcResult.vx>-50&&lcResult.vy<50&&lcResult.vy>-50) {
        flag_if_route_finish_g=1;
     printf("ROUTE FINISH!\n");
        return 1;
        }
    else 
    {
        
        flag_if_route_finish_g=0;
        route_spd_local=change_world_to_local(route_spd,lcResult.r);
        printf("%lf",lcResult.r)  ;
        cha_remote(route_spd_local.x,route_spd_local.y,route_spd_angle);
        //USART_printf("rx:%.1f ry:%.1f ra:%.1f\n",route_spd.x,route_spd.y,route_spd_angle);
    }
    return 0;
}