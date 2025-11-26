#include "chassis_route.h"
#include "chasctrl.h"
#include "main.h"
#include "speed_compose.h"
#include "chassis_path.h"
#include "debug.h"
#include "math.h"
#include "map.h"
#include "bsp_can.h"

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

vec2 getSpd_Route2Point(Point start_point,Point target_point,Dis_spd dis_spd)
{
	static vec2 last_point;
	static vec2 spd;
    double min=0.0001;
    Point now_point=getlocator_point(lcResult);
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
//x方向积分分离
	if(fabs(sum_err_spd.x)>40)
	{
		index_x=0;
	}
	else{
		index_x=1;
	}
//y方向积分分离
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
    double dis=get_length(target_point,now_point);
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
    float vel_x,vel_y,vel_r;
    P_now.x=lcResult.x;
    P_now.y=lcResult.y;
    vec2 P_spd_now;
    P_spd_now.x=route_spd_local.x;
    P_spd_now.y=route_spd_local.y;
    double P_vel_now=sqrt(pow(P_spd_now.x, 2)+pow(P_spd_now.y, 2));
    double route_length=get_length(P_now,start);//已走路径长度
    double route_length_to_target=get_length(P_now,target);//距离目标点的路径长度
    double route_total_len=get_length(start,target);//总路径长度
    //速度分配示意图：
    //   ^abs_lim_route_vel
    //   |
    //   |  ------------------
    //   | /                  \
    //   |/                    \
    //   |                      \
    //   |                        ------------
    //   |                                         
    //    --------------------------------------->route_length
    //代码实现如下：
    if(flag_if_change_target_g==0)
    {
        if(route_length<=dis_spd.up_stage)//加速阶段
        {
            abs_limit_route_vel=(dis_spd.maxspd-dis_spd.addspd)/dis_spd.up_stage*route_length+dis_spd.addspd;
        }
        else if(route_length > dis_spd.up_stage && route_length < route_total_len-dis_spd.down_stage)//匀速阶段
        {
            abs_limit_route_vel=dis_spd.maxspd;
        }
        else if(route_length > route_total_len-dis_spd.down_stage && route_length<route_total_len)//减速阶段
        {
            abs_limit_route_vel=-((dis_spd.maxspd-dis_spd.stopspd)/dis_spd.down_stage)*(dis_spd.down_stage-(route_total_len-route_length))+dis_spd.maxspd;
        }
        else if (route_length>route_total_len)//超调阶段
        {
            abs_limit_route_vel=dis_spd.stopspd;
        }
        else{
        }
    }
    else
    {
        if(route_length_to_target>dis_spd.down_stage)
        {
            abs_limit_route_vel=dis_spd.maxspd;
        }
        else
        {
            abs_limit_route_vel=((dis_spd.maxspd-dis_spd.stopspd)/(dis_spd.down_stage))*route_length_to_target+dis_spd.stopspd;
        }
    }
}

int Route2Point(Point start_point,Point target_point,Dis_spd dis_spd,double target_angle)
{
    double now_angle = lcResult.r;
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
        
    double route_length=get_length(now_point,start_point);//已走路径长度
    double route_total_len=get_length(start_point,target_point);//总路径长度
    double distance=get_length(now_point,target_point);
    vec2 route_spd=getSpd_Route2Point(start_point,target_point,dis_spd);
    double target_angle_route = getAng_Route2Point(route_length,route_total_len,lcResult.r,target_angle);
    double route_spd_angle=pid_path_angle(target_angle_route,lcResult.r);
    if(distance<10&&(target_angle-lcResult.r)<0.05&&(target_angle-lcResult.r)>-0.05&&lcResult.vx<50&&lcResult.vx>-50&&lcResult.vy<50&&lcResult.vy>-50) {
        flag_if_route_finish_g=1;
     USART_printf("ROUTE FINISH!\n");
        return 1;
        }
    else 
    {
        
        flag_if_route_finish_g=0;
        route_spd_local=change_world_to_local(route_spd,lcResult.r);
        USART_printf("%lf",lcResult.r)  ;
        cha_remote(route_spd_local.x,route_spd_local.y,route_spd_angle);
        //USART_printf("rx:%.1f ry:%.1f ra:%.1f\n",route_spd.x,route_spd.y,route_spd_angle);
    }
    return 0;
}