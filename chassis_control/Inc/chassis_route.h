#ifndef CHASSIS_ROUTE_H
#define CHASSIS_ROUTE_H

#include "chassis_path.h"

typedef struct Distri_Speed
{
	double up_stage;       //���ٽ׶�
	double down_stage;     //���ٽ׶�
	double maxspd;        //����ٶȣ�Ҳ���ٶ�����
    double addspd;
	double stopspd;         //ֹͣǰ�����
    double accel;          // 用户可配置的加速度 (units consistent with speed and distance)
    double decel;          // 用户可配置的减速度
}Dis_spd;

typedef struct 
{
	float x;
	float y;
}Point;


// extern float sumerr_ang_spd;
// extern vec2 sum_err_spd;
// extern vec2 sum_err_spd_kj;
// extern int flag_if_route_finish_g;
// extern int flag_if_change_target_g;

vec2 getSpd_Route2Point(Point start_point,Point target_point,Dis_spd dis_spd);
double getAng_Route2Point(float path_pos, float length,float start_ang, float end_ang);
int Route2Point(Point start_point,Point target_point,Dis_spd dis_spd,double target_angle);
void Speed_Distribute(Point start,Point target,Dis_spd dis_spd);

#endif