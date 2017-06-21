/******************** (C) COPYRIGHT 2016 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：anotc_baro_ctrl.c
 * 描述    ：气压计控制
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "anotc_baro_ctrl.h"
#include "ms5611.h"
#include "filter.h"
#include "fly_mode.h"

void fusion_prepare(float dT,float av_arr[],u16 av_num,u16 *av_cnt,float deadzone,_height_st *data,_fusion_p_st *pre_data)
{
	pre_data->dis_deadzone = my_deathzoom(data->relative_height,pre_data->dis_deadzone,deadzone);	
	Moving_Average(av_arr,av_num ,(av_cnt),(10 *pre_data->dis_deadzone ),&(pre_data->displacement)); //厘米->毫米
	
//	Moving_Average(av_arr,av_num ,(av_cnt),(10 *data->relative_height),&(pre_data->dis_deadzone)); //厘米->毫米	
//	pre_data->displacement = my_deathzoom(pre_data->dis_deadzone,pre_data->displacement,10 *deadzone);
	pre_data->speed = safe_div(pre_data->displacement - pre_data->displacement_old,dT,0);
	pre_data->acceleration = safe_div(pre_data->speed - pre_data->speed_old,dT,0);
	
	pre_data->displacement_old = pre_data->displacement;
	pre_data->speed_old = pre_data->speed;
}

void acc_fusion(float dT,_f_set_st *set,float est_acc,_fusion_p_st *pre_data,_fusion_st *fusion)
{
	fusion->fusion_acceleration.out += est_acc - fusion->est_acc_old; //估计
	anotc_filter_1(set->b1,set->g1,dT,pre_data->acceleration,&(fusion->fusion_acceleration));  //pre_data->acceleration //观测、最优
	
	fusion->fusion_speed_m.out += 1.1f *my_deathzoom(fusion->fusion_acceleration.out,0,20) *dT;
	anotc_filter_1(set->b2,set->g2,dT,pre_data->speed,&(fusion->fusion_speed_m));
	anotc_filter_1(set->b2,set->g2,dT,(-pre_data->speed + fusion->fusion_speed_m.out),&(fusion->fusion_speed_me));
	fusion->fusion_speed_me.out = LIMIT(fusion->fusion_speed_me.out,-200,200);
	fusion->fusion_speed_m.a = LIMIT(fusion->fusion_speed_m.a,-1000,1000);
	
	fusion->fusion_displacement.out += 1.05f *(fusion->fusion_speed_m.out - fusion->fusion_speed_me.out) *dT;
	anotc_filter_1(set->b3,set->g3,dT,pre_data->displacement,&(fusion->fusion_displacement));
	
	fusion->est_acc_old = est_acc;
}

//超声波融合参数

#define SONAR_AV_NUM 50
float sonar_av_arr[SONAR_AV_NUM];
u16 sonar_av_cnt;

_fusion_p_st sonar;
_fusion_st sonar_fusion;
_f_set_st sonar_f_set = {
													0.2f,
													0.5f,
													0.8f,
													
													0.2f,
													0.5f,
													0.8f	
												};


			
void baro_ctrl(float dT,_hc_value_st *height_value)
{

			fusion_prepare(dT,sonar_av_arr,SONAR_AV_NUM,&sonar_av_cnt,0,&ultra,&sonar);
			acc_fusion(dT,&sonar_f_set,acc_3d_hg.z,&sonar,&sonar_fusion);
			
			//fusion_prepare(dT,baro_av_arr,BARO_AV_NUM,&baro_av_cnt,2,&baro,&baro_p);	//commentted by Mr.Lin
			//acc_fusion(dT,&baro_f_set,acc_3d_hg.z,&baro_p,&baro_fusion);
	
			float m_speed,f_speed;
			m_speed = sonar.speed;
			f_speed = sonar_fusion.fusion_speed_m.out - sonar_fusion.fusion_speed_me.out;
			
			height_value->m_acc = acc_3d_hg.z;
			height_value->m_speed = m_speed;  
			height_value->m_height = sonar.displacement;
			height_value->fusion_acc = sonar_fusion.fusion_acceleration.out;
			height_value->fusion_speed = my_deathzoom(LIMIT( (f_speed),-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP),height_value->fusion_speed,10);
			height_value->fusion_height = sonar_fusion.fusion_displacement.out; 
	
}



/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/
