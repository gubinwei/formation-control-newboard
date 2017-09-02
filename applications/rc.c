/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：rc.c
 * 描述    ：遥控器通道数据处理
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "include.h"
#include "rc.h"
#include "mymath.h"
#include "mpu6050.h"
#include "filter.h"
#include "ak8975.h"
#include "anotc_baro_ctrl.h"

u16 global_mode_NS;// 发送到地面站显示NS状态，1555代表遥控器控制，1999代表自主控制，1111代表地面站控制

s8 CH_in_Mapping[CH_NUM] = {0,1,2,3,4,5,6,7};    //通道映射
u8 rc_lose = 0;

void CH_Mapping_Fun(u16 *in,u16 *Mapped_CH)
{
	u8 i;
	for( i = 0 ; i < CH_NUM ; i++ )
	{
		*( Mapped_CH + i ) = *( in + CH_in_Mapping[i] );
	}
}

s16 CH[CH_NUM];

float CH_Old[CH_NUM];
float CH_filter[CH_NUM];
float CH_filter_Old[CH_NUM];
float CH_filter_D[CH_NUM];
u8 NS=1,CH_Error[CH_NUM];
u16 NS_cnt,CLR_CH_Error[CH_NUM];
 
s16 MAX_CH[CH_NUM]  = {1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 };	//摇杆最大
s16 MIN_CH[CH_NUM]  = {1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 };	//摇杆最小
char CH_DIR[CH_NUM] = {0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    };  //摇杆方向
#define CH_OFFSET 500


float filter_A;

void RC_Duty( float T , u16 tmp16_CH[CH_NUM] )
{
	u8 i;
	s16 CH_TMP[CH_NUM];
	static u16 Mapped_CH[CH_NUM];

	if( NS == 1 )	//Remote Control
	{
		CH_Mapping_Fun(tmp16_CH,Mapped_CH);
		global_mode_NS= 1555 ;
	}
	else if( NS == 2 )	//Ground Station
	{
		CH_Mapping_Fun(RX_CH,Mapped_CH);
		global_mode_NS = 1111;
	}else if(NS==3){
	*( Mapped_CH + 0 ) = *( RX_CH + CH_in_Mapping[0] );
	*( Mapped_CH + 1 ) = *( RX_CH + CH_in_Mapping[1] );
	*( Mapped_CH + 2 ) = *( tmp16_CH + CH_in_Mapping[2] );
	*( Mapped_CH + 3 ) = *( RX_CH + CH_in_Mapping[3] );
		
		global_mode_NS = 1999;
	}

	
	for( i = 0;i < CH_NUM ; i++ )
	{
		if( (u16)Mapped_CH[i] > 2500 || (u16)Mapped_CH[i] < 500 )
		{
			CH_Error[i]=1;
			CLR_CH_Error[i] = 0;
		}
		else
		{
			CLR_CH_Error[i]++;
			if( CLR_CH_Error[i] > 200 )
			{
				CLR_CH_Error[i] = 2000;
				CH_Error[i] = 0;
			}
		}

		if( NS == 1 || NS == 2 || NS==3)
		{
			if( CH_Error[i] ) //单通道数据错误
			{
				
			}
			else
			{
				//CH_Max_Min_Record();
				CH_TMP[i] = ( Mapped_CH[i] ); //映射拷贝数据，大约 1000~2000
				
				if( MAX_CH[i] > MIN_CH[i] )
				{
					if( !CH_DIR[i] )
					{
						CH[i] =   LIMIT ( (s16)( ( CH_TMP[i] - MIN_CH[i] )/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - CH_OFFSET ), -500, 500); //归一化，输出+-500
					}
					else
					{
						CH[i] = - LIMIT ( (s16)( ( CH_TMP[i] - MIN_CH[i] )/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - CH_OFFSET ), -500, 500); //归一化，输出+-500
					}
				}	
				else
				{
					fly_ready = 0;
				}
			}
			rc_lose = 0;
		}	
		else //未接接收机或无信号（遥控关闭或丢失信号）
		{
			rc_lose = 1;
		}
//=================== filter ===================================
//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
//=================== filter =================================== 		
			
			filter_A = 6.28f *10 *T;
			
			if( ABS(CH_TMP[i] - CH_filter[i]) <100 )
			{
				CH_filter[i] += filter_A *(CH[i] - CH_filter[i]) ;
			}
			else
			{
				CH_filter[i] += 0.5f *filter_A *( CH[i] - CH_filter[i]) ;
			}
			
			if(NS == 0) //无信号
			{
				if(!fly_ready)
				{
					CH_filter[THR] = -500;
				}
			}
			// CH_filter[i] = Fli_Tmp;
			CH_filter_D[i] 	= ( CH_filter[i] - CH_filter_Old[i] );
			CH_filter_Old[i] = CH_filter[i];
			CH_Old[i] 		= CH[i];
	}
	//======================================================================
	Fly_Ready(T,tmp16_CH[3]);		//解锁判断
	//======================================================================
	if(++NS_cnt>200)  // 400ms  未插信号线。
	{
		NS_cnt = 0;
		NS = 0;
	}
}

u8 fly_ready = 0,thr_stick_low;
s16 ready_cnt=0;

//s16 mag_cali_cnt;
//s16 locked_cnt;
extern u8 acc_ng_cali;
void Fly_Ready(float T,float rc_yaw)
{
	if( CH_filter[2] < -400 )  							//油门小于10%
	{
		//thr_stick_low = 1;
		if( fly_ready && ready_cnt != -1 ) //解锁完成，且已退出解锁上锁过程
		{
			//ready_cnt += 1000 *T;
		}
#if(USE_TOE_IN_UNLOCK)		
		if( CH_filter[3] < -400 )							
		{
			if( CH_filter[1] > 400 )
			{
				if( CH_filter[0] > 400 )
				{
					if( ready_cnt != -1 )				   //外八满足且退出解锁上锁过程
					{
						ready_cnt += 3 *1000 *T;
					}
				}

			}

		}
#else
		if( CH_filter[3] < -400 ||(NS==3&&rc_yaw < 1100))					      //左下满足 ,NS等于3时认可通过左下上锁和右下解锁			
		{
			if( ready_cnt != -1 && fly_ready )	//判断已经退出解锁上锁过程且已经解锁
			{
				ready_cnt += 10000 *T;
				if( ready_cnt > 400 ) // 600ms 
				{	
					fly_ready=0;
					ready_cnt=-1;
				}
			}
			//add by ycnalin 2016 3.5
			//作用：上锁时可以通过roll和pitch来调节NS值，坐下满足并且右手边满足右上时NS=3，右手边满足左上时NS=1；
			if(!fly_ready)
			{
					if(Rc_Pwm_In[0]>1900&&Rc_Pwm_In[1]>1900)
						NS=3;
					else if(Rc_Pwm_In[1]>1900 && Rc_Pwm_In[0]<1100)
						NS=1;
			}

		}
		else if( CH_filter[3] > 400 ||(NS==3&& rc_yaw >1900) )      			//右下满足
		{
			if( ready_cnt != -1 && !fly_ready )	//判断已经退出解锁上锁过程且已经上锁
			{
				ready_cnt += 1000 *T;
				if( ready_cnt > 1500 ) // 600ms 
				{	
					fly_ready=1;
					ready_cnt=-1;
					//acc_ng_cali = mpu6050.Gyro_CALIBRATE = 2;
					mpu6050.Acc_CALIBRATE = 1;		
					mpu6050.Gyro_CALIBRATE = 1;
				//	Mag_CALIBRATED = 1;
				}
			}
		}
#endif		
		else if( ready_cnt == -1 )						//4通道(CH[3])回位
		{
			ready_cnt=0;
		}
	}
	else
	{
		ready_cnt=0;
		//thr_stick_low = 0;
	}
}

void Feed_Rc_Dog(u8 ch_mode) //400ms内必须调用一次
{
	if (NS==3){NS_cnt = 0; return;}
	NS = ch_mode;
	NS_cnt = 0;
}

//=================== filter ===================================
//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
//=================== filter =================================== 	
//u8 height_ctrl_mode = 0;

//extern u8 ultra_ok;
//void Mode()
//{
////	if( !fly_ready || CH_filter[THR]<-400 ) //只在上锁时 以及 油门 低于10% 的时候，允许切换模式，否则只能向模式0切换。
////	{
//		if( CH_filter[AUX1] < -200 )
//		{
//			height_ctrl_mode = 0;
//		}
//		else if( CH_filter[AUX1] < 200 )
//		{
//			height_ctrl_mode = 1;
//		}
//		else
//		{
//			if(ultra_ok == 1)
//			{
//				height_ctrl_mode = 2;
//			}
//			else
//			{
//				height_ctrl_mode = 1;
//			}
//		}
////	}
////	else
////	{
////		if( CH_filter[AUX1] < -200 )
////		{
////			height_ctrl_mode = 0;
////		}
////	}
//}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

