#ifndef _IMU_H_
#define _IMU_H_
#include "stm32f4xx.h"
#define RtA 	57.324841				//弧度到角度
#define AtR    	0.0174533				//度到角度
#define Acc_G 	0.0011963				//加速度变成G
#define Gyro_G 	0.0076294//0.0152672//0.0609756				//角速度变成度250：0.0152672//改
#define Gyro_Gr	0.0001332//0.0002663		
#define FILTER_NUM 20
typedef struct{
				float X;
				float Y;
				float Z;}S_FLOAT_XYZ;
typedef struct{
				int16_t X;
				int16_t Y;
				int16_t Z;}S_INT16_XYZ;

extern S_INT16_XYZ ACC_AVG;			//平均值滤波后的ACC
extern S_INT16_XYZ GYRO_AVG;
extern S_FLOAT_XYZ GYRO_I;				//陀螺仪积分
extern S_FLOAT_XYZ EXP_ANGLE;		//期望角度
extern S_FLOAT_XYZ DIF_ANGLE;		//期望角度与实际角度差
extern S_FLOAT_XYZ Q_ANGLE;			//四元数计算出的角度
extern S_FLOAT_XYZ OFFSET;		

void Prepare_Data(void);
void Get_Attitude(void);

#endif
