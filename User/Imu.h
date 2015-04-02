#ifndef _IMU_H_
#define _IMU_H_
#include "stm32f4xx.h"
#define RtA 	57.324841				//���ȵ��Ƕ�
#define AtR    	0.0174533				//�ȵ��Ƕ�
#define Acc_G 	0.0011963				//���ٶȱ��G
#define Gyro_G 	0.0076294//0.0152672//0.0609756				//���ٶȱ�ɶ�250��0.0152672//��
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

extern S_INT16_XYZ ACC_AVG;			//ƽ��ֵ�˲����ACC
extern S_INT16_XYZ GYRO_AVG;
extern S_FLOAT_XYZ GYRO_I;				//�����ǻ���
extern S_FLOAT_XYZ EXP_ANGLE;		//�����Ƕ�
extern S_FLOAT_XYZ DIF_ANGLE;		//�����Ƕ���ʵ�ʽǶȲ�
extern S_FLOAT_XYZ Q_ANGLE;			//��Ԫ��������ĽǶ�
extern S_FLOAT_XYZ OFFSET;		

void Prepare_Data(void);
void Get_Attitude(void);

#endif
