#ifndef __MPU6050_H
#define __MPU6050_H
#include "myiic.h"


typedef struct DMP_Data
{
    float pitch;
    float roll;
    float yaw;
    int16_t gyro[3];
    int16_t accel[3];
} DMP_Data;

uint8_t MPU_Init(void);
int16_t MPU_DMP_Get_Remaining_Data_Count();
int16_t MPU_Get_Temperature();
uint8_t MPU_DMP_Get_Data(DMP_Data *dd);



#endif




































