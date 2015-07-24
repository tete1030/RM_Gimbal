#include "stm32f4xx.h"
#include "myiic.h"
#include "mpu6050.h"
#include "eMPL/inv_mpu.h"
#include "eMPL/inv_mpu_dmp_motion_driver.h"
#include "delay.h"
#include "stm32f4xx_it.h"
#include "gimbal_motor.h"
#include <math.h>

//q30格式,long转float时的除数.
#define q30  1073741824.0f

//陀螺仪方向设置

static signed char gyro_orientation[9] = { -1, 0, 0,
										   0, -1, 0,
										   0, 0, 1};


// 将X和Y颠倒, 在解析Pitch和Roll时再颠倒回来, 以便使Pitch能够支持在-180~180度
//static signed char gyro_orientation[9] = { 0, -1, 0,
//                                           -1, 0, 0,
//                                           0, 0, 1};

uint8_t MPU_Init(void)
{
    GPIO_InitTypeDef    gpio;
    NVIC_InitTypeDef    nvic;
    EXTI_InitTypeDef    exti;
	int res=0;

	IIC_Init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,  ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &gpio);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,GPIO_PinSource5);

    exti.EXTI_Line = EXTI_Line5;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿中断
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = ITP_MPU_EXTI9_5_PREEMPTION;
    nvic.NVIC_IRQChannelSubPriority = ITP_MPU_EXTI9_5_SUB;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

	if(mpu_init()==0)	//初始化MPU6050
	{
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置所需要的传感器
		if(res)return 1;
		res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);//设置FIFO
		if(res)return 2;
		res=mpu_set_sample_rate(DEFAULT_MPU_HZ);	//设置采样率
		if(res)return 3;
		res=dmp_load_motion_driver_firmware();		//加载dmp固件
		if(res)return 4;
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//设置陀螺仪方向
		if(res)return 5;
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	//设置dmp功能
							   DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
							   DMP_FEATURE_GYRO_CAL);
		if(res)return 6;
		res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);	//设置DMP输出速率(最大不超过200Hz)
		if(res)return 7;
        res=mpu_set_int_level(1);
        if(res)return 8;
		res=dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
		if(res)return 9;
		res=run_self_test();		//自检
		if(res)return 10;
		res=mpu_set_dmp_state(1);	//使能DMP
		if(res)return 11;
	}
	else
		return 12;
	return 0;
}

int16_t MPU_DMP_Get_Remaining_Data_Count()
{
	return dmp_get_fifo_packet_count();
}

//得到dmp处理后的数据(注意,本函数需要比较多堆栈,局部变量有点多)
//pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
//roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
//yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
//返回值:0,正常
//    其他,失败
uint8_t MPU_DMP_Get_Data(DMP_Data *dd)
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short sensors;
	unsigned char more;
	long quat[4];
	if(dmp_read_fifo(dd->gyro, dd->accel, quat, &sensor_timestamp, &sensors,&more))return 1;
	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	/*if (sensors & INV_XYZ_GYRO )
	send_packet(PACKET_TYPE_GYRO, gyro);
	if (sensors & INV_XYZ_ACCEL)
	send_packet(PACKET_TYPE_ACCEL, accel); */
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization.
	**/
	if(sensors&INV_WXYZ_QUAT)
	{
		q0 = quat[0] / q30;	//q30格式转换为浮点数
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30;
		//计算得到俯仰角/横滚角/航向角

        dd->yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
		dd->pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
		dd->roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;

        // 在方向定义处将X和Y颠倒, 在解析Pitch和Roll时再颠倒回来, 以便使Pitch能够支持在-180~180度
        //dd->yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
        //dd->pitch  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
        //dd->roll = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;

	}else return 2;
	return 0;
}


int16_t MPU_Get_Temperature()
{
    long temp;
    if(mpu_get_temperature(&temp, NULL) == 0)
        return (int16_t) temp;
    else
        return -1;

}

//MPU6050 外部中断处理函数
void EXTI9_5_IRQHandler(void) {
	uint8_t ret;
    DMP_Data dmp_data;

    if (EXTI_GetITStatus(EXTI_Line5) == SET) {
        EXTI_ClearITPendingBit(EXTI_Line5);

        while ((ret = MPU_DMP_Get_Data(&dmp_data)) == 0 && MPU_DMP_Get_Remaining_Data_Count() > 0);
        if(ret == 0)
		{
            GM_Set_DMP_Data(&dmp_data);
		}

    }
}
