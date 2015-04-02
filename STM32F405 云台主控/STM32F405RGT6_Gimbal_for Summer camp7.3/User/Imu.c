#include "imu.h"
#include "math.h"
#include "mpu6050_driver.h"
#define RtA 	57.324841				//弧度到角度
#define AtR    	0.0174533				//度到角度
#define Acc_G 	0.0011963				//加速度变成G
#define Gyro_G 	0.0076294//0.0152672//0.0609756				//角速度变成度250：0.0152672//改
#define Gyro_Gr	0.0001332//0.0002663		
#define FILTER_NUM 20

S_INT16_XYZ ACC_AVG;			//平均值滤波后的ACC
S_INT16_XYZ GYRO_AVG;
S_FLOAT_XYZ GYRO_I;				//陀螺仪积分
S_FLOAT_XYZ EXP_ANGLE;		//期望角度
S_FLOAT_XYZ DIF_ANGLE;		//期望角度与实际角度差
S_FLOAT_XYZ Q_ANGLE;			//四元数计算出的角度
S_FLOAT_XYZ OFFSET;		

int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];	

void Prepare_Data(void)
{
	static uint8_t filter_cnt=0;
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;
  MPU6050_ReadData();
	ACC_X_BUF[filter_cnt] = MPU6050_ACC_LAST.X;
	ACC_Y_BUF[filter_cnt] = MPU6050_ACC_LAST.Y;
	ACC_Z_BUF[filter_cnt] = MPU6050_ACC_LAST.Z;
	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	ACC_AVG.X = temp1 / FILTER_NUM;
	ACC_AVG.Y = temp2 / FILTER_NUM;
	ACC_AVG.Z = temp3 / FILTER_NUM;
	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;
	
	GYRO_I.Z += MPU6050_GYRO_LAST.Z*Gyro_G*0.002;
}
////////////////////////////////////////////////////////////////////////////////
#define Kp 3.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.002f                          // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.001f                   // half the sample period???????

float qq0 = 1, qq1 = 0, qq2 = 0, qq3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float norm;
//  float hx, hy, hz, bx, bz;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;

  // 先把这些用得到的值算好
  float q0q0 = qq0*qq0;
  float q0q1 = qq0*qq1;
  float q0q2 = qq0*qq2;
//  float q0q3 = qq0*qq3;
  float q1q1 = qq1*qq1;
//  float q1q2 = qq1*qq2;
  float q1q3 = qq1*qq3;
  float q2q2 = qq2*qq2;
  float q2q3 = qq2*qq3;
  float q3q3 = qq3*qq3;
	
	if(ax*ay*az==0)
 		return;
		
  norm = sqrt(ax*ax + ay*ay + az*az);       //acc数据归一化
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;

  // estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
  vx = 2*(q1q3 - q0q2);												//四元素中xyz的表示
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           					 //向量外积在相减得到差分就是误差
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;								  //对误差进行积分
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;					   							//将误差PI后补偿到陀螺仪，即补偿零点漂移
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

  // integrate quaternion rate and normalise						   //四元素的微分方程
  qq0 = qq0 + (-qq1*gx - qq2*gy - qq3*gz)*halfT;
  qq1 = qq1 + (qq0*gx + qq2*gz - qq3*gy)*halfT;
  qq2 = qq2 + (qq0*gy - qq1*gz + qq3*gx)*halfT;
  qq3 = qq3 + (qq0*gz + qq1*gy - qq2*gx)*halfT;

  // normalise quaternion
  norm = sqrt(qq0*qq0 + qq1*qq1 + qq2*qq2 + qq3*qq3);
  qq0 = qq0 / norm;
  qq1 = qq1 / norm;
  qq2 = qq2 / norm;
  qq3 = qq3 / norm;

//  Q_ANGLE.Z = atan2(2 * qq1 * qq2 + 2 * qq0 * qq3, -2 * qq2*qq2 - 2 * qq3* qq3 + 1)* 57.3; // yaw
  Q_ANGLE.Y=asin(-2 * qq1 * qq3 + 2 * qq0* qq2)* 57.3; // roll
  Q_ANGLE.X=atan2(2 * qq2 * qq3 + 2 * qq0 * qq1, -2 * qq1 * qq1 - 2 * qq2* qq2 + 1)* 57.3; // pitch
}



void Get_Attitude(void)
{
	IMUupdate(MPU6050_GYRO_LAST.X*Gyro_Gr,
						MPU6050_GYRO_LAST.Y*Gyro_Gr,
						MPU6050_GYRO_LAST.Z*Gyro_Gr,
						ACC_AVG.X,ACC_AVG.Y,ACC_AVG.Z);	//*0.0174????
}
