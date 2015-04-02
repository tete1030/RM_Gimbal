#include "main.h"
#include "imu.h"
//����MPU6050�ڲ���ַ
#define	SMPLRT_DIV		0x19	//�����ǲ����� ����ֵ 0X07 125Hz
#define	CONFIG			0x1A	//��ͨ�˲�Ƶ�� ����ֵ 0x00 
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ                 ����ֵ 0x18 ���Լ� 2000deg/s
#define	ACCEL_CONFIG	0x1C	//���ٶȼ��Լ켰������Χ����ͨ�˲�Ƶ�� ����ֵ 0x01 ���Լ� 2G 5Hz

#define INT_PIN_CFG     0x37
#define INT_ENABLE      0x38
#define INT_STATUS      0x3A    //ֻ��


#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C

#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E

#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	

#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46

#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B	//��Դ���� ����ֵ 0x00 ��������
#define	WHO_AM_I		0x75	//ֻ��  Ĭ�϶���Ӧ���� MPU6050_ID = 0x68


#define MPU6050_ID              0x68
#define MPU6050_DEVICE_ADDRESS  0xD0
#define MPU6050_DATA_START      ACCEL_XOUT_H   //�������ݴ�ŵ�ַ�������ģ�����һ������

//MPU6050_RAW_DATA    MPU6050_Raw_Data; 
//MPU6050_REAL_DATA   MPU6050_Real_Data;

S_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET;			//��Ư
S_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//����һ�ζ�ȡֵ

//int gyroADC_X_offset=0,gyroADC_Y_offset=0,gyroADC_Z_offset=0;

//MPU6050 ��ʼ�����ɹ�����0  ʧ�ܷ��� 0xff
int MPU6050_Initialization(void)
{
    unsigned char temp_data = 0x00;

    IIC_GPIO_Init();  //��ʼ��IIC�ӿ�
    HEAT_Configuration();
    
    if(IIC_ReadData(MPU6050_DEVICE_ADDRESS,WHO_AM_I ,&temp_data ,1)==0) //ȷ��IIC�����Ϲҽӵ��Ƿ���MPU6050
    {
        if(temp_data != MPU6050_ID)
        {
            printf("error 1A\r\n");
            return 0xff; //У��ʧ�ܣ�����0xff
        }
    }
    else
    {
        printf("error 1B\r\n");
        return 0xff; //��ȡʧ�� ����0xff
    }
    
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,PWR_MGMT_1,0x01) == 0xff)    //�������״̬
    {
        printf("error 1C\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,SMPLRT_DIV,0x07) == 0xff)//cyq��07 ����Ƶ��1khz
    {
        printf("error 1D\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,CONFIG,0x00) == 0xff)  //5hz LOWPASS
    {
        printf("error 1E\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,GYRO_CONFIG,0x00) == 0xff) 
			                                  //0  �� 250 ��/s	 131	  ������
																				//1  �� 500 ��/s	 65.5
																				//2  �� 1000��/s	 32.8
																				//3  �� 2000��/s	 16.4
    {
        printf("error 1F\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,ACCEL_CONFIG,0x08) == 0xff)
			                                  //0  �� 2g		     16384	���ٶ�
																				//1  �� 4g	       8192
																				//2  �� 8g	       4096
																				//3  �� 16g	     2048
    {
        printf("error 1G\r\n");
        return 0xff;
    }
//    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_PIN_CFG,0x00) == 0xff)
//    {
//        printf("error 1H\r\n");
//        return 0xff;
//    }
//    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x01) == 0xff)
//    {
//        printf("error 1I\r\n");
//        return 0xff;
//    }
    
    //MPU6050_Interrupt_Configuration(); //MPU6050�жϳ�ʼ��
    
    return 0;
}

//MPU6050  ���ݶ�ȡ���ɹ�����0  ʧ�ܷ��� 0xff
int MPU6050_ReadData(void)
{
    u8 buf[14];
    
    if(IIC_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,buf,14) == 0xff)
    {
        printf("error 1J\r\n");
        return 0xff;
    }
    else
    {
        //��ȡ�Ĵ���ԭ������
           
        MPU6050_ACC_LAST.X= (buf[0]<<8 | buf[1]);
        MPU6050_ACC_LAST.Y = (buf[2]<<8 | buf[3]);
        MPU6050_ACC_LAST.Z = (buf[4]<<8 | buf[5]); 
//        MPU6050_Raw_Data.Temp =    (buf[6]<<8 | buf[7]);  
        MPU6050_GYRO_LAST.X= (buf[8]<<8 | buf[9])-GYRO_OFFSET.X;
        MPU6050_GYRO_LAST.Y = (buf[10]<<8 | buf[11])-GYRO_OFFSET.Y;
        MPU6050_GYRO_LAST.Z = (buf[12]<<8 | buf[13])-GYRO_OFFSET.Z;

       
//        //��ԭ������ת��Ϊʵ��ֵ�����㹫ʽ���Ĵ����������й�
//        MPU6050_Real_Data.Accel_X = (float)(MPU6050_Raw_Data.Accel_X)/8192.0; //��datasheet 30 of 47
//        MPU6050_Real_Data.Accel_Y = (float)(MPU6050_Raw_Data.Accel_Y)/8192.0; //��datasheet 30 of 47
//        MPU6050_Real_Data.Accel_Z = (float)(MPU6050_Raw_Data.Accel_Z)/8192.0; //��datasheet 30 of 47
//        MPU6050_Real_Data.Temp =   (float)(MPU6050_Raw_Data.Temp)/340.0+36.53;//��datasheet 31 of 47
//        MPU6050_Real_Data.Gyro_X = (float)(MPU6050_Raw_Data.Gyro_X - gyroADC_X_offset)/65.5;     //��datasheet 32 of 47
//        MPU6050_Real_Data.Gyro_Y = (float)(MPU6050_Raw_Data.Gyro_Y - gyroADC_Y_offset)/65.5;     //��datasheet 32 of 47
//        MPU6050_Real_Data.Gyro_Z = (float)(MPU6050_Raw_Data.Gyro_Z - gyroADC_Z_offset)/65.5;     //��datasheet 32 of 47
    } 
    
    return 0;
}


void MPU6050_Gyro_Offset(void)
{
	u16 i;
	float x_temp=0,y_temp=0,z_temp=0;
	
	for(i=0; i<1000; i++)
	{
		MPU6050_ReadData();
		delay_ms(1);
		x_temp=x_temp+MPU6050_GYRO_LAST.X;
		y_temp=y_temp+MPU6050_GYRO_LAST.X;
		z_temp=z_temp+MPU6050_GYRO_LAST.X;
	}			
	
	x_temp=x_temp/1000.00;
	y_temp=y_temp/1000.00;	
	z_temp=z_temp/1000.00;

	GYRO_OFFSET.X=(int)x_temp;
	GYRO_OFFSET.X=(int)y_temp;
	GYRO_OFFSET.X=(int)z_temp;
	
}
