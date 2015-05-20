//
// Created by Texot Qexoq on 5/17/15.
//
#include <stdio.h>
#include <stm32f4-stdperiph/stm32f4xx_can.h>
#include "stm32f4xx.h"
#include "can2.h"
#include "maincontrol.h"
#include "can_packet.h"
#include "gimbal_motor.h"

#define CAN_CENTER_ADDR 0x0

#pragma pack(push,1)
struct Gimbal_Maincontrol_Packet_Others
{
    int8_t friction_state;
    int8_t friction_ready_state;
    int8_t shooter_state;
    uint16_t pitch_angle;
    uint16_t yaw_angle;
};

struct Gimbal_Maincontrol_Packet_Accel
{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct Gimbal_Maincontrol_Packet_Gyro
{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct Gimbal_Maincontrol_Packet_Yaw_Pitch
{
    float yaw;
    float pitch;
};

struct Gimbal_Maincontrol_Packet_Roll
{
    float roll;
};

struct Maincontrol_Gimbal_Packet_Set_Speed
{
    float yaw;
    float pitch;
};

struct Maincontrol_Gimbal_Packet_Set_Friction
{
    uint8_t enable;
};

struct Maincontrol_Gimbal_Packet_Set_Shooter
{
    uint8_t enable;
};

struct Maincontrol_Gimbal_Packet_Set_Laser
{
    uint8_t enable;
};

struct Maincontrol_Gimbal_Packet_Set_Yaw_Pitch
{
    float yaw;
    float pitch;
};

#pragma pack(pop)


void Maincontrol_Configuration(void)
{
    extern void Maincontrol_Can_Receive_Handler(CanRxMsg* rx_msg);
    extern void Maincontrol_Can_Send_Handler(uint16_t id, int8_t code);
    CAN2_Configuration(Maincontrol_Can_Send_Handler, Maincontrol_Can_Receive_Handler,
                       CAN_PACKET_DESTTYPE_GIMBAL | CAN_PACKET_PLACETYPE_SPECIFIC | CAN_CENTER_ADDR,
                       CAN_PACKET_TYPE_MASK_DESTTYPE | CAN_PACKET_TYPE_MASK_PLACETYPE | CAN_PACKET_TYPE_MASK_SOURCEADDR_STRICT,
                       0,
                       CAN_PACKET_TYPE_MASK_DESTTYPE | CAN_PACKET_TYPE_MASK_PLACETYPE | CAN_PACKET_TYPE_MASK_SOURCEADDR_STRICT
    );
}

void Maincontrol_Can_Receive_Handler(CanRxMsg* rx_msg)
{
    uint16_t type,id;

    type = (uint16_t) (rx_msg->StdId & CAN_PACKET_TYPE_MASK_DATATYPE);
    id = (uint16_t) (rx_msg->StdId & CAN_PACKET_TYPE_MASK_SOURCEADDR_STRICT);

    if(type == CAN_PACKET_GIMBAL_DATATYPE_SET_SPEED)
    {
        Gimbal_Consume_Control_Target_Absolute_Yaw_Pitch_Speed(
                ((struct Maincontrol_Gimbal_Packet_Set_Speed *) rx_msg->Data)->yaw,
                ((struct Maincontrol_Gimbal_Packet_Set_Speed *) rx_msg->Data)->pitch
        );

    }
    else if(type == CAN_PACKET_GIMBAL_DATATYPE_SET_FRICTION)
    {

    }
    else if(type == CAN_PACKET_GIMBAL_DATATYPE_SET_SHOOTER)
    {

    }
    else if(type == CAN_PACKET_GIMBAL_DATATYPE_SET_LASER)
    {

    }
    else if(type == CAN_PACKET_GIMBAL_DATATYPE_SET_YAW_PITCH)
    {

    }
}

void Maincontrol_Can_Send_Handler(uint16_t id, int8_t code)
{
    if(code == 0)printf("%u send failed from can2\r\n", id);
}

void Maincontrol_Send_Other_Status()
{
    static struct Gimbal_Maincontrol_Packet_Others gmpo;
    //gmpo.friction_state
}


void Maincontrol_Send_Gyro()
{
    static struct Gimbal_Maincontrol_Packet_Gyro gmpg;

}

void Maincontrol_Send_Accel()
{
    static struct Gimbal_Maincontrol_Packet_Accel gmpa;

}

void Maincontrol_Send_Yaw_Pitch()
{
    static struct Gimbal_Maincontrol_Packet_Yaw_Pitch gmpyp;

}

void Maincontrol_Send_Roll()
{
    static struct Gimbal_Maincontrol_Packet_Roll gmpr;

}