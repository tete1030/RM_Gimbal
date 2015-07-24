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
#include "main.h"
#include "timer.h"
#include "friction.h"
#include "shooter.h"

#define CAN_CENTER_ADDR 0x0

#pragma pack(push,1)
struct Gimbal_Maincontrol_Packet_GM_Mech_Angle_Yaw
{
    int16_t yaw;
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

struct Maincontrol_Gimbal_Packet_Config_Friction
{
    uint16_t remoter_value;
};

struct Maincontrol_Gimbal_Packet_Set_Enable_Control
{
	uint8_t enable;
};

#pragma pack(pop)

#define GIMBAL_PACKET_ID_SEND_MECH_YAW 0x0

void Maincontrol_Send_Mech_Yaw(void)
{
    static struct Gimbal_Maincontrol_Packet_GM_Mech_Angle_Yaw gmpgmay;
    gmpgmay.yaw = gms_yaw.angle;
    CAN2_AsyncTransmit(GIMBAL_PACKET_ID_SEND_MECH_YAW,
                       CAN_PACKET_DESTTYPE_CENTER |
					   CAN_PACKET_CENTER_DATATYPE_STATUS_GM_MECH_ANGLE_YAW |
					   CAN_PACKET_PLACETYPE_SPECIFIC |
					   CAN_ADDR,
                       (char*) &gmpgmay,
                       sizeof(gmpgmay));
}

#define GIMBAL_PACKET_ID_SET_ENABLE_CONTROL 0x1

void Maincontrol_Set_Enable_Control(uint8_t enable)
{
	static struct Maincontrol_Gimbal_Packet_Set_Enable_Control mgpsec;
	mgpsec.enable = enable;

	CAN2_AsyncTransmit(GIMBAL_PACKET_ID_SET_ENABLE_CONTROL,
	                       CAN_PACKET_DESTTYPE_CENTER |
						   CAN_PACKET_CENTER_DATATYPE_STATUS_GM_SET_ENABLE_CONTROL |
						   CAN_PACKET_PLACETYPE_SPECIFIC |
						   CAN_ADDR,
	                       (char*) &mgpsec,
	                       sizeof(mgpsec));
}

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
    	GM_Set_Control_Target_Absolute_Yaw_Pitch_Speed(
                -((struct Maincontrol_Gimbal_Packet_Set_Speed *) rx_msg->Data)->yaw,
#if defined CAR_1
0
                #elif defined CAR_3
                ((struct Maincontrol_Gimbal_Packet_Set_Speed *) rx_msg->Data)->pitch
                #elif defined CAR_2
                -((struct Maincontrol_Gimbal_Packet_Set_Speed *) rx_msg->Data)->pitch
                #endif
        );

    }
    else if(type == CAN_PACKET_GIMBAL_DATATYPE_CONFIG_FRICTION)
    {
        Friction_Set_Remoter_Value(((struct Maincontrol_Gimbal_Packet_Config_Friction *) rx_msg->Data)->remoter_value);
    }
    else if(type == CAN_PACKET_GIMBAL_DATATYPE_SET_FRICTION)
    {
        // turn off the shooter and wait for a while before turn off friction
        Friction_Set_Enable(((struct Maincontrol_Gimbal_Packet_Set_Friction *) rx_msg->Data)->enable);
    }
    else if(type == CAN_PACKET_GIMBAL_DATATYPE_SET_SHOOTER)
    {
        // turn on only when friction is ready
        if(((struct Maincontrol_Gimbal_Packet_Set_Shooter *) rx_msg->Data)->enable)
            Shooter_Start();
        else
            Shooter_Stop();
    }
    else if(type == CAN_PACKET_GIMBAL_DATATYPE_SET_LASER)
    {
        // TODO: Finish this
        // only used under debug
    }
    else if(type == CAN_PACKET_GIMBAL_DATATYPE_SET_YAW_PITCH)
    {
        // TODO: Finish this
        // only used under debug
    }
}

void Maincontrol_Can_Send_Handler(uint16_t id, int8_t code)
{
    if(code != 0)printf("id %u send failed with code %d from can2\r\n", id, code);
}
