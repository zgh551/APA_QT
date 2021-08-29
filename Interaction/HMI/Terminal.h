/*****************************************************************************/
/* FILE NAME: terminal.h                        COPYRIGHT (c) Henry Zhu 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: Interaction terminal            					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      January 8 2019      Initial Version                  */
/*****************************************************************************/

#ifndef TERMINAL_H_
#define TERMINAL_H_

#include <QMainWindow>

//#include "WinZlgCan/win_zlg_can.h"

#include "./Common/Utils/Inc/property.h"
#include "./Common/VehicleState/Interface/vehicle_state.h"
#include "./Common/VehicleState/GeometricTrack/geometric_track.h"

#include "./Interaction/CANBUS/DongFengE70/dong_feng_e70_message.h"
#include "./Interaction/CANBUS/Interface/vehicle_controller.h"
#include "./Interaction/CANBUS/Interface/message_manager.h"
#include "./Interaction/Ultrasonic/Ultrasonic.h"

#include "./Percaption/Interface/percaption.h"
#include "./Percaption/UltrasonicPercaption/ultrasonic_obstacle_percption.h"

typedef union _byte2float
{
	uint8_t b[4];
	float f;
}Byte2Float;

typedef union _Byte2Int
{
	uint8_t b[2];
	uint16_t u16;
	int16_t  i16;
}Byte2Int;

typedef union _Byte2Int32
{
	uint8_t b[4];
	uint32_t u32;
	int32_t  i32;
}Byte2Int32;

class Terminal
{
public:
	Terminal();
	virtual ~Terminal();
	// CAN Module:Vehicle information receive
//    void Parse(VCI_CAN_OBJ *obj,VehicleController *ctl);
//    void Parse(VCI_CAN_OBJ *obj,MessageManager *msg);

//    void Parse(VCI_CAN_OBJ *obj,Ultrasonic *u);

//    void Parse(VCI_CAN_OBJ obj,Percaption *pct);

//    void Parse(VCI_CAN_OBJ *obj);
	// Terminal Control
///////////////////////////////////////////////////////////////////////////
	/*
	 * 直接测量超声波原始信号
	 * */
	void UltrasonicSend(uint8_t id,Ultrasonic_Packet *msg_pk);
	/*
	 * 三角定位的短距离超声波信号
	 * */
	void UltrasonicLocationSend(uint8_t id,Ultrasonic_Packet *msg_pk);
	/*
	 * 载体坐标系的数据发送
	 * */
	void UltrasonicBodyLocationSend(uint8_t id,ObstacleLocationPacket packet);
	/*
	 * 地面坐标系的数据发送
	 * */
	void UltrasonicGroundLocationSend(uint8_t id,ObstacleLocationPacket packet);

	void Ack(void);

	void ParkingMsgSend(Percaption *pi,float fm,float rm);
	void ParkingCenterPointSend(Vector2d v);

	/*** Property ***/
	// AckValid
	uint8_t getAckValid();
	void setAckValid(uint8_t value);
	Property<Terminal,uint8_t,READ_WRITE> AckValid;

	uint8_t getAckEcho();
	void    setAckEcho(uint8_t value);
	Property<Terminal,uint8_t,READ_WRITE> AckEcho;

	// Commond
	uint8_t getCommand();
	void    setCommand(uint8_t value);
	Property<Terminal,uint8_t,READ_WRITE> Command;

	// Commond
	uint8_t getPushActive();
	void    setPushActive(uint8_t value);
	Property<Terminal,uint8_t,READ_WRITE> PushActive;

private:
	/// terminal command
	uint8_t _command;

	/// push state
	uint8_t _push_active;
	/// terminal receive frame state machine
	uint8_t _data_buffer[32];
	uint8_t _send_data_buffer[32];
	uint8_t _frame_id,_frame_length,_frame_cnt,_check_sum;
	uint8_t _frame_err_cnt;
	uint8_t _ack_valid;
	uint8_t _ack_echo;
	/// data type conversion
	Byte2Float _data_temp,_speed_data_temp;
};


#endif /* TERMINAL_H_ */
