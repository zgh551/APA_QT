/*****************************************************************************/
/* FILE NAME: terminal.cpp                      COPYRIGHT (c) Henry Zhu 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: Interaction terminal            					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      January 8 2019      Initial Version                  */
/*****************************************************************************/

#include "./Interaction/HMI/Terminal.h"

Terminal::Terminal() {

	_frame_err_cnt = 0;
	_push_active = 0;
	// ACK Valid
	AckValid.setContainer(this);
	AckValid.getter(&Terminal::getAckValid);
	AckValid.setter(&Terminal::setAckValid);

	AckEcho.setContainer(this);
	AckEcho.getter(&Terminal::getAckEcho);
	AckEcho.setter(&Terminal::setAckEcho);

	// ACK Valid
	Command.setContainer(this);
	Command.getter(&Terminal::getCommand);
	Command.setter(&Terminal::setCommand);

	// Push Active
	PushActive.setContainer(this);
	PushActive.getter(&Terminal::getPushActive);
	PushActive.setter(&Terminal::setPushActive);
}

Terminal::~Terminal() {

}

/// AckValid
uint8_t Terminal::getAckValid()             {return _ack_valid ;}
void    Terminal::setAckValid(uint8_t value){_ack_valid = value;}
uint8_t Terminal::getAckEcho()             {return _ack_echo ;}
void    Terminal::setAckEcho(uint8_t value){_ack_echo = value;}

uint8_t Terminal::getCommand()             {return _command ;}
void    Terminal::setCommand(uint8_t value){_command = value;}
uint8_t Terminal::getPushActive()             {return _push_active ;}
void    Terminal::setPushActive(uint8_t value){_push_active = value;}
/**************************************************************************************/

//void Terminal::Parse(VCI_CAN_OBJ *obj,VehicleController *ctl)
//{
//	uint8_t i,check_sum;
//    switch(obj->ID)
//	{
//		case 0x516://eps status
//			check_sum =0 ;
//			for(i=0;i<7;i++){
//                check_sum += obj->Data[i];
//			}
//			check_sum = check_sum ^ 0xFF;
//            if(check_sum == obj->Data[7])
//			{
//                ctl->GearEnable 		=  obj->Data[0]       & 0x01;
//                ctl->AccelerationEnable = (obj->Data[0] >> 2) & 0x01;
//                ctl->DecelerationEnable = (obj->Data[0] >> 4) & 0x01;
//                ctl->TorqueEnable       = (obj->Data[0] >> 5) & 0x01;
//                ctl->VelocityEnable     = (obj->Data[0] >> 3) & 0x01;
//                if((0 == ctl->SteeringEnable) || (0 == ((obj->Data[0] >> 1) & 0x01)))
//				{
//                    ctl->SteeringEnable 	= (obj->Data[0] >> 1) & 0x01;
//				}
//                ctl->Gear 				= (uint8_t)obj->Data[1];
//                ctl->SteeringAngle 		= (float)(((int16_t)((obj->Data[3] << 8) | obj->Data[2])) * 0.1);
//                ctl->SteeringAngleRate 	= (float)(((uint16_t)((obj->Data[5] << 8) | obj->Data[4])) * 0.01);
//				AckValid = 0xa5;
//			}
//			break;

//		case 0x517://eps status
//			check_sum =0 ;
//			for(i=0;i<7;i++){
//                check_sum += obj->Data[i];
//			}
//			check_sum = check_sum ^ 0xFF;
//            if(check_sum == obj->Data[7])
//			{
//                ctl->Acceleration	= (float)(((int16_t )((obj->Data[1] << 8) | obj->Data[0])) * 0.001);
////				ctl->TargetAcceleration	= (float)(((int16_t )((obj->Data[1] << 8) | obj->Data[0])) * 0.001);
//                ctl->Deceleration	= (float)(((int16_t )((obj->Data[3] << 8) | obj->Data[2])) * 0.001);
//                ctl->Velocity		= (float)(((uint16_t)((obj->Data[5] << 8) | obj->Data[4])) * 0.001);
//                ctl->Torque			= (float)(obj->Data[6] * 2);
//			}
//			break;

//		case 0x518:
//			check_sum =0 ;
//			for(i=0;i<7;i++){
//                check_sum += obj->Data[i];
//			}
//			check_sum = check_sum ^ 0xFF;
//            if(check_sum == obj->Data[7])
//			{
//                ctl->SteeringAngle 		= ((int16_t)((obj->Data[1] << 8) | obj->Data[0])) * 0.1f;
//                ctl->SteeringAngleRate 	= obj->Data[2] * 4.0f;
//                ctl->Velocity		    = obj->Data[3] * 0.01f;
//                ctl->Distance           = ((uint16_t)((obj->Data[5] << 8) | obj->Data[4])) * 0.001f;
//                ctl->Gear 				= (uint8_t)(obj->Data[6] & 0x0f);
//                ctl->APAEnable          = (uint8_t)((obj->Data[6]>>4) & 0x03);
//				AckValid = 0xa5;
//			}
//			break;

//		case 0x519://eps status
//			check_sum =0 ;
//			for(i=0;i<7;i++){
//                check_sum += obj->Data[i];
//			}
//			check_sum = check_sum ^ 0xFF;
//            if(check_sum == obj->Data[7])
//			{
//                ctl->SteeringAngle 		= (float)(((int16_t)((obj->Data[1] << 8) | obj->Data[0])) * 0.1);
//                ctl->SteeringAngleRate 	= (float)(((uint16_t)((obj->Data[3] << 8) | obj->Data[2])) * 0.01);
//                ctl->Torque			    = (float)(((uint16_t)((obj->Data[5] << 8) | obj->Data[4])));

//                ctl->GearEnable 		=  obj->Data[6]       & 0x01;
//                ctl->SteeringEnable 	= (obj->Data[6] >> 1) & 0x01;
//                ctl->AccelerationEnable = (obj->Data[6] >> 2) & 0x01;
//                ctl->VelocityEnable     = (obj->Data[6] >> 3) & 0x01;
//                ctl->DecelerationEnable = (obj->Data[6] >> 4) & 0x01;
//                ctl->TorqueEnable       = (obj->Data[6] >> 5) & 0x01;
//				AckValid = 0xa5;
//			}
//			break;

//		case 0x51A://eps status
//			check_sum =0 ;
//			for(i=0;i<7;i++){
//                check_sum += obj->Data[i];
//			}
//			check_sum = check_sum ^ 0xFF;
//            if(check_sum == obj->Data[7])
//			{
//                ctl->Acceleration	= (float)(((int16_t )((obj->Data[1] << 8) | obj->Data[0])) * 0.001);
////				ctl->TargetAcceleration	= (float)(((int16_t )((obj->Data[1] << 8) | obj->Data[0])) * 0.001);
//                ctl->Deceleration	= (float)(((int16_t )((obj->Data[3] << 8) | obj->Data[2])) * 0.001);
//                ctl->Velocity		= (float)(((uint16_t)((obj->Data[5] << 8) | obj->Data[4])) * 0.001);
//                ctl->Gear 			= (uint8_t)obj->Data[6];
//			}
//			break;
//		default:

//			break;
//	}
//}

//void Terminal::Parse(VCI_CAN_OBJ *obj,Ultrasonic *u)
//{
//	Ultrasonic_Packet ultrasonic_packet;
//	ObstacleLocationPacket obstacle_location_packet;
//    switch(obj->ID)
//	{
//        case 0x508://传感器9
//        case 0x509://传感器10
//        case 0x50A://传感器11
//        case 0x50B://传感器12
//            ultrasonic_packet.Distance1 = (float)(((uint16_t )((obj->Data[1] << 8) | obj->Data[0])) * 0.01);
//            ultrasonic_packet.Distance2 = (float)(((uint16_t )((obj->Data[3] << 8) | obj->Data[2])) * 0.01);
//            ultrasonic_packet.Level = obj->Data[4] * 0.1;
//            ultrasonic_packet.Width = obj->Data[5];
//            ultrasonic_packet.status = obj->Data[6];
//        	u->setUltrasonicPacket(id & 0x00f,ultrasonic_packet);
//			break;

//        case 0x50C:
//            obstacle_location_packet.Position.X = (float)(((int16_t )((obj->Data[1] << 8) | obj->Data[0])) * 0.01);
//            obstacle_location_packet.Position.Y = (float)(((int16_t )((obj->Data[3] << 8) | obj->Data[2])) * 0.01);
//            obstacle_location_packet.Status = (UltrasonicStatus)obj->Data[7];
//        	u->setAbstacleGroundPositionTriangle(5, obstacle_location_packet);
//        	break;

//        case 0x50D:
//            obstacle_location_packet.Position.X = (float)(((int16_t )((obj->Data[1] << 8) | obj->Data[0])) * 0.01);
//            obstacle_location_packet.Position.Y = (float)(((int16_t )((obj->Data[3] << 8) | obj->Data[2])) * 0.01);
//            obstacle_location_packet.Status = (UltrasonicStatus)obj->Data[7];
//        	u->setAbstacleGroundPositionTriangle(6, obstacle_location_packet);
//        	break;

//        case 0x50E:
//            obstacle_location_packet.Position.X = (float)(((int16_t )((obj->Data[1] << 8) | obj->Data[0])) * 0.01);
//            obstacle_location_packet.Position.Y = (float)(((int16_t )((obj->Data[3] << 8) | obj->Data[2])) * 0.01);
//            obstacle_location_packet.Status = (UltrasonicStatus)obj->Data[7];
//        	u->setAbstacleGroundPositionTriangle(10, obstacle_location_packet);
//        	break;

//        case 0x50F:
//            obstacle_location_packet.Position.X = (float)(((int16_t )((obj->Data[1] << 8) | obj->Data[0])) * 0.01);
//            obstacle_location_packet.Position.Y = (float)(((int16_t )((obj->Data[3] << 8) | obj->Data[2])) * 0.01);
//            obstacle_location_packet.Status = (UltrasonicStatus)obj->Data[7];
//        	u->setAbstacleGroundPositionTriangle(11, obstacle_location_packet);
//        	AckValid = 0xa5;
//        	break;

//		default:

//			break;
//	}
//}

//void Terminal::Parse(VCI_CAN_OBJ *obj,MessageManager *msg)
//{
//	Byte2Int temp_int;
//    switch(obj->ID)
//	{
//        case 0x510:
//            temp_int.b[1] = obj->Data[2];
//            temp_int.b[0] = obj->Data[3];
//        	msg->SteeringAngle = temp_int.i16 * 0.1;
//        	break;

//        case 0x520:
//            temp_int.b[1] = obj->Data[4];
//            temp_int.b[0] = obj->Data[5];
//        	msg->WheelSpeedRearLeft = temp_int.u16 * 0.001;
//            temp_int.b[1] = obj->Data[6];
//            temp_int.b[0] = obj->Data[7];
//        	msg->WheelSpeedRearRight = temp_int.u16 * 0.001;
//        	break;
//		default:

//			break;
//	}
//}

//void Terminal::Parse(VCI_CAN_OBJ obj,Percaption *pct)
//{
//    Byte2Int temp_int;
//    ObstacleDistancePacket obstacle_temp;
//    switch(obj.ID)
//	{
//        case 0x44D:
//            temp_int.b[0] = obj.Data[0];
//            temp_int.b[1] = obj.Data[1];
//            obstacle_temp.distance = temp_int.u16 * 0.0001f;
//            obstacle_temp.region   = static_cast<ObstacleRegion>(obj.Data[2]);
//            obstacle_temp.status   = static_cast<UltrasonicStatus>(obj.Data[3]);
//            pct->setFrontObstacleDistance(obstacle_temp);

//            temp_int.b[0] = obj.Data[4];
//            temp_int.b[1] = obj.Data[5];
//            obstacle_temp.distance = temp_int.u16 * 0.0001f;
//            obstacle_temp.region   = static_cast<ObstacleRegion>(obj.Data[6]);
//            obstacle_temp.status   = static_cast<UltrasonicStatus>(obj.Data[7]);
//            pct->setRearObstacleDistance(obstacle_temp);
//        	break;

//		default:
//			break;
//	}
//}

//void Terminal::Parse(VCI_CAN_OBJ *obj)
//{
//    switch(obj->ID)
//	{
//        case 0x530:
//            Command = static_cast<uint8_t>(obj->Data[0]);
//        	break;

//        case 0x512:
//            AckEcho = static_cast<uint8_t>(obj->Data[0]);
//        	break;

//        default:
//        	break;
//	}
//}
/**************************************************************************************/
//void Terminal::Push(MessageManager *msg)
//{
//	CAN_Packet m_CAN_Packet;
//	int16_t temp_int16;
//	uint16_t temp_uint16;

//	m_CAN_Packet.id = 0x410;
//	m_CAN_Packet.length = 8;

//	temp_uint16 = msg->VehicleMiddleSpeed * 1000;
//	m_CAN_Packet.obj->Dataa[0] =  temp_uint16 & 0xff;
//	m_CAN_Packet.obj->Dataa[1] = (temp_uint16 >> 8) & 0xff;
//	temp_int16 = (int16_t)(msg->SteeringAngle * 10);
//	m_CAN_Packet.data[2] = temp_int16 & 0xff;
//	m_CAN_Packet.data[3] = (temp_int16 >> 8) & 0xff;
//	temp_uint16 = (uint16_t)(msg->SteeringAngleRate * 100);
//	m_CAN_Packet.data[4] =  temp_uint16 & 0xff;
//	m_CAN_Packet.data[5] = (temp_uint16 >> 8) & 0xff;
//	m_CAN_Packet.data[6] = msg->Gear;
//	m_CAN_Packet.data[7] = msg->WheelSpeedDirection;
//	CAN2_TransmitMsg(m_CAN_Packet);

	// 车速状态反馈
//	m_CAN_Packet.id = 0x411;
//	m_CAN_Packet.length = 8;
//	temp_uint16 = (uint16_t)(msg->WheelSpeedFrontLeft * 1000);
//	m_CAN_Packet.data[0] =  temp_uint16 & 0xff;
//	m_CAN_Packet.data[1] = (temp_uint16 >> 8) & 0xff;
//	temp_uint16 = (uint16_t)(msg->WheelSpeedFrontRight * 1000);
//	m_CAN_Packet.data[2] = temp_uint16 & 0xff;
//	m_CAN_Packet.data[3] = (temp_uint16 >> 8) & 0xff;
//	temp_uint16 = (uint16_t)(msg->WheelSpeedRearLeft * 1000);
//	m_CAN_Packet.data[4] =  temp_uint16 & 0xff;
//	m_CAN_Packet.data[5] = (temp_uint16 >> 8) & 0xff;
//	temp_uint16 = (uint16_t)(msg->WheelSpeedRearRight * 1000);
//	m_CAN_Packet.data[6] = temp_uint16 & 0xff;
//	m_CAN_Packet.data[7] = (temp_uint16 >> 8) & 0xff;
//	CAN2_TransmitMsg(m_CAN_Packet);

//	// 车速状态反馈
//	m_CAN_Packet.id = 0x412;
//	m_CAN_Packet.length = 8;
//	temp_uint16 = msg->WheelPulseFrontLeft;
//	m_CAN_Packet.data[0] =  temp_uint16 & 0xff;
//	m_CAN_Packet.data[1] = (temp_uint16 >> 8) & 0xff;
//	temp_uint16 = msg->WheelPulseFrontRight;
//	m_CAN_Packet.data[2] = temp_uint16 & 0xff;
//	m_CAN_Packet.data[3] = (temp_uint16 >> 8) & 0xff;
//	temp_uint16 = msg->WheelPulseRearLeft;
//	m_CAN_Packet.data[4] =  temp_uint16 & 0xff;
//	m_CAN_Packet.data[5] = (temp_uint16 >> 8) & 0xff;
//	temp_uint16 = msg->WheelPulseRearRight;
//	m_CAN_Packet.data[6] = temp_uint16 & 0xff;
//	m_CAN_Packet.data[7] = (temp_uint16 >> 8) & 0xff;
//	CAN2_TransmitMsg(m_CAN_Packet);

//	m_CAN_Packet.id = 0x417;
//	temp_int16 = (int16_t)(msg->LonAcc * 1000);
//	m_CAN_Packet.data[0] =  temp_int16 & 0xff ;
//	m_CAN_Packet.data[1] = (temp_int16 >> 8) & 0xff ;
//	temp_int16 = (int16_t)(msg->LatAcc * 1000);
//	m_CAN_Packet.data[2] = temp_int16 & 0xff;
//	m_CAN_Packet.data[3] = (temp_int16 >> 8) & 0xff;
//	temp_int16 = (int16_t)(msg->YawRate * 100);
//	m_CAN_Packet.data[4] = temp_int16 & 0xff;
//	m_CAN_Packet.data[5] = (temp_int16 >> 8) & 0xff;
//	m_CAN_Packet.data[6] = 0;
//	m_CAN_Packet.data[7] = 0;
//	CAN2_TransmitMsg(m_CAN_Packet);
//}

//void Terminal::Push(DongFengE70Message msg)
//{
//	CAN_Packet m_CAN_Packet;

//	m_CAN_Packet.length = 8;
//	m_CAN_Packet.id = 0x418;

//	m_CAN_Packet.data[0] = msg.getEPS_AvailabStatus();
//	m_CAN_Packet.data[1] = msg.getESC_APA_EnableStatus();
//	m_CAN_Packet.data[2] = msg.getVCU_APA_ControlStatus();
//	m_CAN_Packet.data[3] = 0;

//	m_CAN_Packet.data[4] = 0;
//	m_CAN_Packet.data[5] = 0;
//	m_CAN_Packet.data[6] = 0;
//	m_CAN_Packet.data[7] = 0;
//	CAN2_TransmitMsg(m_CAN_Packet);
//}
///*
// * 控制信号
// * */
//void Terminal::Push(VehicleController *msg)
//{
//	CAN_Packet m_CAN_Packet;
//	int16_t temp_int16;
//	uint16_t temp_uint16;

//	m_CAN_Packet.id = 0x414;
//	m_CAN_Packet.length = 8;
//	m_CAN_Packet.data[0] = 	 msg->AccelerationEnable 	   |
//							(msg->DecelerationEnable << 1) |
//							(msg->TorqueEnable       << 2) |
//							(msg->VelocityEnable     << 3) |
//							(msg->SteeringEnable     << 4) |
//							(msg->GearEnable         << 6) ;
//	m_CAN_Packet.data[1] = msg->Gear ;
//	temp_int16 = (int16_t)(msg->Acceleration * 100);
//	m_CAN_Packet.data[2] =  temp_int16       & 0xff ;
//	m_CAN_Packet.data[3] = (temp_int16 >> 8) & 0xff ;
//	temp_uint16 = (uint16_t)msg->Torque;
//	m_CAN_Packet.data[4] =  temp_uint16       & 0xff ;
//	m_CAN_Packet.data[5] = (temp_uint16 >> 8) & 0xff ;
//	temp_uint16 = (uint16_t)(msg->Velocity * 100);
//	m_CAN_Packet.data[6] =  temp_uint16       & 0xff ;
//	m_CAN_Packet.data[7] = (temp_uint16 >> 8) & 0xff ;
//	CAN2_TransmitMsg(m_CAN_Packet);

//	m_CAN_Packet.id = 0x415;
//	temp_uint16 = (uint16_t)(msg->Distance * 1000);
//	m_CAN_Packet.data[0] =  temp_uint16 & 0xff ;
//	m_CAN_Packet.data[1] = (temp_uint16 >> 8) & 0xff ;
//	temp_int16 = (int16_t)(msg->TargetAcceleration * 1000);
//	m_CAN_Packet.data[2] = temp_int16 & 0xff;
//	m_CAN_Packet.data[3] = (temp_int16 >> 8) & 0xff;

//	m_CAN_Packet.data[4] = 0;
//	m_CAN_Packet.data[5] = 0;
//	m_CAN_Packet.data[6] = 0;
//	m_CAN_Packet.data[7] = 0;
//	CAN2_TransmitMsg(m_CAN_Packet);
//}

//void Terminal::Push(VehicleState *msg)
//{
//	CAN_Packet m_CAN_Packet;
//	Byte2Int temp_int;
//	m_CAN_Packet.id = 0x442;
//	m_CAN_Packet.length = 8;

//	temp_int.i16 = (int16_t)(msg->getPosition().X * 100);
//	m_CAN_Packet.data[0] = temp_int.b[1];
//	m_CAN_Packet.data[1] = temp_int.b[0];
//	temp_int.i16 = (int16_t)(msg->getPosition().Y * 100);
//	m_CAN_Packet.data[2] = temp_int.b[1];
//	m_CAN_Packet.data[3] = temp_int.b[0];
//	temp_int.i16 = (int16_t)(msg->Yaw * 100);
//	m_CAN_Packet.data[4] = temp_int.b[1];
//	m_CAN_Packet.data[5] = temp_int.b[0];

//	m_CAN_Packet.data[6] = 0;
//	m_CAN_Packet.data[7] = 0;
//	CAN2_TransmitMsg(m_CAN_Packet);

//	m_CAN_Packet.id = 0x44E;
//	m_CAN_Packet.length = 8;

//	temp_int.u16 = (uint16_t)(msg->getPulseUpdateVelocity() * 10000);
//	m_CAN_Packet.data[0] = temp_int.b[1];
//	m_CAN_Packet.data[1] = temp_int.b[0];
//	temp_int.u16 = (uint16_t)(msg->getAccUpdateVelocity() * 10000);
//	m_CAN_Packet.data[2] = temp_int.b[1];
//	m_CAN_Packet.data[3] = temp_int.b[0];

//	m_CAN_Packet.data[4] = 0;
//	m_CAN_Packet.data[5] = 0;
//	m_CAN_Packet.data[6] = 0;
//	m_CAN_Packet.data[7] = 0;
//	CAN2_TransmitMsg(m_CAN_Packet);
//}

//void Terminal::Push(GeometricTrack track)
//{
//	CAN_Packet m_CAN_Packet;
//	Byte2Int32 temp_int32;
//	m_CAN_Packet.id = 0x413;
//	m_CAN_Packet.length = 8;

//	temp_int32.i32 = track.SumRearLeftPulse;
//	m_CAN_Packet.data[0] = temp_int32.b[3];
//	m_CAN_Packet.data[1] = temp_int32.b[2];
//	m_CAN_Packet.data[2] = temp_int32.b[1];
//	m_CAN_Packet.data[3] = temp_int32.b[0];
//	temp_int32.i32 = track.SumRearRightPulse;
//	m_CAN_Packet.data[4] = temp_int32.b[3];
//	m_CAN_Packet.data[5] = temp_int32.b[2];
//	m_CAN_Packet.data[6] = temp_int32.b[1];
//	m_CAN_Packet.data[7] = temp_int32.b[0];
//	CAN2_TransmitMsg(m_CAN_Packet);
//}

//void Terminal::Push(Ultrasonic *u)
//{
//#if ULTRASONIC_PACKET == 1

//#if ULTRASONIC_SCHEDULE_MODO == 2
//	switch(u->ScheduleTimeCnt)
//	{
//		case 7:
//			UltrasonicSend(1,u->UltrasonicPacket);
//			UltrasonicSend(7,u->UltrasonicPacket);
//			break;

//		case 10:
//		case 23:
//			UltrasonicSend(8,u->UltrasonicPacket);
//			UltrasonicSend(11,u->UltrasonicPacket);
//			break;

//		case 12:
//		case 25:
//			UltrasonicSend(9,u->UltrasonicPacket);
//			UltrasonicSend(10,u->UltrasonicPacket);
//			break;

//		case 13:
//			UltrasonicSend(3,u->UltrasonicPacket);
//			UltrasonicSend(5,u->UltrasonicPacket);
//			break;

//		case 20:
//			UltrasonicSend(0,u->UltrasonicPacket);
//			UltrasonicSend(6,u->UltrasonicPacket);
//			break;

//		case 0:
//			UltrasonicSend(2,u->UltrasonicPacket);
//			UltrasonicSend(4,u->UltrasonicPacket);
//			break;

//		default:
//			break;
//	}
//#endif

//#if ULTRASONIC_SCHEDULE_MODO == 3
//	switch(u->ScheduleTimeCnt)
//	{
//		case 9:
//			// 直接测量的传感器值
//			UltrasonicSend(1,u->UltrasonicPacket);
//			UltrasonicSend(6,u->UltrasonicPacket);

//			// 三角定位测量的传感器值
//			UltrasonicLocationSend(0 ,u->UltrasonicLocationPacket);
//			UltrasonicLocationSend(1 ,u->UltrasonicLocationPacket);
//			UltrasonicLocationSend(2 ,u->UltrasonicLocationPacket);
//			UltrasonicLocationSend(9 ,u->UltrasonicLocationPacket);
//			UltrasonicLocationSend(10,u->UltrasonicLocationPacket);
//			UltrasonicLocationSend(11,u->UltrasonicLocationPacket);

//			// 三角定位车体坐标系
//			UltrasonicBodyLocationSend(0,u->AbstacleBodyPositionTriangle[0]);
//			UltrasonicBodyLocationSend(1,u->AbstacleBodyPositionTriangle[1]);
//			UltrasonicBodyLocationSend(2,u->AbstacleBodyPositionTriangle[2]);
//			UltrasonicBodyLocationSend(3,u->AbstacleBodyPositionTriangle[3]);
//			// 三角定位地面坐标系
//			UltrasonicGroundLocationSend(0,u->AbstacleGroundPositionTriangle[0]);
//			UltrasonicGroundLocationSend(1,u->AbstacleGroundPositionTriangle[1]);
//			UltrasonicGroundLocationSend(2,u->AbstacleGroundPositionTriangle[2]);
//			UltrasonicGroundLocationSend(3,u->AbstacleGroundPositionTriangle[3]);
//			break;

//		case 23:
//			UltrasonicSend(2,u->UltrasonicPacket);
//			UltrasonicSend(5,u->UltrasonicPacket);
//			// 三角定位测量的传感器值
//			UltrasonicLocationSend(3,u->UltrasonicLocationPacket);
//			UltrasonicLocationSend(4,u->UltrasonicLocationPacket);
//			UltrasonicLocationSend(5,u->UltrasonicLocationPacket);
//			UltrasonicLocationSend(6,u->UltrasonicLocationPacket);
//			UltrasonicLocationSend(7,u->UltrasonicLocationPacket);
//			UltrasonicLocationSend(8,u->UltrasonicLocationPacket);
//			// 三角定位车体坐标系
//			UltrasonicBodyLocationSend(4,u->AbstacleBodyPositionTriangle[4]);
//			UltrasonicBodyLocationSend(5,u->AbstacleBodyPositionTriangle[5]);
//			UltrasonicBodyLocationSend(6,u->AbstacleBodyPositionTriangle[6]);
//			UltrasonicBodyLocationSend(7,u->AbstacleBodyPositionTriangle[7]);
//			// 三角定位地面坐标系
//			UltrasonicGroundLocationSend(4,u->AbstacleGroundPositionTriangle[4]);
//			UltrasonicGroundLocationSend(5,u->AbstacleGroundPositionTriangle[5]);
//			UltrasonicGroundLocationSend(6,u->AbstacleGroundPositionTriangle[6]);
//			UltrasonicGroundLocationSend(7,u->AbstacleGroundPositionTriangle[7]);
//			break;

//		case 11:
//		case 25:
//			UltrasonicSend(8 ,u->UltrasonicPacket);
//			UltrasonicSend(10,u->UltrasonicPacket);

//			UltrasonicBodyLocationSend(8,u->AbstacleBodyPositionDirect[8]);
//			UltrasonicBodyLocationSend(10,u->AbstacleBodyPositionDirect[10]);

//			UltrasonicGroundLocationSend(8,u->AbstacleGroundPositionTriangle[8]);
//			UltrasonicGroundLocationSend(10,u->AbstacleGroundPositionTriangle[10]);
//		break;

//		case 13:
//		case 27:
//			UltrasonicSend(9,u->UltrasonicPacket);
//			UltrasonicSend(11,u->UltrasonicPacket);

//			UltrasonicBodyLocationSend(9,u->AbstacleBodyPositionDirect[9]);
//			UltrasonicBodyLocationSend(11,u->AbstacleBodyPositionDirect[11]);

//			UltrasonicGroundLocationSend(9,u->AbstacleGroundPositionTriangle[9]);
//			UltrasonicGroundLocationSend(11,u->AbstacleGroundPositionTriangle[11]);
//			break;

//		case 14:
//			UltrasonicSend(3,u->UltrasonicPacket);
//			UltrasonicSend(4,u->UltrasonicPacket);
//			break;

//		case 0:
//			UltrasonicSend(0,u->UltrasonicPacket);
//			UltrasonicSend(7,u->UltrasonicPacket);
//			break;

//		default:
//			break;
//	}
//#endif

//#else
//#if ULTRASONIC_SCHEDULE_MODO == 2
//	switch(u->ReadStage)
//	{
//		case 0:
//			UltrasonicSend(1,u->UltrasonicDatas);
//			UltrasonicSend(7,u->UltrasonicDatas);
//			break;

//		case 1:
//		case 5:
//			UltrasonicSend(8,u->UltrasonicDatas);
//			UltrasonicSend(11,u->UltrasonicDatas);
//			break;

//		case 2:
//		case 6:
//			UltrasonicSend(9,u->UltrasonicDatas);
//			UltrasonicSend(10,u->UltrasonicDatas);
//			break;

//		case 3:
//			UltrasonicSend(3,u->UltrasonicDatas);
//			UltrasonicSend(5,u->UltrasonicDatas);
//			break;

//		case 4:
//			UltrasonicSend(0,u->UltrasonicDatas);
//			UltrasonicSend(6,u->UltrasonicDatas);
//			break;

//		case 7:
//			UltrasonicSend(2,u->UltrasonicDatas);
//			UltrasonicSend(4,u->UltrasonicDatas);
//			break;

//		default:
//			break;
//	}
//#endif

//#if ULTRASONIC_SCHEDULE_MODO == 3
//	switch(u->ReadStage)
//	{
//		case 0:
//			UltrasonicSend(1,u->UltrasonicDatas);
//			UltrasonicSend(6,u->UltrasonicDatas);
//			UltrasonicLocationSend(0,u->UltrasonicLocationDatas);
//			UltrasonicLocationSend(1,u->UltrasonicLocationDatas);
//			UltrasonicLocationSend(2,u->UltrasonicLocationDatas);
//			UltrasonicLocationSend(9,u->UltrasonicLocationDatas);
//			UltrasonicLocationSend(10,u->UltrasonicLocationDatas);
//			UltrasonicLocationSend(11,u->UltrasonicLocationDatas);
//			break;

//		case 1:
//		case 5:
//			UltrasonicSend(8,u->UltrasonicDatas);
//			UltrasonicSend(10,u->UltrasonicDatas);
//			break;

//		case 2:
//		case 6:
//			UltrasonicSend(9,u->UltrasonicDatas);
//			UltrasonicSend(11,u->UltrasonicDatas);
//			break;

//		case 3:
//			UltrasonicSend(3,u->UltrasonicDatas);
//			UltrasonicSend(4,u->UltrasonicDatas);
//			break;

//		case 4:
//			UltrasonicSend(2,u->UltrasonicDatas);
//			UltrasonicSend(5,u->UltrasonicDatas);
//			UltrasonicLocationSend(3,u->UltrasonicLocationDatas);
//			UltrasonicLocationSend(4,u->UltrasonicLocationDatas);
//			UltrasonicLocationSend(5,u->UltrasonicLocationDatas);
//			UltrasonicLocationSend(6,u->UltrasonicLocationDatas);
//			UltrasonicLocationSend(7,u->UltrasonicLocationDatas);
//			UltrasonicLocationSend(8,u->UltrasonicLocationDatas);
//			break;

//		case 7:
//			UltrasonicSend(0,u->UltrasonicDatas);
//			UltrasonicSend(7,u->UltrasonicDatas);
//			break;

//		default:
//			break;
//	}
//#endif
//#endif
//}

//void Terminal::Push(Percaption *p)
//{
//	CAN_Packet m_CAN_Packet;
//	Byte2Int temp_int;

//	m_CAN_Packet.length = 8;

//	m_CAN_Packet.id = 0x440;
//	temp_int.i16 = (int16_t)(p->PositionX * 100);
//	m_CAN_Packet.data[0] = temp_int.b[1];
//	m_CAN_Packet.data[1] = temp_int.b[0];
//	temp_int.i16 = (int16_t)(p->PositionY * 100);
//	m_CAN_Packet.data[2] = temp_int.b[1];
//	m_CAN_Packet.data[3] = temp_int.b[0];
//	temp_int.i16 = (int16_t)(p->AttitudeYaw * 100);
//	m_CAN_Packet.data[4] = temp_int.b[1];
//	m_CAN_Packet.data[5] = temp_int.b[0];
//	m_CAN_Packet.data[6] = p->DetectParkingStatus;
//	m_CAN_Packet.data[7] = 0;
//	CAN2_TransmitMsg(m_CAN_Packet);

//	m_CAN_Packet.id = 0x441;
//	temp_int.u16 = (uint16_t)(p->ParkingLength * 1000);
//	m_CAN_Packet.data[0] = temp_int.b[1];
//	m_CAN_Packet.data[1] = temp_int.b[0];
//	temp_int.u16 = (uint16_t)(p->ParkingWidth * 1000);
//	m_CAN_Packet.data[2] = temp_int.b[1];
//	m_CAN_Packet.data[3] = temp_int.b[0];
//	m_CAN_Packet.data[4] = 0;
//	m_CAN_Packet.data[5] = 0;
//	m_CAN_Packet.data[6] = 0;
//	m_CAN_Packet.data[7] = 0;
//	CAN2_TransmitMsg(m_CAN_Packet);

//	m_CAN_Packet.id = 0x449;
//	temp_int.i16 = (int16_t)(p->getValidParkingEdgePosition().First_Position.getX() * 1000);
//	m_CAN_Packet.data[0] = temp_int.b[1];
//	m_CAN_Packet.data[1] = temp_int.b[0];
//	temp_int.i16 = (int16_t)(p->getValidParkingEdgePosition().First_Position.getY() * 1000);
//	m_CAN_Packet.data[2] = temp_int.b[1];
//	m_CAN_Packet.data[3] = temp_int.b[0];
//	temp_int.i16 = (int16_t)(p->getValidParkingEdgePosition().Second_Position.getX() * 1000);
//	m_CAN_Packet.data[4] = temp_int.b[1];
//	m_CAN_Packet.data[5] = temp_int.b[0];
//	temp_int.i16 = (int16_t)(p->getValidParkingEdgePosition().Second_Position.getY() * 1000);
//	m_CAN_Packet.data[6] = temp_int.b[1];
//	m_CAN_Packet.data[7] = temp_int.b[0];
//	CAN2_TransmitMsg(m_CAN_Packet);

//	m_CAN_Packet.id = 0x44B;
//	temp_int.i16 = (int16_t)(p->getValidParkingCenterPosition().position.getX() * 1000);
//	m_CAN_Packet.data[0] = temp_int.b[1];
//	m_CAN_Packet.data[1] = temp_int.b[0];
//	temp_int.i16 = (int16_t)(p->getValidParkingCenterPosition().position.getY() * 1000);
//	m_CAN_Packet.data[2] = temp_int.b[1];
//	m_CAN_Packet.data[3] = temp_int.b[0];
//	temp_int.i16 = (int16_t)(p->getValidParkingCenterPosition().angle * 10000);
//	m_CAN_Packet.data[4] = temp_int.b[1];
//	m_CAN_Packet.data[5] = temp_int.b[0];
//	temp_int.i16 = (int16_t)(p->getCenterFitLinePacket().offset * 1000);
//	m_CAN_Packet.data[6] = temp_int.b[1];
//	m_CAN_Packet.data[7] = temp_int.b[0];
//	CAN2_TransmitMsg(m_CAN_Packet);

//	m_CAN_Packet.id = 0x44C;
//	temp_int.i16 = (int16_t)(p->getLeftFitLinePacket().angle * 10000);
//	m_CAN_Packet.data[0] = temp_int.b[1];
//	m_CAN_Packet.data[1] = temp_int.b[0];
//	temp_int.i16 = (int16_t)(p->getLeftFitLinePacket().offset * 1000);
//	m_CAN_Packet.data[2] = temp_int.b[1];
//	m_CAN_Packet.data[3] = temp_int.b[0];
//	temp_int.i16 = (int16_t)(p->getRightFitLinePacket().angle * 10000);
//	m_CAN_Packet.data[4] = temp_int.b[1];
//	m_CAN_Packet.data[5] = temp_int.b[0];
//	temp_int.i16 = (int16_t)(p->getRightFitLinePacket().offset * 1000);
//	m_CAN_Packet.data[6] = temp_int.b[1];
//	m_CAN_Packet.data[7] = temp_int.b[0];
//	CAN2_TransmitMsg(m_CAN_Packet);
//}

//void Terminal::Push(UltrasonicObstaclePercption p)
//{
//	CAN_Packet m_CAN_Packet;
//	uint16_t temp_uint16;

//	m_CAN_Packet.id = 0x44A;
//	m_CAN_Packet.length = 8;

//	m_CAN_Packet.data[0] = (uint8_t)p.UltrasonicLocationStatus;
//	m_CAN_Packet.data[1] = 0;
//	m_CAN_Packet.data[2] = 0;
//	m_CAN_Packet.data[3] = 0;

//	m_CAN_Packet.data[4] = (uint8_t)(p.getPositionListLength()  & 0xff);
//	m_CAN_Packet.data[5] = (uint8_t)(p.getLocationListLength()  & 0xff);
//	m_CAN_Packet.data[6] = (uint8_t)(p.getLeftEdgeListLength()  & 0xff);
//	m_CAN_Packet.data[7] = (uint8_t)(p.getRightEdgeListLength() & 0xff);
//	CAN2_TransmitMsg(m_CAN_Packet);

//	m_CAN_Packet.id = 0x44D;
//	temp_uint16 = p.getFrontObstacleDistance().distance * 10000;
//	m_CAN_Packet.data[0] = (uint8_t)( temp_uint16       & 0xff);
//	m_CAN_Packet.data[1] = (uint8_t)((temp_uint16 >> 8) & 0xff);
//	m_CAN_Packet.data[2] = (uint8_t)p.getFrontObstacleDistance().region;
//	m_CAN_Packet.data[3] = (uint8_t)p.getFrontObstacleDistance().status;

//	temp_uint16 = p.getRearObstacleDistance().distance * 10000;
//	m_CAN_Packet.data[4] = (uint8_t)( temp_uint16       & 0xff);
//	m_CAN_Packet.data[5] = (uint8_t)((temp_uint16 >> 8) & 0xff);
//	m_CAN_Packet.data[6] = (uint8_t)p.getRearObstacleDistance().region;
//	m_CAN_Packet.data[7] = (uint8_t)p.getRearObstacleDistance().status;
//	CAN2_TransmitMsg(m_CAN_Packet);
//}
///**************************************************************************************/
///*
// * 直接测量超声波原始信号
// * */
//void Terminal::UltrasonicSend(uint8_t id,LIN_RAM *msg)
//{
//	CAN_Packet m_CAN_Packet;
//	m_CAN_Packet.id = 0x400 | id;
//	m_CAN_Packet.length = 8;
//	if(id < 8)
//	{

//		m_CAN_Packet.data[0] =  msg[id].STP318.TOF       & 0xff;
//		m_CAN_Packet.data[1] = (msg[id].STP318.TOF >> 8) & 0xff;
//		m_CAN_Packet.data[2] = 0;
//		m_CAN_Packet.data[3] = 0;
//		m_CAN_Packet.data[4] = 0;
//		m_CAN_Packet.data[5] = 0;
//		m_CAN_Packet.data[6] =  msg[id].STP318.Status;
//		m_CAN_Packet.data[7] = 0;
//	}
//	else
//	{
//		m_CAN_Packet.data[0] =  msg[id].STP313.TOF1       & 0xff;
//		m_CAN_Packet.data[1] = (msg[id].STP313.TOF1 >> 8) & 0xff;
//		m_CAN_Packet.data[2] =  msg[id].STP313.TOF2       & 0xff;
//		m_CAN_Packet.data[3] = (msg[id].STP313.TOF2 >> 8) & 0xff;
//		m_CAN_Packet.data[4] =  msg[id].STP313.Level;
//		m_CAN_Packet.data[5] =  msg[id].STP313.Width;
//		m_CAN_Packet.data[6] =  msg[id].STP313.Status;
//		m_CAN_Packet.data[7] =  0;
//	}
//	CAN2_TransmitMsg(m_CAN_Packet);
//}
//void Terminal::UltrasonicSend(uint8_t id,Ultrasonic_Packet *msg_pk)
//{
//	CAN_Packet m_CAN_Packet;
//	uint16_t temp;

//	m_CAN_Packet.id = 0x400 | id;
//	m_CAN_Packet.length = 8;
//	if(id < 8)
//	{
//		temp = msg_pk[id].Distance1 * 100;
//		m_CAN_Packet.data[0] = (uint8_t) temp;
//		m_CAN_Packet.data[1] = (uint8_t)(temp >> 8 );
//		m_CAN_Packet.data[2] = 0;
//		m_CAN_Packet.data[3] = 0;
//		m_CAN_Packet.data[4] = 0;
//		m_CAN_Packet.data[5] = 0;
//		m_CAN_Packet.data[6] = msg_pk[id].status;
//		m_CAN_Packet.data[7] = msg_pk->Time_Tx;
//	}
//	else
//	{
//		temp = msg_pk[id].Distance1 * 100;
//		m_CAN_Packet.data[0] = (uint8_t) temp;
//		m_CAN_Packet.data[1] = (uint8_t)(temp >> 8 );
//		temp = msg_pk[id].Distance2 * 100;
//		m_CAN_Packet.data[2] = (uint8_t) temp;
//		m_CAN_Packet.data[3] = (uint8_t)(temp >> 8 );
//		m_CAN_Packet.data[4] = (uint8_t)( msg_pk[id].Level * 10 ) ;
//		m_CAN_Packet.data[5] = (uint8_t)( msg_pk[id].Width);
//		m_CAN_Packet.data[6] =  msg_pk[id].status;
//		m_CAN_Packet.data[7] =  msg_pk->Time_Tx;
//	}
//	CAN2_TransmitMsg(m_CAN_Packet);
//}
///*
// * 三角定位的短距离超声波信号
// * */
//void Terminal::UltrasonicLocationSend(uint8_t id,LIN_RAM *msg)
//{
//	CAN_Packet m_CAN_Packet;
//	m_CAN_Packet.id = 0x470 | id;
//	m_CAN_Packet.length = 8;
//	m_CAN_Packet.data[0] =  msg[id].STP318.TOF       & 0xff;
//	m_CAN_Packet.data[1] = (msg[id].STP318.TOF >> 8) & 0xff;
//	m_CAN_Packet.data[2] = 0;
//	m_CAN_Packet.data[3] = 0;
//	m_CAN_Packet.data[4] = 0;
//	m_CAN_Packet.data[5] = 0;
//	m_CAN_Packet.data[6] = msg[id].STP318.Status;
//	m_CAN_Packet.data[7] = 0;
//	CAN2_TransmitMsg(m_CAN_Packet);
//}
//void Terminal::UltrasonicLocationSend(uint8_t id,Ultrasonic_Packet *msg_pk)
//{
//	CAN_Packet m_CAN_Packet;
//	uint16_t temp;
//	m_CAN_Packet.id = 0x470 | id;
//	m_CAN_Packet.length = 8;

//	temp = msg_pk[id].Distance1 * 100;
//	m_CAN_Packet.data[0] = (uint8_t) temp ;
//	m_CAN_Packet.data[1] = (uint8_t)(temp >> 8 );
//	m_CAN_Packet.data[2] = 0;
//	m_CAN_Packet.data[3] = 0;
//	m_CAN_Packet.data[4] = 0;
//	m_CAN_Packet.data[5] = 0;
//	m_CAN_Packet.data[6] =  msg_pk[id].status;
//	m_CAN_Packet.data[7] = 0;

//	CAN2_TransmitMsg(m_CAN_Packet);
//}

///*
// * 载体坐标系的数据发送
// * */
//void Terminal::UltrasonicBodyLocationSend(uint8_t id,ObstacleLocationPacket packet)
//{
//	CAN_Packet m_CAN_Packet;
//	int16_t temp;
//	m_CAN_Packet.id = 0x480 | id;
//	m_CAN_Packet.length = 8;

//	temp = packet.Position.getX()*1000;
//	m_CAN_Packet.data[0] = (uint8_t)((temp     ) & 0xff );
//	m_CAN_Packet.data[1] = (uint8_t)((temp >> 8) & 0xff );
//	temp = packet.Position.getY()*1000;
//	m_CAN_Packet.data[2] = (uint8_t)((temp     ) & 0xff );
//	m_CAN_Packet.data[3] = (uint8_t)((temp >> 8) & 0xff );

//	m_CAN_Packet.data[4] = 0;
//	m_CAN_Packet.data[5] = 0;
//	m_CAN_Packet.data[6] = 0;
//	m_CAN_Packet.data[7] = packet.Status;

//	CAN2_TransmitMsg(m_CAN_Packet);
//}
///*
// * 地面坐标系的数据发送
// * */
//void Terminal::UltrasonicGroundLocationSend(uint8_t id,ObstacleLocationPacket packet)
//{
//	CAN_Packet m_CAN_Packet;
//	int16_t temp;
//	m_CAN_Packet.id = 0x490 | id;
//	m_CAN_Packet.length = 8;

//	temp = packet.Position.getX()*100;
//	m_CAN_Packet.data[0] = (uint8_t)((temp     ) & 0xff );
//	m_CAN_Packet.data[1] = (uint8_t)((temp >> 8) & 0xff );
//	temp = packet.Position.getY()*100;
//	m_CAN_Packet.data[2] = (uint8_t)((temp     ) & 0xff );
//	m_CAN_Packet.data[3] = (uint8_t)((temp >> 8) & 0xff );
//	m_CAN_Packet.data[4] = 0;
//	m_CAN_Packet.data[5] = 0;
//	m_CAN_Packet.data[6] = 0;
//	m_CAN_Packet.data[7] = packet.Status;

//	CAN2_TransmitMsg(m_CAN_Packet);
//}

//void Terminal::Ack(void)
//{
//	CAN_Packet m_CAN_Packet;
//	m_CAN_Packet.id = 0x416;
//	m_CAN_Packet.length = 8;
//	m_CAN_Packet.data[0] = 0x5A;
//	m_CAN_Packet.data[1] = 0xA5;
//	m_CAN_Packet.data[2] = 0;
//	m_CAN_Packet.data[3] = 0;
//	m_CAN_Packet.data[4] = 0;
//	m_CAN_Packet.data[5] = 0;
//	m_CAN_Packet.data[6] = 0;
//	m_CAN_Packet.data[7] = 0;
//	CAN2_TransmitMsg(m_CAN_Packet);
//}

//void Terminal::ParkingMsgSend(Percaption *p,float fm,float rm)
//{
//	CAN_Packet m_CAN_Packet;
//	Byte2Int temp_int;
//	Vector2d temp_v;
//	m_CAN_Packet.id = 0x441;
//	m_CAN_Packet.length = 8;

//	temp_int.u16 = (uint16_t)(p->ParkingLength * 1000);
//	m_CAN_Packet.data[0] = temp_int.b[1];
//	m_CAN_Packet.data[1] = temp_int.b[0];
//	temp_int.u16 = (uint16_t)(p->ParkingWidth * 1000);
//	m_CAN_Packet.data[2] = temp_int.b[1];
//	m_CAN_Packet.data[3] = temp_int.b[0];
//	temp_int.u16 = (uint16_t)(fm * 1000);
//	m_CAN_Packet.data[4] = temp_int.b[1];
//	m_CAN_Packet.data[5] = temp_int.b[0];
//	temp_int.u16 = (uint16_t)(rm * 1000);
//	m_CAN_Packet.data[6] = temp_int.b[1];
//	m_CAN_Packet.data[7] = temp_int.b[0];
//	CAN2_TransmitMsg(m_CAN_Packet);
//}



//void Terminal::ParkingCenterPointSend(Vector2d v)
//{
//	CAN_Packet m_CAN_Packet;
//	Byte2Int temp_int;
//	Vector2d temp_v;
//	m_CAN_Packet.id = 0x447;
//	m_CAN_Packet.length = 8;

//	temp_v = v;
//	temp_int.i16 = (int16_t)(temp_v.getX() * 100);
//	m_CAN_Packet.data[0] = temp_int.b[1];
//	m_CAN_Packet.data[1] = temp_int.b[0];
//	temp_int.i16 = (int16_t)(temp_v.getY() * 100);
//	m_CAN_Packet.data[2] = temp_int.b[1];
//	m_CAN_Packet.data[3] = temp_int.b[0];

//	m_CAN_Packet.data[4] = 0;
//	m_CAN_Packet.data[5] = 0;

//	m_CAN_Packet.data[6] = 0;
//	m_CAN_Packet.data[7] = 0;
//	CAN2_TransmitMsg(m_CAN_Packet);
//}
