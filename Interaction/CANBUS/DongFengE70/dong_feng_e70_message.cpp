/*
 * dong_feng_e70_message.cpp
 *
 *  Created on: 2019年6月20日
 *      Author: Henry Zhu
 */

#include "./Interaction/CANBUS/DongFengE70/dong_feng_e70_message.h"

DongFengE70Message::DongFengE70Message() {
	// TODO Auto-generated constructor stub

}

DongFengE70Message::~DongFengE70Message() {
	// TODO Auto-generated destructor stub
}

void DongFengE70Message::Init()
{
	VCU_APA_ControlStatus.setContainer(this);
	VCU_APA_ControlStatus.getter(&DongFengE70Message::getVCU_APA_ControlStatus);
	VCU_APA_ControlStatus.setter(&DongFengE70Message::setVCU_APA_ControlStatus);

	EPS_AvailabStatus.setContainer(this);
	EPS_AvailabStatus.getter(&DongFengE70Message::getEPS_AvailabStatus);
	EPS_AvailabStatus.setter(&DongFengE70Message::setEPS_AvailabStatus);

	ESC_APA_EnableStatus.setContainer(this);
	ESC_APA_EnableStatus.getter(&DongFengE70Message::getESC_APA_EnableStatus);
	ESC_APA_EnableStatus.setter(&DongFengE70Message::setESC_APA_EnableStatus);
}

void DongFengE70Message::Parse(const uint32_t id,const uint8_t *data,const uint32_t lenght)
{
//    switch(id)
//	{
//		case 0x122:
//			LatAcc = (uint16_t)(((data[0] & 0x0f) << 8) | data[1] ) * 0.1 - 204.8;
//			YawRate = (((data[2] & 0x07) << 8 ) | data[3]) * 0.03 - 15.36;
//			break;

//		case 0xFA:
//			_esc_apa_enable_status = (uint8_t)((data[1] >> 6) & 0x03);
//			break;

//		case 0x165:
//			_eps_availab_status = (uint8_t)((data[0] >> 6) & 0x03);
//			break;

//		case 0x176://VCU 10
//			_vcu_apa_control_st = (uint8_t)(( data[0] >> 5 ) & 0x03);
//			switch((uint8_t)( data[0] & 0x07))
//			{
//				case 1:
//					Gear = Parking;
//					break;

//				case 2:
//					Gear = Reverse;
//					break;

//				case 3:
//					Gear = Neutral;
//					break;

//				case 4:
//					Gear = Drive;
//					break;

//				default:
//					Gear = None;
//					break;
//			}
//			break;

//		case 0xA0:
////			VehicleMiddleSpeed      = (uint16_t)((data[6] << 8) | data[7]) * V_M_S;
////			VehicleMiddleSpeedValid = (uint8_t)((data[3] >> 1) & 0x01);
//			break;

//		case 0xA3://ESC
//			if( 0 == ((data[0] >> 7) & 0x01))
//			{
//				WheelSpeedFrontLeft  = ((uint16_t)(((data[0] & 0x7F) << 8) | data[1])) * V_M_S;
//			}
//			if( 0 == ((data[2] >> 7) & 0x01))
//			{
//				WheelSpeedFrontRight  = ((uint16_t)(((data[2] & 0x7F) << 8) | data[3])) * V_M_S;
//			}
//			if( 0 == ((data[4] >> 7) & 0x01))
//			{
//				WheelSpeedRearLeft  = ((uint16_t)(((data[4] & 0x7F) << 8) | data[5])) * V_M_S;
//			}
//			if( 0 == ((data[6] >> 7) & 0x01))
//			{
//				WheelSpeedRearRight  = ((uint16_t)(((data[6] & 0x7F) << 8) | data[7])) * V_M_S;
//			}
//			break;

//			case 0xA6://ESC
//				if( 0 == ((data[5] >> 7) & 0x01))
//				{
//					WheelPulseFrontLeft  = (uint16_t)(((data[0] & 0xff) << 2) | ((data[1] >> 6) & 0x03));
//				}
//				if( 0 == ((data[5] >> 6) & 0x01))
//				{
//					WheelPulseFrontRight  = (uint16_t)(((data[1] & 0x3f) << 4) | ((data[2] >> 4) & 0x0f));
//				}
//				if( 0 == ((data[5] >> 5) & 0x01))
//				{
//					WheelPulseRearLeft = (uint16_t)(((data[2] & 0x0f) << 6) | ((data[3] >> 2) & 0x3f));
//				}
//				if( 0 == ((data[5] >> 4) & 0x01))
//				{
//					WheelPulseRearRight  = (uint16_t)(((data[3] & 0x03) << 8) |   data[4]              );
//				}
//				if( 0 == ((data[5] >> 3) & 0x01))
//				{
//					LonAcc = (uint16_t)(((data[5] & 0x07) << 8) | data[6]) * 0.03 - 15.36;
//				}
//				break;

//			case 0xA5:
//				SteeringAngle = (float)(((int16_t)((data[0] << 8) | data[1])) * 0.1);
//				SteeringAngleRate = (uint16_t)(data[2] * 4);
////				SAS_Failure = (uint8_t)( data[3] >> 7 ) & 0x01;
//				break;

//			default:
//				break;
//	}
}


uint8_t DongFengE70Message::getVCU_APA_ControlStatus()			   {return _vcu_apa_control_st ;}
void    DongFengE70Message::setVCU_APA_ControlStatus(uint8_t value){_vcu_apa_control_st = value;}

uint8_t DongFengE70Message::getEPS_AvailabStatus()			   {return _eps_availab_status ;}
void    DongFengE70Message::setEPS_AvailabStatus(uint8_t value){_eps_availab_status = value;}

uint8_t DongFengE70Message::getESC_APA_EnableStatus()			  {return _esc_apa_enable_status ;}
void    DongFengE70Message::setESC_APA_EnableStatus(uint8_t value){_esc_apa_enable_status = value;}
