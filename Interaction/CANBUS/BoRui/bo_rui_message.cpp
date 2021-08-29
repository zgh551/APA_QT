/*
 * bo_rui_message.cpp
 *
 *  Created on: 2019年3月15日
 *      Author: Henry Zhus
 */

#include "./Interaction/CANBUS/BoRui/bo_rui_message.h"

CRC8 crc8(CRC8::eAUTOSAR);

BoRuiMessage::BoRuiMessage() {
	// TODO Auto-generated constructor stub
//	CRC8 crc8 = CRC8(CRC8::eAUTOSAR);
}

BoRuiMessage::~BoRuiMessage() {
	// TODO Auto-generated destructor stub
}

void BoRuiMessage::Init()
{

}

void BoRuiMessage::Parse(const uint32_t id,const uint8_t *dat,const uint32_t lenght)
{
//	uint8_t crc_temp,i;
//	uint8_t dat_temp[7];
	switch(id)
	{
		case 0x2A0://eps status

			break;

		case 0x122:// wheel speed
			WheelSpeedFrontRight = ((uint16_t)( (dat[0] << 5) | (dat[1] >> 3))) * V_M_S;
			WheelSpeedFrontLeft  = ((uint16_t)( (dat[2] << 5) | (dat[3] >> 3))) * V_M_S;
			break;

		case 0x123:// wheel speed
            switch(static_cast<uint8_t>(dat[1] >> 1) & 0x03)
			{
				case 0:
					WheelSpeedDirection = StandStill;
					break;
				case 1:
					WheelSpeedDirection = Forward;
					break;
				case 2:
					WheelSpeedDirection = Backward;
					break;
				case 3:
					WheelSpeedDirection = Invalid;
					break;

				default:
					WheelSpeedDirection = Invalid;
					break;
			}
			WheelSpeedRearRight  = ((uint16_t)( (dat[0] << 5) | (dat[1] >> 3))) * V_M_S;
			WheelSpeedRearLeft   = ((uint16_t)( (dat[2] << 5) | (dat[3] >> 3))) * V_M_S;
			break;

		case 0x124://Wheel speed pulse
//			crc_temp = crc8.crcCompute((uint8_t*)dat, 7);
//			if(crc_temp == dat[7])
//			{
				WheelPulseFrontLeft  = (uint16_t)(( (dat[0] << 4) | (dat[1] >> 4)) & 0x0fff);
				WheelPulseFrontRight = (uint16_t)(( (dat[1] << 8) |  dat[2]      ) & 0x0fff);
				WheelPulseRearLeft   = (uint16_t)(( (dat[3] << 4) | (dat[4] >> 4)) & 0x0fff);
				WheelPulseRearRight  = (uint16_t)(( (dat[4] << 8) |  dat[5]      ) & 0x0fff);
//			}
			break;

//		case 0x113:// TCU GEAR
//			switch((uint8_t)(dat[5] >> 3))
//			{
//				case 0:
//					Gear = None;
//					break;
//
//				case 1:
//					Gear = Parking;
//					break;
//
//				case 2:
//					Gear = Reverse;
//					break;
//
//				case 3:
//					Gear = Neutral;
//					break;
//
//				default:
//					Gear = Drive;
//					break;
//			}
//			break;

		case 0x165:
			switch((uint8_t)(dat[1] & 0x1f))
			{
				case 0:
					Gear = None;
					break;

				case 1:
					Gear = Parking;
					break;

				case 2:
					Gear = Reverse;
					break;

				case 3:
					Gear = Neutral;
					break;

				case 4:
					Gear = Drive;
					break;

				default:
					Gear = None;
					break;
			}
			break;
		case 0x125:// ESC

			break;

		case 0x0E0://SAS
//			for(i=0;i<7;i++)
//			{
//				dat_temp[i] = dat[i];
//			}
//			dat_temp[4] = 0x4A;
//			crc_temp = crc8.crcCompute(dat_temp, 7);
//			if(crc_temp == dat[7])
//			{
				SteeringAngle     = ((int16_t)((dat[0] << 8) | dat[1])) * 0.1f;
				SteeringAngleRate = dat[2]*4.0f;
//			}
			break;

		default:

			break;
	}
}
