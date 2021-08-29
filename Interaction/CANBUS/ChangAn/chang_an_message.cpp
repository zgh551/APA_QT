/*****************************************************************************/
/* FILE NAME: chang_an_message.cpp              COPYRIGHT (c) Henry Zhu 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: chang an vehicle message class     					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      December 28 2018    Initial Version                  */
/*****************************************************************************/

#include <./Interaction/CANBUS/ChangAn/chang_an_message.h>

ChangAnMessage::ChangAnMessage() {
	Init();
}

ChangAnMessage::~ChangAnMessage() {

}

void ChangAnMessage::Init()
{
	// EPS
	EPS_Failed.setContainer(this);
	EPS_Failed.getter(&ChangAnMessage::getEPS_Failed);
	EPS_Failed.setter(&ChangAnMessage::setEPS_Failed);

	APA_EPAS_Failed.setContainer(this);
	APA_EPAS_Failed.getter(&ChangAnMessage::getAPA_EPAS_Failed);
	APA_EPAS_Failed.setter(&ChangAnMessage::setAPA_EPAS_Failed);

	APA_ControlFeedback.setContainer(this);
	APA_ControlFeedback.getter(&ChangAnMessage::getAPA_ControlFeedback);
	APA_ControlFeedback.setter(&ChangAnMessage::setAPA_ControlFeedback);

	TorqueSensorStatus.setContainer(this);
	TorqueSensorStatus.getter(&ChangAnMessage::getTorqueSensorStatus);
	TorqueSensorStatus.setter(&ChangAnMessage::setTorqueSensorStatus);

	SteeringTorque.setContainer(this);
	SteeringTorque.getter(&ChangAnMessage::getSteeringTorque);
	SteeringTorque.setter(&ChangAnMessage::setSteeringTorque);

	SAS_Failure.setContainer(this);
	SAS_Failure.getter(&ChangAnMessage::getSAS_Failure);
	SAS_Failure.setter(&ChangAnMessage::setSAS_Failure);
	//ESP
	ESP_QDC_ACC.setContainer(this);
	ESP_QDC_ACC.getter(&ChangAnMessage::getESP_QDC_ACC);
	ESP_QDC_ACC.setter(&ChangAnMessage::setESP_QDC_ACC);
	//EMS
	EMS_QEC_ACC.setContainer(this);
	EMS_QEC_ACC.getter(&ChangAnMessage::getEMS_QEC_ACC);
	EMS_QEC_ACC.setter(&ChangAnMessage::setEMS_QEC_ACC);

	//ACM
	ACM_APA_RequestEnable.setContainer(this);
	ACM_APA_RequestEnable.getter(&ChangAnMessage::getACM_APA_RequestEnable);
	ACM_APA_RequestEnable.setter(&ChangAnMessage::setACM_APA_RequestEnable);


}

void ChangAnMessage::Parse(const uint32_t id,const uint8_t *dat,const uint32_t lenght)
{
	switch(id)
	{
		case 0x24C:
			_acm_apa_request_enable = (uint8_t)((dat[4] >> 6) & 0x03);
			break;

		case 0x2A0://eps status
			EPS_Failed = (uint8_t)((dat[1] >> 7) & 0x01);
			APA_EPAS_Failed = (uint8_t)((dat[1] >> 1) & 0x01);
			TorqueSensorStatus = (uint8_t)( dat[1] & 0x01 );
			APA_ControlFeedback = (uint8_t)((dat[3] >> 5) & 0x01);
			SteeringTorque = (float)(((int8_t)dat[2]) * 0.17);
			break;

		case 0x2A3:
			EPS_Failed = (uint8_t)((dat[2] >> 7) & 0x01);
			APA_EPAS_Failed = (uint8_t)((dat[2] >> 1) & 0x01);
			TorqueSensorStatus = (uint8_t)( dat[2] & 0x01 );
			APA_ControlFeedback = (uint8_t)((dat[3] >> 5) & 0x01);
			SteeringTorque = (float)(((int8_t)dat[4]) * 0.17);
			break;

		case 0x20B:// wheel speed
			switch((uint8_t)(dat[0] >> 5) & 0x03)
			{
			case 0:
				WheelSpeedDirection  = Forward;
				break;

			case 1:
				WheelSpeedDirection  = Backward;
				break;

			case 2:
				WheelSpeedDirection  = StandStill;
				break;

			case 3:
				WheelSpeedDirection  = Invalid;
				break;

			default:
				WheelSpeedDirection  = Invalid;
				break;
			}

			WheelSpeedRearRight  = ((uint16_t)(((dat[0] & 0x1F) << 8) | dat[1])) * V_M_S;
			WheelSpeedRearLeft   = ((uint16_t)(((dat[2] & 0x1F) << 8) | dat[3])) * V_M_S;
			WheelSpeedFrontRight = ((uint16_t)(((dat[4] & 0x1F) << 8) | dat[5])) * V_M_S;
			WheelSpeedFrontLeft  = ((uint16_t)(((dat[6] & 0x1F) << 8) | dat[7])) * V_M_S;
			break;

		case 0x25B://Wheel speed pulse
			switch((uint8_t)(dat[2] & 0x03))
			{
				case 0:
					WheelPulseDirection  = Forward;
					break;

				case 1:
					WheelPulseDirection  = Backward;
					break;

				case 2:
					WheelPulseDirection  = StandStill;
					break;

				case 3:
					WheelPulseDirection  = Invalid;
					break;

				default:
					WheelPulseDirection  = Invalid;
					break;
			}
			WheelPulseRearRight  = (uint8_t)dat[4];
			WheelPulseRearLeft   = (uint8_t)dat[5];
			WheelPulseFrontRight = (uint8_t)dat[6];
			WheelPulseFrontLeft  = (uint8_t)dat[7];
			break;

		case 0x26B:// TCU GEAR
			switch((uint8_t)(dat[1] & 0x0f))
			{
				case 0:
					Gear = Neutral;
					break;

				case 1:
				case 2:
				case 3:
				case 4:
				case 5:
				case 6:
					Gear = Drive;
					break;

				case 9:
					Gear = Reverse;
					break;

				case 10:
					Gear = Parking;
					break;

				default:
					Gear = None;
					break;
			}
			break;

		case 0x27A:// ESP
			ESP_QDC_ACC = (uint8_t)( (dat[5] >> 1) & 0x03);
			break;

		case 0x27B:
			LonAcc = ((uint16_t)((dat[3] << 2) | (dat[4] >> 6)))*0.03125 - 16;
			LatAcc = dat[2] * 0.1f - 12.7f;
			YawRate = (((dat[4] & 0x3f) << 8) | dat[5]) * 0.01 - 81.91;
			break;

		case 0x26D:// EMS
			EMS_QEC_ACC = (uint8_t)( (dat[3] >> 1) & 0x03);
			break;

		case 0x183://SAS
			SteeringAngle = (float)(((int16_t)((dat[3] << 8) | dat[4])) * 0.1);
			SteeringAngleRate = (uint16_t)(dat[0] * 4);
			SAS_Failure = (uint8_t)( dat[5] >> 6 ) & 0x01;
			break;

		default:

			break;
	}
}

/// EPS
uint8_t ChangAnMessage::getEPS_Failed()             {return  _eps_failed;}
void    ChangAnMessage::setEPS_Failed(uint8_t value){_eps_failed = value;}

uint8_t ChangAnMessage::getAPA_EPAS_Failed()             {return  _apa_epas_failed;}
void    ChangAnMessage::setAPA_EPAS_Failed(uint8_t value){_apa_epas_failed = value;}

uint8_t ChangAnMessage::getAPA_ControlFeedback()             {return _apa_control_feedback ;}
void    ChangAnMessage::setAPA_ControlFeedback(uint8_t value){_apa_control_feedback = value;}

uint8_t ChangAnMessage::getTorqueSensorStatus()             {return _torque_sensor_status ;}
void    ChangAnMessage::setTorqueSensorStatus(uint8_t value){_torque_sensor_status = value;}

float ChangAnMessage::getSteeringTorque()             {return _steering_torque ;}
void  ChangAnMessage::setSteeringTorque(float value){_steering_torque = value;}

uint8_t ChangAnMessage::getSAS_Failure()			 {return _sas_failure ;}
void    ChangAnMessage::setSAS_Failure(uint8_t value){_sas_failure = value;}

uint8_t ChangAnMessage::getESP_QDC_ACC() 			 {return esp_qdc_acc ;}
void    ChangAnMessage::setESP_QDC_ACC(uint8_t value){esp_qdc_acc = value;}

uint8_t ChangAnMessage::getEMS_QEC_ACC()             {return ems_qec_acc ;}
void    ChangAnMessage::setEMS_QEC_ACC(uint8_t value){ems_qec_acc = value;}

uint8_t ChangAnMessage::getACM_APA_RequestEnable()             {return _acm_apa_request_enable ;}
void    ChangAnMessage::setACM_APA_RequestEnable(uint8_t value){_acm_apa_request_enable = value;}
