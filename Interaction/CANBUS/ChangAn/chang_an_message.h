/*****************************************************************************/
/* FILE NAME: chang_an_message.h                COPYRIGHT (c) Henry Zhu 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: chang an vehicle message class     					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      December 28 2018    Initial Version                  */
/*****************************************************************************/

#ifndef CANBUS_CHANGAN_CHANG_AN_MESSAGE_H_
#define CANBUS_CHANGAN_CHANG_AN_MESSAGE_H_

#include <QMainWindow>
#include "./Common/Utils/Inc/property.h"
#include "./Interaction/CANBUS/Interface/message_manager.h"

#define V_M_S 0.015625

class ChangAnMessage : public MessageManager
{
public:
	ChangAnMessage();
	virtual ~ChangAnMessage();

	void Init() override;
    void Parse(const uint32_t id,const uint8_t *data,const uint32_t lenght) override;
	// EPS
	uint8_t getEPS_Failed();
	void    setEPS_Failed(uint8_t value);
	Property<ChangAnMessage,uint8_t,READ_WRITE> EPS_Failed;

	uint8_t getAPA_EPAS_Failed();
	void    setAPA_EPAS_Failed(uint8_t value);
	Property<ChangAnMessage,uint8_t,READ_WRITE> APA_EPAS_Failed;

	uint8_t getAPA_ControlFeedback();
	void    setAPA_ControlFeedback(uint8_t value);
	Property<ChangAnMessage,uint8_t,READ_WRITE> APA_ControlFeedback;

	uint8_t getTorqueSensorStatus();
	void    setTorqueSensorStatus(uint8_t value);
	Property<ChangAnMessage,uint8_t,READ_WRITE> TorqueSensorStatus;

	float getSteeringTorque();
	void  setSteeringTorque(float value);
	Property<ChangAnMessage,float,READ_WRITE> SteeringTorque;

	uint8_t getSAS_Failure();
	void    setSAS_Failure(uint8_t value);
	Property<ChangAnMessage,uint8_t,READ_WRITE> SAS_Failure;

	// ESP
	uint8_t getESP_QDC_ACC();
	void    setESP_QDC_ACC(uint8_t value);
	Property<ChangAnMessage,uint8_t,READ_WRITE> ESP_QDC_ACC;

	// EMS
	uint8_t getEMS_QEC_ACC();
	void    setEMS_QEC_ACC(uint8_t value);
	Property<ChangAnMessage,uint8_t,READ_WRITE> EMS_QEC_ACC;

	// EMS
	uint8_t getACM_APA_RequestEnable();
	void    setACM_APA_RequestEnable(uint8_t value);
	Property<ChangAnMessage,uint8_t,READ_WRITE> ACM_APA_RequestEnable;
private:
	/*** Receive messege form vehicle ***/
	//
	uint8_t _eps_failed;
	uint8_t _apa_epas_failed;
	uint8_t _apa_control_feedback;
	uint8_t _torque_sensor_status;
	float   _steering_torque;

	// SAS
	uint8_t  _sas_failure;

	// TCU
	uint8_t _gear_actual;

	// ESP
	uint8_t esp_qdc_acc;

	// EMS
	uint8_t ems_qec_acc;

	// ACM
	uint8_t _acm_apa_request_enable;

};

#endif /* CANBUS_CHANGAN_CHANG_AN_MESSAGE_H_ */
