/*****************************************************************************/
/* FILE NAME: chang_an_controller.h             COPYRIGHT (c) Henry Zhu 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: chang an vehicle control class     					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      December 28 2018    Initial Version                  */
/*****************************************************************************/

#ifndef CANBUS_CHANGAN_CHANGANCONTROLLER_H_
#define CANBUS_CHANGAN_CHANGANCONTROLLER_H_

#include <QMainWindow>
#include "./Common/Utils/Inc/property.h"
#include "./Interaction/CANBUS/Interface/vehicle_controller.h"

typedef enum _SteeringAngleActiveControl
{
	WaitSteeringAngleControlSingleState = 0,
	WaitFeedbackSingleState,
	WaitExistState
}SteeringAngleActiveControl;

typedef enum _GearActiveControlState
{
	WaitGearControlState = 0,
	WaitGearFeedbackSingleState,
	WaitGearExistState
}GearActiveControlState;

class ChangAnController : public VehicleController
{
public:
	ChangAnController();
	virtual ~ChangAnController();
	/**
	 * @brief initialize the vehicle controller.
	 * @return none
	 */
	void Init() override;
	/**
	 * @brief start the vehicle controller.
	 * @return true if successfully started.
	 */
	void Start() override;

	/**
	 * @brief stop the vehicle controller.
	 */
	void Stop() override;
	  /**
	   * @brief update the vehicle controller.
	   * @param command the control command
	   * @return error_code
	   */
//	void Update(ControlCommand cmd) override;
	void Update(APAControlCommand cmd) override;

	// push the command to the vehiclevoid
	void Push();

	/*** Function ***/
	// Vehicle control command function
	void VehicleContorlStep1();
	void VehicleContorlStep2();
	void VehicleContorlStep3();

	void VehicleContorlNew();
	void VehicleContorl();

	// Steeing angle control base on the angle speed
    void SteeringAngleControl(float dt);

	// Steering Angle control state machine
	void SteeringAngleControlStateMachine(uint8_t fd);

	void GearControlStateMachine(uint8_t fd);

	void EnableControl();
private:
	SteeringAngleActiveControl _steerig_angle_active_control_state;
	GearActiveControlState     _gear_active_control_state;
	/*** Send to Vehicle Messege ***/
	/* Roolling Counter */
	uint8_t _rolling_counter_torque_AEB;
	uint8_t _rolling_counter_brake_ACC;
	uint8_t _rolling_counter_steering_control;
	uint8_t _rolling_counter_gear_control;

	/* ACC */
	// current value
	uint8_t _current_target_acceleration_ACC;
	uint8_t _current_target_acceleration_enable_single;

	/* AEB */
	// current value
	uint16_t _current_target_deceleration_AEB;
	uint8_t _current_target_deceleration_enable_single;

	/* Torque */
	// current value
	uint16_t _current_torque;
	uint8_t _current_torque_enable_single;

	/* SteeringAngle */
	// actual value
	float _steering_angle_set;
	// current value
	int16_t _current_steering_angle_target;
	uint8_t _current_steering_angle_target_active_single;

	/* Gear */
	// current value
	uint8_t _current_gear;
	uint8_t _current_gear_enable_single;
	uint8_t _current_gear_valid_single;
};

#endif /* CANBUS_CHANGAN_CHANGANCONTROLLER_H_ */
