/*****************************************************************************/
/* FILE NAME: vehicle_state.cp                  COPYRIGHT (c) Henry Zhu 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: return the vehicle position ans attitude state		         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      December 29 2018    Initial Version                  */
/*****************************************************************************/

#include "../Interface/vehicle_state.h"

VehicleState::VehicleState() {

	Init();
}

VehicleState::~VehicleState() {

}

void VehicleState::Init()
{
	Position.setContainer(this);
	Position.getter(&VehicleState::getPosition);
	Position.setter(&VehicleState::setPosition);

	Yaw.setContainer(this);
	Yaw.getter(&VehicleState::getYaw);
	Yaw.setter(&VehicleState::setYaw);

	LinearVelocity.setContainer(this);
	LinearVelocity.getter(&VehicleState::getLinearVelocity);
	LinearVelocity.setter(&VehicleState::setLinearVelocity);

	LinearRate.setContainer(this);
	LinearRate.getter(&VehicleState::getLinearRate);
	LinearRate.setter(&VehicleState::setLinearRate);

	TurnningRadius.setContainer(this);
	TurnningRadius.getter(&VehicleState::getTurnningRadius);
	TurnningRadius.setter(&VehicleState::setTurnningRadius);

	PulseUpdateVelocity.setContainer(this);
	PulseUpdateVelocity.getter(&VehicleState::getPulseUpdateVelocity);
	PulseUpdateVelocity.setter(&VehicleState::setPulseUpdateVelocity);

	AccUpdateVelocity.setContainer(this);
	AccUpdateVelocity.getter(&VehicleState::getAccUpdateVelocity);
	AccUpdateVelocity.setter(&VehicleState::setAccUpdateVelocity);
}

Vector2d VehicleState::getPosition()              { return  _position;}
void     VehicleState::setPosition(Vector2d value){ _position = value;}

float VehicleState::getYaw()           { return _yaw;}
void  VehicleState::setYaw(float value){_yaw = value;}

float VehicleState::getLinearVelocity()           { return _linear_velocity;}
void  VehicleState::setLinearVelocity(float value){_linear_velocity = value;}

float VehicleState::getLinearRate()           { return _linear_rate;}
void  VehicleState::setLinearRate(float value){_linear_rate = value;}

float VehicleState::getTurnningRadius()           { return _turnning_radius;}
void  VehicleState::setTurnningRadius(float value){_turnning_radius = value;}

float VehicleState::getPulseUpdateVelocity()           { return _pulse_update_velocity;}
void  VehicleState::setPulseUpdateVelocity(float value){_pulse_update_velocity = value;}

float VehicleState::getAccUpdateVelocity()           { return _acc_update_velocity;}
void  VehicleState::setAccUpdateVelocity(float value){_acc_update_velocity = value;}
