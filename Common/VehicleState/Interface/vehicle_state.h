/*****************************************************************************/
/* FILE NAME: vehicle_state.h                   COPYRIGHT (c) Henry Zhu 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: return the vehicle position ans attitude state		         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      December 29 2018    Initial Version                  */
/*****************************************************************************/

#ifndef VEHICLESTATE_VEHICLESTATE_H_
#define VEHICLESTATE_VEHICLESTATE_H_

#include <QMainWindow>
#include "math.h"
#include "./Common/Utils/Inc/property.h"
#include "./Common/Math/vector_2d.h"
#include "./Common/Configure/Configs/vehilce_config.h"
#include "./Interaction/CANBUS/Interface/message_manager.h"

class VehicleState
{
public:
	VehicleState();
	virtual ~VehicleState();

	void Init(void);

	virtual void VelocityUpdate(MessageManager *msg,float dt) = 0;
	virtual void PulseUpdate(MessageManager *msg) = 0;
	virtual void PulseTrackUpdate(MessageManager *msg) = 0;


	Vector2d getPosition();
	void     setPosition(Vector2d value);
	Property<VehicleState,Vector2d,READ_WRITE> Position;

	float getYaw();
	void  setYaw(float value);
	Property<VehicleState,float,READ_WRITE> Yaw;

	float getLinearVelocity();
	void  setLinearVelocity(float value);
	Property<VehicleState,float,READ_WRITE> LinearVelocity;

	float getLinearRate();
	void  setLinearRate(float value);
	Property<VehicleState,float,READ_WRITE> LinearRate;

	float getTurnningRadius();
	void  setTurnningRadius(float value);
	Property<VehicleState,float,READ_WRITE> TurnningRadius;

	float getPulseUpdateVelocity();
	void  setPulseUpdateVelocity(float value);
	Property<VehicleState,float,READ_WRITE> PulseUpdateVelocity;

	float getAccUpdateVelocity();
	void  setAccUpdateVelocity(float value);
	Property<VehicleState,float,READ_WRITE> AccUpdateVelocity;
protected:
	Vector2d _position;
	float    _yaw;
	float _linear_velocity;
	float _linear_rate;
	float _turnning_radius;

	float _pulse_update_velocity;
	float _acc_update_velocity;
};

#endif /* VEHICLESTATE_VEHICLESTATE_H_ */
