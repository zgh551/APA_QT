/*****************************************************************************/
/* FILE NAME: lon_control.h                     COPYRIGHT (c) Henry Zhu 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: this class using to interpolation data  			         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      January 3 2019      Initial Version                  */
/* 1.0	 Henry Zhu      July   30 2019      Add Dongfeng Control Function    */
/*****************************************************************************/

#ifndef LONCONTROL_LON_CONTROL_H_
#define LONCONTROL_LON_CONTROL_H_

//#include "derivative.h"
//#include "property.h"
#include "Control/Interface/controller.h"

#include "math.h"
#include "Common/Math/vector_2d.h"
#include "Common/Math/interpolation.h"
#include "Common/VehicleState/GeometricTrack/geometric_track.h"
#include "Common/Configure/Configs/vehilce_config.h"

/**************************速度控制******************************/
#define MAX_POSITION            ( 0.9 ) // 速度控制上限点
#define MIN_POSITION            ( 0.4 ) // 速度控制下限点
#define MAX_VELOCITY	  		( 1.0 ) // 直线段的速度
#define MIN_VELOCITY	      	( 0.3 ) // 曲线段的速度
/**************************加速度控制******************************/
#define START_ACC               ( 0.2f  ) // 车辆起步时的正向加速度
#define DT                      ( 0.02f ) // 控制时间间隔

typedef enum _Lon_VelocityControlState
{
	VelocityStartStatus = 0,
	WaitVelocityStableStatus
}Lon_VelocityControlState;

class LonControl:public Controller
{
public:
	LonControl();
	virtual ~LonControl();

	void Init() override;

	void Proc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid) override;

	void VelocityProc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid);

	void AccProc(MessageManager *msg,VehicleController *ctl,PID *acc_pid);

	void VelocityLookupProc(MessageManager *msg,VehicleController *ctl,PID *start_velocity_pid,PID *velocity_pid);
	void VelocityLookupProc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid);

	void DistanceProc(MessageManager *msg,VehicleController *ctl);

	float VelocityPlanningControl(float distance);
	float VelocityControl(float distance,float velocity);
	float AcceleratePlanningControl(float cur_velocity,float stop_distance);
	float AccAcceleratePlanningControl(float cur_velocity,float stop_distance);

	float getControlStateFlag();
	void  setControlStateFlag(float value);
	Property<LonControl,float,READ_WRITE> ControlStateFlag;
private:
	float _max_position,_min_position;
	float _max_velocity,_min_velocity;

	float _vehicle_velocity;	// 车辆速度
	float _throttle_lowerbound; // 油门最低边界
	float _brake_lowerbound;    // 制动最低边界
	float _calibration_value;   // 校准值

	VehilceConfig _lon_VehilceConfig;
	Interpolation _lon_Interpolation;

	Lon_VelocityControlState _lon_velocity_control_state;
	GearStatus _current_gear,_last_gear;
	float _current_velocity,_last_velocity;
	uint8_t _control_state_flag;
	float _target_velocity;
	float _delta_velocity;
	float _distance_update_distance_value;
	float _variable_distance_value;
	float _delta_distance;
	float _distance_update_pulse_value;
	float _vehicle_stop_acc;
	float _vehicle_stop_acc_acc;
	float _vehicle_slow_down_acc;
	uint16_t _err_velocity_cnt;
};

#endif /* LONCONTROL_LON_CONTROL_H_ */
