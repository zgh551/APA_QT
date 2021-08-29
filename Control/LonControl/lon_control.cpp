/*****************************************************************************/
/* FILE NAME: lon_control.cpp                   COPYRIGHT (c) Henry Zhu 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: this class using to interpolation data  			         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      January 3 2019      Initial Version                  */
/* 1.0	 Henry Zhu      July   30 2019      Add Dongfeng Control Function    */
/*****************************************************************************/

#include "Control/LonControl/lon_control.h"

LonControl::LonControl() {
	ControlStateFlag.setContainer(this);
	ControlStateFlag.getter(&LonControl::getControlStateFlag);
	ControlStateFlag.setter(&LonControl::setControlStateFlag);

	Init();
}

LonControl::~LonControl() {

}

void LonControl::Init()
{
	_max_position = MAX_POSITION;// 速度控制上限点
	_min_position = MIN_POSITION;// 速度控制下限点
	_max_velocity = MAX_VELOCITY;// 直线段的速度
	_min_velocity = MIN_VELOCITY;// 曲线段的速度

	_throttle_lowerbound = 30;//Nm
	_brake_lowerbound = -0.123;// m/s2

	_control_state_flag = 0;
	_lon_velocity_control_state = VelocityStartStatus;

	_delta_velocity = START_ACC * DT;

	_err_velocity_cnt = 0;
}

//根据距离规划速度
float LonControl::VelocityPlanningControl(float distance)
{
	if(distance > _max_position)
	{
		return _max_velocity;
	}
	else if(distance > _min_position)
	{
		return _min_velocity + (_max_velocity - _min_velocity)*(distance - _min_position)/(_max_position - _min_position);
	}
	else if(distance > 0.05)
	{
		return distance * _min_velocity / _min_position ;
	}
	else
	{
		return 0;
	}
}


float LonControl::VelocityControl(float distance,float velocity)
{
	float temp_v;
	temp_v = VelocityPlanningControl(distance);
	return velocity < temp_v ? velocity :temp_v;
}

void LonControl::Proc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid)
{
	if(ctl->VelocityEnable)
	{
		_vehicle_velocity = (msg->WheelSpeedRearLeft + msg->WheelSpeedRearRight) * 0.5;
//		velocity_pid->Desired = ctl->Velocity;
        velocity_pid->setDesired(VelocityControl(ctl->Distance,ctl->Velocity));
		ctl->Acceleration = velocity_pid->pidUpdateIntegralSeparation(_vehicle_velocity);
        if((ctl->getAcceleration() < 1.0e-6) && (velocity_pid->getDesired() < 1.0e-6))
		{
			ctl->Acceleration = -0.5;
		}
	}
}

void LonControl::VelocityProc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid)
{
	if(ctl->VelocityEnable)
	{
		_vehicle_velocity = (msg->WheelSpeedRearLeft + msg->WheelSpeedRearRight) * 0.5;
        velocity_pid->setDesired(ctl->Velocity);
		ctl->TargetAcceleration = velocity_pid->pidUpdateIntegralSeparation(_vehicle_velocity);
	}
}

void LonControl::AccProc(MessageManager *msg,VehicleController *ctl,PID *acc_pid)
{
	if(ctl->TorqueEnable)
	{
		if(ctl->TargetAcceleration < 0)
		{
			ctl->Torque = 0;
			ctl->Acceleration = ctl->TargetAcceleration;
			ctl->AccelerationEnable = 1;
		}
		else
		{
            acc_pid->setDesired(ctl->TargetAcceleration);
			if(msg->Gear == Drive)
			{
				ctl->Torque = acc_pid->pidUpdateIntegralSeparation(msg->LonAcc);
			}
			else if(msg->Gear == Reverse)
			{
				ctl->Torque = acc_pid->pidUpdateIntegralSeparation(-msg->LonAcc);
			}
			ctl->Acceleration = 0;
			ctl->AccelerationEnable = 0;
		}
	}
}

void LonControl::VelocityLookupProc(MessageManager *msg,VehicleController *ctl,PID *start_velocity_pid,PID *velocity_pid)
{
	_current_gear = msg->getGear();

//	_target_velocity = ctl->Velocity;//纯速度控制
	_target_velocity = VelocityControl(ctl->Distance,ctl->Velocity);//距离速度控制

	_vehicle_velocity = msg->VehicleMiddleSpeed;
	_current_velocity = _vehicle_velocity;

	switch(_lon_velocity_control_state)
	{
		case VelocityStartStatus:
			if(_current_gear != _last_gear)//换挡
			{
				_control_state_flag = 0xAA;
				_lon_velocity_control_state = WaitVelocityStableStatus;
			}
			else if(_target_velocity < 1.0e-6f)//目标速度接近0
			{
				_control_state_flag = 0xAA;
				_lon_velocity_control_state = WaitVelocityStableStatus;
			}
			break;

		case WaitVelocityStableStatus:
			if( (fabs(_vehicle_velocity - _target_velocity) < 0.1f) && (_target_velocity > 0.1F))//接近目标速度
			{
				_control_state_flag = 0x55;
				_lon_velocity_control_state = VelocityStartStatus;
			}
			else if(_current_velocity < _last_velocity)//速度变慢
			{
				_control_state_flag = 0x55;
				_lon_velocity_control_state = VelocityStartStatus;
			}
			break;

		default:

			break;
	}
	_last_gear = _current_gear;
	_last_velocity = _current_velocity;

	if(ctl->VelocityEnable)
	{
//		if(0x55 == _control_state_flag)//启动后速度控制
//		{
            velocity_pid->setDesired(_target_velocity);
			ctl->TargetAcceleration = velocity_pid->pidUpdate(_vehicle_velocity);
//		}
//		else if(0xAA == _control_state_flag)//启动前
//		{
//			start_velocity_pid->Desired = _target_velocity;
//			ctl->TargetAcceleration = start_velocity_pid->pidUpdate(_vehicle_velocity);
//		}

		if(ctl->TargetAcceleration > 0)
		{
			ctl->TorqueEnable       = 1;
			_calibration_value = _lon_Interpolation.Interpolation2D(ctl->TargetAcceleration, _vehicle_velocity,
																	_lon_VehilceConfig.AccelerateTable, _lon_VehilceConfig.AccNum,
																	_lon_VehilceConfig.VelocityTable, _lon_VehilceConfig.VlcNum,
																	_lon_VehilceConfig.TorqueTable);

            if(velocity_pid->getDesired() < 1.0e-6)//速度为0时，直接刹住
			{
				ctl->Torque = 0;
				ctl->Acceleration = -0.5;
				ctl->AccelerationEnable = 1;
			}
			else
			{
				if(ctl->TargetAcceleration >= 1.0e-2)
				{
					ctl->AccelerationEnable = 0;
					ctl->Torque = fmax(_calibration_value,_throttle_lowerbound);//添加最小死区的判断
					ctl->Acceleration = 0;
				}
				else
				{
					ctl->Torque = _throttle_lowerbound;
					ctl->Acceleration = 0;
					ctl->AccelerationEnable = 0;
				}
			}
		}
		else
		{
			ctl->TorqueEnable  = 1;
			_calibration_value = ctl->TargetAcceleration;

            if(velocity_pid->getDesired() < 1.0e-6)
			{
				ctl->AccelerationEnable = 1;
				ctl->Acceleration = -0.6;
			}
			else
			{
				ctl->Torque = _throttle_lowerbound;
//				if(_calibration_value >= -1.0e-6)
//				{
//					ctl->Acceleration = _brake_lowerbound;
//				}
//				else
//				{
//					ctl->Acceleration = _calibration_value;
//				}
			}
		}
	}
	else
	{
		ctl->AccelerationEnable = 0;
		ctl->TorqueEnable       = 0;
		ctl->Acceleration = 0;
		ctl->Torque       = 0;
	}
}

float LonControl::AcceleratePlanningControl(float cur_velocity,float stop_distance)
{
	float target_acc;
	if((cur_velocity < 1.0e-6f) || (stop_distance < 1.0e-6f))
	{
		return MAX_DECELERATION;
	}
	else
	{
		target_acc = -0.5f * cur_velocity * cur_velocity / stop_distance;
		return target_acc < MAX_DECELERATION ? MAX_DECELERATION : target_acc;
	}
}

float LonControl::AccAcceleratePlanningControl(float cur_velocity,float stop_distance)
{
	if((cur_velocity < 1.0e-6f) || (stop_distance < 1.0e-6f))
	{
		return MAX_DECELERATION/DT;
	}
	else
	{
		return (-8.0f * cur_velocity * cur_velocity * cur_velocity)/(9.0f * stop_distance * stop_distance);
	}
}

void LonControl::VelocityLookupProc(MessageManager *msg,VehicleController *ctl,PID *velocity_pid)
{
//	_target_velocity = ctl->Velocity;//纯速度控制
	_target_velocity = VelocityControl(ctl->getDistance(),ctl->getVelocity());//距离速度控制
	_vehicle_velocity = msg->getVehicleMiddleSpeed();

	if(ctl->getVelocityEnable())
	{
		if(_target_velocity < 1.0e-6f)
		{
			if(_distance_update_distance_value < 1.0e-6f)
			{
				_distance_update_distance_value = ctl->getDistance();
				_distance_update_pulse_value = msg->getWheelSumPulse();
				_vehicle_stop_acc_acc = AccAcceleratePlanningControl(_vehicle_velocity,_distance_update_distance_value);
			}
			else
			{
				// 距离值更新
				if((ctl->getDistance() < (_distance_update_distance_value - 1.0e-6f)) || (ctl->getDistance() > (_distance_update_distance_value + 1.0e-6f)))
				{
					_distance_update_distance_value = ctl->getDistance();
					_distance_update_pulse_value = msg->getWheelSumPulse();
					_vehicle_stop_acc_acc = AccAcceleratePlanningControl(_vehicle_velocity,_distance_update_distance_value);
				}
				else
				{
					_delta_distance          = (msg->getWheelSumPulse() - _distance_update_pulse_value) * WHEEL_PUSLE_RATIO;
					_variable_distance_value = (_delta_distance > 0) ? _distance_update_distance_value - _delta_distance
							                                         : _distance_update_distance_value;
					_variable_distance_value = _variable_distance_value < 1.0e-6f ? 0 : _variable_distance_value;

					_vehicle_stop_acc_acc = AccAcceleratePlanningControl(_vehicle_velocity,_variable_distance_value);
				}
			}
			_vehicle_stop_acc += _vehicle_stop_acc_acc * DT;
			_vehicle_stop_acc  = _vehicle_stop_acc < MAX_DECELERATION ? MAX_DECELERATION : _vehicle_stop_acc;
		}
		else
		{
			_vehicle_stop_acc = 0;
			_distance_update_distance_value = 0;
		}

		if(msg->getBrakePressure() > 0)
		{
			velocity_pid->setDesired(0.0f);
		}
		else
		{
			if(velocity_pid->getDesired() <= (_target_velocity - _delta_velocity))
			{
				velocity_pid->setDesired(velocity_pid->getDesired() + _delta_velocity);
			}
			else
			{
				velocity_pid->setDesired(_target_velocity);
			}
		}

		ctl->setTorqueEnable(1);

		if(SpeedNormal == msg->getVehicleMiddleSpeedAbnormal())/*速度正常，更新控制信息*/
		{
			ctl->setTargetAcceleration(velocity_pid->pidUpdate(_vehicle_velocity));
			if(ctl->getTargetAcceleration() > 0)
			{
				_calibration_value = _lon_Interpolation.Interpolation2D(ctl->getTargetAcceleration(), _vehicle_velocity,
																		_lon_VehilceConfig.AccelerateTable, _lon_VehilceConfig.AccNum,
																		_lon_VehilceConfig.VelocityTable, _lon_VehilceConfig.VlcNum,
																		_lon_VehilceConfig.TorqueTable);
				if(velocity_pid->getDesired() < 1.0e-6)//速度为0时，直接刹住
				{
					ctl->setAccelerationEnable(1);
					ctl->setAcceleration(_vehicle_stop_acc);
					ctl->setTorque(0);
				}
				else
				{
					ctl->setTorque(fmax(_calibration_value,_throttle_lowerbound));//添加最小死区的判断
					ctl->setAcceleration(0);
					ctl->setAccelerationEnable(0);
				}
				_vehicle_slow_down_acc = 0.0f;
			}
			else
			{
				_calibration_value = ctl->getTargetAcceleration();
				if(velocity_pid->getDesired() < 1.0e-6)//刹车
				{
					ctl->setTorque(0);
					ctl->setAccelerationEnable(1);
					ctl->setAcceleration(_vehicle_stop_acc);
					_vehicle_slow_down_acc = 0.0f;
				}
				else//减速控制
				{
					ctl->setAccelerationEnable(0);
					ctl->setTorque(0);
					if( (velocity_pid->getDesired() - _vehicle_velocity) < -0.1f)
					{
						_vehicle_slow_down_acc -= START_ACC * DT;
						_vehicle_slow_down_acc = _vehicle_slow_down_acc < -5.0f? -5.0f : _vehicle_slow_down_acc;
					}
					else
					{
						_vehicle_slow_down_acc = 0.0f;
					}
					ctl->setAcceleration(_vehicle_slow_down_acc);
				}
			}
		}
		// 异常处理
		// (1)速度反馈异常
		if(0 == msg->getBrakePressure())
		{
			if(SpeedAbnormal == msg->getVehicleMiddleSpeedAbnormal())/*速度值异常*/
			{
				_err_velocity_cnt++;
			}
			else
			{
				_err_velocity_cnt = 0;
			}
		}
		if(_err_velocity_cnt > 100)
		{
			// 策略：制动减速度设置为最大减速度，强制车辆停止
			ctl->setAccelerationEnable(1);
			ctl->setTorqueEnable(1);
			ctl->setAcceleration(MAX_DECELERATION);
			ctl->setTorque(0);
		}

		// (2)ESC异常
		if(ActuatorErr == msg->getESC_Status())
		{
			// 策略：失效扭矩控制
			ctl->setAccelerationEnable(0);
			ctl->setTorqueEnable(0);
			ctl->setAcceleration(0);
			ctl->setTorque(0);
		}
	}
	else
	{
		ctl->setAccelerationEnable(0);
		ctl->setTorqueEnable(0);
		ctl->setAcceleration(0);
		ctl->setTorque(0);
	}
}

void LonControl::DistanceProc(MessageManager *msg,VehicleController *ctl)
{
	_target_velocity = VelocityControl(ctl->getDistance(),ctl->getVelocity());//距离速度控制
	_vehicle_velocity = msg->getVehicleMiddleSpeed();

	if(_target_velocity < 1.0e-6f)
	{
		if(_distance_update_distance_value < 1.0e-6f)/*变量初始化*/
		{
			_distance_update_distance_value = ctl->getDistance();
			_distance_update_pulse_value    = msg->getWheelSumPulse();
			_variable_distance_value        = _distance_update_distance_value;
		}
		else
		{
			// 距离值更新
			if((ctl->getDistance() < (_distance_update_distance_value - 1.0e-6f)) || (ctl->getDistance() > (_distance_update_distance_value + 1.0e-6f)))
			{
				_distance_update_distance_value = ctl->getDistance();
				_distance_update_pulse_value    = msg->getWheelSumPulse();
				_variable_distance_value        = _distance_update_distance_value;
			}
			else
			{
				_delta_distance          = (msg->getWheelSumPulse() - _distance_update_pulse_value) * WHEEL_PUSLE_RATIO;
				_variable_distance_value = (_delta_distance > 0) ? _distance_update_distance_value - _delta_distance
						                                         : _distance_update_distance_value;
				_variable_distance_value = _variable_distance_value < 1.0e-6f ? 0 : _variable_distance_value;
			}
		}
		if(_vehicle_velocity < 1.0e-6f)
		{
			_variable_distance_value = 0;
		}
		ctl->setDistanceSet(_variable_distance_value);
	}
	else
	{
		_distance_update_distance_value = 0;
		_distance_update_pulse_value    = 0;
		ctl->setDistanceSet(ctl->getDistance());
	}
}

float LonControl::getControlStateFlag()           { return _control_state_flag;}
void  LonControl::setControlStateFlag(float value){_control_state_flag = value;}
