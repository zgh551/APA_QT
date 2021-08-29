/*****************************************************************************/
/* FILE NAME: path_plannig.cpp                  COPYRIGHT (c) Henry Zhu 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the trajectory planning interface  					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      January 9 2019      Initial Version                  */
/*****************************************************************************/
#include "./Planning/Interface/planning.h"

Planning::Planning() {
	MinParkingLength.setContainer(this);
	MinParkingLength.getter(&Planning::getMinParkingLength);
	MinParkingLength.setter(&Planning::setMinParkingLength);

	MinParkingWidth.setContainer(this);
	MinParkingWidth.getter(&Planning::getMinParkingWidth);
	MinParkingWidth.setter(&Planning::setMinParkingWidth);

	OuterVirtualBoundary.setContainer(this);
	OuterVirtualBoundary.getter(&Planning::getOuterVirtualBoundary);
	OuterVirtualBoundary.setter(&Planning::setOuterVirtualBoundary);

	InsideVirtualBoundary.setContainer(this);
	InsideVirtualBoundary.getter(&Planning::getInsideVirtualBoundary);
	InsideVirtualBoundary.setter(&Planning::setInsideVirtualBoundary);

	FrontVirtualBoundary.setContainer(this);
	FrontVirtualBoundary.getter(&Planning::getFrontVirtualBoundary);
	FrontVirtualBoundary.setter(&Planning::setFrontVirtualBoundary);

	RearVirtualBoundary.setContainer(this);
	RearVirtualBoundary.getter(&Planning::getRearVirtualBoundary);
	RearVirtualBoundary.setter(&Planning::setRearVirtualBoundary);
	/**********************************************************************/
	OuterMarginMove.setContainer(this);
	OuterMarginMove.getter(&Planning::getOuterMarginMove);
	OuterMarginMove.setter(&Planning::setOuterMarginMove);

	InsideMarginBoundary.setContainer(this);
	InsideMarginBoundary.getter(&Planning::getInsideMarginBoundary);
	InsideMarginBoundary.setter(&Planning::setInsideMarginBoundary);

	FrontMarginBoundary.setContainer(this);
	FrontMarginBoundary.getter(&Planning::getFrontMarginBoundary);
	FrontMarginBoundary.setter(&Planning::setFrontMarginBoundary);

	RearMarginBoundary.setContainer(this);
	RearMarginBoundary.getter(&Planning::getRearMarginBoundary);
	RearMarginBoundary.setter(&Planning::setRearMarginBoundary);
	/**********************************************************************/
	Command.setContainer(this);
	Command.getter(&Planning::getCommand);
	Command.setter(&Planning::setCommand);

	ConsoleState.setContainer(this);
	ConsoleState.getter(&Planning::getConsoleState);
	ConsoleState.setter(&Planning::setConsoleState);

	ParkingStatus.setContainer(this);
	ParkingStatus.getter(&Planning::getParkingStatus);
	ParkingStatus.setter(&Planning::setParkingStatus);
	/**********************************************************************/
	PlanningBrakingAcc.setContainer(this);
	PlanningBrakingAcc.getter(&Planning::getPlanningBrakingAcc);
	PlanningBrakingAcc.setter(&Planning::setPlanningBrakingAcc);

	PlanningBrakingAccR.setContainer(this);
	PlanningBrakingAccR.getter(&Planning::getPlanningBrakingAccR);
	PlanningBrakingAccR.setter(&Planning::setPlanningBrakingAccR);

	PlanningBrakingAeb.setContainer(this);
	PlanningBrakingAeb.getter(&Planning::getPlanningBrakingAeb);
	PlanningBrakingAeb.setter(&Planning::setPlanningBrakingAeb);

	TurnningFeedforwardTime.setContainer(this);
	TurnningFeedforwardTime.getter(&Planning::getTurnningFeedforwardTime);
	TurnningFeedforwardTime.setter(&Planning::setTurnningFeedforwardTime);

	AccDisableTime.setContainer(this);
	AccDisableTime.getter(&Planning::getAccDisableTime);
	AccDisableTime.setter(&Planning::setAccDisableTime);

	PositionMax.setContainer(this);
	PositionMax.getter(&Planning::getPositionMax);
	PositionMax.setter(&Planning::setPositionMax);

	PositionMin.setContainer(this);
	PositionMin.getter(&Planning::getPositionMin);
	PositionMin.setter(&Planning::setPositionMin);

	ParkingMargin.setContainer(this);
	ParkingMargin.getter(&Planning::getParkingMargin);
	ParkingMargin.setter(&Planning::setParkingMargin);

	KpYaw.setContainer(this);
	KpYaw.getter(&Planning::getKpYaw);
	KpYaw.setter(&Planning::setKpYaw);
	/**********************************************************************/
	//边界内margin为正值
	_outer_margin_move      =  0.0f;
	_inside_margin_boundary =  0.15f;
	_front_margin_boundary  =  0.15f;
	_rear_margin_boundary   =  0.15f;

	_parking_status = 0;

	planning_braking_acc_      = PLANNING_BRAKING;
	planning_braking_acc_r_    = fabs(1/planning_braking_acc_/0.6f);
	planning_braking_aeb_      = EMERGENCY_BRAKING;
	turnning_feedforward_time_ = TURN_FEEDFORWARD_TIME;
	acc_disable_time_          = ACC_DISABLE_TIME;
	position_min_              = POSITION_A;
	position_max_              = POSITION_B;
	parking_margin_            = PARKING_MARGIN;
	kp_yaw_                    = 30;
}

Planning::~Planning() {

}

int8_t Planning::ForecastCircleParkingPointMargin(VehicleState *s,Vector2d stop_point,float radius,float margin,uint8_t quadrant,int8_t mode)
{
	float angle_vector;
	float min_value,max_value;
	switch(quadrant)
	{
		case 1:
			min_value = 0;
			max_value = PI_2;
			break;

		case 2:
			min_value = PI_2;
			max_value = PI;
			break;

		case 3:
			min_value = -PI;
			max_value = -PI_2;
			break;

		case 4:
			min_value = -PI_2;
			max_value = 0;
			break;

		default:
			min_value = 0;
			max_value = 0;
			break;
	}
	angle_vector = (s->getPosition() - stop_point).Angle();
	if((angle_vector > min_value) && (angle_vector < max_value))//在规划区间内，执行提前规划停车控制
	{
		if(-1 == mode)
		{
			if(_plan_algebraic_geometry.ArcLength(s->getPosition(), stop_point, radius) < margin)
			{
				_control_command.ControlEnable.B.VelocityEnable     = 0;
				_control_command.ControlEnable.B.DecelerationEnable = 1;
				_control_command.ControlEnable.B.AccelerationEnable = 1;
				_control_command.Deceleration  = planning_braking_aeb_;
				_control_command.Acceleration  = planning_braking_aeb_;
				return SUCCESS;
			}
			else if((_plan_algebraic_geometry.ArcLength(s->getPosition(), stop_point, radius) - margin) < ( s->LinearRate * s->LinearRate * 0.5 * planning_braking_acc_r_))
			{
				_control_command.ControlEnable.B.VelocityEnable     = 0;
				_control_command.ControlEnable.B.AccelerationEnable = 1;
				_control_command.Acceleration                       = planning_braking_acc_;
				return SUCCESS;
			}
			else
			{
				return FAIL;
			}
		}
		else if( 0 == mode)
		{
			if( _plan_algebraic_geometry.ArcLength(s->getPosition(), stop_point, radius) < ( s->LinearRate * s->LinearRate * 0.5 * planning_braking_acc_r_))
			{
				_control_command.ControlEnable.B.VelocityEnable     = 0;
				_control_command.ControlEnable.B.AccelerationEnable = 1;
				_control_command.Acceleration                       = planning_braking_acc_;
				return SUCCESS;
			}
			else
			{
				return FAIL;
			}
		}
		else
		{
			return FAIL;
		}
	}
	else
	{
		if(1 == mode)
		{
			if(margin < _plan_algebraic_geometry.ArcLength(s->getPosition(), stop_point, radius))
			{
				_control_command.ControlEnable.B.VelocityEnable     = 0;
				_control_command.ControlEnable.B.DecelerationEnable = 1;
				_control_command.ControlEnable.B.AccelerationEnable = 1;
				_control_command.Deceleration  = planning_braking_aeb_;
				_control_command.Acceleration  = planning_braking_aeb_;
				return SUCCESS;
			}
			if((margin - _plan_algebraic_geometry.ArcLength(s->getPosition(), stop_point, radius)) < ( s->LinearRate * s->LinearRate * 0.5 * planning_braking_acc_r_))
			{
				_control_command.Acceleration                       = planning_braking_acc_;
				_control_command.ControlEnable.B.VelocityEnable     = 0;
				_control_command.ControlEnable.B.AccelerationEnable = 1;
				return SUCCESS;
			}
			else
			{
				return FAIL;
			}
		}
		else
		{
			_control_command.ControlEnable.B.VelocityEnable     = 0;
			_control_command.ControlEnable.B.DecelerationEnable = 1;
			_control_command.ControlEnable.B.AccelerationEnable = 1;
			_control_command.Deceleration  = planning_braking_aeb_;
			_control_command.Acceleration  = planning_braking_aeb_;
			return SUCCESS;
		}
	}
}

int8_t Planning::ForecastLineParkingPointMargin(VehicleState *s,Vector2d stop_point,float margin,uint8_t quadrant,int8_t mode)
{
	float angle_vector;
	float min_value,max_value;
	switch(quadrant)
	{
		case 1:
			min_value = 0;
			max_value = PI_2;
			break;

		case 2:
			min_value = PI_2;
			max_value = PI;
			break;

		case 3:
			min_value = -PI;
			max_value = -PI_2;
			break;

		case 4:
			min_value = -PI_2;
			max_value = 0;
			break;

		default:
			min_value = 0;
			max_value = 0;
			break;
	}
	angle_vector = (s->getPosition() - stop_point).Angle();
	if((angle_vector > min_value) && (angle_vector < max_value))//在规划区间内，执行提前规划停车控制
	{
		if(-1 == mode)
		{
			if((s->getPosition() - stop_point).Length() < margin)
			{
				_control_command.ControlEnable.B.AccelerationEnable = 1;
				_control_command.ControlEnable.B.VelocityEnable     = 0;
				_control_command.Velocity                           = 0;
				_control_command.Acceleration                       = planning_braking_aeb_;
				return SUCCESS;
			}
			else if(((s->getPosition() - stop_point).Length() - margin) < ( s->LinearRate * s->LinearRate * 0.5 * planning_braking_acc_r_))
			{
				_control_command.ControlEnable.B.AccelerationEnable = 1;
				_control_command.ControlEnable.B.VelocityEnable     = 0;
				_control_command.Velocity                           = 0;
				_control_command.Acceleration                       = planning_braking_acc_;
				return SUCCESS;
			}
			else
			{
				return FAIL;
			}
		}
		else if( 0 == mode)
		{
			if( (s->getPosition() - stop_point).Length() < ( s->LinearRate * s->LinearRate * 0.5 * planning_braking_acc_r_))
			{
				_control_command.ControlEnable.B.AccelerationEnable = 1;
				_control_command.ControlEnable.B.VelocityEnable     = 0;
				_control_command.Velocity                           = 0;
				_control_command.Acceleration                       = planning_braking_acc_;
				return SUCCESS;
			}
			else
			{
				return FAIL;
			}
		}
		else
		{
			return FAIL;
		}
	}
	else
	{
		if(1 == mode)
		{
			if(margin < (s->getPosition() - stop_point).Length())
			{
				_control_command.ControlEnable.B.AccelerationEnable = 1;
				_control_command.ControlEnable.B.VelocityEnable     = 0;
				_control_command.Velocity                           = 0;
				_control_command.Acceleration                       = planning_braking_aeb_;
				return SUCCESS;
			}
			if((margin - (s->getPosition() - stop_point).Length()) < ( s->LinearRate * s->LinearRate * 0.5 * planning_braking_acc_r_))
			{
				_control_command.ControlEnable.B.AccelerationEnable = 1;
				_control_command.ControlEnable.B.VelocityEnable     = 0;
				_control_command.Velocity                           = 0;
				_control_command.Acceleration                       = planning_braking_acc_;
				return SUCCESS;
			}
			else
			{
				return FAIL;
			}
		}
		else
		{
			_control_command.ControlEnable.B.AccelerationEnable = 1;
			_control_command.ControlEnable.B.VelocityEnable     = 0;
			_control_command.Velocity                           = 0;
			_control_command.Acceleration  = planning_braking_aeb_;
			return SUCCESS;
		}
	}
}

int8_t Planning::ForecastCircleParking(VehicleState *s,Vector2d stop_point,float radius,uint8_t quadrant)
{
	float angle_vector;
	float min_value,max_value;
	switch(quadrant)
	{
		case 1:
			min_value = 0;
			max_value = PI_2;
			break;

		case 2:
			min_value = PI_2;
			max_value = PI;
			break;

		case 3:
			min_value = -PI;
			max_value = -PI_2;
			break;

		case 4:
			min_value = -PI_2;
			max_value = 0;
			break;

		default:
			min_value = 0;
			max_value = 0;
			break;
	}
	angle_vector = (s->getPosition() - stop_point).Angle();
	if((angle_vector > min_value) && (angle_vector < max_value))//在规划区间内，执行提前规划停车控制
	{
		if( _plan_algebraic_geometry.ArcLength(s->getPosition(), stop_point, radius) < ( s->LinearRate * s->LinearRate * 0.5 * planning_braking_acc_r_))
		{
			_control_command.ControlEnable.B.VelocityEnable     = 0;
			_control_command.ControlEnable.B.AccelerationEnable = 1;
			_control_command.Acceleration                       = planning_braking_acc_;
			return SUCCESS;
		}
		else
		{
			return FAIL;
		}
	}
	else
	{
		_control_command.ControlEnable.B.VelocityEnable = 0;
		_control_command.ControlEnable.B.DecelerationEnable = 1;
		_control_command.ControlEnable.B.AccelerationEnable = 1;
		_control_command.Deceleration  = planning_braking_aeb_;
		_control_command.Acceleration  = planning_braking_aeb_;
		return SUCCESS;
	}
}

int8_t Planning::ForecastCircleBoundaryMargin(VehicleState *s,Vector2d stop_point,float radius,float margin)
{
	if( _plan_algebraic_geometry.ArcLength(s->getPosition(), stop_point, radius) < margin )
	{
		_control_command.ControlEnable.B.VelocityEnable = 0;
		_control_command.ControlEnable.B.AccelerationEnable = 1;
		_control_command.ControlEnable.B.DecelerationEnable = 1;
		_control_command.Deceleration  = planning_braking_aeb_;
		_control_command.Acceleration  = planning_braking_aeb_;
		return SUCCESS;
	}
	else
	{
		if( (_plan_algebraic_geometry.ArcLength(s->getPosition(), stop_point, radius) - margin) < ( s->LinearRate * s->LinearRate * 0.5 * planning_braking_acc_r_))
		{
			_control_command.ControlEnable.B.VelocityEnable     = 0;
			_control_command.ControlEnable.B.AccelerationEnable = 1;
			_control_command.ControlEnable.B.DecelerationEnable = 0;
			_control_command.Acceleration                       = planning_braking_acc_;
			return SUCCESS;
		}
		else
		{
			return FAIL;
		}
	}
}

float Planning::ForecastLineParkingPointMarginDistance(VehicleState *s,Vector2d stop_point,float margin,uint8_t quadrant)
{
	float angle_vector;
	float min_value,max_value;
	switch(quadrant)
	{
		case 1:
			min_value = 0;
			max_value = PI_2;
			break;

		case 2:
			min_value = PI_2;
			max_value = PI;
			break;

		case 3:
			min_value = -PI;
			max_value = -PI_2;
			break;

		case 4:
			min_value = -PI_2;
			max_value = 0;
			break;

		default:
			min_value = 0;
			max_value = 0;
			break;
	}
	angle_vector = (s->getPosition() - stop_point).Angle();
	if((angle_vector > min_value) && (angle_vector < max_value))//在规划区间内，执行提前规划停车控制
	{
		if(margin <= 0)
		{
			if((s->getPosition() - stop_point).Length() + margin > 0 )
			{
				return (s->getPosition() - stop_point).Length() + margin;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			return (s->getPosition() - stop_point).Length() + margin;
		}
	}
	else
	{
		if(margin > 0)
		{
			if(margin > (s->getPosition() - stop_point).Length())
			{
				return margin - (s->getPosition() - stop_point).Length();
			}
			else
			{
				return 0;
			}
		}
		else
		{
			return 0;
		}
	}
}
/**
 * 该函数适用于圆弧停止点的判断
 * */
int8_t Planning::ForecastYawParking(int8_t state,float radius,float target_yaw,VehicleState *s)
{
	float circle_length;
	// 车辆偏航角的趋势偏向于由大变小
	if(-1 == state)
	{
		circle_length = fabs(s->Yaw - target_yaw) * radius;
		if(s->Yaw > target_yaw)
		{
			_control_command.ControlEnable.B.VelocityEnable     = 1;
			_control_command.ControlEnable.B.AccelerationEnable = 1;
			if(circle_length > position_max_)
			{
				_control_command.Velocity = STRAIGHT_VELOCITY;
				return FAIL;
			}
			else if(circle_length > position_min_)
			{
				_control_command.Velocity = CURVE_VELOCITY + (STRAIGHT_VELOCITY - CURVE_VELOCITY)*(circle_length - position_min_)/(position_max_ - position_min_);
				return FAIL;
			}
			else if(circle_length > 0.03)
			{
				_control_command.Velocity = circle_length * CURVE_VELOCITY / position_min_ ;
				return FAIL;
			}
			else
			{
				_control_command.Velocity = 0;
				if(0 == s->LinearVelocity)
				{
					return SUCCESS;
				}
				else
				{
					return FAIL;
				}
			}
		}
		else
		{
			_control_command.Velocity                           = 0;
			_control_command.ControlEnable.B.VelocityEnable     = 0;
			_control_command.ControlEnable.B.AccelerationEnable = 1;
			_control_command.Acceleration  = -0.5;//-circle_length * 0.5;
			if(0 == s->LinearVelocity)
			{
				return SUCCESS;
			}
			else
			{
				return FAIL;
			}
		}
	}
	else if(1 == state)
	{
		circle_length = fabs(s->Yaw - target_yaw) * radius;
		if(s->Yaw < target_yaw)
		{
			_control_command.ControlEnable.B.VelocityEnable     = 1;
			_control_command.ControlEnable.B.AccelerationEnable = 1;
			if(circle_length > position_max_)
			{
				_control_command.Velocity = STRAIGHT_VELOCITY;
				return FAIL;
			}
			else if(circle_length > position_min_)
			{
				_control_command.Velocity = CURVE_VELOCITY + (STRAIGHT_VELOCITY - CURVE_VELOCITY)*(circle_length - position_min_)/(position_max_ - position_min_);
				return FAIL;
			}
			else if(circle_length > 0.03)
			{
				_control_command.Velocity = circle_length * CURVE_VELOCITY / position_min_ ;
				return FAIL;
			}
			else
			{
				_control_command.Velocity = 0;
				if(0 == s->LinearVelocity)
				{
					return SUCCESS;
				}
				else
				{
					return FAIL;
				}
			}
		}
		else
		{
			_control_command.Velocity                           = 0;
			_control_command.ControlEnable.B.VelocityEnable     = 0;
			_control_command.ControlEnable.B.AccelerationEnable = 1;
			_control_command.Acceleration  = -0.5;//-circle_length * 0.5;
			if(0 == s->LinearVelocity)
			{
				return SUCCESS;
			}
			else
			{
				return FAIL;
			}
		}
	}
	else
	{
		return FAIL;
	}
}

float Planning::ForecastYawParkingDistance(float target_yaw,VehicleState *s)
{
	float circle_length;
	float radius;
	radius = fabs(_plan_vehilce_config.TurnRadiusCalculate(_apa_control_command.SteeringAngle));
	// 车辆偏航角的趋势偏向于由大变小
	if(Reverse == _apa_control_command.Gear)
	{
		circle_length = fabs(s->Yaw - target_yaw) * radius;
        if(s->Yaw < target_yaw)
		{
			return circle_length;
		}
		else
		{
			return 0;
		}
	}
	else if(Drive == _apa_control_command.Gear)
	{
		circle_length = fabs(s->Yaw - target_yaw) * radius;
        if(s->Yaw > target_yaw)
		{
			return circle_length;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
}
/**************************************************************************************************/
int8_t Planning::BoundaryCollision(int8_t motion,VehicleState *s)
{
	_boundary_collision_body.Center      = s->getPosition();
	_boundary_collision_body.AttitudeYaw = s->getYaw();
	_boundary_collision_body.EdgePoint();

	if(-1 == motion)
	{
		if( (_boundary_collision_body.getRearLeft().getX()  < RearVirtualBoundary  ) ||
			(_boundary_collision_body.getRearRight().getY() < InsideVirtualBoundary) )
		{
			_control_command.Acceleration  = planning_braking_aeb_;
			_control_command.ControlEnable.B.VelocityEnable     = 0;
			_control_command.Velocity                           = 0;
			_control_command.ControlEnable.B.AccelerationEnable = 1;
			return SUCCESS;
		}
		else
		{
			// TODO 后期可以优化到弧线判定
			if( (_boundary_collision_body.getRearLeft().getX()  - RearVirtualBoundary  ) < ( s->LinearRate * s->LinearRate * 0.5 * planning_braking_acc_r_) ||
				(_boundary_collision_body.getRearRight().getY() - InsideVirtualBoundary) < ( s->LinearRate * s->LinearRate * 0.5 * planning_braking_acc_r_) )
			{
				_control_command.Acceleration                       = planning_braking_acc_;
				_control_command.ControlEnable.B.AccelerationEnable = 1;
				_control_command.Velocity                           = 0;
				_control_command.ControlEnable.B.VelocityEnable     = 0;
				return SUCCESS;
			}
			else
			{
				return FAIL;
			}
		}
	}
	else if(1 == motion)
	{
		if(( _boundary_collision_body.getFrontRight().getX() > FrontVirtualBoundary ) ||
		   ( _boundary_collision_body.getFrontRight().getY() < InsideVirtualBoundary) )
		{
			_control_command.Acceleration  = planning_braking_aeb_;
			_control_command.ControlEnable.B.VelocityEnable     = 0;
			_control_command.Velocity                           = 0;
			_control_command.ControlEnable.B.AccelerationEnable = 1;
			return SUCCESS;
		}
		else
		{
			// TODO 后期可以优化到弧线判定
			if(( FrontVirtualBoundary - _boundary_collision_body.getFrontRight().getX()) <( s->LinearRate * s->LinearRate * 0.5 * planning_braking_acc_r_ ) )
			{
				_control_command.Acceleration                       = planning_braking_acc_;
				_control_command.ControlEnable.B.VelocityEnable     = 0;
				_control_command.Velocity                           = 0;
				_control_command.ControlEnable.B.AccelerationEnable = 1;
				return SUCCESS;
			}
			else
			{
				return FAIL;
			}
		}
	}
	else
	{
		return FAIL;
	}
}

int8_t Planning::BoundaryCollisionVelocity(int8_t motion,float target,VehicleState *s)
{
 	float theta1,theta2,theta3,theta_min;
 	float circle_length;
	_boundary_collision_body.Center      = s->getPosition();
	_boundary_collision_body.AttitudeYaw = s->getYaw();

	if(-1 == motion)
	{
		_boundary_collision_body.RotationCenter(MIN_LEFT_TURN_RADIUS);
		_boundary_collision_body.EdgePoint();

		if( (_boundary_collision_body.getRearLeft().getX()  <= RearVirtualBoundary  ) ||
			(_boundary_collision_body.getRearRight().getY() <= InsideVirtualBoundary) ||
			(s->getYaw() <= (target + 0.02)))
		{
			_apa_control_command.Velocity = 0;
			_apa_control_command.Distance = 0;
			if(s->getYaw() <= (target + 0.02))
			{
				_control_command.SteeringAngle     = 0;
				_control_command.SteeringAngleRate = MAX_STEERING_ANGLE_RATE;
			}
			if(0 == s->LinearVelocity)
			{
				return SUCCESS;
			}
			else
			{
				return FAIL;
			}
		}
		else
		{
			theta1 = _boundary_collision_body.RotateAngleCollision(_boundary_collision_body.getRearRight(), _parking_inside_rear_point);
			theta2 = _boundary_collision_body.RotateAngleCollision(_boundary_collision_body.getRearLeft() , _parking_inside_rear_point);
			theta3 = s->getYaw() - target ;
			theta_min = theta1 < theta2    ? theta1 : theta2;
			theta_min = theta3 < theta_min ? theta3 : theta_min;

			circle_length = MIN_LEFT_TURN_RADIUS * theta_min;

			if(circle_length > position_max_)
			{
				_control_command.Velocity = STRAIGHT_VELOCITY;
				return FAIL;
			}
			else if(circle_length > position_min_)
			{
				_control_command.Velocity = CURVE_VELOCITY + (STRAIGHT_VELOCITY - CURVE_VELOCITY)*(circle_length - position_min_)/(position_max_ - position_min_);
				return FAIL;
			}
			else if(circle_length > 0.05)
			{
				_control_command.Velocity = circle_length * CURVE_VELOCITY / position_min_ ;
				return FAIL;
			}
			else
			{
				_control_command.Velocity = 0;
				if(0 == s->LinearVelocity)
				{
					return SUCCESS;
				}
				else
				{
					return FAIL;
				}
			}
		}
	}
	else if(1 == motion)
	{
		_boundary_collision_body.RotationCenter(-MIN_RIGHT_TURN_RADIUS);
		_boundary_collision_body.EdgePoint();

		if(( _boundary_collision_body.getFrontRight().getX() > FrontVirtualBoundary ) ||
		   ( _boundary_collision_body.getFrontRight().getY() < InsideVirtualBoundary) ||
		   (s->getYaw() <= (target + 0.02)))
		{
			_control_command.ControlEnable.B.VelocityEnable     = 0;
			_control_command.Velocity                           = 0;
			_control_command.ControlEnable.B.AccelerationEnable = 1;
			// TODO 需要考虑加速度如何给定合适
			_control_command.Acceleration                       = planning_braking_aeb_;
			if(s->getYaw() <= (target + 0.02))
			{
				_control_command.SteeringAngle     = 0;
				_control_command.SteeringAngleRate = MAX_STEERING_ANGLE_RATE;
			}
			if(0 == s->LinearVelocity)
			{
				return SUCCESS;
			}
			else
			{
				return FAIL;
			}
		}
		else
		{
			_control_command.ControlEnable.B.VelocityEnable     = 1;
			_control_command.ControlEnable.B.AccelerationEnable = 1;

			theta1 = _boundary_collision_body.RotateAngleCollision(_boundary_collision_body.getFrontRight(), _parking_inside_front_point);
			theta2 = _boundary_collision_body.RotateAngleCollision(_boundary_collision_body.getFrontLeft() , _parking_inside_front_point);
			theta3 = s->getYaw() - target ;
			theta_min = theta1 < theta2    ? theta1 : theta2;
			theta_min = theta3 < theta_min ? theta3 : theta_min;

			circle_length = MIN_RIGHT_TURN_RADIUS * theta_min;

			if(circle_length > position_max_)
			{
				_control_command.Velocity = STRAIGHT_VELOCITY;
				return FAIL;
			}
			else if(circle_length > position_min_)
			{
				_control_command.Velocity = CURVE_VELOCITY + (STRAIGHT_VELOCITY - CURVE_VELOCITY)*(circle_length - position_min_)/(position_max_ - position_min_);
				return FAIL;
			}
			else if(circle_length > 0.05)
			{
				_control_command.Velocity = circle_length * CURVE_VELOCITY / position_min_ ;
				return FAIL;
			}
			else
			{
				_control_command.Velocity = 0;
				if(0 == s->LinearVelocity)
				{
					return SUCCESS;
				}
				else
				{
					return FAIL;
				}
			}
		}
	}
	else
	{
		return FAIL;
	}
}

int8_t Planning::BoundaryCollisionCircle(int8_t motion,VehicleState *s)
{
	float theta1,theta2,theta_min;
	_boundary_collision_body.Center      = s->getPosition();
	_boundary_collision_body.AttitudeYaw = s->getYaw();

	if(-1 == motion)
	{
		_boundary_collision_body.RotationCenter(MIN_LEFT_TURN_RADIUS);
		_boundary_collision_body.EdgePoint();

		theta1 = _boundary_collision_body.RotateAngleCollision(_boundary_collision_body.getRearRight(), _parking_inside_rear_point);
		theta2 = _boundary_collision_body.RotateAngleCollision(_boundary_collision_body.getRearLeft() , _parking_inside_rear_point);
		theta_min = theta1 < theta2 ? theta1 : theta2;

		if( (_boundary_collision_body.getRearLeft().getX()  < RearVirtualBoundary  ) ||
			(_boundary_collision_body.getRearRight().getY() < InsideVirtualBoundary) )
		{
			_control_command.Acceleration  = planning_braking_aeb_;
			_control_command.ControlEnable.B.VelocityEnable     = 0;
			_control_command.Velocity                           = 0;
			_control_command.ControlEnable.B.AccelerationEnable = 1;
			return SUCCESS;
		}
		else
		{
			// TODO 后期可以优化到弧线判定
			if( ( theta_min * MIN_LEFT_TURN_RADIUS ) < ( s->LinearRate * s->LinearRate * 0.5 * planning_braking_acc_r_))
			{
				_control_command.Acceleration                       = planning_braking_acc_;
				_control_command.ControlEnable.B.AccelerationEnable = 1;
				_control_command.Velocity                           = 0;
				_control_command.ControlEnable.B.VelocityEnable     = 0;
				return SUCCESS;
			}
			else
			{
				return FAIL;
			}
		}
	}
	else if(1 == motion)
	{
		_boundary_collision_body.RotationCenter(-MIN_RIGHT_TURN_RADIUS);
		_boundary_collision_body.EdgePoint();

		theta_min = _boundary_collision_body.RotateAngleCollision(_boundary_collision_body.getFrontRight(), _parking_inside_front_point);

		if(( _boundary_collision_body.getFrontRight().getX() > FrontVirtualBoundary ) ||
		   ( _boundary_collision_body.getFrontRight().getY() < InsideVirtualBoundary) )
		{
			_control_command.Acceleration  = planning_braking_aeb_;
			_control_command.ControlEnable.B.VelocityEnable     = 0;
			_control_command.Velocity                           = 0;
			_control_command.ControlEnable.B.AccelerationEnable = 1;
			return SUCCESS;
		}
		else
		{
			// TODO 后期可以优化到弧线判定
			if( (theta_min * MIN_RIGHT_TURN_RADIUS) < ( s->LinearRate * s->LinearRate * 0.5 * planning_braking_acc_r_ ) )
			{
				_control_command.Acceleration                       = planning_braking_acc_;
				_control_command.ControlEnable.B.VelocityEnable     = 0;
				_control_command.Velocity                           = 0;
				_control_command.ControlEnable.B.AccelerationEnable = 1;
				return SUCCESS;
			}
			else
			{
				return FAIL;
			}
		}
	}
	else
	{
		return FAIL;
	}
}

float Planning::BoundaryCollisionDistance(float target,VehicleState *s)
{
 	float theta1,theta2,theta3,theta_min;
 	float radius;

 	radius = _plan_vehilce_config.TurnRadiusCalculate(_apa_control_command.SteeringAngle);
	_boundary_collision_body.Center      = s->getPosition();
	_boundary_collision_body.AttitudeYaw = s->getYaw();
	_boundary_collision_body.RotationCenter(radius);
	_boundary_collision_body.EdgePoint();

	if(Reverse == _apa_control_command.Gear)
	{
		if( (_boundary_collision_body.getRearLeft().getX()  <= RearVirtualBoundary  ) ||
			(_boundary_collision_body.getRearRight().getY() <= InsideVirtualBoundary) )
		{
			return 0;
		}
		else
		{
			theta1 = _boundary_collision_body.RotateAngleCollision(_boundary_collision_body.getRearRight(), _parking_inside_rear_point);
			theta2 = _boundary_collision_body.RotateAngleCollision(_boundary_collision_body.getRearLeft() , _parking_inside_rear_point);
			theta3 = s->getYaw() - target ;
			theta_min = theta1 < theta2    ? theta1 : theta2;
			theta_min = theta3 < theta_min ? theta3 : theta_min;
			return fabs(radius * theta_min);
		}
	}
	else if(Drive == _apa_control_command.Gear)
	{
		if(( _boundary_collision_body.getFrontRight().getX() > FrontVirtualBoundary ) ||
		   ( _boundary_collision_body.getFrontRight().getY() < InsideVirtualBoundary) )
		{
			return 0;
		}
		else
		{
			theta1 = _boundary_collision_body.RotateAngleCollision(_boundary_collision_body.getFrontRight(), _parking_inside_front_point);
			theta2 = _boundary_collision_body.RotateAngleCollision(_boundary_collision_body.getFrontLeft() , _parking_inside_front_point);
			theta3 = s->getYaw() - target ;
			theta_min = theta1 < theta2    ? theta1 : theta2;
			theta_min = theta3 < theta_min ? theta3 : theta_min;
			return fabs(radius * theta_min);
		}
	}
	else
	{
		return 0;
	}
}

int8_t Planning::UltrasonicCollision(int8_t motion,VehicleState *s,Ultrasonic *u)
{
	uint8_t i;
	if(1 == motion)
	{
		for(i = 0;i < 4;i++)
		{
			if(u->UltrasonicPacket[i].Distance1 != 0.0f)
			{
				if( (u->UltrasonicPacket[i].Distance1 < 0.4) || (u->UltrasonicPacket[i].status == 16) )
				{
					_control_command.Acceleration  = planning_braking_aeb_;
					_control_command.ControlEnable.B.VelocityEnable     = 0;
					_control_command.Velocity                           = 0;
					_control_command.ControlEnable.B.AccelerationEnable = 1;
					return SUCCESS;
				}
			}
		}
		return FAIL;
	}
	else if(-1 == motion)
	{
		for(i = 4;i < 8;i++)
		{
			if(u->UltrasonicPacket[i].Distance1 != 0.0f)
			{
				if( (u->UltrasonicPacket[i].Distance1 < 0.4) || (u->UltrasonicPacket[i].status == 16))
				{
					_control_command.Acceleration  = planning_braking_aeb_;
					_control_command.ControlEnable.B.VelocityEnable     = 0;
					_control_command.Velocity                           = 0;
					_control_command.ControlEnable.B.AccelerationEnable = 1;
					return SUCCESS;
				}
			}
		}
		return FAIL;
	}
	else
	{
		return FAIL;
	}
}

int8_t Planning::UltrasonicCollisionDiatance(Ultrasonic *u)
{
	uint8_t i;
	if(Reverse == _apa_control_command.Gear)
	{
		for(i = 0; i < 4; i++)
		{
			if(u->UltrasonicPacket[i].Distance1 != 0.0f)
			{
				if( (u->UltrasonicPacket[i].Distance1 < 0.4) || (u->UltrasonicPacket[i].status == 16) )
				{
					return SUCCESS;
				}
			}
		}
		return FAIL;
	}
	else if(Drive == _apa_control_command.Gear)
	{
		for(i = 4;i < 8;i++)
		{
			if(u->UltrasonicPacket[i].Distance1 != 0.0f)
			{
				if( (u->UltrasonicPacket[i].Distance1 < 0.4) || (u->UltrasonicPacket[i].status == 16))
				{
					return SUCCESS;
				}
			}
		}
		return FAIL;
	}
	else
	{
		return FAIL;
	}
}
/**************************************************************************************************/
float Planning::VelocityPlanningCircle(VehicleState *s,Vector2d stop_point,float radius)
{
	float distance;
	distance = _plan_algebraic_geometry.ArcLength(s->getPosition(), stop_point, radius);
	if(distance > position_max_)
	{
		return STRAIGHT_VELOCITY;
	}
	else if(distance > position_min_)
	{
		return CURVE_VELOCITY + (STRAIGHT_VELOCITY - CURVE_VELOCITY)*(distance - position_min_)/(position_max_ - position_min_);
	}
	else
	{
		return CURVE_VELOCITY;
	}
}

float Planning::VelocityPlanningLine(VehicleState *s,Vector2d stop_point)
{
	float distance;
	distance = (s->getPosition() - stop_point).Length();
	if(distance > position_max_)
	{
		return STRAIGHT_VELOCITY;
	}
	else if(distance > position_min_)
	{
		return CURVE_VELOCITY + (STRAIGHT_VELOCITY - CURVE_VELOCITY)*(distance - position_min_)/(position_max_ - position_min_);
	}
	else
	{
		return CURVE_VELOCITY;
	}
}

int8_t Planning::CircleTurnningPointDetermination(VehicleState *s,Turn turn_point,float radius,uint8_t quadrant)
{
	float angle_vector;
	float min_value,max_value;

	switch(quadrant)
	{
		case 1:
			min_value = 0;
			max_value = PI_2;
			break;

		case 2:
			min_value = PI_2;
			max_value = PI;
			break;

		case 3:
			min_value = -PI;
			max_value = -PI_2;
			break;

		case 4:
			min_value = -PI_2;
			max_value = 0;
			break;

		default:
			min_value = 0;
			max_value = 0;
			break;
	}
	angle_vector = (s->getPosition() - turn_point.Point).Angle();
	if(angle_vector > min_value && angle_vector < max_value )
	{
		if(_plan_algebraic_geometry.ArcLength(s->getPosition(), turn_point.Point, radius) < s->LinearRate * TURN_FEEDFORWARD_TIME)
		{
			_apa_control_command.SteeringAngle = turn_point.SteeringAngle;
			return SUCCESS;
		}
		else
		{
			return FAIL;
		}
	}
	else
	{
		_apa_control_command.SteeringAngle = turn_point.SteeringAngle;
		return SUCCESS;
	}
}

int8_t Planning::LineTurnningPointDetermination(VehicleState *s,Turn turn_point,uint8_t quadrant)
{
	float angle_vector;
	float min_value,max_value;

	switch(quadrant)
	{
		case 1:
			min_value = 0;
			max_value = PI_2;
			break;

		case 2:
			min_value = PI_2;
			max_value = PI;
			break;

		case 3:
			min_value = -PI;
			max_value = -PI_2;
			break;

		case 4:
			min_value = -PI_2;
			max_value = 0;
			break;

		default:
			min_value = 0;
			max_value = 0;
			break;
	}
	angle_vector = (s->getPosition() - turn_point.Point).Angle();
	if(angle_vector >= min_value && angle_vector <= max_value )
	{
		if((s->getPosition() - turn_point.Point).Length() < s->LinearRate * TURN_FEEDFORWARD_TIME)
		{
			_control_command.SteeringAngle = turn_point.SteeringAngle;
			_control_command.SteeringAngleRate = s->LinearRate * RK;
			return SUCCESS;
		}
		else
		{
			return FAIL;
		}
	}
	else
	{
		_control_command.SteeringAngle = turn_point.SteeringAngle;
		_control_command.SteeringAngleRate = s->LinearRate * RK;
		return SUCCESS;
	}
}
/**************************************************************************************************/
int8_t Planning::WaitVehicleStartMove(uint8_t d,MessageManager *msg)
{
	if(d == msg->WheelSpeedDirection)
	{
		_control_command.ControlEnable.B.AccelerationEnable = 1;
		_control_command.ControlEnable.B.VelocityEnable     = 1;
		return SUCCESS;
	}
	else
	{
		return FAIL;
	}

	_acc_disable_cnt++;
	if(_acc_disable_cnt > acc_disable_time_)
	{
		_control_command.ControlEnable.B.AccelerationEnable = 1;
		_control_command.ControlEnable.B.VelocityEnable     = 1;
		return SUCCESS;
	}
	else
	{
		return FAIL;
	}
}

/*
 * front_state:
 * -1:进入盲区;
 *  0:正常距离;
 *  1:距离太远，无回波;
 * */
void Planning::ParkingCenterAdjustment(VehicleState *s,Percaption *p)
{
	float rear_boundary,front_boundary;
	int8_t rear_state,front_state;
	uint8_t left_id,right_id;

	left_id  = 1;
	right_id = 2;

//	if( (0 == u->UltrasonicPacket[left_id].status) && (0 == u->UltrasonicPacket[right_id].status))
//	{
//		if( (0 != u->UltrasonicPacket[left_id].Distance1) && (0 != u->UltrasonicPacket[right_id].Distance1) )
//		{
//			front_boundary = s->getPosition().getX() + FRONT_EDGE_TO_CENTER +
//							 u->UltrasonicPacket[left_id].Distance1 < u->UltrasonicPacket[right_id].Distance1 ?
//							 u->UltrasonicPacket[left_id].Distance1 : u->UltrasonicPacket[right_id].Distance1 ;
//			front_state = 0;
//		}
//		else if( (0 != u->UltrasonicPacket[left_id].Distance1) && (0 == u->UltrasonicPacket[right_id].Distance1) )
//		{
//			front_boundary = s->getPosition().getX() + FRONT_EDGE_TO_CENTER + u->UltrasonicPacket[left_id].Distance1;
//			front_state = 0;
//		}
//		else if( (0 == u->UltrasonicPacket[left_id].Distance1) && (0 != u->UltrasonicPacket[right_id].Distance1) )
//		{
//			front_boundary = s->getPosition().getX() + FRONT_EDGE_TO_CENTER + u->UltrasonicPacket[right_id].Distance1;
//			front_state = 0;
//		}
//		else//无回波 无穷远
//		{
//			front_state = 1;
//		}
//	}
//	else if( (0 == u->UltrasonicPacket[left_id].status) && (0 != u->UltrasonicPacket[right_id].status) )
//	{
//		if(16 == u->UltrasonicPacket[right_id].status)
//		{
//			front_boundary = s->getPosition().getX() + FRONT_EDGE_TO_CENTER ;
//			front_state = -1;
//		}
//		else
//		{
//			if(0 == u->UltrasonicPacket[left_id].Distance1)
//			{
//				front_state = 1;
//			}
//			else
//			{
//				front_boundary = s->getPosition().getX() + FRONT_EDGE_TO_CENTER + u->UltrasonicPacket[left_id].Distance1;
//				front_state = 0;
//			}
//		}
//	}
//	else if( (0 != u->UltrasonicPacket[left_id].status) && (0 == u->UltrasonicPacket[right_id].status) )
//	{
//		if(16 == u->UltrasonicPacket[left_id].status)
//		{
//			front_boundary = s->getPosition().getX() + FRONT_EDGE_TO_CENTER ;
//			front_state = -1;
//		}
//		else
//		{
//			if(0 == u->UltrasonicPacket[right_id].Distance1)
//			{
//				front_state = 1;
//			}
//			else
//			{
//				front_boundary = s->getPosition().getX() + FRONT_EDGE_TO_CENTER + u->UltrasonicPacket[right_id].Distance1;
//				front_state = 0;
//			}
//		}
//	}
//	else
//	{
//		if((16 == u->UltrasonicPacket[left_id].status) || (16 == u->UltrasonicPacket[right_id].status))
//		{
//			front_boundary = s->getPosition().getX() + FRONT_EDGE_TO_CENTER;
//			front_state = -1;
//		}
//		else
//		{
//			front_state = -2;
//		}
//	}

//	left_id  = 5;
//	right_id = 6;
//	if( (0 == u->UltrasonicPacket[left_id].status) && (0 == u->UltrasonicPacket[right_id].status))
//	{
//		if( (0 != u->UltrasonicPacket[left_id].Distance1) && (0 != u->UltrasonicPacket[right_id].Distance1) )
//		{
//			rear_boundary = s->getPosition().getX() - REAR_EDGE_TO_CENTER -
//							 u->UltrasonicPacket[left_id].Distance1 < u->UltrasonicPacket[right_id].Distance1 ?
//							 u->UltrasonicPacket[left_id].Distance1 : u->UltrasonicPacket[right_id].Distance1 ;
//			rear_state = 0;
//		}
//		else if( (0 != u->UltrasonicPacket[left_id].Distance1) && (0 == u->UltrasonicPacket[right_id].Distance1) )
//		{
//			rear_boundary = s->getPosition().getX() - REAR_EDGE_TO_CENTER - u->UltrasonicPacket[left_id].Distance1;
//			rear_state = 0;
//		}
//		else if( (0 == u->UltrasonicPacket[left_id].Distance1) && (0 != u->UltrasonicPacket[right_id].Distance1) )
//		{
//			rear_boundary = s->getPosition().getX() - REAR_EDGE_TO_CENTER - u->UltrasonicPacket[right_id].Distance1;
//			rear_state = 0;
//		}
//		else//无回波 无穷远
//		{
//			rear_state = 1;
//		}
//	}
//	else if( (0 == u->UltrasonicPacket[left_id].status) && (0 != u->UltrasonicPacket[right_id].status) )
//	{
//		if(16 == u->UltrasonicPacket[right_id].status)
//		{
//			rear_boundary = s->getPosition().getX() - REAR_EDGE_TO_CENTER ;
//			rear_state = -1;
//		}
//		else
//		{
//			if(0 == u->UltrasonicPacket[left_id].Distance1)
//			{
//				rear_state = 1;
//			}
//			else
//			{
//				rear_boundary = s->getPosition().getX() - REAR_EDGE_TO_CENTER - u->UltrasonicPacket[left_id].Distance1;
//				rear_state = 0;
//			}
//		}
//	}
//	else if( (0 != u->UltrasonicPacket[left_id].status) && (0 == u->UltrasonicPacket[right_id].status) )
//	{
//		if(16 == u->UltrasonicPacket[left_id].status)
//		{
//			rear_boundary = s->getPosition().getX() - REAR_EDGE_TO_CENTER ;
//			rear_state = -1;
//		}
//		else
//		{
//			if(0 == u->UltrasonicPacket[right_id].Distance1)
//			{
//				rear_state = 1;
//			}
//			else
//			{
//				rear_boundary = s->getPosition().getX() - REAR_EDGE_TO_CENTER - u->UltrasonicPacket[right_id].Distance1;
//				rear_state = 0;
//			}
//		}
//	}
//	else
//	{
//		if((16 == u->UltrasonicPacket[left_id].status) || (16 == u->UltrasonicPacket[right_id].status))
//		{
//			rear_boundary = s->getPosition().getX() - REAR_EDGE_TO_CENTER;
//			rear_state = -1;
//		}
//		else
//		{
//			rear_state = -2;
//		}
//	}

//	if((1 == front_state) && (1 != rear_state))
//	{
//		_parking_center_point.X = rear_boundary + 0.6f + REAR_EDGE_TO_CENTER;
//	}
//	else if((1 != front_state) && (1 == rear_state))
//	{
//		_parking_center_point.X = front_boundary - 0.6f + FRONT_EDGE_TO_CENTER;
//	}
//	else if((1 != front_state) && (1 != rear_state))
//	{
//		_parking_center_point.X = rear_boundary + (front_boundary - rear_boundary - LENGHT)*0.5 + REAR_EDGE_TO_CENTER;
//	}
//	else//无穷远
//	{

//	}
}

/**************************************************************************************************/
float Planning::getMinParkingLength()           { return  _min_parking_length;}
void  Planning::setMinParkingLength(float value){ _min_parking_length = value;}

float Planning::getMinParkingWidth()           { return  _min_parking_width;}
void  Planning::setMinParkingWidth(float value){ _min_parking_width = value;}
/**************************************************************************************************/
float Planning::getOuterVirtualBoundary()           { return  _outer_virtual_boundary;}
void  Planning::setOuterVirtualBoundary(float value){ _outer_virtual_boundary = value;}

float Planning::getInsideVirtualBoundary()           { return  _inside_virtual_boundary;}
void  Planning::setInsideVirtualBoundary(float value){ _inside_virtual_boundary = value;}

float Planning::getFrontVirtualBoundary()           { return  _front_virtual_boundary;}
void  Planning::setFrontVirtualBoundary(float value){ _front_virtual_boundary = value;}

float Planning::getRearVirtualBoundary()           { return  _rear_virtual_boundary;}
void  Planning::setRearVirtualBoundary(float value){ _rear_virtual_boundary = value;}
/**************************************************************************************************/
float Planning::getOuterMarginMove()           { return  _outer_margin_move;}
void  Planning::setOuterMarginMove(float value){ _outer_margin_move = value;}

float Planning::getInsideMarginBoundary()           { return  _inside_margin_boundary;}
void  Planning::setInsideMarginBoundary(float value){ _inside_margin_boundary = value;}

float Planning::getFrontMarginBoundary()           { return  _front_margin_boundary;}
void  Planning::setFrontMarginBoundary(float value){ _front_margin_boundary = value;}

float Planning::getRearMarginBoundary()           { return  _rear_margin_boundary;}
void  Planning::setRearMarginBoundary(float value){ _rear_margin_boundary = value;}
/**************************************************************************************************/
uint8_t Planning::getCommand()             { return  _command;}
void    Planning::setCommand(uint8_t value){ _command = value;}

uint8_t Planning::getConsoleState()             { return  _console_state;}
void    Planning::setConsoleState(uint8_t value){ _console_state = value;}

uint8_t Planning::getParkingStatus()             { return  _parking_status;}
void    Planning::setParkingStatus(uint8_t value){ _parking_status = value;}
/**************************************************************************************************/
float Planning::getPlanningBrakingAcc()           { return  planning_braking_acc_;}
void  Planning::setPlanningBrakingAcc(float value){ planning_braking_acc_ = value;}

float Planning::getPlanningBrakingAccR()           { return  planning_braking_acc_r_;}
void  Planning::setPlanningBrakingAccR(float value){ planning_braking_acc_r_ = value;}

float Planning::getPlanningBrakingAeb()           { return  planning_braking_aeb_;}
void  Planning::setPlanningBrakingAeb(float value){ planning_braking_aeb_ = value;}

float Planning::getTurnningFeedforwardTime()           { return  turnning_feedforward_time_;}
void  Planning::setTurnningFeedforwardTime(float value){ turnning_feedforward_time_ = value;}

uint8_t Planning::getAccDisableTime()             { return  acc_disable_time_;}
void    Planning::setAccDisableTime(uint8_t value){ acc_disable_time_ = value;}

float Planning::getPositionMax()           { return  position_max_;}
void  Planning::setPositionMax(float value){ position_max_ = value;}

float Planning::getPositionMin()           { return  position_min_;}
void  Planning::setPositionMin(float value){ position_min_ = value;}

float Planning::getParkingMargin()           { return  parking_margin_;}
void  Planning::setParkingMargin(float value){ parking_margin_ = value;}

float Planning::getKpYaw()           { return  kp_yaw_;}
void  Planning::setKpYaw(float value){ kp_yaw_ = value;}
