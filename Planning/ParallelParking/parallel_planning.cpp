/*****************************************************************************/
/* FILE NAME: parallel_planning.cpp               COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the parallel parking trajectory planning                     */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      January 9  2019      Initial Version                 */
/* 1.0	 Henry Zhu      January 16 2019      Add ReversedTrial Function      */
/* 1.0	 Henry Zhu      January 17 2019      Add TransitionArc Function      */
/* 1.0	 Henry Zhu      January 21 2019      Add  Control State Machine      */
/* 1.0	 Henry Zhu      January 23 2019      Add  Control Other Machine      */
/*****************************************************************************/
#include "./Planning/ParallelParking/parallel_planning.h"

//Terminal m_ParallelPlanningTerminal;

ParallelPlanning::ParallelPlanning() {
	LineInit.setContainer(this);
	LineInit.getter(&ParallelPlanning::getLineInit);
	LineInit.setter(&ParallelPlanning::setLineInit);
}

ParallelPlanning::~ParallelPlanning() {

}

void ParallelPlanning::Init()
{
	// whole state
	_parallel_planning_state = WaitStart;
	_parallel_control_state  = WaitPlanningFinish;
	///////////////////// state machine //////////////////////////
	_adjust_state = InitPointFrontAdjust;
	_curve_state  = GearShift;
	_right_front_state = RightFrontTrialGearShift;
	_left_rear_state   = LeftRearTrialGearShift;
	_parking_complete_state = ParkingCenterAdjust;

	//
	_trial_status = 0;
	_reverse_cnt = 0;
	_acc_disable_cnt = 0;
	//
	_ahead_distance = 0.0f;
	//
}

void ParallelPlanning::Work(Percaption *p)
{
	switch(_parallel_planning_state)
	{
		case WaitStart:
			if(0x60 == Command)
			{
				ParkingStatus = 1;
				_parallel_planning_state = EnterParkingPointPlanning;
			}
			break;

		case EnterParkingPointPlanning:
			ReversedTrial(p);
			if( (0 == _trial_status) && (_reverse_cnt >= 9))//fail
			{
				Command = 0;
                qDebug("trial more than 9 time");
//				m_ParallelPlanningTerminal.EnterParkingPositionSend(_enter_parking, _reverse_cnt,0);
				_parallel_planning_state = WaitStart;
			}
			else
			{
                qDebug("trial %d time",_reverse_cnt);
//				m_ParallelPlanningTerminal.EnterParkingPositionSend(_enter_parking, _reverse_cnt,0x5A);
				_parallel_planning_state = FirstArcPlanning;
			}
			break;

		case FirstArcPlanning:
			TransitionArc(p);
			_parallel_planning_state = SteeringTurnningCalculate;
			break;

		case SteeringTurnningCalculate:
			TurnningPoint();
			Command = 0x70;
			_parallel_planning_state = WaitStart;
			break;

		default:
			break;
	}
}

void ParallelPlanning::Control(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p)
{
	int8_t status;
	switch(_parallel_control_state)
	{
		case WaitPlanningFinish:
			if( 0x70 == Command )
			{
				ParkingStatus = 2;
				Command = 0x00;
				_apa_control_command.ControlEnable.B.APAEnable   = 1;
				_apa_control_command.Gear              = Parking;
				_apa_control_command.SteeringAngle     = 0;
				_apa_control_command.SteeringAngleRate = STEERING_RATE;
				_apa_control_command.Velocity          = 0;
				_apa_control_command.Distance          = 0;
				ctl->Update(_apa_control_command);
				_parallel_control_state = InitPointJudge;
			}
			break;

		case InitPointJudge:
			if(s->getPosition().getX() < _line_init_circle_right_turn.Point.getX())
			{
				_parallel_control_state = InitPointAdjust;
			}
			else
			{
				_parallel_control_state = CurveTrajectory;
			}
			break;

		case InitPointAdjust:
            if(SUCCESS == InitPositionAdjustMachine(ctl,msg,s,p))
			{
				_parallel_control_state = CurveTrajectory;
			}
			break;

		case CurveTrajectory:
            status = CurveTrajectoryMachine(ctl,msg,s,p);
			if(SUCCESS == status)
			{
				_parallel_control_state = RightFrontTrial;
			}
			else if(PARKING_FINISH == status)
			{
				_parallel_control_state = ParkingComplete;
			}
			break;

		case RightFrontTrial:
            status = RightFrontTrialMachine(ctl,msg,s,p);
			if(SUCCESS == status)
			{
				_parallel_control_state = LeftRearTrial;
			}
			else if(PARKING_FINISH == status)
			{
				_parallel_control_state = ParkingComplete;
			}
			break;

		case LeftRearTrial:
            status = LeftRearTrialMachine(ctl,msg,s,p);
			if(SUCCESS == status)
			{
				_parallel_control_state = RightFrontTrial;
			}
			else if( PARKING_FINISH == status )
			{
				_parallel_control_state = ParkingComplete;
			}
			break;

		case ParkingComplete:
            status = ParkingCompletedMachine(ctl,msg,s,p);
			if(SUCCESS == status)
			{
				ParkingStatus = 3;
				_parallel_control_state = WaitPlanningFinish;
			}
			break;
		default:

			break;
	}
}

int8_t ParallelPlanning::InitPositionAdjustMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p)
{
	switch(_adjust_state)
	{
		case InitPointFrontAdjust:
				_apa_control_command.Gear              = Drive;
				_apa_control_command.SteeringAngle     = 0;
				_apa_control_command.SteeringAngleRate = STEERING_RATE;
				_apa_control_command.Velocity          = 0;
				_apa_control_command.Distance          = 0;
				_adjust_state = InitPointMove;
			break;

		case InitPointMove:
			if( (msg->Gear == Drive) && (fabs(msg->SteeringAngle - _apa_control_command.SteeringAngle) < STEER_ANGLE_ARRIVE_ERR))
			{
				_apa_control_command.Velocity          = STRAIGHT_VELOCITY;
				_apa_control_command.Distance          = MOTION_DISTANCE;
				_adjust_state = WaitVehicleStop;
			}
			break;

		case WaitVehicleStop:
            if(s->getPosition().getX() < (_line_init_circle_right_turn.Point.getX() + INIT_POINT_MARGIN))
			{
                _apa_control_command.Distance = _line_init_circle_right_turn.Point.getX() + INIT_POINT_MARGIN - s->getPosition().getX();
			}
			else
			{
				_apa_control_command.Distance = 0;
			}
			if(StandStill == msg->WheelSpeedDirection)
			{
				_adjust_state = InitPointFrontAdjust;
				return SUCCESS;
			}
			break;

		default:

			break;
	}
	ctl->Update(_apa_control_command);
	return FAIL;
}

int8_t ParallelPlanning::CurveTrajectoryMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p)
{
	switch(_curve_state)
	{
		case GearShift:
			_apa_control_command.Gear              = Reverse;
			_apa_control_command.SteeringAngle     = 0;
			_apa_control_command.SteeringAngleRate = STEERING_RATE;
			_apa_control_command.Velocity          = 0;
			_apa_control_command.Distance          = 0;
			_curve_state = VehicleMove;
			break;

		case VehicleMove:
			if( (Reverse == msg->Gear) && (fabs(msg->SteeringAngle - _apa_control_command.SteeringAngle) < STEER_ANGLE_ARRIVE_ERR))
			{
				_apa_control_command.Velocity = STRAIGHT_VELOCITY;
				_apa_control_command.Distance = MOTION_DISTANCE;
				_curve_state = FirstTurnPoint;
			}
			break;

		case FirstTurnPoint:
			// 考虑转向角执行延迟时间
			if( (s->getPosition().getX() -_line_init_circle_right_turn.Point.getX()) < s->LinearRate * turnning_feedforward_time_)
			{
				_apa_control_command.SteeringAngle     = _line_init_circle_right_turn.SteeringAngle;
				_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
				_curve_state = SecondTurnPoint;
			}
			break;

		case SecondTurnPoint:
			// 根据当前车速，实时更新转向角速度
			_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
			// 象限点判定控制
			// 运行过程中计算转向点
			if( (_line_middle_circle_right_turn.Yaw - s->getYaw()) * _circle_right.Radius < K * fabs(_line_init_circle_right_turn.SteeringAngle) * 0.5 )
			{
				_apa_control_command.SteeringAngle = _line_middle_circle_right_turn.SteeringAngle;
				_curve_state = ThirdTurnPoint;
			}
			// 基于规划的转向点进行转向
//			if(SUCCESS ==  CircleTurnningPointDetermination(s,_line_middle_circle_right_turn,_circle_right.Radius,1))
//			{
//				_curve_state = ThirdTurnPoint;
//			}
			break;

		case ThirdTurnPoint:
			// 根据当前车速，实时更新转向角速度
			_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
			if( (_line_middle_circle_left_tangent - s->getPosition()).Length() < K * fabs(_line_middle_circle_left_turn.SteeringAngle) * 0.5)
			{
				_apa_control_command.SteeringAngle = _line_middle_circle_left_turn.SteeringAngle;
				_curve_state = WaitStill;
			}
			// 基于规划的转向点进行转向
//			if(SUCCESS == LineTurnningPointDetermination(s,_line_middle_circle_left_turn,1))
//			{
//                _apa_control_command.SteeringAngle = _line_middle_circle_left_turn.SteeringAngle;
//				_curve_state = WaitStill;
//			}
			break;

		case WaitStill:
			_apa_control_command.SteeringAngleRate = s->LinearRate * RK;

            if(Normal == p->getRearObstacleDistance().status)
			{
                _apa_control_command.Distance = p->getRearObstacleDistance().distance;
			}
            else if(BlindZone == p->getRearObstacleDistance().status)
            {
                _apa_control_command.Distance = 0;
            }
            else
			{
				_apa_control_command.Distance = BoundaryCollisionDistance(_enter_parking.getAttitudeYaw(),s);
			}

			if(StandStill == msg->WheelSpeedDirection)
			{
				_curve_state = GearShift;
				if( s->Yaw < 0.02)
				{
					return PARKING_FINISH;
				}
				else
				{
					return SUCCESS;
				}
			}
			else
			{
				if( s->Yaw < 0.02f)
				{
					_apa_control_command.SteeringAngle     = 0;
					_apa_control_command.SteeringAngleRate = MAX_STEERING_ANGLE_RATE;
				}
			}
			break;
		default:

			break;
	}
	ctl->Update(_apa_control_command);
	return FAIL;
}

int8_t ParallelPlanning::RightFrontTrialMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p)
{
	switch(_right_front_state)
	{
		case RightFrontTrialGearShift:
			_apa_control_command.Gear              =  Drive;
			_apa_control_command.SteeringAngle     = -MAX_STEERING_ANGLE;
			_apa_control_command.SteeringAngleRate = STEERING_RATE;
			_apa_control_command.Velocity          = 0;
			_apa_control_command.Distance          = 0;
			_right_front_state = RightFrontTrialVehicleMove;
			break;

		case RightFrontTrialVehicleMove:
			if(( msg->Gear == Drive ) && (fabs(msg->SteeringAngle - _apa_control_command.SteeringAngle) < STEER_ANGLE_ARRIVE_ERR))
			{
				_apa_control_command.Velocity = STRAIGHT_VELOCITY;
				_apa_control_command.Distance = MOTION_DISTANCE;
				_right_front_state = RightFrontTrialWaitStill;
			}
			break;

		case RightFrontTrialWaitStill:
            if(Normal == p->getFrontObstacleDistance().status)
            {
                _apa_control_command.Distance = p->getFrontObstacleDistance().distance;
            }
            else if(BlindZone == p->getFrontObstacleDistance().status)
            {
                _apa_control_command.Distance = 0;
            }
            else
            {
                _apa_control_command.Distance = BoundaryCollisionDistance(0,s);
            }

			if(StandStill == msg->WheelSpeedDirection)
			{
				_right_front_state = RightFrontTrialGearShift;
				if( s->Yaw < 0.02)
				{
					return PARKING_FINISH;
				}
				else
				{
					return SUCCESS;
				}
			}
			else
			{
				if( s->Yaw < 0.02f)
				{
					_apa_control_command.SteeringAngle     = 0;
					_apa_control_command.SteeringAngleRate = MAX_STEERING_ANGLE_RATE;
				}
			}
			break;
		default:

			break;
	}
	ctl->Update(_apa_control_command);
	return FAIL;
}

int8_t ParallelPlanning::LeftRearTrialMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p)
{
	switch(_left_rear_state)
	{
		case LeftRearTrialGearShift:
			_apa_control_command.Gear              = Reverse;
			_apa_control_command.SteeringAngle     = MAX_STEERING_ANGLE;
			_apa_control_command.SteeringAngleRate = STEERING_RATE;
			_apa_control_command.Velocity          = 0;
			_apa_control_command.Distance          = 0;
			_left_rear_state = LeftRearTrialVehicleMove;
			break;

		case LeftRearTrialVehicleMove:
            if((msg->getGear() == Reverse) && (fabs(msg->getSteeringAngle() - _apa_control_command.SteeringAngle ) < STEER_ANGLE_ARRIVE_ERR))
			{
				_apa_control_command.Velocity = STRAIGHT_VELOCITY;
				_apa_control_command.Distance = MOTION_DISTANCE;
				_left_rear_state = LeftRearTrialWaitStill;
			}
			break;

		case LeftRearTrialWaitStill:
            if(Normal == p->getRearObstacleDistance().status)
            {
                _apa_control_command.Distance = p->getRearObstacleDistance().distance;
            }
            else if(BlindZone == p->getRearObstacleDistance().status)
            {
                _apa_control_command.Distance = 0;
            }
            else
            {
                _apa_control_command.Distance = BoundaryCollisionDistance(0,s);
            }

			if(StandStill == msg->WheelSpeedDirection)
			{
				_left_rear_state = LeftRearTrialGearShift;
				if( s->Yaw < 0.02)
				{
					return PARKING_FINISH;
				}
				else
				{
					return SUCCESS;
				}
			}
			else
			{
				if( s->Yaw < 0.02f)
				{
					_apa_control_command.SteeringAngle     = 0;
					_apa_control_command.SteeringAngleRate = MAX_STEERING_ANGLE_RATE;
				}
			}
			break;

		default:

			break;
	}
	ctl->Update(_apa_control_command);
	return FAIL;
}

int8_t ParallelPlanning::ParkingCompletedMachine(VehicleController *ctl, MessageManager *msg, VehicleState *s, Percaption *p)
{
	float temp_distance;
	switch(_parking_complete_state)
	{
		case ParkingCenterAdjust:
            ParkingCenterAdjustment(s,p);
//			m_ParallelPlanningTerminal.ParkingCenterPointSend(_parking_center_point);
			_parking_complete_state = GearShiftJudge;
			break;

		case GearShiftJudge:
			_apa_control_command.SteeringAngle     = 0;
			_apa_control_command.SteeringAngleRate = STEERING_RATE;
			if(s->getPosition().getX() < ( _parking_center_point.getX() - PARKING_CENTER_MARGIN ))
			{
				_apa_control_command.Gear =  Drive;
				_parking_complete_state = FrontMoveAdjust;
			}
			else if(s->getPosition().getX() > ( _parking_center_point.getX() + PARKING_CENTER_MARGIN ))
			{
				_apa_control_command.Gear =  Reverse;
				_parking_complete_state = RearMoveAdjust;
			}
			else
			{
				_apa_control_command.Gear =  Parking;
				_parking_complete_state = ParkingStill;
			}
			break;

		case FrontMoveAdjust:
			if((Drive == msg->Gear) && (fabs(msg->SteeringAngle - _apa_control_command.SteeringAngle ) < STEER_ANGLE_ARRIVE_ERR))
			{
				_apa_control_command.Velocity = CURVE_VELOCITY;
				_apa_control_command.Distance = MOTION_DISTANCE;
				_parking_complete_state = FrontWaitArrive;
			}
			break;

		case RearMoveAdjust:
			if((Reverse == msg->Gear) && (fabs(msg->SteeringAngle - _apa_control_command.SteeringAngle ) < STEER_ANGLE_ARRIVE_ERR))
			{
				_apa_control_command.Velocity = CURVE_VELOCITY;
				_apa_control_command.Distance = MOTION_DISTANCE;
				_parking_complete_state = RearWaitArrive;
			}
			break;

		case FrontWaitArrive:
			if(StandStill == msg->WheelSpeedDirection)
			{
				_apa_control_command.Gear   = Parking;
				_parking_complete_state = WaitParkingGear;
			}
			else
			{
				temp_distance = _parking_center_point.getX() - s->getPosition().getX();
				if(temp_distance >= 0)
				{
					_apa_control_command.Distance = temp_distance;
				}
				else
				{
					_apa_control_command.Distance = 0;
				}
			}
			break;

		case RearWaitArrive:
			if(StandStill == msg->WheelSpeedDirection)
			{
				_apa_control_command.Gear   = Parking;
				_parking_complete_state = WaitParkingGear;
			}
			else
			{
				temp_distance = s->getPosition().getX() - _parking_center_point.getX();
				if(temp_distance >= 0)
				{
					_apa_control_command.Distance = temp_distance;
				}
				else
				{
					_apa_control_command.Distance = 0;
				}
			}
			break;

		case ParkingStill:
			if(StandStill == msg->WheelSpeedDirection)
			{
				_apa_control_command.Gear   = Parking;
				_parking_complete_state = WaitParkingGear;
			}
			break;

		case WaitParkingGear:
			if( Parking == msg->Gear)
			{
				ctl->Stop();
				_parking_complete_state = GearShiftJudge;
				return SUCCESS;
			}
			break;

		default:

			break;
	}
	ctl->Update(_apa_control_command);
	return FAIL;
}

void ParallelPlanning::ReversedTrial(Percaption *inf)
{
	// 车位信息发送
//	m_ParallelPlanningTerminal.ParkingMsgSend(inf,FrontMarginBoundary,RearMarginBoundary);
    // 车辆初始位置信息
//	_init_parking.Center      = Vector2d(inf->PositionX,inf->PositionY);
//	_init_parking.AttitudeYaw = inf->AttitudeYaw;
	// TODO 终端信息 车辆初始位置信息
//	m_ParallelPlanningTerminal.VehicleInitPositionSend(_init_parking);
	/// 车位虚拟边界计算
	InsideVirtualBoundary = -inf->ParkingWidth  + InsideMarginBoundary;
	FrontVirtualBoundary =  inf->ParkingLength - FrontMarginBoundary;
	RearVirtualBoundary  = RearMarginBoundary;
	// 车库点计算
	_parking_inside_rear_point  = Vector2d(RearVirtualBoundary,InsideVirtualBoundary);
	_parking_inside_front_point = Vector2d(FrontVirtualBoundary,InsideVirtualBoundary);
	_parking_outer_front_point = Vector2d(FrontVirtualBoundary,0);

	// 根据车位宽度，确定车辆最终停车的横向位置
	_plan_vehilce_config.EdgeRadius(MIN_LEFT_TURN_RADIUS);
	MinParkingWidth  = LEFT_EDGE_TO_CENTER + _plan_vehilce_config.RadiusRearRight - MIN_LEFT_TURN_RADIUS + InsideMarginBoundary;
	if(inf->ParkingWidth >= MinParkingWidth)//库位宽度足够
	{
        enter_point.setY( -LEFT_EDGE_TO_CENTER + OuterMarginMove );
	}
	else //库位宽度太小，调整y轴方向位置
	{
        enter_point.setY( -LEFT_EDGE_TO_CENTER + MinParkingWidth - inf->ParkingWidth + InsideMarginBoundary + OuterMarginMove );
	}
    _parking_center_point = Vector2d( RearVirtualBoundary + (FrontVirtualBoundary - RearVirtualBoundary - LENGHT)*0.5 + REAR_EDGE_TO_CENTER,enter_point.getY());

//	m_ParallelPlanningTerminal.ParkingCenterPointSend(_parking_center_point);
	// 根据车位长度，确定车辆最终的纵向位置
    MinParkingLength = REAR_EDGE_TO_CENTER + sqrtf(powf(_plan_vehilce_config.RadiusFrontRight,2) - powf(MIN_LEFT_TURN_RADIUS + enter_point.getY(),2));
	if( inf->ParkingLength > (MinParkingLength + FrontMarginBoundary + RearMarginBoundary))//满足一次入库条件
	{
        enter_point.setX( RearMarginBoundary  + REAR_EDGE_TO_CENTER + (inf->ParkingLength - FrontMarginBoundary - RearMarginBoundary - MinParkingLength)*0.5 );
		_enter_parking.setCenter(enter_point);
		_enter_parking.setAttitudeYaw(0.0f);
		_enter_parking.RotationCenter(MIN_LEFT_TURN_RADIUS);

		_trial_status = 1;
		rear_trial_body.Center = enter_point;
		rear_trial_body.AttitudeYaw = 0.0f;
//		m_ParallelPlanningTerminal.RearTrialPositionSend(rear_trial_body,_reverse_cnt);
	}
	else//不满足一次入库，需多次尝试
	{
        enter_point.setX( inf->ParkingLength - FrontMarginBoundary - FRONT_EDGE_TO_CENTER );
		front_trial_body.Center = enter_point;
		front_trial_body.AttitudeYaw = 0.0f;
		front_trial_arrary[_reverse_cnt] = enter_point;

        enter_point.setX( RearMarginBoundary  + REAR_EDGE_TO_CENTER );
		rear_trial_body.Center = enter_point;
		rear_trial_body.AttitudeYaw = 0.0f;
		rear_trial_arrary[_reverse_cnt]  = enter_point;

//		m_ParallelPlanningTerminal.FrontTrialPositionSend(front_trial_body,_reverse_cnt);
//		m_ParallelPlanningTerminal.RearTrialPositionSend(rear_trial_body,_reverse_cnt);
		while( (0 == _trial_status) && (_reverse_cnt < 9))
		{
			/*+--------------+----+----+----+*/
			/*+ Init Status  + 1  + 2  + 3  +*/
			/*+--------------+----+----+----+*/
			/*+    Front     + -R + +L + -R +*/
			/*+--------------+----+----+----+*/
			/*+    Rear      + +L + -R + +L +*/
			/*+--------------+----+----+----+*/
			_reverse_cnt++;
			if(	_reverse_cnt % 2 )// 1 3 5
			{
				front_trial_body.OneTrial(-MIN_RIGHT_TURN_RADIUS, _parking_inside_rear_point);
				rear_trial_body.OneTrial(MIN_LEFT_TURN_RADIUS, _parking_inside_front_point);
//				m_ParallelPlanningTerminal.FrontTrialPositionSend(front_trial_body,_reverse_cnt);
//				m_ParallelPlanningTerminal.RearTrialPositionSend(rear_trial_body,_reverse_cnt);

				front_trial_body.RotationCenter(MIN_LEFT_TURN_RADIUS);
				_plan_vehilce_config.EdgeRadius(MIN_LEFT_TURN_RADIUS);
				_trial_status = (front_trial_body.getRotation() - _parking_outer_front_point).Length() >= (_plan_vehilce_config.RadiusFrontRight + 0.3) ? 1 : 0;
				if(_trial_status)
				{
					front_trial_body.EdgePoint();
					_enter_parking = front_trial_body;
				}
			}
			else // 2 4 6
			{
				front_trial_body.OneTrial(MIN_LEFT_TURN_RADIUS, _parking_inside_front_point);
				rear_trial_body.OneTrial(-MIN_RIGHT_TURN_RADIUS, _parking_inside_rear_point);
//				m_ParallelPlanningTerminal.FrontTrialPositionSend(front_trial_body,_reverse_cnt);
//				m_ParallelPlanningTerminal.RearTrialPositionSend(rear_trial_body,_reverse_cnt);

				rear_trial_body.RotationCenter(MIN_LEFT_TURN_RADIUS);
				_plan_vehilce_config.EdgeRadius(MIN_LEFT_TURN_RADIUS);
				_trial_status = (rear_trial_body.getRotation() - _parking_outer_front_point).Length() >= (_plan_vehilce_config.RadiusFrontRight + 0.3) ? 1 : 0;
				if(_trial_status)
				{
					rear_trial_body.EdgePoint();
					_enter_parking = rear_trial_body;
				}
			}
			front_trial_arrary[_reverse_cnt]      = front_trial_body.Center;
			rear_trial_arrary[_reverse_cnt]       = rear_trial_body.Center;
		}
	}
}

void ParallelPlanning::TransitionArc(Percaption *inf)
{
	Line cr_line;

	//圆心和直线变量初始化
    _line_init.Point.setX( inf->PositionX );
    _line_init.Point.setY( inf->PositionY );
	_line_init.Angle   = inf->AttitudeYaw;

	_circle_left.Center = _enter_parking.getRotation();
	_circle_left.Radius = MIN_LEFT_TURN_RADIUS;

    emit sCircleCenterPoint(1,&_circle_left);

	_circle_right.Radius = MIN_RIGHT_TURN_RADIUS;
	// 计算初始右侧圆心位置
	_plan_algebraic_geometry.Tangent_CCL(_line_init,_circle_left,&_circle_right);
	do
	{
		// 沿右下方向移动圆心坐标
		cr_line.Point = _circle_right.Center;
		cr_line.Angle = _line_init.Angle - PI_4;
        _circle_right.Center.setX( _circle_right.Center.getX() + 0.1 );
        _circle_right.Center.setY( _plan_algebraic_geometry.LinearAlgebra(cr_line, _circle_right.Center.getX()) );
		// 重新根据右圆心坐标计算右圆半径和切点坐标
		_plan_algebraic_geometry.Tangent_CL(_line_init,&_circle_right,&_line_init_circle_right_tangent);
		// 计算左右圆之间切线的切点坐标
		_plan_algebraic_geometry.Tangent_CLC(_circle_left, _circle_right,&_line_middle,&_line_middle_circle_left_tangent,&_line_middle_circle_right_tangent);
        emit sCircleCenterPoint(0,&_circle_right);
	}
	//碰撞判定
	while(
			(_circle_right.Radius - RIGHT_EDGE_TO_CENTER) < (_parking_outer_front_point - _circle_right.Center).Length() ||
			(_line_middle_circle_left_tangent - _line_middle_circle_right_tangent).Length() < 2 * K * _plan_vehilce_config.SteeringAngleCalculate(_circle_right.Radius)
	);
    emit sCircleCenterPoint(1,&_circle_left);
    emit sCircleCenterPoint(0,&_circle_right);
    qDebug("middle circle left tangent point:x=%f,y=%f",_line_middle_circle_left_tangent.getX(),_line_middle_circle_left_tangent.getY());
    qDebug("middle circle right tangent point:x=%f,y=%f",_line_middle_circle_right_tangent.getX(),_line_middle_circle_right_tangent.getY());
}

void ParallelPlanning::TurnningPoint()
{
	Vector2d Ahead;
	float ahead_angle;

	// line first point
	_line_init_circle_right_turn.SteeringAngle = _plan_vehilce_config.SteeringAngleCalculate(-_circle_right.Radius);
	_ahead_distance = - K * _line_init_circle_right_turn.SteeringAngle * 0.5;
	Ahead = Vector2d(_ahead_distance,0);
	_line_init_circle_right_turn.Point = _line_init_circle_right_tangent + Ahead.rotate(_line_init.Angle);
//	m_ParallelPlanningTerminal.TurnPointSend(_line_init_circle_right_turn,0);
	// turning arc:sencond point
	ahead_angle = _ahead_distance / _circle_right.Radius;
	_line_middle_circle_right_turn.Point = _circle_right.Center +
   (_line_middle_circle_right_tangent    - _circle_right.Center).rotate(-ahead_angle);
	_line_middle_circle_right_turn.SteeringAngle = 0;
	_line_middle_circle_right_turn.Yaw = _line_middle.Angle;
//	m_ParallelPlanningTerminal.TurnPointSend(_line_middle_circle_right_turn,1);
	// line:third point
	_line_middle_circle_left_turn.SteeringAngle = _plan_vehilce_config.SteeringAngleCalculate(_circle_left.Radius);
    _ahead_distance = K* _line_middle_circle_left_turn.SteeringAngle * 0.5f;
	Ahead = Vector2d(_ahead_distance,0);
	_line_middle_circle_left_turn.Point = _line_middle_circle_left_tangent + Ahead.rotate(_line_middle.Angle);
//	m_ParallelPlanningTerminal.TurnPointSend(_line_middle_circle_left_turn,2);
}

/**************************************************************************************************/
Line ParallelPlanning::getLineInit()          { return  _line_init;}
void ParallelPlanning::setLineInit(Line value){ _line_init = value;}

