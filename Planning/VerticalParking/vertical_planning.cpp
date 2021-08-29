/*****************************************************************************/
/* FILE NAME: vertical_planning.cpp             COPYRIGHT (c) Henry Zhu 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the parallel parking trajectory planning                     */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      February 13 2019      Initial Version                */
/*****************************************************************************/
#include "vertical_planning.h"

//Terminal m_VerticalPlanningTerminal;

VerticalPlanning::VerticalPlanning() {
	LineInit.setContainer(this);
	LineInit.getter(&VerticalPlanning::getLineInit);
	LineInit.setter(&VerticalPlanning::setLineInit);
	Init();
}

VerticalPlanning::~VerticalPlanning() {

}

void VerticalPlanning::Init()
{
	_outer_parking_boundary = 6;
    _trial_margin = 0.3f;

	_vertical_planning_state = VerticalWaitStart;
	_vertical_control_state  = VerticalWaitPlanningFinish;//初始化控制状态机
	// low level state machine init
	_init_point_adjust_state = VerticalInitPointFrontAdjust;
	_circle_trajectory_state = VerticalGearShift;
	_curve_trajectory_state  = VerticalCurveGearShift;
	_enter_trial_state       = EnterTrialRearGearShift;
	_outer_trial_state       = OuterTrialDriveGearShift;
}

void VerticalPlanning::Work(Percaption *p)
{
	Line temp_line;
	switch(_vertical_planning_state)
	{
		case VerticalWaitStart:
			if(0x60 == Command)
			{
				//泊车状态：规划
				ParkingStatus = 1;
				_vertical_parking_correct = StepOneParkingLocation;
				_vertical_planning_state = ParkingAnalysisState;
			}
			break;

		case ParkingAnalysisState:
			_analysis_state = ParkingAnalysis(p);
			if(FAIL == _analysis_state)
			{
				_vertical_planning_state = VerticalWaitStart;
			}
			else if(1 == _analysis_state)
			{
				_vertical_planning_state = ArcPlanning;
			}
			else if(2 == _analysis_state)
			{
				_vertical_planning_state = TrialPlanning;
			}
			else
			{
				_vertical_planning_state = VerticalWaitStart;
			}
			break;

		case ArcPlanning:
            temp_line.Point.setX( p->getPositionX() );
            temp_line.Point.setY( p->getPositionY() );
			temp_line.Angle   = p->getAttitudeYaw();
			PlanningArc(temp_line);
			Command = 0x70;
			_vertical_planning_state = VerticalWaitStart;
			break;

		case CurvePlanning:
			TransitionCurve(p);
			Command = 0x70;
			_vertical_planning_state = VerticalWaitStart;
			break;

		case TrialPlanning:
            temp_line.Point.setX( p->getPositionX() );
            temp_line.Point.setY( p->getPositionY());
			temp_line.Angle   = p->getAttitudeYaw();
			EnterTrialWithMargin(temp_line);
			Command = 0x70;
			_vertical_planning_state = WaitEnterOuterPlanning;
			break;

		case WaitEnterOuterPlanning:
			if(0x61 == Command)
			{
				_vertical_planning_state = EnterOuterTrialPlanning;
			}
			break;

		case EnterOuterTrialPlanning:
            temp_line.Point.setX( p->getPositionX() );
            temp_line.Point.setY( p->getPositionY() );
			temp_line.Angle   = p->getAttitudeYaw();
			OuterAndEnterTrial(temp_line);
			if(1 == _analysis_state)
			{
				_vertical_planning_state = VerticalWaitStart;
			}
			else if(2 == _analysis_state)
			{
				_vertical_planning_state = WaitEnterOuterPlanning;
			}
			Command = 0x71;
			break;

		default:
			break;
	}
}

void VerticalPlanning::Work(Percaption *p,VehicleState *s)
{
	Line temp_line;
	switch(_vertical_planning_state)
	{
		case VerticalWaitStart:
			if(0x60 == Command)
			{
				//泊车状态：规划
				ParkingStatus = 1;
				_vertical_planning_state = ParkingAnalysisState;
			}
			break;

		case ParkingAnalysisState:
			_analysis_state = ParkingAnalysis(p);
			if(FAIL == _analysis_state)
			{
                Command = 0;
				_vertical_planning_state = VerticalWaitStart;
			}
			else if(1 == _analysis_state)
			{
				_vertical_planning_state = ArcPlanning;
			}
			else if(2 == _analysis_state)
			{
				_vertical_planning_state = TrialPlanning;
			}
			else
			{
				_vertical_planning_state = VerticalWaitStart;
			}
			break;

		case ArcPlanning:
            temp_line.Point.setX( s->getPosition().getX() );
            temp_line.Point.setY( s->getPosition().getY() );
			temp_line.Angle   = s->getYaw();
			PlanningArc(temp_line);
			Command = 0x70;
			_vertical_planning_state = VerticalWaitStart;
			break;

		case CurvePlanning:
			TransitionCurve(p);
			Command = 0x70;
			_vertical_planning_state = VerticalWaitStart;
			break;

		case TrialPlanning:
            temp_line.Point.setX( s->getPosition().getX() );
            temp_line.Point.setY( s->getPosition().getY() );
			temp_line.Angle   = s->getYaw();
			EnterTrialWithMargin(temp_line);
			Command = 0x70;
			_vertical_planning_state = WaitEnterOuterPlanning;
			break;

		case WaitEnterOuterPlanning:
			if(0x61 == Command)
			{
				_vertical_planning_state = EnterOuterTrialPlanning;
			}
			break;

		case EnterOuterTrialPlanning:
			//TODO 实现库位的矫正
            if((p->getValidParkingEdgePosition().FrontOutSide.position - p->getValidParkingEdgePosition().RearOutSide.position).Length() > 3)
			{
                _line_center.Point.setX( (p->getValidParkingEdgePosition().FrontOutSide.position.getX() + p->getValidParkingEdgePosition().RearOutSide.position.getX()) * 0.5 );
			}
            temp_line.Point.setX( s->getPosition().getX() );
            temp_line.Point.setY( s->getPosition().getY() );
			temp_line.Angle   = s->getYaw();
			OuterAndEnterTrial(temp_line);
			Command = 0x71;
			_vertical_planning_state = WaitEnterNewPlanning;
			break;

		case WaitEnterNewPlanning:
			if(0x62 == Command)
			{
				if(1 == _analysis_state)
				{
					_vertical_planning_state = ArcPlanning;
				}
				else if(2 == _analysis_state)
				{
					_vertical_planning_state = TrialPlanning;
				}
			}
			break;

		default:

			break;
	}
}

void VerticalPlanning::Control(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p)
{
	int8_t status;
	switch(_vertical_control_state)
	{
		case VerticalWaitPlanningFinish:
			if( 0x70 == Command )
			{
				// 泊车状态：控制状态
				ParkingStatus = 2;
				Command = 0x00;
				_apa_control_command.ControlEnable.B.APAEnable = 1;
				_apa_control_command.Gear              = Parking;
				_apa_control_command.SteeringAngle     = 0;
				_apa_control_command.SteeringAngleRate = STEERING_RATE;
				_apa_control_command.Distance          = 0;
				_apa_control_command.Velocity          = 0;
				ctl->Update(_apa_control_command);
				_vertical_control_state  = VerticalInitPointJudge;
			}
			break;

		case VerticalInitPointJudge:
			if(1 == _analysis_state)
			{
				if(s->getPosition().getX() < _line_init_circle_parking_enter_turn.Point.getX())
				{
					_vertical_control_state = VerticalInitPointAdjust;
				}
				else
				{
					_vertical_control_state = VerticalCircleTrajectory;
				}
			}
			else if(2 == _analysis_state)
			{
				if(s->getPosition().getX() < _line_to_circle_enter_turn.Point.getX())
				{
					_vertical_control_state = VerticalInitPointAdjust;
				}
				else
				{
					_vertical_control_state = VerticalEnterTrial;
				}
			}
			else
			{
				_vertical_control_state = VerticalWaitPlanningFinish;
			}
			break;

		case VerticalInitPointAdjust:
			if(SUCCESS == InitPositionAdjustMachine(ctl,msg,s,p))
			{
				if(1 == _analysis_state)
				{
					_vertical_control_state = VerticalCircleTrajectory;
				}
				else if(2 == _analysis_state)
				{
					_vertical_control_state = VerticalEnterTrial;
				}
				else
				{

				}
			}
			break;

		case VerticalCircleTrajectory://一次入库的状态控制
			status = CircleTrajectoryMachine(ctl,msg,s,p);
			if(PARKING_FINISH == status)
			{
				ParkingStatus = 3;
				_vertical_control_state = VerticalWaitPlanningFinish;
			}
			break;

		case VerticalEnterTrial:
			status = EnterTrialMachine(ctl,msg,s,p);
			if(SUCCESS == status)
			{
				if(_vertical_parking_correct == StepOneParkingLocation)
				{

				}
				else
				{
					Command = 0x61;
				}
				_vertical_control_state = VerticalWaitMarginPlanFinish;
			}
			break;

		case VerticalWaitMarginPlanFinish:
			if(0x71 == Command)
			{
				_vertical_control_state = VerticalOuterTrial;
			}
			break;

		case VerticalOuterTrial:
			status = OuterTrialMachine(ctl,msg,s,p);
			if(SUCCESS == status)
			{
				Command = 0x62;
				_vertical_control_state = VerticalWaitPlanningFinish;
			}
			break;

		case VerticalCurveTrajectory://保留
			status = CurveTrajectoryMachine(ctl,msg,s,p);
			if(PARKING_FINISH == status)
			{
				_vertical_control_state = VerticalWaitPlanningFinish;
			}
			break;

		case VerticalParkingComplete:

			break;

		default:

			break;
	}
}

int8_t VerticalPlanning::InitPositionAdjustMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p)
{
	switch(_init_point_adjust_state)
	{
		case VerticalInitPointFrontAdjust:
			_apa_control_command.Gear              = Drive;
			_apa_control_command.SteeringAngle     = 0;
			_apa_control_command.SteeringAngleRate = STEERING_RATE;
			_apa_control_command.Distance          = 0;
			_apa_control_command.Velocity          = 0;
			_init_point_adjust_state       = VerticalInitPointMove;
			break;

		case VerticalInitPointMove:
			if((Drive == msg->Gear) && (fabsf(msg->SteeringAngle - _apa_control_command.SteeringAngle ) < STEER_ANGLE_ARRIVE_ERR))
			{
				_apa_control_command.Velocity          = STRAIGHT_VELOCITY;
				_apa_control_command.Distance          = MOTION_DISTANCE;
				_init_point_adjust_state = VerticalWaitVehicleStop;
			}
			break;

		case VerticalWaitVehicleStop:
			if(1 == _analysis_state)
			{
                if(s->getPosition().getX() < (_line_init_circle_parking_enter_turn.Point.getX() + INIT_POINT_MARGIN))
				{
                    _apa_control_command.Distance = _line_init_circle_parking_enter_turn.Point.getX() + INIT_POINT_MARGIN - s->getPosition().getX();
				}
				else
				{
					_apa_control_command.Distance = 0;
				}
			}
			else if(2 == _analysis_state)
			{
                if(s->getPosition().getX() < (_line_to_circle_enter_turn.Point.getX() + INIT_POINT_MARGIN))
				{
                    _apa_control_command.Distance = _line_init_circle_parking_enter_turn.Point.getX() + INIT_POINT_MARGIN - s->getPosition().getX();
				}
				else
				{
					_apa_control_command.Distance = 0;
				}
			}
			else
			{
				_apa_control_command.Distance = 0;
			}
			if(StandStill == msg->WheelSpeedDirection)
			{
				_init_point_adjust_state = VerticalInitPointFrontAdjust;
				return SUCCESS;
			}
			break;

		default:

			break;
	}
	ctl->Update(_apa_control_command);
	return FAIL;
}

// 一次入库，圆弧轨迹控制状态机
int8_t VerticalPlanning::CircleTrajectoryMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p)
{
	VehicleBody motion_body;
	switch(_circle_trajectory_state)
	{
		case VerticalGearShift:
			_apa_control_command.Gear              = Reverse;
			_apa_control_command.SteeringAngle     = 0;
			_apa_control_command.SteeringAngleRate = STEERING_RATE;
			_apa_control_command.Distance          = 0;
			_apa_control_command.Velocity          = 0;
			_circle_trajectory_state = VerticalVehicleMove;
			break;

		case VerticalVehicleMove:
			if((Reverse == msg->Gear) && (fabsf(msg->SteeringAngle - _apa_control_command.SteeringAngle ) < STEER_ANGLE_ARRIVE_ERR))
			{
				_apa_control_command.Velocity          = STRAIGHT_VELOCITY;
				_apa_control_command.Distance          = MOTION_DISTANCE;
				_circle_trajectory_state = VerticalFirstTurnPoint;
			}
			break;

		case VerticalFirstTurnPoint:
			if(1 == _analysis_state)
			{
				// 考虑转向角执行延迟时间
				if( (s->getPosition().getX() -_line_init_circle_parking_enter_turn.Point.getX()) < s->LinearRate * turnning_feedforward_time_)
				{
					_apa_control_command.SteeringAngle = _line_init_circle_parking_enter_turn.SteeringAngle;
					_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
					_circle_trajectory_state = VerticalSecondTurnPoint;
				}
			}
			else if(2 == _analysis_state)
			{
				// 考虑转向角执行延迟时间
				if( (s->getPosition().getX() - _line_to_circle_enter_turn.Point.getX()) < s->LinearRate * turnning_feedforward_time_)
				{
					_apa_control_command.SteeringAngle = _line_to_circle_enter_turn.SteeringAngle;
					_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
					_circle_trajectory_state = VerticalSecondTurnPoint;
				}
			}
			else
			{

			}
			break;

		case VerticalSecondTurnPoint:
			// 根据当前车速，实时更新转向角速度
			_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
			_apa_control_command.Velocity = VelocityPlanningCircle(s,_line_center_circle_parking_enter_turn.Point,_circle_parking_enter.Radius);
			// 象限点判定控制
			if( (PI_2 - s->getYaw())*_circle_parking_enter.Radius < K * MAX_STEERING_ANGLE * 0.5 )
			{
				_apa_control_command.SteeringAngle = _line_center_circle_parking_enter_turn.SteeringAngle;
				_circle_trajectory_state = verticalWaitSteeringArrive;
			}
			//通过转向点判断
//			if(SUCCESS == CircleTurnningPointDetermination(s,_line_center_circle_parking_enter_turn,_circle_parking_enter.Radius,1))
//			{
//				_circle_trajectory_state = verticalWaitSteeringArrive;
//			}

			break;

		case verticalWaitSteeringArrive:
			_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
			if(fabs(msg->SteeringAngle) < 0.1)
			{
				_parking_center_x_middle = 0.5 * (s->getPosition().getX() + _parking_center_point.getX());
                _circle_trajectory_state = VerticalWaitStill;
			}
			break;

		case VerticalWaitYawArrive:
			_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
			//通过偏航角误差消除
//			_apa_control_command.SteeringAngle = kp_yaw_*(s->getYaw() - PI_2)*R2A;
			// 通过x坐标误差消除
//			_apa_control_command.SteeringAngle = kp_yaw_*(s->getPosition().getX() - _parking_center_point.getX());

			if((PI_2 - s->getYaw()) < 0.01)
			{
				_apa_control_command.SteeringAngleRate = STEERING_RATE;
				_apa_control_command.SteeringAngle = 0;
                _circle_trajectory_state = VerticalWaitStill;
			}
			break;

		case VerticalWaitStill:
			_apa_control_command.SteeringAngleRate = MAX_STEERING_ANGLE;//STEERING_RATE;//s->LinearRate * RK;
			// 通过x坐标误差消除
//			_apa_control_command.SteeringAngle = kp_yaw_*(s->getPosition().getX() - _parking_center_x_middle)*100;
			if(s->getPosition().getY() > _parking_center_point.getY())
			{
				_apa_control_command.Distance = s->getPosition().getY() - _parking_center_point.getY();
			}
			else
			{
				_apa_control_command.Distance = 0;
			}
			if(StandStill == msg->WheelSpeedDirection)
			{
				_apa_control_command.Gear    = Parking;
				_circle_trajectory_state = VerticalWaitParkingGear;
			}
			break;

		case VerticalWaitParkingGear:
			if(Parking == msg->Gear)
			{
				ctl->Stop();
				_circle_trajectory_state = VerticalGearShift;
				return PARKING_FINISH;
			}
			break;

		default:

			break;
	}
	ctl->Update(_apa_control_command);
	return FAIL;
}

// 特殊情况，不太实用
int8_t VerticalPlanning::CurveTrajectoryMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p)
{
	float angle_vector;
	switch(_curve_trajectory_state)
	{
		case VerticalCurveGearShift:
			_apa_control_command.Gear              = Reverse;
			_apa_control_command.SteeringAngle     = 0;
			_apa_control_command.SteeringAngleRate = STEERING_RATE;
			_apa_control_command.Distance          = 0;
			_apa_control_command.Velocity          = 0;
			_curve_trajectory_state = VerticalCurveVehicleMove;
			break;

		case VerticalCurveVehicleMove:
			if( (Reverse == msg->Gear) && (fabsf(msg->SteeringAngle - _apa_control_command.SteeringAngle ) < STEER_ANGLE_ARRIVE_ERR))
			{
				_apa_control_command.Velocity = CURVE_VELOCITY;
				_apa_control_command.Distance = MOTION_DISTANCE;
                p->Command = 0x50;//超声重定位算法
				_curve_trajectory_state = VerticalCurveFirstTurnPoint;
			}
			break;

		case VerticalCurveFirstTurnPoint:
			// 考虑转向角执行延迟时间
			if( (s->getPosition().getX() - _line_init_circle_transition_turn.Point.getX()) < s->LinearRate * turnning_feedforward_time_)
			{
				_apa_control_command.SteeringAngle = _line_init_circle_transition_turn.SteeringAngle;
				_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
				_curve_trajectory_state = VerticalCurveSecondTurnPoint;
			}
			break;

		case VerticalCurveSecondTurnPoint:
			// 根据当前车速，实时更新转向角速度
			_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
			// 象限点判定控制
			angle_vector = (s->getPosition() - _line_middle_circle_transition_turn.Point).Angle();
			if(angle_vector > -PI_2 && angle_vector < 0 )
			{
				if(_plan_algebraic_geometry.ArcLength(s->getPosition(), _line_middle_circle_transition_turn.Point, _circle_transition.Radius) < s->LinearRate * turnning_feedforward_time_)
				{
					_apa_control_command.SteeringAngle = _line_middle_circle_transition_turn.SteeringAngle;
					_curve_trajectory_state = VerticalCurveThirdTurnPoint;
				}
			}
			else
			{
				_apa_control_command.SteeringAngle = _line_middle_circle_transition_turn.SteeringAngle;
				_curve_trajectory_state = VerticalCurveThirdTurnPoint;
			}
			break;

		case VerticalCurveThirdTurnPoint:
			// 根据当前车速，实时更新转向角速度
			_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
			// 象限点判定控制
			angle_vector = (s->getPosition() - _line_middle_circle_parking_enter_turn.Point).Angle();
			if(angle_vector > -PI_2 && angle_vector < 0 )
			{
				if((s->getPosition() -_line_middle_circle_parking_enter_turn.Point).Length() < s->LinearRate * turnning_feedforward_time_)
				{
					_apa_control_command.SteeringAngle = _line_middle_circle_parking_enter_turn.SteeringAngle;
					_curve_trajectory_state = VerticalCurveFourthTurnPoint;
				}
			}
			else
			{
				_apa_control_command.SteeringAngle = _line_middle_circle_parking_enter_turn.SteeringAngle;
				_curve_trajectory_state = VerticalCurveFourthTurnPoint;
			}
			break;

		case VerticalCurveFourthTurnPoint:
			// 根据当前车速，实时更新转向角速度
			_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
			// 象限点判定控制
			angle_vector = (s->getPosition() - _line_center_circle_parking_enter_turn.Point).Angle();
			if(angle_vector > 0 && angle_vector < PI_2 )
			{
				if(_plan_algebraic_geometry.ArcLength(s->getPosition(), _line_center_circle_parking_enter_turn.Point, _circle_parking_enter.Radius) < s->LinearRate * turnning_feedforward_time_)
				{
					_apa_control_command.SteeringAngle = _line_center_circle_parking_enter_turn.SteeringAngle;
					_curve_trajectory_state = VerticalCurveWaitStill;
				}
			}
			else
			{
				_apa_control_command.SteeringAngle = _line_center_circle_parking_enter_turn.SteeringAngle;
				_curve_trajectory_state = VerticalCurveWaitStill;
			}
			break;

		case VerticalCurveWaitStill:
			_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
			if(s->getPosition().getY() > _parking_center_point.getY())
			{
				_apa_control_command.Distance = s->getPosition().getY() - _parking_center_point.getY();
			}
			else
			{
				_apa_control_command.Distance = 0;
			}
			if(StandStill == msg->WheelSpeedDirection)
			{
				p->Command = 0x60;
				_curve_trajectory_state = VerticalCurveGearShift;
				return PARKING_FINISH;
			}
			break;

		default:

			break;
	}
	ctl->Update(_apa_control_command);
	return FAIL;
}

// 曲线入库，多次尝试
int8_t VerticalPlanning::EnterTrialMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p)
{
	switch(_enter_trial_state)
	{
		case EnterTrialRearGearShift:
			_apa_control_command.Gear              = Reverse;
			_apa_control_command.SteeringAngle     = 0;
			_apa_control_command.SteeringAngleRate = STEERING_RATE;
			_apa_control_command.Distance          = 0;
			_apa_control_command.Velocity          = 0;
			_enter_trial_state = EnterTrialVehicleMoveRear;
			break;

		case EnterTrialVehicleMoveRear:
			if((Reverse == msg->Gear) && (fabsf(msg->SteeringAngle - _apa_control_command.SteeringAngle ) < STEER_ANGLE_ARRIVE_ERR))
			{
				_apa_control_command.Velocity = CURVE_VELOCITY;
				_apa_control_command.Distance = MOTION_DISTANCE;
//				if(_vertical_parking_correct == StepOneParkingLocation)
//				{
//					p->Command  = 0x50;
//				}
				_enter_trial_state = EnterTrialTurnPoint;
			}
			break;

		case EnterTrialTurnPoint:
			// 考虑转向角执行延迟时间
//			if(SUCCESS == LineTurnningPointDetermination(s,_line_to_circle_enter_turn,1))
//			{
//				_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
//				_enter_trial_state = EnterTrialStopWaitArrive;
//			}
			if( (s->getPosition().getX() - _line_to_circle_enter_turn.Point.getX()) < s->LinearRate * turnning_feedforward_time_)
			{
				_apa_control_command.SteeringAngle     = _line_to_circle_enter_turn.SteeringAngle;
				_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
				_enter_trial_state = EnterTrialStopWaitStill;
			}
			break;

		case EnterTrialStopWaitStill:
			// 根据当前车速，实时更新转向角速度
			_apa_control_command.SteeringAngleRate = s->LinearRate * RK;
            _apa_control_command.Distance = ForecastYawParkingDistance(_circle_enter_stop_point_turn.Yaw,s);
//			_apa_control_command.Distance = p->getObstacleDistance().distance;
			if(StandStill == msg->WheelSpeedDirection)
			{
				if(_vertical_parking_correct == StepOneParkingLocation)
				{
					p->Command = 0x60;
					_vertical_parking_correct = StepTwoEnterParkingRight;
				}
				_enter_trial_state = EnterTrialRearGearShift;
				return SUCCESS;
			}
			else
			{

			}
			break;

		default:

			break;
	}
	ctl->Update(_apa_control_command);
	return FAIL;
}

// 曲线出库，多次尝试
int8_t VerticalPlanning::OuterTrialMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p)
{
	switch(_outer_trial_state)
	{
		case OuterTrialDriveGearShift:
			_apa_control_command.Gear              = Drive;
			_apa_control_command.SteeringAngle     = _stop_point_steering_angle;
			_apa_control_command.SteeringAngleRate = STEERING_RATE;
			_apa_control_command.Distance          = 0;
			_apa_control_command.Velocity          = 0;
			_outer_trial_state = OuterTrialVehicleMoveDrive;
			break;

		case OuterTrialVehicleMoveDrive:
			if((Drive == msg->Gear) && (fabs(msg->SteeringAngle - _stop_point_steering_angle) < STEER_ANGLE_ARRIVE_ERR))
			{
				_apa_control_command.Velocity = CURVE_VELOCITY;
				_apa_control_command.Distance = MOTION_DISTANCE;
				_outer_trial_state = OuterTrialTurnPoint;
			}
			break;

		case OuterTrialTurnPoint:
			// 象限点判定控制
			if(SUCCESS == CircleTurnningPointDetermination(s,_circle_outer_to_line_turn,_circle_outer.Radius,3))
			{
				_outer_trial_state = OuterTrialWaitStill;
			}
			break;


		case OuterTrialWaitStill:
			if(1 == _analysis_state)
			{
				_apa_control_command.Distance = ForecastLineParkingPointMarginDistance(s,_line_init_circle_parking_enter_turn.Point,0.3,3);
			}
			else if(2 == _analysis_state)
			{
				_apa_control_command.Distance = ForecastLineParkingPointMarginDistance(s,_line_to_circle_enter_turn.Point,0.3,3);
			}
			if(StandStill == msg->WheelSpeedDirection)
			{
				_outer_trial_state = OuterTrialDriveGearShift;
				return SUCCESS;
			}
			break;
	}
	ctl->Update(_apa_control_command);
	return FAIL;
}

/******************************************************************************************************************/
// 泊车库位初始位置分析，确定泊车模式
int8_t VerticalPlanning::ParkingAnalysis(Percaption *inf)
{
    _line_init.Point.setX( inf->PositionX );
    _line_init.Point.setY( inf->PositionY );
	_line_init.Angle   = inf->AttitudeYaw;

	// 车位信息发送
//	m_VerticalPlanningTerminal.ParkingMsgSend(inf,FrontMarginBoundary,RearMarginBoundary);
	FrontVirtualBoundary =  inf->ParkingLength - FrontMarginBoundary;
	RearVirtualBoundary  = RearMarginBoundary;

	_parking_outer_front_point = Vector2d(FrontVirtualBoundary,0);
	_parking_outer_rear_point  = Vector2d(RearVirtualBoundary,0);

	// TODO _parking_center_point 可以放到 Planning 类中
    _parking_center_point.setX( (FrontVirtualBoundary + RearVirtualBoundary) * 0.5 );
    _parking_center_point.setY( -FRONT_EDGE_TO_CENTER );
//	m_VerticalPlanningTerminal.ParkingCenterPointSend(_parking_center_point);
	_line_center.Point = _parking_center_point;
	_line_center.Angle = PI_2;

    _circle_parking_enter.Center.setX( _parking_center_point.getX() + MIN_RIGHT_TURN_RADIUS );
	_circle_parking_enter.Radius   = MIN_RIGHT_TURN_RADIUS;
	_plan_vehilce_config.EdgeRadius(-_circle_parking_enter.Radius);

    _min_circle_parking_enter_y = -sqrtf( powf(_circle_parking_enter.Radius - RIGHT_EDGE_TO_CENTER,2) - powf(_circle_parking_enter.Center.getX() - FrontVirtualBoundary,2));
	_max_circle_parking_enter_y = _outer_parking_boundary - _plan_vehilce_config.RadiusFrontLeft;

    _circle_parking_enter.Center.setY( _min_circle_parking_enter_y );
    _critical_boundary = _circle_parking_enter.Center.getY() + _circle_parking_enter.Radius;
    if((RearVirtualBoundary + _plan_vehilce_config.RadiusRearLeft) > _circle_parking_enter.Center.getX()) // 库位长度太小不满足一次入库条件
	{
		return FAIL;
	}
	else//库位足够长，满足一次入库的条件
	{
		if(inf->PositionY < _critical_boundary)// 车辆初始位置不满足一次入库条件
		{
			return 2;
		}
		else
		{
			return 1;
		}
	}
}

// 老函数，目前不用了
void VerticalPlanning::EnterTrial(Line l_init)
{
	Vector2d Ahead;
	float ahead_angle;

	_line_init = l_init;

	_circle_parking_enter.Radius = MIN_RIGHT_TURN_RADIUS;

	_plan_algebraic_geometry.Tangent_LLC(_line_center, _line_init, &_circle_parking_enter, &_line_center_circle_parking_enter_tangent, &_line_init_circle_parking_enter_tangent);

	if( (_circle_parking_enter.Center - _parking_outer_front_point).Length() < (MIN_RIGHT_TURN_RADIUS - RIGHT_EDGE_TO_CENTER))//满足一次入库条件
	{
		_analysis_state = 1;
		// 第一个转向点计算
		_line_init_circle_parking_enter_turn.SteeringAngle = _plan_vehilce_config.SteeringAngleCalculate(-_circle_parking_enter.Radius);
        _ahead_distance = - K * _line_init_circle_parking_enter_turn.SteeringAngle * 0.5f;
		Ahead = Vector2d(_ahead_distance,0);
		_line_init_circle_parking_enter_turn.Point = _line_init_circle_parking_enter_tangent + Ahead.rotate(_line_init.Angle);
//		m_VerticalPlanningTerminal.TurnPointSend(_line_init_circle_parking_enter_turn,0);

		// 第二个转向点计算
		ahead_angle = _ahead_distance / _circle_parking_enter.Radius;
		_line_center_circle_parking_enter_turn.Point = _circle_parking_enter.Center +
				(_line_center_circle_parking_enter_tangent - _circle_parking_enter.Center).rotate(-ahead_angle);
		_line_center_circle_parking_enter_turn.SteeringAngle = 0;
//		m_VerticalPlanningTerminal.TurnPointSend(_line_center_circle_parking_enter_turn,1);
	}
	else// 不能一次入库，需要尝试
	{
		_analysis_state = 2;

		_circle_enter.Radius = MIN_RIGHT_TURN_RADIUS;
		_plan_algebraic_geometry.Tanget_LC_Margin(_line_init,_parking_outer_front_point,_trial_margin,&_circle_enter,&_init_circle_tangent);

		_trial_body.AttitudeYaw = _line_init.Angle;
		_trial_body.Center      = _line_init.Point;
		_trial_body.VerticalTrial(-MIN_RIGHT_TURN_RADIUS, _parking_outer_rear_point);
		_enter_circle_stop_Line.Point = _trial_body.Center;
		_enter_circle_stop_Line.Angle = _trial_body.AttitudeYaw;

		_circle_outer.Radius = MIN_LEFT_TURN_RADIUS;
		_trial_body.RotationCenter(MIN_LEFT_TURN_RADIUS);
		_circle_outer.Center = _trial_body.Rotation;
        _trial_margin = _trial_margin * 0.5f;
		_plan_algebraic_geometry.Tanget_CC_Margin(_circle_outer,_parking_outer_front_point,_trial_margin,&_circle_enter,&_outer_enter_circle_line_tangent);
//		_outer_circle_tangent = _outer_enter_circle_line_tangent.Point;

		// 第一个转向点计算
		_line_to_circle_enter_turn.SteeringAngle = _plan_vehilce_config.SteeringAngleCalculate(-_circle_enter.Radius);
        _ahead_distance = - K * _line_to_circle_enter_turn.SteeringAngle * 0.5f;
		Ahead = Vector2d(_ahead_distance,0);
		_line_to_circle_enter_turn.Point = _init_circle_tangent + Ahead.rotate(_line_init.Angle);
//		m_VerticalPlanningTerminal.TurnPointSend(_line_to_circle_enter_turn,0);

		// 停车点,以切点的形式进行发送
		_circle_enter_stop_point_turn.Point = _enter_circle_stop_Line.Point;
		_circle_enter_stop_point_turn.Yaw   = _enter_circle_stop_Line.Angle;
		_circle_enter_stop_point_turn.SteeringAngle = _plan_vehilce_config.SteeringAngleCalculate(_circle_outer.Radius);
//		m_VerticalPlanningTerminal.TurnPointSend(_circle_enter_stop_point_turn,1);

		// 第二个转向点计算
		ahead_angle = _ahead_distance / _circle_outer.Radius;
		_circle_outer_to_line_turn.Point = _circle_outer.Center +
		(_outer_enter_circle_line_tangent.Point - _circle_outer.Center).rotate(-ahead_angle);
		_circle_outer_to_line_turn.SteeringAngle = 0;
//		m_VerticalPlanningTerminal.TurnPointSend(_circle_outer_to_line_turn,2);

		_next_stage_line_init_circle_enter_turn.SteeringAngle = _plan_vehilce_config.SteeringAngleCalculate(-_circle_enter.Radius);
        _ahead_distance = - K * _next_stage_line_init_circle_enter_turn.SteeringAngle * 0.5f;
		Ahead = Vector2d(_ahead_distance,0);
		_next_stage_line_init_circle_enter_turn.Point = _outer_enter_circle_line_tangent.Point + Ahead.rotate( _outer_enter_circle_line_tangent.Angle );
//		m_VerticalPlanningTerminal.TurnPointSend(_next_stage_line_init_circle_enter_turn,3);
	}
}

// 入库第一步，有余量的入库
void VerticalPlanning::EnterTrialWithMargin(Line l_init)
{
	Vector2d Ahead;

	_line_init = l_init;
	_circle_outer.Radius = MIN_LEFT_TURN_RADIUS;
	_circle_enter.Radius = MIN_RIGHT_TURN_RADIUS;
	_plan_algebraic_geometry.Tanget_LC_Margin(_line_init,_parking_outer_front_point,_trial_margin,&_circle_enter,&_init_circle_tangent);

    emit sCircleCenterPoint(0,&_circle_enter);

	_trial_body.AttitudeYaw = _line_init.Angle;
	_trial_body.Center      = Vector2d(_init_circle_tangent.getX(),_init_circle_tangent.getY());
	_trial_body.VerticalTrial(-MIN_RIGHT_TURN_RADIUS, _parking_outer_rear_point);
	_enter_circle_stop_Line.Point = _trial_body.Center;
	_enter_circle_stop_Line.Angle = _trial_body.AttitudeYaw;
	// 第一个转向点计算
	_line_to_circle_enter_turn.SteeringAngle = _plan_vehilce_config.SteeringAngleCalculate(-_circle_enter.Radius);
    _ahead_distance = - K * _line_to_circle_enter_turn.SteeringAngle * 0.5f;
	Ahead = Vector2d(_ahead_distance,0);
	_line_to_circle_enter_turn.Point = _init_circle_tangent + Ahead.rotate(_line_init.Angle);
//	m_VerticalPlanningTerminal.TurnPointSend(_line_to_circle_enter_turn,0);

	// 停车点,以切点的形式进行发送
	_circle_enter_stop_point_turn.Point         = _enter_circle_stop_Line.Point;
	_circle_enter_stop_point_turn.Yaw           = _enter_circle_stop_Line.Angle;
	_circle_enter_stop_point_turn.SteeringAngle = _plan_vehilce_config.SteeringAngleCalculate(_circle_outer.Radius);
//	m_VerticalPlanningTerminal.TurnPointSend(_circle_enter_stop_point_turn,1);
}

// 带有余量地，同时计算出库和入库的转向点
void VerticalPlanning::OuterAndEnterTrial(Line l_init)
{
	Vector2d Ahead;
	float ahead_angle;

	// 初始化车体对象
	_trial_body.AttitudeYaw = l_init.Angle;
	_trial_body.Center      = l_init.Point;

	// 求左圆的圆心
	_trial_body.RotationCenter(MIN_LEFT_TURN_RADIUS);
	_circle_outer.Center = _trial_body.Rotation;
	_circle_outer.Radius = MIN_LEFT_TURN_RADIUS;

	// 设置右圆的半径
	_circle_enter.Radius = MIN_RIGHT_TURN_RADIUS;

	_plan_algebraic_geometry.Tangent_CCL_VerticalLine(_line_center,_circle_outer,&_circle_enter,&_line_center_circle_parking_enter_tangent,&_line_init_circle_parking_enter_tangent);

    emit sCircleCenterPoint(0,&_circle_enter);
    emit sCircleCenterPoint(1,&_circle_outer);

	if( (_circle_enter.Center - _parking_outer_front_point).Length() < (MIN_RIGHT_TURN_RADIUS - RIGHT_EDGE_TO_CENTER))//满足一次入库条件
	{
		_analysis_state = 1;

		// 第一个转向点计算
		_line_init_circle_parking_enter_turn.SteeringAngle = _plan_vehilce_config.SteeringAngleCalculate(-_circle_enter.Radius);
        _ahead_distance = - K * _line_init_circle_parking_enter_turn.SteeringAngle * 0.5f;
		Ahead = Vector2d(_ahead_distance,0);
		_line_init_circle_parking_enter_turn.Point = _line_init_circle_parking_enter_tangent + Ahead.rotate( (_circle_enter.Center - _circle_outer.Center).Angle() + PI_2);
//		m_VerticalPlanningTerminal.TurnPointSend(_line_init_circle_parking_enter_turn,0);

		// 第二个转向点计算
		ahead_angle = _ahead_distance / _circle_enter.Radius;
		_line_center_circle_parking_enter_turn.Point = _circle_enter.Center +
          (_line_center_circle_parking_enter_tangent - _circle_enter.Center).rotate(-ahead_angle);
		_line_center_circle_parking_enter_turn.SteeringAngle = 0;
//		m_VerticalPlanningTerminal.TurnPointSend(_line_center_circle_parking_enter_turn,1);

		// 使出转向点计算
		_stop_point_steering_angle = _plan_vehilce_config.SteeringAngleCalculate(_circle_outer.Radius);
        _ahead_distance = K * _stop_point_steering_angle * 0.5f;
		ahead_angle = _ahead_distance / _circle_outer.Radius;
		_circle_outer_to_line_turn.Point = _circle_outer.Center +
       (_line_init_circle_parking_enter_tangent - _circle_outer.Center).rotate(-ahead_angle);
		_circle_outer_to_line_turn.SteeringAngle = 0;
//		m_VerticalPlanningTerminal.TurnPointSend(_circle_outer_to_line_turn,2);
//		temp_turn.Point = _line_init_circle_parking_enter_tangent;
//		temp_turn.SteeringAngle = 0;
//		m_VerticalPlanningTerminal.TurnPointSend(temp_turn,3);
	}
	else
	{
		_analysis_state = 2;
        _trial_margin = _trial_margin * 0.8f;
		_plan_algebraic_geometry.Tanget_CC_Margin(_circle_outer,_parking_outer_front_point,_trial_margin,&_circle_enter,&_outer_enter_circle_line_tangent);

		_trial_body.AttitudeYaw = _outer_enter_circle_line_tangent.Angle;
		_trial_body.Center      = _outer_enter_circle_line_tangent.Point;
		_trial_body.VerticalTrial(-MIN_RIGHT_TURN_RADIUS, _parking_outer_rear_point);
		_enter_circle_stop_Line.Point = _trial_body.Center;
		_enter_circle_stop_Line.Angle = _trial_body.AttitudeYaw;

		// 使出转向点计算
		_stop_point_steering_angle = _plan_vehilce_config.SteeringAngleCalculate(_circle_outer.Radius);
        _ahead_distance = K * _stop_point_steering_angle * 0.5f;
		ahead_angle = _ahead_distance / _circle_outer.Radius;
		_circle_outer_to_line_turn.Point = _circle_outer.Center +
       (_outer_enter_circle_line_tangent.Point - _circle_outer.Center).rotate(-ahead_angle);
		_circle_outer_to_line_turn.SteeringAngle = 0;
		_circle_outer_to_line_turn.Yaw = _outer_enter_circle_line_tangent.Angle;
//		m_VerticalPlanningTerminal.TurnPointSend(_circle_outer_to_line_turn,0);

		// 驶入转向点计算
		_line_to_circle_enter_turn.SteeringAngle = _plan_vehilce_config.SteeringAngleCalculate(-_circle_enter.Radius);
        _ahead_distance = - K * _line_to_circle_enter_turn.SteeringAngle * 0.5f;
		Ahead = Vector2d(_ahead_distance,0);
		_line_to_circle_enter_turn.Point = _outer_enter_circle_line_tangent.Point + Ahead.rotate(_outer_enter_circle_line_tangent.Angle);
//		m_VerticalPlanningTerminal.TurnPointSend(_line_to_circle_enter_turn,1);

		// 停车点,以切点的形式进行发送
		_circle_enter_stop_point_turn.Point = _enter_circle_stop_Line.Point;
		_circle_enter_stop_point_turn.Yaw   = _enter_circle_stop_Line.Angle;
		_circle_enter_stop_point_turn.SteeringAngle = _plan_vehilce_config.SteeringAngleCalculate(_circle_outer.Radius);
//		m_VerticalPlanningTerminal.TurnPointSend(_circle_enter_stop_point_turn,2);
	}
}

//一次入库圆弧，规划
void VerticalPlanning::PlanningArc(Line l_init)
{
	Vector2d Ahead;
	float ahead_angle;

	_line_init = l_init;
	_circle_parking_enter.Radius = MIN_RIGHT_TURN_RADIUS;
	_plan_algebraic_geometry.Tangent_LLC(_line_center, _line_init, &_circle_parking_enter, &_line_center_circle_parking_enter_tangent, &_line_init_circle_parking_enter_tangent);
    emit sCircleCenterPoint(0,&_circle_parking_enter);

	// 第一个转向点计算
	_line_init_circle_parking_enter_turn.SteeringAngle = _plan_vehilce_config.SteeringAngleCalculate(-_circle_parking_enter.Radius);
    _ahead_distance = - K * _line_init_circle_parking_enter_turn.SteeringAngle * 0.5f;
	Ahead = Vector2d(_ahead_distance,0);
	_line_init_circle_parking_enter_turn.Point = _line_init_circle_parking_enter_tangent + Ahead.rotate(_line_init.Angle);
//	m_VerticalPlanningTerminal.TurnPointSend(_line_init_circle_parking_enter_turn,0);

	// 第二个转向点计算
	ahead_angle = _ahead_distance / _circle_parking_enter.Radius;
	_line_center_circle_parking_enter_turn.Point = _circle_parking_enter.Center +
			(_line_center_circle_parking_enter_tangent - _circle_parking_enter.Center).rotate(-ahead_angle);
	_line_center_circle_parking_enter_turn.SteeringAngle = 0;
//	m_VerticalPlanningTerminal.TurnPointSend(_line_center_circle_parking_enter_turn,1);
}

// 不用了，不太实用
void VerticalPlanning::TransitionCurve(Percaption *inf)
{
	Line circle_move_line;
	Vector2d cure_middle_point;

	Vector2d Ahead;
	float ahead_angle;

//	_line_init.Point.X = inf->PositionX;
//	_line_init.Point.Y = inf->PositionY;
//	_line_init.Angle   = inf->AttitudeYaw;

    _circle_parking_enter.Center.setY( (_min_circle_parking_enter_y + _max_circle_parking_enter_y) * 0.5 );
	_circle_transition.Radius = MIN_LEFT_TURN_RADIUS;

	_plan_algebraic_geometry.Tangent_CCL_Up(_line_init,_circle_parking_enter,&_circle_transition);

    _line_center_circle_parking_enter_tangent.setX( _parking_center_point.getX() );
    _line_center_circle_parking_enter_tangent.setY( _circle_parking_enter.Center.getY() );
	do
	{
		// 确定圆心移动方向
		circle_move_line.Point = _circle_transition.Center;
		circle_move_line.Angle = _line_init.Angle + PI_4;
		// x方向按照0.1步长移动
        _circle_transition.Center.setX( _circle_transition.Center.getX() + 0.1 );
        _circle_transition.Center.setY( _plan_algebraic_geometry.LinearAlgebra(circle_move_line,_circle_transition.Center.getX()) );
		// 根据新的圆心坐标计算与初始直线的切点和相切圆的半径
		_plan_algebraic_geometry.Tangent_CL(_line_init,&_circle_transition,&_line_init_circle_transition_tangent);
		// 计算左右圆之间切线的切点坐标
		_plan_algebraic_geometry.Tangent_CLC_Up(_circle_parking_enter,_circle_transition,&_line_middle,&_line_middle_circle_parking_enter_tangent,&_line_middle_circle_transition_tangent);

		cure_middle_point = (_line_init_circle_transition_tangent + _line_middle_circle_transition_tangent) * 0.5;
	}
	while(
			(_line_middle_circle_parking_enter_tangent - _line_middle_circle_transition_tangent).Length() < K * _plan_vehilce_config.SteeringAngleCalculate(_circle_parking_enter.Radius) ||
			2 * asinf((_line_init_circle_transition_tangent - cure_middle_point).Length() / _circle_transition.Radius) * _circle_transition.Radius < 1.5 * K * _plan_vehilce_config.SteeringAngleCalculate(_circle_transition.Radius)
	);
    emit sCircleCenterPoint(0,&_circle_parking_enter);
    emit sCircleCenterPoint(1,&_circle_transition);
	// 转向点计算
	// 第一个转向点的计算
	_line_init_circle_transition_turn.SteeringAngle = _plan_vehilce_config.SteeringAngleCalculate(_circle_transition.Radius);
	_ahead_distance = K * _line_init_circle_transition_turn.SteeringAngle * 0.5;
	Ahead = Vector2d(_ahead_distance,0);
	_line_init_circle_transition_turn.Point = _line_init_circle_transition_tangent + Ahead.rotate(_line_init.Angle);
//	m_VerticalPlanningTerminal.TurnPointSend(_line_init_circle_transition_turn,0);

	// 第二个转向点计算
	ahead_angle = _ahead_distance / _circle_transition.Radius;
	_line_middle_circle_transition_turn.Point = _circle_transition.Center +
			                                  ( _line_middle_circle_transition_tangent - _circle_transition.Center).rotate(ahead_angle);
	_line_middle_circle_transition_turn.SteeringAngle = 0;
//	m_VerticalPlanningTerminal.TurnPointSend(_line_middle_circle_transition_turn,1);

	// 第三个转向点计算
	_line_middle_circle_parking_enter_turn.SteeringAngle = _plan_vehilce_config.SteeringAngleCalculate(-_circle_parking_enter.Radius);
	_ahead_distance = - K * _line_middle_circle_parking_enter_turn.SteeringAngle * 0.5;
	Ahead = Vector2d(_ahead_distance,0);
	_line_middle_circle_parking_enter_turn.Point = _line_middle_circle_parking_enter_tangent + Ahead.rotate(_line_middle.Angle);
//	m_VerticalPlanningTerminal.TurnPointSend(_line_middle_circle_parking_enter_turn,2);

	// 第四个转向点
	ahead_angle = _ahead_distance / _circle_parking_enter.Radius;
	_line_center_circle_parking_enter_turn.Point = _circle_parking_enter.Center +
			                                     ( _line_center_circle_parking_enter_tangent - _circle_parking_enter.Center).rotate(-ahead_angle);
	_line_center_circle_parking_enter_turn.SteeringAngle = 0;
//	m_VerticalPlanningTerminal.TurnPointSend(_line_center_circle_parking_enter_turn,3);
}

/**************************************************************************************************/
Line VerticalPlanning::getLineInit()          { return  _line_init;}
void VerticalPlanning::setLineInit(Line value){ _line_init = value;}

