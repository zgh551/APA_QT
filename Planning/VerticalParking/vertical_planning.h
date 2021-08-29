/*****************************************************************************/
/* FILE NAME: vertical_planning.h               COPYRIGHT (c) Henry Zhu 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the parallel parking trajectory planning                     */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      February 13 2019      Initial Version                */
/*****************************************************************************/

#ifndef VERTICALPARKING_VERTICAL_PLANNING_H_
#define VERTICALPARKING_VERTICAL_PLANNING_H_

#include "./Interaction/HMI/Terminal.h"
#include "./Planning/Interface/planning.h"
// 垂直轨迹规划状态
typedef enum _VerticalPlanningState
{
	VerticalWaitStart = 0,
	ParkingAnalysisState,
	CurvePlanning,
	ArcPlanning,
	TrialPlanning,
	WaitEnterOuterPlanning,
	EnterOuterTrialPlanning,
	WaitEnterNewPlanning,
//	NewArcPlanning,
//	NewTrialPlanning,
}VerticalPlanningState;

// 平行泊车控制总体状态
typedef enum _VerticalControlState
{
	VerticalWaitPlanningFinish = 0,
	VerticalInitPointJudge,       //Initial Position Judge State
	VerticalInitPointAdjust,
	VerticalCircleTrajectory,
	VerticalEnterTrial,
	VerticalOuterTrial,
	VerticalWaitMarginPlanFinish,
	VerticalCurveTrajectory,
	VerticalParkingComplete
}VerticalControlState;

// 初始位置调整状态
typedef enum _VerticalInitPointAdjustState //Initial Position Adjustment Relative State
{
	VerticalInitPointFrontAdjust,
	VerticalInitPointMove,
	VerticalInitPointDisableACC,
	VerticalInitPoitArriveJudge,
	VerticalWaitVehicleStop,
}VerticalInitPointAdjustState;

// 弧线估计段状态
typedef enum _VerticalCircleTrajectoryState
{
	VerticalGearShift,
	VerticalVehicleMove,
	VerticalDisableACC,
	VerticalFirstTurnPoint,
	VerticalSecondTurnPoint,
	verticalWaitSteeringArrive,
	VerticalWaitYawArrive,
	VerticalWaitArrive,
	VerticalWaitStill,
	VerticalWaitParkingGear
}VerticalCircleTrajectoryState;

// 曲线估计段状态
typedef enum _VerticalCurveTrajectoryState
{
	VerticalCurveGearShift,
	VerticalCurveVehicleMove,
	VerticalCurveFirstTurnPoint,
	VerticalCurveSecondTurnPoint,
	VerticalCurveThirdTurnPoint,
	VerticalCurveFourthTurnPoint,
	VerticalCurveWaitArrive,
	VerticalCurveWaitStill
}VerticalCurveTrajectoryState;

// 曲线估计段状态
typedef enum _VerticalTrialTrajectoryState
{
	VerticalTrialRearGearShift,
	VerticalTrialVehicleMoveRear,
	VerticalTrialFirstTurnPoint,
	VerticalTrialStopWaitArrive,
	VerticalTrialStopWaitStill,
	VerticalTrialDriveGearShift,
	VerticalTrialVehicleMoveDrive,
	VerticalTrialSencondTurnPoint,
	VerticalTrialWaitArrive,
	VerticalTrialWaitStill
}VerticalTrialTrajectoryState;


// 进入尝试状态
typedef enum _EnterTrialState
{
	EnterTrialRearGearShift,
	EnterTrialVehicleMoveRear,
	EnterTrialDisableACC,
	EnterTrialTurnPoint,
	EnterTrialStopWaitArrive,
	EnterTrialStopWaitStill
}EnterTrialState;

// 驶出尝试状态
typedef enum _OuterTrialState
{
	OuterTrialDriveGearShift,
	OuterTrialVehicleMoveDrive,
//	OuterTrialDisableACC,
	OuterTrialTurnPoint,
//	OuterTrialWaitArrive,
	OuterTrialWaitStill
}OuterTrialState;

typedef enum _ParkingCorrectStatus
{
	StepOneParkingLocation,
	StepTwoEnterParkingRight
}ParkingCorrectStatus;

class VerticalPlanning : public Planning
{
    Q_OBJECT
public:
	VerticalPlanning();
	virtual ~VerticalPlanning();

	void Init() override;
	void Work(Percaption *p) override;
	void Work(Percaption *p,VehicleState *s);
    void Control(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p) override;
	/*
	 * 基于感知的
	 * */
//	void Control(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p);
	/************************************************************************************************/
	/************************************ branch state machine **************************************/
	/************************************************************************************************/
	int8_t InitPositionAdjustMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p);
	int8_t CircleTrajectoryMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p);
	// 不加修正的版本
	int8_t CurveTrajectoryMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p);
//	int8_t TrialTrajectoryMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p);
	int8_t EnterTrialMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p);
	int8_t OuterTrialMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p);
	/************************************************************************************************/
	// Planning function
	int8_t ParkingAnalysis(Percaption *inf);
	void EnterTrial(Line l_init);
	void EnterTrialWithMargin(Line l_init);
	void OuterAndEnterTrial(Line l_init);
	void PlanningArc(Line l_init);
	void TransitionCurve(Percaption *inf);

	Line getLineInit();
	void setLineInit(Line value);
	Property<VerticalPlanning,Line,READ_WRITE> LineInit;
private:
	// Overall state machine
	VerticalPlanningState _vertical_planning_state;
	VerticalControlState  _vertical_control_state;

	VerticalInitPointAdjustState _init_point_adjust_state;
	VerticalCircleTrajectoryState _circle_trajectory_state;
	VerticalCurveTrajectoryState  _curve_trajectory_state;
//	VerticalTrialTrajectoryState  _trial_trajectory_state;
	EnterTrialState _enter_trial_state;
	OuterTrialState _outer_trial_state;

	float _critical_boundary;
	float _outer_parking_boundary;
	float _max_circle_parking_enter_y;
	float _min_circle_parking_enter_y;
	/***********************************/
	uint8_t _trial_state;
	VehicleBody _trial_body;
	//trial circle
	Circle _circle_enter;
	Circle _circle_outer;

	// tangent point
	Vector2d _init_circle_tangent;
	Line _enter_circle_stop_Line;

	float _parking_center_x_middle;
	// 一次入库的
	float _stop_point_steering_angle;
	Turn _circle_outer_to_line_turn;
	Turn _line_to_circle_enter_turn;
	Turn _circle_enter_stop_point_turn;

	Line _outer_enter_circle_line_tangent;

	Turn _next_stage_line_init_circle_enter_turn;
	/***********************************/
	//
	Circle _circle_parking_enter;
	Circle _circle_transition;
	Line   _line_init;	 // 车辆初始位置所在的直线
	Line   _line_center; // 最终车辆入库时的中心线
	Line   _line_middle; // 两圆中间插入的直线
	// tangent point
	Vector2d _line_init_circle_transition_tangent;

	Vector2d _line_middle_circle_transition_tangent;
	Vector2d _line_middle_circle_parking_enter_tangent;

	Vector2d _line_init_circle_parking_enter_tangent;
	Vector2d _line_center_circle_parking_enter_tangent;

	// turning point
	Turn _line_init_circle_parking_enter_turn;

	Turn _line_init_circle_transition_turn;
	Turn _line_middle_circle_transition_turn;
	Turn _line_middle_circle_parking_enter_turn;

	Turn _line_center_circle_parking_enter_turn;

	float _trial_margin;
	float _ahead_distance;
	int8_t _analysis_state;

	ParkingCorrectStatus _vertical_parking_correct;
};

#endif /* VERTICALPARKING_VERTICAL_PLANNING_H_ */
