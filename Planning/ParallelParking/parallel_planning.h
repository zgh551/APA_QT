/*****************************************************************************/
/* FILE NAME: parallel_planning.h                 PYRIGHT (c) Henry Zhu 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the parallel parking trajectory planning                     */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      January 9  2019      Initial Version                 */
/* 1.0	 Henry Zhu      January 16 2019      Add ReversedTrial Function      */
/* 1.0	 Henry Zhu      January 17 2019      Add TransitionArc Function      */
/*****************************************************************************/
#ifndef PARALLELPARKING_PARALLEL_PLANNING_H_
#define PARALLELPARKING_PARALLEL_PLANNING_H_

#include "./Interaction/HMI/Terminal.h"
#include "./Planning/Interface/planning.h"

// 轨迹规划状态
typedef enum _ParallelPlanningState
{
	WaitStart = 0,
	EnterParkingPointPlanning,
	FirstArcPlanning,
	SteeringTurnningCalculate,
}ParallelPlanningState;

// 平行泊车控制总体状态
typedef enum _ParallelControlState
{
	WaitPlanningFinish = 0,
	InitPointJudge,       //Initial Position Judge State
	InitPointAdjust,
	CurveTrajectory,
	RightFrontTrial,
	LeftRearTrial,
	ParkingComplete
}ParallelControlState;

// 初始位置调整状态
typedef enum _InitPointAdjustState //Initial Position Adjustment Relative State
{
	InitPointFrontAdjust,
	InitPointMove,
	InitPointDisableACC,
	InitPoitArriveJudge,
	WaitVehicleStop,
}InitPointAdjustState;

// 曲线估计段状态
typedef enum _CurveTrajectoryState
{
	GearShift,
	VehicleMove,
	FirstTurnPoint,
	SecondTurnPoint,
	ThirdTurnPoint,
	WaitArrive,
	WaitStill
}CurveTrajectoryState;

typedef enum _RightFrontTrialState
{
	RightFrontTrialGearShift,
	RightFrontTrialVehicleMove,
	RightFrontTrialDisableACC,
	RightFrontTrialWaitArrive,
	RightFrontTrialWaitStill
}RightFrontTrialState;

typedef enum _LeftRearTrialState
{
	LeftRearTrialGearShift,
	LeftRearTrialVehicleMove,
	LeftRearTrialDisableACC,
	LeftRearTrialWaitArrive,
	LeftRearTrialWaitStill
}LeftRearTrialState;

typedef enum _ParkingCompleteState
{
	ParkingCenterAdjust,
	GearShiftJudge,
	FrontMoveAdjust,
	RearMoveAdjust,
	FrontMoveDisableACC,
	RearMoveDisableACC,
	FrontWaitArrive,
	RearWaitArrive,
	FrontMoveStill,
	RearMoveStill,
	ParkingStill,
	WaitParkingGear
}ParkingCompleteState;

class ParallelPlanning : public Planning
{
    Q_OBJECT
public:
	ParallelPlanning();
	virtual ~ParallelPlanning();

	void Init() override;
	void Work(Percaption *p) override;
        void Control(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p) override;

	/***************************************************************/
         int8_t InitPositionAdjustMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *u);
         int8_t CurveTrajectoryMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *u);
         int8_t RightFrontTrialMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *u);
         int8_t LeftRearTrialMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *u);
         int8_t ParkingCompletedMachine(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *u);
	/***************************************************************/

	void ReversedTrial(Percaption *inf);

	void TransitionArc(Percaption *inf);

	void TurnningPoint();

	Line getLineInit();
	void setLineInit(Line value);
	Property<ParallelPlanning,Line,READ_WRITE> LineInit;

protected:
	VehicleBody front_trial_body,rear_trial_body;
	Vector2d    enter_point;

private:
	ParallelPlanningState _parallel_planning_state;
	ParallelControlState  _parallel_control_state;
	///////////////////// state machine //////////////////////////
	InitPointAdjustState _adjust_state;
	CurveTrajectoryState _curve_state;

	RightFrontTrialState _right_front_state;
	LeftRearTrialState   _left_rear_state;

	ParkingCompleteState _parking_complete_state;
	///////////////////////////////////////////////
	uint8_t _reverse_cnt;
	uint8_t _trial_status;

//	Vector2d _parking_left_front;

	VehicleBody _init_parking; // 泊车初始车辆位置
	VehicleBody _enter_parking;// 泊车入库位置

	/********************* Geometry Variable ************************/
	Line   _line_init;
	Line   _line_middle;
	Circle _circle_left;
	Circle _circle_right;

	// tangent point
	Vector2d _line_init_circle_right_tangent;
	Vector2d _line_middle_circle_right_tangent;
	Vector2d _line_middle_circle_left_tangent;

	// turning point
	Turn _line_init_circle_right_turn;
	Turn _line_middle_circle_right_turn;
	Turn _line_middle_circle_left_turn;

	Vector2d _right_front_trial;
	Vector2d _left_rear_trial;

	// trial point
	Vector2d front_trial_arrary[12];
	Vector2d rear_trial_arrary[12];

	float _ahead_distance;
};

#endif /* PARALLELPARKING_PARALLEL_PLANNING_H_ */
