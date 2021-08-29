/*****************************************************************************/
/* FILE NAME: path_plannig.h                    COPYRIGHT (c) Henry Zhu 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the trajectory planning interface  					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu     January 9 2019      Initial Version                  */
/*****************************************************************************/

#ifndef INTERFACE_PATH_PLANNING_H_
#define INTERFACE_PATH_PLANNING_H_
// Common
#include <QMainWindow>

#include "./Common/Utils/Inc/property.h"
#include "./Percaption/Interface/percaption.h"
// math
#include "math.h"
#include "./Common/Math/vector_2d.h"
#include "./Common/Math/vehicle_body.h"
#include "./Common/Math/algebraic_geometry.h"
//configure
#include "./Common/Configure/Configs/vehilce_config.h"
// Controller
#include "./Interaction/CANBUS/Interface/vehicle_controller.h"
//#include "ChangAn/chang_an_controller.h"
// Track
#include "./Common/VehicleState/GeometricTrack/geometric_track.h"
// perception
#include "./Interaction/Ultrasonic/Ultrasonic.h"

//state
#define PARKING_FINISH        ( 1   ) // 泊车完成的状态标志
//
#define STEERING_RATE         ( 300 ) // 转向角速度
#define EMERGENCY_BRAKING	  (-2   ) // 紧急制动的减速度
#define PLANNING_BRAKING	  (-1  )  // 规划减速度
#define PLANNING_BRAKING_R	  ( 0.1f ) // 规划减速度的倒数
#define STRAIGHT_VELOCITY	  ( 0.8f ) // 直线段的速度
#define CURVE_VELOCITY	      ( 0.6f ) // 曲线段的速度
#define MOTION_DISTANCE       ( 5.0f ) // 车辆满足运动的距离
#define TURN_FEEDFORWARD_TIME ( 0.0f ) // 转向角前向反馈的补偿时间
#define PARKING_CENTER_MARGIN ( 0.05f ) // 泊车中心点余量
#define ACC_DISABLE_TIME      ( 50 ) // ACC失效时间
#define STEER_ANGLE_ARRIVE_ERR ( 5 )  // 转向角允许的到位误差
#define INIT_POINT_MARGIN      ( 0.3 ) // 初始位置调整的余量
/**************************速度控制******************************/
#define POSITION_A            ( 0.4f ) // 速度控制下限点
#define POSITION_B            ( 0.8f ) // 速度控制上限点
/**************************泊车余量******************************/
#define PARKING_MARGIN        ( 0.2f ) // 停车余量
/*************************END********************************/
//extern GeometricTrack    m_GeometricTrack;
//extern ChangAnController m_ChangAnController;
#define K  0.0016f   //  0.8/500
#define RK 625
//#define K  0.001   //  0.5/500
//#define RK 1000
typedef struct _Turn
{
	Vector2d Point;
	float Yaw;
	float SteeringAngle;
}Turn;

class Planning : public QObject
{
    Q_OBJECT
public:
	Planning();
	virtual ~Planning();

	virtual void Init() = 0;
	virtual void Work(Percaption *p) = 0;
        virtual void Control(VehicleController *ctl,MessageManager *msg,VehicleState *s,Percaption *p) = 0;
	/***************************************************************************************************/
	// 圆弧上提前停车判定，使车辆停在停止点
	int8_t ForecastCircleParking(VehicleState *s,Vector2d stop_point,float radius,uint8_t quadrant);
	// 圆弧上提前停车判定，根据停止点与车辆中心点的距离，使车辆停留在停止点前方
	int8_t ForecastCircleBoundaryMargin(VehicleState *s,Vector2d stop_point,float radius,float margin);
	// 圆弧上提前停车判定，根据停止点与车辆中心点的距离，使车辆停留在停止点上、前方或者后方
	// quadrant:判定运动点的象限值
	// mode:选择停车模式-> -1 --> 提前margin值停车
	//                  0 --> 停在停止点上
	//                  1 --> 滞后margin值停车
	int8_t ForecastCircleParkingPointMargin(VehicleState *s,Vector2d stop_point,float radius,float margin,uint8_t quadrant,int8_t mode);

	int8_t ForecastLineParkingPointMargin(VehicleState *s,Vector2d stop_point,float margin,uint8_t quadrant,int8_t mode);

	// 输出距离值
	float ForecastLineParkingPointMarginDistance(VehicleState *s,Vector2d stop_point,float margin,uint8_t quadrant);
	/********************转向点判定************************************************************************/
	// 弧线上的转向点判定，根据实际车辆控制延迟决定提前量
	int8_t CircleTurnningPointDetermination(VehicleState *s,Turn turn_point,float radius,uint8_t quadrant);
	// 直线上的转向点判定，根据实际车辆控制延迟决定提前量
	int8_t LineTurnningPointDetermination(VehicleState *s,Turn turn_point,uint8_t quadrant);
	/******************** 通过偏航角判定停车条件 *****************************************************************/
	/**
	 * 车辆偏航角满足要求预估停车，使车辆的偏航角满足目标值
	 * 适用于圆弧上的停车点估计
	 * state: -1 -> 偏航角趋势变小； 1 -> 偏航角趋势变大
	 * radius: 圆弧的半径
	 * target_yaw: 目标偏航角大小(弧度)
	 * s : 车辆实时位置和姿态信息
	 * */
	int8_t ForecastYawParking(int8_t state,float radius,float target_yaw,VehicleState *s);

	/**
	 * 车辆偏航角满足要求预估停车，使车辆的偏航角满足目标值
	 * 适用于圆弧上的停车点估计
	 * state: -1 -> 偏航角趋势变小； 1 -> 偏航角趋势变大
	 * radius: 圆弧的半径
	 * target_yaw: 目标偏航角大小(弧度)
	 * s : 车辆实时位置和姿态信息
	 * */
	float ForecastYawParkingDistance(float target_yaw,VehicleState *s);
	/******************** 边界障碍物碰撞判定 *****************************************************************/
	// 边界障碍物停车
	int8_t BoundaryCollision(int8_t motion,VehicleState *s);
	// 通过速度控制车辆在边界处停止
	int8_t BoundaryCollisionVelocity(int8_t motion,float target,VehicleState *s);
	// 通过弧线距离判定停止
	int8_t BoundaryCollisionCircle(int8_t motion,VehicleState *s);

	// 车辆离障碍物的距离
	float BoundaryCollisionDistance(float target,VehicleState *s);
	// 基于超声波的避障停车
	int8_t UltrasonicCollision(int8_t motion,VehicleState *s,Ultrasonic *u);

	int8_t UltrasonicCollisionDiatance(Ultrasonic *u);

	/********************速度规划************************************************************************/
	// 弧线上的速度规划 速度根据目标距离重规划
	float VelocityPlanningCircle(VehicleState *s,Vector2d stop_point,float radius);
	// 直线上的速度规划
	float VelocityPlanningLine(VehicleState *s,Vector2d stop_point);
	/***************************************************************************************************/
	int8_t WaitVehicleStartMove(uint8_t d,MessageManager *msg);
	/***************************************泊车完成后的车位调整************************************************************/
        void ParkingCenterAdjustment(VehicleState *s,Percaption *p);
	/***************************************************************************************************/
	float getMinParkingLength();
	void  setMinParkingLength(float value);
	Property<Planning,float,READ_WRITE> MinParkingLength;

	float getMinParkingWidth();
	void  setMinParkingWidth(float value);
	Property<Planning,float,READ_WRITE> MinParkingWidth;

	/***************************************************************/
	float getOuterVirtualBoundary();
	void  setOuterVirtualBoundary(float value);
	Property<Planning,float,READ_WRITE> OuterVirtualBoundary;

	float getInsideVirtualBoundary();
	void  setInsideVirtualBoundary(float value);
	Property<Planning,float,READ_WRITE> InsideVirtualBoundary;

	float getFrontVirtualBoundary();
	void  setFrontVirtualBoundary(float value);
	Property<Planning,float,READ_WRITE> FrontVirtualBoundary;

	float getRearVirtualBoundary();
	void  setRearVirtualBoundary(float value);
	Property<Planning,float,READ_WRITE> RearVirtualBoundary;
	/***************************************************************/
	float getOuterMarginMove();
	void  setOuterMarginMove(float value);
	Property<Planning,float,READ_WRITE> OuterMarginMove;

	float getInsideMarginBoundary();
	void  setInsideMarginBoundary(float value);
	Property<Planning,float,READ_WRITE> InsideMarginBoundary;

	float getFrontMarginBoundary();
	void  setFrontMarginBoundary(float value);
	Property<Planning,float,READ_WRITE> FrontMarginBoundary;

	float getRearMarginBoundary();
	void  setRearMarginBoundary(float value);
	Property<Planning,float,READ_WRITE> RearMarginBoundary;
	/***************************************************************/
	uint8_t getCommand();
	void    setCommand(uint8_t value);
	Property<Planning,uint8_t,READ_WRITE> Command;

	uint8_t getConsoleState();
	void    setConsoleState(uint8_t value);
	Property<Planning,uint8_t,READ_WRITE> ConsoleState;

	uint8_t getParkingStatus();
	void    setParkingStatus(uint8_t value);
	Property<Planning,uint8_t,READ_WRITE> ParkingStatus;
	/***************************************************************/
	float getPlanningBrakingAcc();
	void  setPlanningBrakingAcc(float value);
	Property<Planning,float,READ_WRITE> PlanningBrakingAcc;

	float getPlanningBrakingAccR();
	void  setPlanningBrakingAccR(float value);
	Property<Planning,float,READ_WRITE> PlanningBrakingAccR;

	float getPlanningBrakingAeb();
	void  setPlanningBrakingAeb(float value);
	Property<Planning,float,READ_WRITE> PlanningBrakingAeb;

	float getTurnningFeedforwardTime();
	void  setTurnningFeedforwardTime(float value);
	Property<Planning,float,READ_WRITE> TurnningFeedforwardTime;

	uint8_t getAccDisableTime();
	void    setAccDisableTime(uint8_t value);
	Property<Planning,uint8_t,READ_WRITE> AccDisableTime;

	float getPositionMax();
	void  setPositionMax(float value);
	Property<Planning,float,READ_WRITE> PositionMax;

	float getPositionMin();
	void  setPositionMin(float value);
	Property<Planning,float,READ_WRITE> PositionMin;

	float getParkingMargin();
	void  setParkingMargin(float value);
	Property<Planning,float,READ_WRITE> ParkingMargin;

	float getKpYaw();
	void  setKpYaw(float value);
	Property<Planning,float,READ_WRITE> KpYaw;
	/***************************************************************/
protected:
	AlgebraicGeometry _plan_algebraic_geometry;
	VehilceConfig     _plan_vehilce_config;

	Vector2d _parking_center_point;
	Vector2d _parking_outer_front_point;
	Vector2d _parking_outer_rear_point;
	Vector2d _parking_inside_rear_point;
	Vector2d _parking_inside_front_point;//库位边角点
	ControlCommand _control_command;
	APAControlCommand _apa_control_command;
//	Terminal _vertical_planning_terminal;
	uint8_t _acc_disable_cnt;
	/***************************************************/
	float planning_braking_acc_;
	float planning_braking_acc_r_;
	float planning_braking_aeb_;

	float turnning_feedforward_time_;
	uint8_t acc_disable_time_;

	float position_max_;
	float position_min_;
	float parking_margin_;

	float kp_yaw_;
private:
	float _min_parking_length;
	float _min_parking_width;

	float _outer_virtual_boundary;
	float _inside_virtual_boundary;
	float _front_virtual_boundary;
	float _rear_virtual_boundary;

	float _outer_margin_move;
	float _inside_margin_boundary;
	float _front_margin_boundary;
	float _rear_margin_boundary;

	uint8_t _command;
	uint8_t _console_state;
	uint8_t _parking_status;


	VehicleBody _boundary_collision_body;
Q_SIGNALS:
    void sCircleCenterPoint(uint8_t id,Circle *c);
};

#endif /* INTERFACE_PATH_PLANNIG_H_ */
