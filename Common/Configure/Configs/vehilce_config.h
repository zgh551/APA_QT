/*
 * vehilce_config.h
 *
 *  Created on: 2019年1月10日
 *      Author: Henry Zhu
 */

#ifndef CONFIGURE_CONFIGS_VEHILCE_CONFIG_H_
#define CONFIGURE_CONFIGS_VEHILCE_CONFIG_H_

#include <QMainWindow>
#include "math.h"
#include "./Common/Utils/Inc/property.h"
#include "./Common/Math/vector_2d.h"
#include "system_config.h"

#ifdef CHANGAN
#include "./Common/Configure/Data/chang_an_configure.h"
#endif
#ifdef BORUI
#include "./Common/Configure/Data/bo_rui_configure.h"
#endif
#ifdef DONG_FENG_E70
#include "./Common/Configure/Data/dong_feng_configure.h"
#endif

typedef struct _Location
{
	Vector2d Point;
	float Angle;
}Location;

typedef struct _Polar
{
    double Length;
    double Angle;
}Polar;

class VehilceConfig {
public:
	VehilceConfig();
	virtual ~VehilceConfig();

	void Init();
	void EdgeRadius(float r);
	/**********************************************************************/
	float SteeringAngle2TurnningRadiusExp(float steer,float a,float b);
	float TurnningRadius2SteeringAngleExp(float radius,float a,float b);

	float SteeringAngle2TurnningRadius(float steer,float a,float b);
	float TurnningRadius2SteeringAngle(float radius,float a,float b);

	float TurnRadiusCurveFitting(float steering_angle);
	float TurnRadiusFindingTable(float steering_angle);

	float SteeringAngleCurveFitting(float radius);
	float SteeringAngleFindingCallback(uint16_t left_id,uint16_t right_id,float radius);
	float SteeringAngleFindingLoop(uint16_t left_id,uint16_t right_id,float radius);

	float TurnRadiusCalculate(float steering_angle);
	float SteeringAngleCalculate(float radius);
	/**********************************************************************/
    double getFrontDiagonalAxis();
    void  setFrontDiagonalAxis(double value);
    Property<VehilceConfig,double,READ_WRITE> FrontDiagonalAxis;

    double getFrontDiagonalAngle();
    void  setFrontDiagonalAngle(double value);
    Property<VehilceConfig,double,READ_WRITE> FrontDiagonalAngle;

    double getRearDiagonalAxis();
    void  setRearDiagonalAxis(double value);
    Property<VehilceConfig,double,READ_WRITE> RearDiagonalAxis;

    double getRearDiagonalAngle();
    void  setRearDiagonalAngle(double value);
    Property<VehilceConfig,double,READ_WRITE> RearDiagonalAngle;

	float getRadiusFrontLeft();
	void  setRadiusFrontLeft(float value);
	Property<VehilceConfig,float,READ_WRITE> RadiusFrontLeft;

	float getRadiusFrontRight();
	void  setRadiusFrontRight(float value);
	Property<VehilceConfig,float,READ_WRITE> RadiusFrontRight;

	float getRadiusRearLeft();
	void  setRadiusRearLeft(float value);
	Property<VehilceConfig,float,READ_WRITE> RadiusRearLeft;

	float getRadiusRearRight();
	void  setRadiusRearRight(float value);
	Property<VehilceConfig,float,READ_WRITE> RadiusRearRight;

	Location* getUltrasonicLocationArray();
	Property<VehilceConfig,Location*,READ_ONLY> UltrasonicLocationArray;

    Polar getFrontLeftDiagonal();
    Property<VehilceConfig,Polar,READ_ONLY> FrontLeftDiagonal;

    Polar getFrontRightDiagonal();
    Property<VehilceConfig,Polar,READ_ONLY> FrontRightDiagonal;

    Polar getRearLeftDiagonal();
    Property<VehilceConfig,Polar,READ_ONLY> RearLeftDiagonal;

    Polar getRearRightDiagonal();
    Property<VehilceConfig,Polar,READ_ONLY> RearRightDiagonal;

	float* getAccelerateTable();
	Property<VehilceConfig,float*,READ_ONLY> AccelerateTable;

	float* getVelocityTable();
	Property<VehilceConfig,float*,READ_ONLY> VelocityTable;

	float* getTorqueTable();
	Property<VehilceConfig,float*,READ_ONLY> TorqueTable;

	uint16_t getAccNum();
	Property<VehilceConfig,uint16_t,READ_ONLY> AccNum;

	uint16_t getVlcNum();
	Property<VehilceConfig,uint16_t,READ_ONLY> VlcNum;

    double getTs();
    double getCf();
    double getCr();
    double getMassFl();
    double getMassFr();
    double getMassRl();
    double getMassRr();
    double getEps();
    uint16_t getMaxIteration();
    double getMatrixQ1();
    double getMatrixQ2();
    double getMatrixQ3();
    double getMatrixQ4();

    double *getMatrixQ();

    double getWheelBase();
    double getSteeringRatio();
    double getMaxSteeringAngle();
    double getMaxSteeringAngleRate();

    double getMinSpeedProtection();

private:
	// the diagonal axis of the four edge to the center point and the angle
    double _front_diagonal_axis;
    double _front_diagonal_angle;
    double _rear_diagonal_axis;
    double _rear_diagonal_angle;

	float _radius_front_left;
	float _radius_front_right;
	float _radius_rear_left;
	float _radius_rear_right;

	Location _ultrasonic_location_array[12];

    Polar _front_left_diagonal;
    Polar _front_right_diagonal;
    Polar _rear_left_diagonal;
    Polar _rear_right_diagonal;

	uint16_t _acc_num,_vlc_num;
	float _accelerate_table[ACC_ARRAY_NUM];
	float _velocity_table[VELOCITY_ARRAY_NUM];
	float _torque_table[ACC_ARRAY_NUM * VELOCITY_ARRAY_NUM];

    double _wheel_base;
    double _steering_ratio;
    double _max_steering_angle;
    double _max_steering_angle_rate;
    double _min_speed_protection;


    double _ts;
    double _cf;
    double _cr;
    double _mass_fl;
    double _mass_fr;
    double _mass_rl;
    double _mass_rr;
    double _eps;
    uint16_t _max_iteration;

    double _matrix_q1;
    double _matrix_q2;
    double _matrix_q3;
    double _matrix_q4;

    double _matrix_q[4];

};

#endif /* CONFIGURE_CONFIGS_VEHILCE_CONFIG_H_ */
