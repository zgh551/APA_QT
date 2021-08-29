/*****************************************************************************/
/* FILE NAME: vehicle_body.h                    COPYRIGHT (c) Henry Zhu 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the vector property             					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      January 10 2019      Initial Version                 */
/*****************************************************************************/

#ifndef MATH_VEHICLE_BODY_H_
#define MATH_VEHICLE_BODY_H_

#include <QMainWindow>
#include "math.h"
#include "./Common/Utils/Inc/property.h"
#include "./Common/Math/vector_2d.h"
#include "./Common/Math/algebraic_geometry.h"
#include "./Common/Configure/Configs/vehilce_config.h"


class VehicleBody {
public:
	VehicleBody();
	virtual ~VehicleBody();

	// 已知车库边界和车辆的某个边角点，求该边角点不碰撞边沿的最大旋转角度
	float RotateAngle(Vector2d vehicle_edge,Vector2d boundary);
	float RotateAngleCollision(Vector2d vehicle_edge,Vector2d boundary);
	float RotateAngle(Vector2d boundary);
	// 跟据旋转半径求旋转中心点的坐标
	void RotationCenter(float radius);
	// 车体模型旋转
	void Rotate(float angle);
	// 求车体模型的四个边角点的坐标
	void EdgePoint(void);
	// 已知边界和旋转半径，车体模型尝试一次旋转
	void OneTrial(float radius,Vector2d boundary);

	void VerticalTrial(float radius,Vector2d boundary);

	float getAttitudeYaw();
	void  setAttitudeYaw(float value);
	Property<VehicleBody,float,READ_WRITE> AttitudeYaw;

	Vector2d getCenter();
	void     setCenter(Vector2d value);
	Property<VehicleBody,Vector2d,READ_WRITE> Center;

	Vector2d getRotation();
	void     setRotation(Vector2d value);
	Property<VehicleBody,Vector2d,READ_WRITE> Rotation;

	Vector2d getFrontLeft();
	void     setFrontLeft(Vector2d value);
	Property<VehicleBody,Vector2d,READ_WRITE> FrontLeft;

	Vector2d getFrontRight();
	void     setFrontRight(Vector2d value);
	Property<VehicleBody,Vector2d,READ_WRITE> FrontRight;

	Vector2d getRearLeft();
	void     setRearLeft(Vector2d value);
	Property<VehicleBody,Vector2d,READ_WRITE> RearLeft;

	Vector2d getRearRight();
	void     setRearRight(Vector2d value);
	Property<VehicleBody,Vector2d,READ_WRITE> RearRight;

private:
	// the rotation radisu of the four edge
	float _attitude_yaw;
	Vector2d _center;
	Vector2d _rotation;
	Vector2d _front_left;
	Vector2d _front_right;
	Vector2d _rear_left;
	Vector2d _rear_right;
};

#endif /* MATH_VEHICLE_BODY_H_ */
