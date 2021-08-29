/*****************************************************************************/
/* FILE NAME: vehicle_body.cpp                  COPYRIGHT (c) Henry Zhu 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: achieve vehicle rotation property and the interface	         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      January 12 2019      Initial Version                 */
/*****************************************************************************/
#include "./Common/Math/vehicle_body.h"

VehilceConfig m_VehilceConfig;

VehicleBody::VehicleBody() {
	m_VehilceConfig.Init();

	AttitudeYaw.setContainer(this);
	AttitudeYaw.getter(&VehicleBody::getAttitudeYaw);
	AttitudeYaw.setter(&VehicleBody::setAttitudeYaw);

	Center.setContainer(this);
	Center.getter(&VehicleBody::getCenter);
	Center.setter(&VehicleBody::setCenter);

	Rotation.setContainer(this);
	Rotation.getter(&VehicleBody::getRotation);
	Rotation.setter(&VehicleBody::setRotation);

	FrontLeft.setContainer(this);
	FrontLeft.getter(&VehicleBody::getFrontLeft);
	FrontLeft.setter(&VehicleBody::setFrontLeft);

	FrontRight.setContainer(this);
	FrontRight.getter(&VehicleBody::getFrontRight);
	FrontRight.setter(&VehicleBody::setFrontRight);

	RearLeft.setContainer(this);
	RearLeft.getter(&VehicleBody::getRearLeft);
	RearLeft.setter(&VehicleBody::setRearLeft);

	RearRight.setContainer(this);
	RearRight.getter(&VehicleBody::getRearRight);
	RearRight.setter(&VehicleBody::setRearRight);
}

VehicleBody::~VehicleBody() {

}

void VehicleBody::RotationCenter(float radius)
{
	Vector2d v = Vector2d(radius * cosf(_attitude_yaw),
			              radius * sinf(_attitude_yaw));
	_rotation = _center + v.rotate(PI/2);
}

float VehicleBody::RotateAngle(Vector2d vehicle_edge,Vector2d boundary)
{
	float theta_edge,theta_x,theta_y;
	float theta_d1,theta_d2;
	float radius_square;
	float x_axis_distance,y_axis_distance;
	float y_axis_len,x_axis_len;
	Vector2d boundary_x,boundary_y;
	radius_square = (vehicle_edge -_rotation).LengthSquare();
	/////////////////////////////////////////////////////////////////////////////////
	y_axis_distance = powf(boundary.getY() - _rotation.getY(),2);
	if(radius_square > y_axis_distance)
	{
		x_axis_len = sqrtf( radius_square - y_axis_distance);

        boundary_x.setX( (vehicle_edge -_rotation).getX() > 0 ?  _rotation.getX() + x_axis_len : _rotation.getX() - x_axis_len );
        boundary_x.setY( boundary.getY() );

		theta_x = (boundary_x -_rotation).Angle();
	}
	else
	{
		theta_x = 0;
	}
	/////////////////////////////////////////////////////////////////////////////////
	x_axis_distance = powf(boundary.getX() - _rotation.getX(),2);
	if(radius_square > x_axis_distance)
	{
		y_axis_len = sqrtf( radius_square - x_axis_distance);

        boundary_y.setX( boundary.getX());
        boundary_y.setY( (vehicle_edge -_rotation).getY() > 0 ? _rotation.getY() + y_axis_len : _rotation.getY() - y_axis_len );

		theta_y = (boundary_y -_rotation).Angle();
	}
	else
	{
		theta_y = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////
	theta_edge = (vehicle_edge -_rotation).Angle();

	theta_d1 = theta_x - theta_edge;
	theta_d2 = theta_y - theta_edge;

	if(theta_d1 > 0 && theta_d2 > 0)
	{
		return theta_d1 < theta_d2 ? theta_d1 : theta_d2;
	}
	else if(theta_d1 > 0 && theta_d2 < 0)
	{
		return theta_d1;
	}
	else if(theta_d1 < 0 && theta_d2 > 0)
	{
		return theta_d2;
	}
	else
	{
		return 0;
	}
}

float VehicleBody::RotateAngleCollision(Vector2d vehicle_edge,Vector2d boundary)
{
	float theta_edge,theta_x,theta_y;
	float theta_d1,theta_d2;
	float radius_square;
	float x_axis_distance,y_axis_distance;
	float y_axis_len,x_axis_len;
	Vector2d boundary_x,boundary_y;
	radius_square = (vehicle_edge -_rotation).LengthSquare();
	/////////////////////////////////////////////////////////////////////////////////
	y_axis_distance = powf(boundary.getY() - _rotation.getY(),2);
	if(radius_square > y_axis_distance)
	{
		x_axis_len = sqrtf( radius_square - y_axis_distance);

        boundary_x.setX( (vehicle_edge -_rotation).getX() > 0 ?  _rotation.getX() + x_axis_len : _rotation.getX() - x_axis_len );
        boundary_x.setY( boundary.getY() );

		theta_x = (boundary_x -_rotation).Angle();
	}
	else
	{
		theta_x = 0;
	}
	/////////////////////////////////////////////////////////////////////////////////
	x_axis_distance = powf(boundary.getX() - _rotation.getX(),2);
	if(radius_square > x_axis_distance)
	{
		y_axis_len = sqrtf( radius_square - x_axis_distance);

        boundary_y.setX( boundary.getX() );
        boundary_y.setY( (vehicle_edge -_rotation).getY() > 0 ? _rotation.getY() + y_axis_len : _rotation.getY() - y_axis_len );

		theta_y = (boundary_y -_rotation).Angle();
	}
	else
	{
		theta_y = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////
	theta_edge = (vehicle_edge -_rotation).Angle();

	theta_d1 = theta_x - theta_edge;
	theta_d2 = theta_y - theta_edge;

	if(theta_d1 < 0 && theta_d2 < 0)
	{
		return fabs(theta_d1) < fabs(theta_d2) ? fabs(theta_d1) : fabs(theta_d2);
	}
	else if(theta_d1 < 0 && theta_d2 > 0)
	{
		return fabs(theta_d1);
	}
	else if(theta_d1 > 0 && theta_d2 < 0)
	{
		return fabs(theta_d2);
	}
	else
	{
		return 0;
	}
}

float VehicleBody::RotateAngle(Vector2d boundary)
{
	float theta,alpha,beta,gama,lamda;
	float radius_square,x_axis_distance,y_axis_len,theta_edge,theta_y,theta_d2;
	Vector2d boundary_y,edge_points;

	theta = (boundary - _rotation).Angle();
	alpha = asinf(REAR_EDGE_TO_CENTER/(boundary - _rotation).Length());
	gama  = (_center - _rotation).Angle();
	if(theta < PI_2)//往右边界倒库情况
	{
		beta = theta + alpha;
		edge_points = _rear_right;
	}
	else//往左边界倒库情况
	{
		beta = theta - alpha;
		edge_points = _rear_left;
	}
	lamda = beta - gama;

	radius_square = (edge_points -_rotation).LengthSquare();
	x_axis_distance = powf(boundary.getX() - _rotation.getX(),2);
	if(radius_square > x_axis_distance)
	{
		y_axis_len = sqrtf( radius_square - x_axis_distance);

        boundary_y.setX( boundary.getX() );
        boundary_y.setY( (edge_points -_rotation).getY() > 0 ? _rotation.getY() + y_axis_len : _rotation.getY() - y_axis_len );

		theta_y = (boundary_y -_rotation).Angle();
		theta_edge = (edge_points -_rotation).Angle();
		theta_d2 = theta_y - theta_edge;

		return fabs(lamda) > fabs(theta_d2) ? lamda : theta_d2;
	}
	else
	{
		return 0;
	}
}

// radius is +-
void VehicleBody::Rotate(float angle)
{
	_center = (_center - _rotation).rotate(angle) + _rotation;
	_attitude_yaw += angle;
}

void VehicleBody::EdgePoint(void)
{
	Vector2d v;

	v = Vector2d( m_VehilceConfig.FrontDiagonalAxis * cosf(_attitude_yaw),
				  m_VehilceConfig.FrontDiagonalAxis * sinf(_attitude_yaw));

	_front_left  = _center + v.rotate( m_VehilceConfig.FrontDiagonalAngle);
	_front_right = _center + v.rotate(-m_VehilceConfig.FrontDiagonalAngle);

	v = Vector2d( -m_VehilceConfig.RearDiagonalAxis * cosf(_attitude_yaw),
				  -m_VehilceConfig.RearDiagonalAxis * sinf(_attitude_yaw));

	_rear_left  = _center + v.rotate(-m_VehilceConfig.RearDiagonalAngle);
	_rear_right = _center + v.rotate( m_VehilceConfig.RearDiagonalAngle);
}


void VehicleBody::OneTrial(float radius,Vector2d boundary)
{
	float theta1,theta2,min_theta;
	RotationCenter(radius);
	EdgePoint();
	if(radius < 0)
	{
		theta1 = RotateAngle(RearLeft, boundary);
		theta2 = RotateAngle(RearRight, boundary);
	}
	else
	{
		theta1 = RotateAngle(FrontLeft, boundary);
		theta2 = RotateAngle(FrontRight, boundary);
	}
	min_theta = fabsf(theta1) < fabsf(theta2) ? theta1 : theta2;
	Rotate(min_theta);
}

void VehicleBody::VerticalTrial(float radius,Vector2d boundary)
{
	float min_theta;
	RotationCenter(radius);
	EdgePoint();
	if(radius < 0)
	{
//		theta1 = RotateAngle(RearLeft, boundary);
		min_theta = RotateAngle(boundary);
	}
	else
	{
//		theta1 = RotateAngle(FrontLeft, boundary);
//		theta2 = RotateAngle(FrontRight, boundary);
	}
//	min_theta = fabsf(theta1) > fabsf(theta2) ? theta1 : theta2;
	Rotate(min_theta);
}


float VehicleBody::getAttitudeYaw()           { return  _attitude_yaw;}
void  VehicleBody::setAttitudeYaw(float value){ _attitude_yaw = value;}

Vector2d VehicleBody::getCenter()              { return  _center;}
void     VehicleBody::setCenter(Vector2d value){ _center = value;}

Vector2d VehicleBody::getRotation()              { return  _rotation;}
void     VehicleBody::setRotation(Vector2d value){ _rotation = value;}

Vector2d VehicleBody::getFrontLeft()              { return  _front_left;}
void     VehicleBody::setFrontLeft(Vector2d value){ _front_left = value;}

Vector2d VehicleBody::getFrontRight()              { return  _front_right;}
void     VehicleBody::setFrontRight(Vector2d value){ _front_right = value;}

Vector2d VehicleBody::getRearLeft()              { return  _rear_left;}
void     VehicleBody::setRearLeft(Vector2d value){ _rear_left = value;}

Vector2d VehicleBody::getRearRight()              { return  _rear_right;}
void     VehicleBody::setRearRight(Vector2d value){ _rear_right = value;}
