/*****************************************************************************/
/* FILE NAME: vehilce_config.cpp                COPYRIGHT (c) Henry Zhu 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: configure the vehile information and the steering and radius */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      January 10 2019     Initial Version                  */
/* 1.1	 Henry Zhu      May     13 2019     Steering angle calculatge        */
/*****************************************************************************/
#include "./Common/Configure/Configs/vehilce_config.h"

VehilceConfig::VehilceConfig() {
	FrontDiagonalAxis.setContainer(this);
	FrontDiagonalAxis.getter(&VehilceConfig::getFrontDiagonalAxis);
	FrontDiagonalAxis.setter(&VehilceConfig::setFrontDiagonalAxis);

	FrontDiagonalAngle.setContainer(this);
	FrontDiagonalAngle.getter(&VehilceConfig::getFrontDiagonalAngle);
	FrontDiagonalAngle.setter(&VehilceConfig::setFrontDiagonalAngle);

	RearDiagonalAxis.setContainer(this);
	RearDiagonalAxis.getter(&VehilceConfig::getRearDiagonalAxis);
	RearDiagonalAxis.setter(&VehilceConfig::setRearDiagonalAxis);

	RearDiagonalAngle.setContainer(this);
	RearDiagonalAngle.getter(&VehilceConfig::getRearDiagonalAngle);
	RearDiagonalAngle.setter(&VehilceConfig::setRearDiagonalAngle);

	RadiusFrontLeft.setContainer(this);
	RadiusFrontLeft.getter(&VehilceConfig::getRadiusFrontLeft);
	RadiusFrontLeft.setter(&VehilceConfig::setRadiusFrontLeft);

	RadiusFrontRight.setContainer(this);
	RadiusFrontRight.getter(&VehilceConfig::getRadiusFrontRight);
	RadiusFrontRight.setter(&VehilceConfig::setRadiusFrontRight);

	RadiusRearLeft.setContainer(this);
	RadiusRearLeft.getter(&VehilceConfig::getRadiusRearLeft);
	RadiusRearLeft.setter(&VehilceConfig::setRadiusRearLeft);

	RadiusRearRight.setContainer(this);
	RadiusRearRight.getter(&VehilceConfig::getRadiusRearRight);
	RadiusRearRight.setter(&VehilceConfig::setRadiusRearRight);

	UltrasonicLocationArray.setContainer(this);
	UltrasonicLocationArray.getter(&VehilceConfig::getUltrasonicLocationArray);

    FrontLeftDiagonal.setContainer(this);
    FrontLeftDiagonal.getter(&VehilceConfig::getFrontLeftDiagonal);

    FrontRightDiagonal.setContainer(this);
    FrontRightDiagonal.getter(&VehilceConfig::getFrontRightDiagonal);

    RearLeftDiagonal.setContainer(this);
    RearLeftDiagonal.getter(&VehilceConfig::getRearLeftDiagonal);

    RearRightDiagonal.setContainer(this);
    RearRightDiagonal.getter(&VehilceConfig::getRearRightDiagonal);

	AccelerateTable.setContainer(this);
	AccelerateTable.getter(&VehilceConfig::getAccelerateTable);

	VelocityTable.setContainer(this);
	VelocityTable.getter(&VehilceConfig::getVelocityTable);

	TorqueTable.setContainer(this);
	TorqueTable.getter(&VehilceConfig::getTorqueTable);

	AccNum.setContainer(this);
	AccNum.getter(&VehilceConfig::getAccNum);

	VlcNum.setContainer(this);
	VlcNum.getter(&VehilceConfig::getVlcNum);

	Init();
}

VehilceConfig::~VehilceConfig() {

}

void VehilceConfig::Init()
{
	uint8_t i;

	FrontDiagonalAxis = sqrtf( powf(LEFT_EDGE_TO_CENTER,2) + powf(FRONT_EDGE_TO_CENTER,2));
	RearDiagonalAxis  = sqrtf( powf(LEFT_EDGE_TO_CENTER,2) + powf(REAR_EDGE_TO_CENTER,2));

	FrontDiagonalAngle = atanf(LEFT_EDGE_TO_CENTER/FRONT_EDGE_TO_CENTER);
	RearDiagonalAngle  = atanf(LEFT_EDGE_TO_CENTER/REAR_EDGE_TO_CENTER);

    _front_left_diagonal.Length = FrontDiagonalAxis;
    _front_left_diagonal.Angle  = FrontDiagonalAngle;

    _front_right_diagonal.Length =  FrontDiagonalAxis;
    _front_right_diagonal.Angle  = -FrontDiagonalAngle;

    _rear_left_diagonal.Length = RearDiagonalAxis;
    _rear_left_diagonal.Angle  = -RearDiagonalAngle;

    _rear_right_diagonal.Length =  RearDiagonalAxis;
    _rear_right_diagonal.Angle  =  RearDiagonalAngle;

    _ultrasonic_location_array[0].Point.setX( SENSOR1_X );
    _ultrasonic_location_array[0].Point.setY( SENSOR1_Y );
	_ultrasonic_location_array[0].Angle   = SENSOR1_ANGLE;

    _ultrasonic_location_array[1].Point.setX( SENSOR2_X );
    _ultrasonic_location_array[1].Point.setY( SENSOR2_Y );
	_ultrasonic_location_array[1].Angle   = SENSOR2_ANGLE;

    _ultrasonic_location_array[2].Point.setX( SENSOR3_X );
    _ultrasonic_location_array[2].Point.setY( SENSOR3_Y );
	_ultrasonic_location_array[2].Angle   = SENSOR3_ANGLE;

    _ultrasonic_location_array[3].Point.setX( SENSOR4_X );
    _ultrasonic_location_array[3].Point.setY( SENSOR4_Y );
	_ultrasonic_location_array[3].Angle   = SENSOR4_ANGLE;

    _ultrasonic_location_array[4].Point.setX( SENSOR5_X );
    _ultrasonic_location_array[4].Point.setY( SENSOR5_Y );
	_ultrasonic_location_array[4].Angle   = SENSOR5_ANGLE;

    _ultrasonic_location_array[5].Point.setX( SENSOR6_X );
    _ultrasonic_location_array[5].Point.setY( SENSOR6_Y );
	_ultrasonic_location_array[5].Angle   = SENSOR6_ANGLE;

    _ultrasonic_location_array[6].Point.setX( SENSOR7_X );
    _ultrasonic_location_array[6].Point.setY( SENSOR7_Y );
	_ultrasonic_location_array[6].Angle   = SENSOR7_ANGLE;

    _ultrasonic_location_array[7].Point.setX( SENSOR8_X );
    _ultrasonic_location_array[7].Point.setY( SENSOR8_Y );
	_ultrasonic_location_array[7].Angle   = SENSOR8_ANGLE;

    _ultrasonic_location_array[8].Point.setX( SENSOR9_X );
    _ultrasonic_location_array[8].Point.setY( SENSOR9_Y );
	_ultrasonic_location_array[8].Angle   = SENSOR9_ANGLE;

    _ultrasonic_location_array[9].Point.setX( SENSOR10_X );
    _ultrasonic_location_array[9].Point.setX( SENSOR10_Y );
    _ultrasonic_location_array[9].Angle     = SENSOR10_ANGLE;

    _ultrasonic_location_array[10].Point.setX( SENSOR11_X );
    _ultrasonic_location_array[10].Point.setY( SENSOR11_Y );
	_ultrasonic_location_array[10].Angle   = SENSOR11_ANGLE;

    _ultrasonic_location_array[11].Point.setX( SENSOR12_X );
    _ultrasonic_location_array[11].Point.setY( SENSOR12_Y );
	_ultrasonic_location_array[11].Angle   = SENSOR12_ANGLE;

	/*
	 * 初始化扭矩标定表
	 * */

	_acc_num = ACC_ARRAY_NUM;
	_vlc_num  = VELOCITY_ARRAY_NUM;
	for(i=0;i<ACC_ARRAY_NUM;i++)
	{
		_accelerate_table[i] = acc_table[i];
	}
	for(i=0;i<VELOCITY_ARRAY_NUM;i++)
	{
		_velocity_table[i] = velocity_table[i];
	}
	for(i=0;i< (ACC_ARRAY_NUM * VELOCITY_ARRAY_NUM);i++)
	{
		_torque_table[i] = torque_table[i/VELOCITY_ARRAY_NUM][i%VELOCITY_ARRAY_NUM];
	}

    _ts = TS;
    _cf = CF;
    _cr = CR;
    _mass_fl = MASS_FL;
    _mass_fr = MASS_FR;
    _mass_rl = MASS_RL;
    _mass_rr = MASS_RR;
    _eps = EPS;
    _max_iteration = MAX_ITERATION;

    _matrix_q1 = MATRIX_Q1;
    _matrix_q2 = MATRIX_Q2;
    _matrix_q3 = MATRIX_Q3;
    _matrix_q4 = MATRIX_Q4;

    _matrix_q[0] = MATRIX_Q1;
    _matrix_q[1] = MATRIX_Q2;
    _matrix_q[2] = MATRIX_Q3;
    _matrix_q[3] = MATRIX_Q4;

    _wheel_base = WHEEL_BASE;
    _steering_ratio = STEERING_RATIO;
    _max_steering_angle = MAX_STEERING_ANGLE;
    _max_steering_angle_rate = MAX_STEERING_ANGLE_RATE;
    _min_speed_protection = MIN_SPEED_PROTECTION;
}

// r is + and -
void VehilceConfig::EdgeRadius(float r)
{
	RadiusFrontLeft = sqrtf( powf( r - LEFT_EDGE_TO_CENTER , 2 ) +
			                 powf( FRONT_EDGE_TO_CENTER , 2 ) );

	RadiusFrontRight = sqrtf( powf( r + RIGHT_EDGE_TO_CENTER , 2 ) +
                              powf( FRONT_EDGE_TO_CENTER , 2 ) );

	RadiusRearLeft = sqrtf( powf( r -  LEFT_EDGE_TO_CENTER , 2 ) +
			                powf( REAR_EDGE_TO_CENTER , 2 ) );

	RadiusRearRight = sqrtf( powf( r + RIGHT_EDGE_TO_CENTER , 2 ) +
                             powf( REAR_EDGE_TO_CENTER , 2 ) );
}

float VehilceConfig::SteeringAngle2TurnningRadius(float steer,float a,float b)
{
    return steer < 0 ? -WHEEL_BASE / tanf((a * -steer + b) * PI / 180.0f) :
                        WHEEL_BASE / tanf((a *  steer + b) * PI / 180.0f) ;
}

float VehilceConfig::SteeringAngle2TurnningRadiusExp(float steer,float a,float b)
{
	return steer < 0 ? -a * expf(b * -steer) : a * expf(b * steer);
}

float VehilceConfig::TurnningRadius2SteeringAngleExp(float radius,float a,float b)
{
	return radius < 0 ? -logf(-radius/a)/b : logf(radius/a)/b;
}

float VehilceConfig::TurnningRadius2SteeringAngle(float radius,float a,float b)
{
	return radius < 0 ? -(atanf(-WHEEL_BASE/radius) * 180 / PI - b)/a :
			             (atanf( WHEEL_BASE/radius) * 180 / PI - b)/a ;
}

float VehilceConfig::TurnRadiusCurveFitting(float steering_angle)
{
	return 	steering_angle >  400 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A1,FIT_RADIUS_B1) :
			steering_angle >  300 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A2,FIT_RADIUS_B2) :
			steering_angle >  200 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A3,FIT_RADIUS_B3) :
			steering_angle >  100 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A4,FIT_RADIUS_B4) :
			steering_angle >   50 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A5,FIT_RADIUS_B5) :
			steering_angle >    0 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A6,FIT_RADIUS_B6) :
			steering_angle >  -50 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A7,FIT_RADIUS_B7) :
			steering_angle > -100 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A8,FIT_RADIUS_B8) :
			steering_angle > -200 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A9,FIT_RADIUS_B9) :
			steering_angle > -300 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A10,FIT_RADIUS_B10) :
			steering_angle > -400 ? SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A11,FIT_RADIUS_B11) :
									SteeringAngle2TurnningRadius(steering_angle,FIT_RADIUS_A12,FIT_RADIUS_B12);
}

float VehilceConfig::TurnRadiusFindingTable(float steering_angle)
{
    return steering_angle >=  0 ? SteerAngle2Radius[static_cast<uint16_t>(steering_angle)][0] : -SteerAngle2Radius[static_cast<uint16_t>(-steering_angle)][1];
}

float VehilceConfig::SteeringAngleCurveFitting(float radius)
{
	return
           radius >   48.308f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A6,FIT_RADIUS_B6) :
           radius >   24.018f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A5,FIT_RADIUS_B5) :
           radius >   11.910f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A4,FIT_RADIUS_B4) :
           radius >   7.6736f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A3,FIT_RADIUS_B3) :
           radius >    5.463f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A2,FIT_RADIUS_B2) :
           radius >      0.0f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A1,FIT_RADIUS_B1) :
           radius > - 5.4745f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A12,FIT_RADIUS_B12) :
           radius > - 7.7214f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A11,FIT_RADIUS_B11):
           radius > - 11.975f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A10,FIT_RADIUS_B10):
           radius > - 24.975f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A9,FIT_RADIUS_B9):
           radius > - 49.975f ? TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A8,FIT_RADIUS_B8):
							   TurnningRadius2SteeringAngle(radius,FIT_RADIUS_A7,FIT_RADIUS_B7);
}

float VehilceConfig::SteeringAngleFindingCallback(uint16_t left_id,uint16_t right_id,float radius)
{
	uint16_t middle_id;
	middle_id = (left_id + right_id) * 0.5;
	if(radius > 0)
	{
		if( (right_id - left_id) > 1)
		{
			if(radius < SteerAngle2Radius[middle_id][0])
			{
				return SteeringAngleFindingCallback(middle_id,right_id,radius);
			}
			else if(radius > SteerAngle2Radius[middle_id][0])
			{
				return SteeringAngleFindingCallback(left_id,middle_id,radius);
			}
			else
			{
				return middle_id;
			}
		}
		else
		{
            return (left_id + right_id) * 0.5f;
		}
	}
	else
	{
		if( (right_id - left_id) > 1)
		{
			if(-radius < SteerAngle2Radius[middle_id][1])
			{
				return SteeringAngleFindingCallback(middle_id,right_id,radius);
			}
			else if(-radius > SteerAngle2Radius[middle_id][1])
			{
				return SteeringAngleFindingCallback(left_id,middle_id,radius);
			}
			else
			{
				return -middle_id;
			}
		}
		else
		{
            return -(left_id + right_id) * 0.5f;
		}
	}
}

float VehilceConfig::SteeringAngleFindingLoop(uint16_t left_id,uint16_t right_id,float radius)
{
	uint16_t middle_id;
	if(radius > 0)
	{
		while( (right_id - left_id) > 1)
		{
			middle_id = (uint16_t)((left_id + right_id) * 0.5);
			if(radius < SteerAngle2Radius[middle_id][0])
			{
				left_id = middle_id;
			}
			else if(radius > SteerAngle2Radius[middle_id][0])
			{
				right_id = middle_id;
			}
			else
			{
				return middle_id;
			}
		}
        return (left_id + right_id) * 0.5f;
	}
	else
	{
		while( (right_id - left_id) > 1)
		{
            middle_id = (uint16_t)((left_id + right_id) * 0.5f);
			if(-radius < SteerAngle2Radius[middle_id][1])
			{
				left_id = middle_id;
			}
			else if(-radius > SteerAngle2Radius[middle_id][1])
			{
				right_id = middle_id;
			}
			else
			{
				return -middle_id;
			}
		}
        return -(left_id + right_id) * 0.5f;
	}
}

float VehilceConfig::TurnRadiusCalculate(float steering_angle)
{
	return TurnRadiusFindingTable(steering_angle);
}

float VehilceConfig::SteeringAngleCalculate(float radius)
{
	return SteeringAngleFindingCallback(MIN_ARRAY_ID,MAX_ARRAY_ID,radius);
//	return SteeringAngleFindingLoop(MIN_ARRAY_ID,MAX_ARRAY_ID,radius);
}

float VehilceConfig::getRadiusFrontLeft()           { return  _radius_front_left;}
void  VehilceConfig::setRadiusFrontLeft(float value){ _radius_front_left = value;}
float VehilceConfig::getRadiusFrontRight()           { return  _radius_front_right;}
void  VehilceConfig::setRadiusFrontRight(float value){ _radius_front_right = value;}
float VehilceConfig::getRadiusRearLeft()           { return  _radius_rear_left;}
void  VehilceConfig::setRadiusRearLeft(float value){ _radius_rear_left = value;}
float VehilceConfig::getRadiusRearRight()           { return  _radius_rear_right;}
void  VehilceConfig::setRadiusRearRight(float value){ _radius_rear_right = value;}

double VehilceConfig::getFrontDiagonalAxis()           { return  _front_diagonal_axis;}
void  VehilceConfig::setFrontDiagonalAxis(double value){ _front_diagonal_axis = value;}
double VehilceConfig::getFrontDiagonalAngle()           { return  _front_diagonal_angle;}
void  VehilceConfig::setFrontDiagonalAngle(double value){ _front_diagonal_angle = value;}
double VehilceConfig::getRearDiagonalAxis()           { return  _rear_diagonal_axis;}
void  VehilceConfig::setRearDiagonalAxis(double value){ _rear_diagonal_axis = value;}
double VehilceConfig::getRearDiagonalAngle()           { return  _rear_diagonal_angle;}
void  VehilceConfig::setRearDiagonalAngle(double value){ _rear_diagonal_angle = value;}

Location* VehilceConfig::getUltrasonicLocationArray() { return  _ultrasonic_location_array;}

Polar VehilceConfig::getFrontLeftDiagonal()  { return  _front_left_diagonal;}
Polar VehilceConfig::getFrontRightDiagonal() { return  _front_right_diagonal;}
Polar VehilceConfig::getRearLeftDiagonal()   { return  _rear_left_diagonal;}
Polar VehilceConfig::getRearRightDiagonal()  { return  _rear_right_diagonal;}

float* VehilceConfig::getAccelerateTable() { return  _accelerate_table;}
float* VehilceConfig::getVelocityTable() { return  _velocity_table;}
float* VehilceConfig::getTorqueTable() { return  _torque_table;}

uint16_t VehilceConfig::getAccNum() { return  _acc_num;}
uint16_t VehilceConfig::getVlcNum() { return  _vlc_num;}

double VehilceConfig::getTs() { return  _ts;}
double VehilceConfig::getCf() { return  _cf;}
double VehilceConfig::getCr() { return  _cr;}
double VehilceConfig::getMassFl() { return  _mass_fl;}
double VehilceConfig::getMassFr() { return  _mass_fr;}
double VehilceConfig::getMassRl() { return  _mass_rl;}
double VehilceConfig::getMassRr() { return  _mass_rr;}
double VehilceConfig::getEps() { return  _eps;}
uint16_t VehilceConfig::getMaxIteration() { return  _max_iteration;}
double VehilceConfig::getMatrixQ1() { return  _matrix_q1;}
double VehilceConfig::getMatrixQ2() { return  _matrix_q2;}
double VehilceConfig::getMatrixQ3() { return  _matrix_q3;}
double VehilceConfig::getMatrixQ4() { return  _matrix_q4;}
double *VehilceConfig::getMatrixQ() { return  _matrix_q;}

double VehilceConfig::getWheelBase() { return  _wheel_base;}
double VehilceConfig::getSteeringRatio(){ return  _steering_ratio;}
double VehilceConfig::getMaxSteeringAngle(){ return  _max_steering_angle;}
double VehilceConfig::getMaxSteeringAngleRate(){ return  _max_steering_angle_rate;}

double VehilceConfig::getMinSpeedProtection(){ return _min_speed_protection; }
