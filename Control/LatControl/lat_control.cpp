/*
 * lat_control.cpp
 *
 *  Created on: 2019年4月15日
 *      Author: Henry Zhu
 */

#include "Control/LatControl/lat_control.h"



LatControl::LatControl() {
	// TODO Auto-generated constructor stub
    Init();

}

LatControl::~LatControl() {
	// TODO Auto-generated destructor stub
}

void LatControl::Init()
{
    std::vector<double> den(3, 0.0);
    std::vector<double> num(3, 0.0);

    last_cross_err = 0.0f;
	_lat_control_status = init_status;
    common::LpfCoefficients(0.02,1,&den, &num);
    _digital_filter.set_coefficients(den, num);
}

void LatControl::Proc(MessageManager *msg,VehicleController *ctl,PID *pid)
{
	if(ctl->SteeringEnable)
	{
		ctl->SteeringAngle = 0;
	}
}

float LatControl::pi2pi(float angle)
{
    while(angle > MV_PI)
    {
        angle = angle - 2*MV_PI;
    }
    while(angle < -MV_PI)
    {
        angle = angle + 2*MV_PI;
    }
    return angle;
}

/**
 * @brief LatControl::SatFunction Sat函数
 * @param x：输入
 * @return 返回Sat数值
 */
float LatControl::SatFunction(float x)
{
	if(x > COEFFICIENT_DELTA)
	{
		return  1.0f;
	}
	else if(x < -COEFFICIENT_DELTA)
	{
		return -1.0f;
	}
	else
	{
		return x/COEFFICIENT_DELTA;
	}
}

/**
 * @brief LatControl::ProcV1_0 基于非时间参考的滑模控制算法
 * @param msg：车辆信号反馈，主要获取车辆速度信号
 * @param ctl：车辆控制信号量，控制转向角和角速度信号
 * @param a_track       :实际跟踪变量
 * @param a_track.point :实时跟踪坐标
 * @param a_track.yaw   :实时航向角@(-pi,pi)
 * @param t_track           :目标路径参数变量
 * @param t_track.point     :目标路径与车辆后轴最近的点
 * @param t_track.yaw       :目标路径的航向角 in(-pi,pi)
 * @param t_track.curvature :目标路径处的曲率 左正右负
 */
void LatControl::ProcV1_0(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track)
{
	float cos_psi_a,cos_psi_t;
	float tan_psi_a,tan_psi_t;
	float delta_t,rote_angle;
	float temp_a,temp_b;
	float delta_ctl;

	float yaw_a,yaw_t;
	Vector2d point_a,point_t;

    if((a_track->getYaw() >= -MV_PI) && (a_track->getYaw() < -MV_3PI4))
	{
        rote_angle = MV_3PI4;
	}
    else if((a_track->getYaw() >= -MV_3PI4) && (a_track->getYaw() < -MV_PI2))
	{
        rote_angle = MV_PI2;
	}
    else if((a_track->getYaw() >= -MV_PI2) && (a_track->getYaw() < -MV_PI4))
	{
        rote_angle = MV_PI4;
	}
    else if((a_track->getYaw() >= -MV_PI4) && (a_track->getYaw() < 0))
	{
		rote_angle = 0;
	}
    else if((a_track->getYaw() >= 0) && (a_track->getYaw() < MV_PI4))
	{
		rote_angle = 0;
	}
    else if((a_track->getYaw() >= MV_PI4) && (a_track->getYaw() < MV_PI2))
	{
        rote_angle = -MV_PI4;
	}
    else if((a_track->getYaw() >= MV_PI2) && (a_track->getYaw() < MV_3PI4))
	{
        rote_angle = -MV_PI2;
	}
    else if((a_track->getYaw() >= MV_3PI4) && (a_track->getYaw() <= MV_PI))
	{
        rote_angle = -MV_3PI4;
	}
	else
	{
        rote_angle = 0;
        APA_DEBUG("Warn:angle over the range!");
        return;
	}

	point_a = a_track->getPosition().rotate(rote_angle);
	point_t = t_track.point.rotate(rote_angle);

	yaw_a = a_track->getYaw() + rote_angle;
	yaw_t = t_track.yaw       + rote_angle;

	cos_psi_t = cosf(yaw_t);
	tan_psi_t = tanf(yaw_t);

	cos_psi_a = cosf(yaw_a);
	tan_psi_a = tanf(yaw_a);

	delta_t = atanf(WHEEL_BASE*t_track.curvature);

	_x1 = point_t.getY() - point_a.getY();

	if(Drive == msg->getGear())
	{
		_x2 = tan_psi_t - tan_psi_a;
	}
	else if(Reverse == msg->getGear())
	{
		_x2 = tan_psi_a - tan_psi_t;
	}
	else
	{

	}
	_sliding_variable = COEFFICIENT_SMV * _x1 + _x2;

	temp_a = WHEEL_BASE * cos_psi_a * cos_psi_a * cos_psi_a;
	temp_b = tanf(delta_t)/(WHEEL_BASE * cos_psi_t * cos_psi_t * cos_psi_t) +
			COEFFICIENT_SMV * _x2 +
			COEFFICIENT_RHO * SatFunction(_sliding_variable) +
			COEFFICIENT_K   * _sliding_variable;

	delta_ctl = atanf(temp_a * temp_b);

    delta_ctl = delta_ctl > 0.54f ? 0.54f : delta_ctl < -0.54f ? -0.54f:delta_ctl;
    ctl->SteeringAngle 		= _digital_filter.Filter( delta_ctl * 16 * 57.3f);
	ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
}

/**
 * @brief LatControl::RearWheelFeedback 基于车辆后轴位置反馈的路径跟踪算法
 * @param msg：车辆信号反馈，主要获取车辆速度信号
 * @param ctl：车辆控制信号量，控制转向角和角速度信号
 * @param a_track       :实际跟踪变量
 * @param a_track.point :实时跟踪坐标
 * @param a_track.yaw   :实时航向角@(-pi,pi)
 * @param t_track           :目标路径参数变量
 * @param t_track.point     :目标路径与车辆后轴最近的点
 * @param t_track.yaw       :目标路径的航向角 in(-pi,pi)
 * @param t_track.curvature :目标路径处的曲率 左正右负
 */
void LatControl::RearWheelFeedback(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track)
{
    double err_yaw=0.0,err_cro=0.0;
	Vector2d vec_d,vec_t;
    double psi_omega;
    double v_x = 0.0,k = 0.0;
    double delta_ctl;

    vec_d = a_track->getPosition() - t_track.point;

    if(Drive == msg->getGear())
    {
        vec_t = Vector2d(cos(t_track.yaw),sin(t_track.yaw));
        err_yaw = pi2pi(a_track->getYaw() - t_track.yaw);
//        v_x = msg->getVehicleMiddleSpeed();
        v_x = ctl->getVelocity();
        k =  t_track.curvature;
    }
    else if(Reverse == msg->getGear())
    {
        vec_t = Vector2d(cosf(t_track.yaw + MV_PI),sinf(t_track.yaw + MV_PI));
        err_yaw = pi2pi(a_track->getYaw() - t_track.yaw - MV_PI);
        v_x = msg->getVehicleMiddleSpeed();
        k = -t_track.curvature;
    }
    else
    {
        vec_t = Vector2d(0.0,0.0);
        err_yaw = 0.0f;
        v_x = 0.0f;
    }
    err_cro = vec_t.CrossProduct(vec_d);


    psi_omega = v_x * k * cosf(err_yaw)/(1.0f - k * err_cro)
              - COEFFICIENT_KE   * v_x  * sinf(err_yaw) * err_cro / err_yaw
              - COEFFICIENT_KPSI * fabs(v_x) * err_yaw;

    // TODO 对于err_yaw为0，输出上一时刻的转向角
	if(fabs(psi_omega) < 1.0e-6f || fabs(err_yaw) < 1.0e-6f)
	{
		ctl->SteeringAngle 		= 0.0f;
		ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
	}
	else
	{
        delta_ctl = atanf(psi_omega * WHEEL_BASE / v_x);
        delta_ctl = delta_ctl > 0.54f ? 0.54f : delta_ctl < -0.54f ? -0.54f:delta_ctl;
        ctl->SteeringAngle 		= _digital_filter.Filter(delta_ctl * 16 * 57.3f);
		ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
	}
}


/**
 * @brief LatControl::Work 横向控制状态切换控制
 * @param msg：车辆信息->挡位、车速
 * @param ctl：车辆控制->转向角、转向角速度、车速、挡位
 * @param a_track：当前跟踪位置信息
 * @param t_track：目标曲线信息
 * @param last_track：车辆终点信息
 */
void LatControl::Work(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TargetTrack t_track,TargetTrack last_track)
{
    float nerest_distance;
    float cross_err;
	switch(_lat_control_status)
	{
		case init_status:
			if(ctl->getAPAEnable() && (msg->getSteeringAngle() < 5.0f) && (msg->getSteeringAngle() > -5.0f))
			{
                a_track->Init(); // 跟踪位置初始化，从坐标零点开始控制
				_lat_control_status = process_status;
			}
			else
			{
				ctl->SteeringAngle 		= 0;
				ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
			}
			break;

		case process_status:
			if(ctl->getAPAEnable())
			{
//                ProcV1_0(msg,ctl,a_track,t_track);
                RearWheelFeedback(msg,ctl,a_track,t_track);
                nerest_distance = (a_track->getPosition() - last_track.point).Length();
                cross_err = last_track.point.CrossProduct(a_track->getPosition());
                if( nerest_distance < 0.1f)
                {
                    ctl->setDistance(0);
                    ctl->setVelocity(0.0);
                    ctl->setAPAEnable(0);
                    ctl->setGear(Parking);
                    _lat_control_status = init_status;
                }
                else
                {

                }
                last_cross_err = cross_err;
			}
			else
			{
				_lat_control_status = init_status;
			}
			break;

		default:
            a_track->Init(); // 跟踪位置初始化，从坐标零点开始控制
            ctl->SteeringAngle 		= 0;
            ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
            _lat_control_status = init_status;
			break;
	}
}

void LatControl::Work(MessageManager *msg,VehicleController *ctl,GeometricTrack *a_track,TrajectoryAnalyzer *track_aly)
{
    float nerest_distance;
    float init_steering;
    TargetTrack temp_track = track_aly->CalculateNearestPointByPosition(a_track->getPosition().getX() ,
                                                                        a_track->getPosition().getY());
    switch(_lat_control_status)
    {
        case init_status:
            init_steering = atan(WHEEL_BASE*temp_track.curvature)*16*57.3;
            if((msg->getSteeringAngle() < (init_steering + 5.0f)) && (msg->getSteeringAngle() > (init_steering - 5.0f)))
            {
//                printf("steering angle arrive\r\n");
                _lat_control_status = process_status;
            }
            else
            {
                ctl->setSteeringAngle(init_steering);
//                printf("init steering angle:%f\r\n",init_steering);
                ctl->setSteeringAngleRate(MAX_STEERING_ANGLE_RATE);
            }
            break;

        case process_status:
            nerest_distance = track_aly->DistanceToEnd(a_track->getPosition().getX(),a_track->getPosition().getY());
            if( nerest_distance < 0.2f)
            {
                ctl->setDistance(0);
                ctl->setVelocity(0.0);
                ctl->setAPAEnable(0);
                ctl->setGear(Parking);
                _lat_control_status = init_status;
            }
            else
            {
//                ProcV1_0(msg,ctl,a_track,temp_track);
                RearWheelFeedback(msg,ctl,a_track,temp_track);
            }
            break;

        default:
            a_track->Init();
            ctl->setSteeringAngle(0.0);
            ctl->setSteeringAngleRate(MAX_STEERING_ANGLE_RATE);
            _lat_control_status = init_status;
            break;
    }
}

float LatControl::getX1()            { return  _x1;}
void  LatControl::setX1(float value) { _x1 = value;}

float LatControl::getX2()            { return  _x2;}
void  LatControl::setX2(float value) { _x2 = value;}

float LatControl::getSlidingVariable()            { return  _sliding_variable;}
void  LatControl::setSlidingVariable(float value) { _sliding_variable = value;}
