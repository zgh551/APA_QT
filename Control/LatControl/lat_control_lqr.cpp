#include "lat_control_lqr.h"

using Matrix = Eigen::MatrixXd;
using namespace math;

LatControl_LQR::LatControl_LQR()
{
    _lat_control_lqr_status = lqr_init_status;
}

void LatControl_LQR::Init(VehilceConfig *vehicle_conf)
{
    if(!LoadControlConf(vehicle_conf))
    {
        qDebug() << "failed to load control conf";
        return;
    }

    KinematicsModuleInit();

//    DynamicsModuleInit();

    _matrix_state = Matrix::Zero(_basic_state_size, 1);

    _matrix_k = Matrix::Zero(1, _basic_state_size);
    // control authority weighting matrix
    _matrix_r = Matrix::Identity(1, 1);
    // state weighting matrix
    _matrix_q = Matrix::Zero(_basic_state_size, _basic_state_size);

    for(uint8_t i=0;i<_basic_state_size;i++)
    {
        _matrix_q(i,i) = vehicle_conf->getMatrixQ()[i];
    }
    _matrix_q_updated = _matrix_q;

}

bool LatControl_LQR::LoadControlConf(VehilceConfig *vehicle_conf)
{
    if(!vehicle_conf)
    {
        return false;
    }
    _ts = vehicle_conf->getTs();
    if(_ts <= 0.0)
    {
        return false;
    }

    _cf = vehicle_conf->getCf();
    _cr = vehicle_conf->getCr();

    _wheelbase = vehicle_conf->getWheelBase();
    _steer_ratio = vehicle_conf->getSteeringRatio();
    _max_steer_angle = vehicle_conf->getMaxSteeringAngle();
    _max_steer_angle_rate = vehicle_conf->getMaxSteeringAngleRate();

    const double mass_fl = vehicle_conf->getMassFl();
    const double mass_fr = vehicle_conf->getMassFr();
    const double mass_rl = vehicle_conf->getMassRl();
    const double mass_rr = vehicle_conf->getMassRr();

    const double mass_front = mass_fl + mass_fr;
    const double mass_rear = mass_rl + mass_rr;

    _mass = mass_front + mass_rear;

    _lf = _wheelbase * (1.0 - mass_front/_mass);
    _lr = _wheelbase * (1.0 - mass_rear/_mass);

    _iz = _lf * _lf * mass_front + _lr * _lr * mass_rear;

    _lqr_eps = vehicle_conf->getEps();

    _lqr_max_iteration = vehicle_conf->getMaxIteration();

    _min_speed_protection = vehicle_conf->getMinSpeedProtection();
    return true;
}

void LatControl_LQR::KinematicsModuleInit()
{
    _matrix_a  = Matrix::Zero(_basic_state_size,_basic_state_size);
    _matrix_ad = Matrix::Zero(_basic_state_size,_basic_state_size);

    // 运动学方程
    _matrix_ad(0,0) = 1.0;
    _matrix_ad(0,1) = _ts;
    _matrix_ad(2,2) = 1.0;
    _matrix_ad(2,3) = _ts;

    _matrix_a_coeff = Matrix::Zero(_basic_state_size,_basic_state_size);
    _matrix_a_coeff(1,2) =  1.0;
    _matrix_a_coeff(3,0) = -1.0;

    _matrix_b  = Matrix::Zero(_basic_state_size,1);
    _matrix_bd = Matrix::Zero(_basic_state_size,1);

    _matrix_b_coeff = Matrix::Zero(_basic_state_size,1);
    _matrix_b_coeff(3,0) = 1.0/_wheelbase;
}

void LatControl_LQR::DynamicsModuleInit()
{
    _matrix_a  = Matrix::Zero(_basic_state_size,_basic_state_size);
    _matrix_ad = Matrix::Zero(_basic_state_size,_basic_state_size);

    _matrix_a(0, 1) = 1.0;
    _matrix_a(1, 2) = (_cf + _cr) / _mass;
    _matrix_a(2, 3) = 1.0;
    _matrix_a(3, 2) = (_lf * _cf - _lr * _cr) / _iz;

    _matrix_a_coeff = Matrix::Zero(_basic_state_size, _basic_state_size);
    _matrix_a_coeff(1, 1) = -(_cf + _cr) / _mass;
    _matrix_a_coeff(1, 3) = (_lr * _cr - _lf * _cf) / _mass;
    _matrix_a_coeff(3, 1) = (_lr * _cr - _lf * _cf) / _iz;
    _matrix_a_coeff(3, 3) = -1.0 * (_lf * _lf * _cf + _lr * _lr * _cr) / _iz;

    _matrix_b  = Matrix::Zero(_basic_state_size,1);
    _matrix_bd = Matrix::Zero(_basic_state_size,1);

    _matrix_b(1, 0) = _cf / _mass;
    _matrix_b(3, 0) = _lf * _cf / _iz;

    _matrix_bd = _matrix_b * _ts;
}

void LatControl_LQR::KinematicsModuleUpdate()
{
    // 运动学方程
    _matrix_ad(0,0) = 1.0;
    _matrix_ad(0,1) = _ts;
    _matrix_ad(2,2) = 1.0;
    _matrix_ad(2,3) = _ts;

    _matrix_a_coeff(1,2) =  1.0;
    _matrix_a_coeff(3,0) = -1.0;

    _matrix_b_coeff(3,0) = 1.0/_wheelbase;
}

void LatControl_LQR::DynamicsModuleUpdate()
{
    // A Matrix
    _matrix_a(0, 1) = 1.0;
    _matrix_a(1, 2) = (_cf + _cr) / _mass;
    _matrix_a(3, 2) = (_lf * _cf - _lr * _cr) / _iz;

    _matrix_a_coeff(0, 2) = 0.0;
    _matrix_a_coeff(1, 1) = -(_cf + _cr) / _mass;
    _matrix_a_coeff(1, 3) = (_lr * _cr - _lf * _cf) / _mass;
    _matrix_a_coeff(3, 1) = (_lr * _cr - _lf * _cf) / _iz;
    _matrix_a_coeff(3, 3) = -1.0 * (_lf * _lf * _cf + _lr * _lr * _cr) / _iz;

    // B Matrix
    _matrix_b(1, 0) = _cf / _mass;
    _matrix_b(3, 0) = _lf * _cf / _iz;

    _matrix_bd = _matrix_b * _ts;
}

void LatControl_LQR::UpdateState(LateralErr *err)
{
//    Vector2d com = _vehicle_track->ComputeCOMPosition(_lr);
    Vector2d com = _vehicle_track->getPosition();
    ComputeLateralErrors(static_cast<double>(com.getX()),
                         static_cast<double>(com.getY()),
                         static_cast<double>(_vehicle_track->getYaw()),
                         static_cast<double>(_message_manager->getVehicleMiddleSpeed()),
                         static_cast<double>(_message_manager->getYawRate()),
                         _trajectory_analyzer,err);

    _matrix_state(0,0) = err->getLateralError();
    _matrix_state(1,0) = err->getLateralErrorRate();
    _matrix_state(2,0) = err->getHeadingError();
    _matrix_state(3,0) = err->getHeadingErrorRate();
}


void LatControl_LQR::KinematicsUpdateMatrix(double v, double k)
{
    _matrix_ad(1,2) = _matrix_a_coeff(1,2) * v;
    _matrix_ad(3,0) = _matrix_a_coeff(3,0) * v * k * k;

    _matrix_bd(3,0) = _matrix_b_coeff(3,0) * v;
}

void LatControl_LQR::DynamicsUpdateMatrix(double v)
{
    _matrix_a(1, 1) = _matrix_a_coeff(1, 1) / v;
    _matrix_a(1, 3) = _matrix_a_coeff(1, 3) / v;
    _matrix_a(3, 1) = _matrix_a_coeff(3, 1) / v;
    _matrix_a(3, 3) = _matrix_a_coeff(3, 3) / v;

    Matrix matrix_i = Matrix::Identity(_matrix_a.cols(), _matrix_a.cols());

    _matrix_ad = (matrix_i - _ts * 0.5 * _matrix_a).inverse() *
                 (matrix_i + _ts * 0.5 * _matrix_a);
}


void LatControl_LQR::UpdateMatrix(const double k)
{
    double v;
    if(_message_manager->getGear() == Reverse)
    {
        v = fmin(_message_manager->getVehicleMiddleSpeed(),-_min_speed_protection);
    }
    else
    {
        v = fmax(_message_manager->getVehicleMiddleSpeed(),_min_speed_protection);
    }
    KinematicsUpdateMatrix(v,k);
//    DynamicsUpdateMatrix(v);
}

double LatControl_LQR::ComputeFeedForward(const double ref_curvature)const
{
    const double kv =
        _lr * _mass / 2 / _cf / _wheelbase - _lf * _mass / 2 / _cr / _wheelbase;

    const double v = static_cast<double>(_message_manager->getVehicleMiddleSpeed());

    double steer_angle_feedforward_term;
    if(_message_manager->getGear() == Reverse)
    {
        steer_angle_feedforward_term = _wheelbase * ref_curvature * 180.0 / static_cast<double>(M_PI) *_steer_ratio;
    }
    else
    {
        steer_angle_feedforward_term = _wheelbase * ref_curvature * 180.0 / static_cast<double>(M_PI) *_steer_ratio;
//        steer_angle_feedforward_term = (_wheelbase * ref_curvature +
//                                        kv * v * v * ref_curvature -
//                                        _matrix_k(0,2) *
//                                        (_lr * ref_curvature -
//                                         _lf * _mass * v * v * ref_curvature / 2 / _cr / _wheelbase)) *
//                                        180.0 / static_cast<double>(M_PI) *_steer_ratio;
    }
    return steer_angle_feedforward_term;
}

void LatControl_LQR::ComputeLateralErrors(const double x, const double y, const double yaw,
                                          const double linear_v, const double angular_v,
                                          TrajectoryAnalyzer &trajectory_analyzer,LateralErr *error)
{
    TargetTrack target_point;
    Vector2d vec_a,vec_d,vec_t;

    target_point = trajectory_analyzer.CalculateNearestPointByPosition(x,y);

    vec_a.setX(static_cast<float>(x));
    vec_a.setY(static_cast<float>(y));

    vec_d = vec_a - target_point.point;

    vec_t.setX(cosf(static_cast<float>(target_point.yaw)));
    vec_t.setY(sinf(static_cast<float>(target_point.yaw)));

    float lateral_error = vec_t.CrossProduct(vec_d);

    error->setLateralError(static_cast<double>(lateral_error));

    error->setRefHeading(static_cast<double>(target_point.yaw));
    error->setHeading(yaw);

    double head_error = math::NormallizeAngle(error->getHeading() - error->getRefHeading());

    error->setHeadingError(head_error);

    //error first dot
    // base on the actual velocity
//    error->setLateralErrorRate(linear_v * sin(error->getHeadingError()));
    // base on last error value calculate the dot
    error->setLateralErrorRate((error->getLateralError() - _previous_lateral_error) / _ts);
    error->setHeadingErrorRate((error->getHeadingError() - _previous_heading_error) / _ts);

    _previous_lateral_error = error->getLateralError();
    _previous_heading_error = error->getHeadingError();

    error->setCurvature(static_cast<double>(target_point.curvature));
}

void LatControl_LQR::ComputeControlCommand(GeometricTrack *act_track,MessageManager *msg, TrajectoryAnalyzer track,VehicleController *ctl)
{
//    _m_time.start();
    gettimeofday(&tpstart,NULL);

    LateralErr *error = new LateralErr();

    _vehicle_track          = act_track;
    _message_manager        = msg;
    _trajectory_analyzer    = track;

    KinematicsModuleUpdate();
//    DynamicsModuleUpdate();

    UpdateState(error);
    UpdateMatrix(error->getCurvature());

    math::SolveLQR(_matrix_ad,_matrix_bd,_matrix_q,_matrix_r,_lqr_eps,_lqr_max_iteration,&_matrix_k);

    // feedback = - K * state
    const double steer_angle_feedback = -(_matrix_k *_matrix_state)(0,0) * 180.0 / static_cast<double>(M_PI) *_steer_ratio;
    const double steer_angle_feedforward = ComputeFeedForward(error->getCurvature());

    double steer_angle = steer_angle_feedback + steer_angle_feedforward;

    steer_angle = math::Clamp(steer_angle,-_max_steer_angle,_max_steer_angle);

    ctl->setSteeringAngle(static_cast<float>(steer_angle));
    ctl->setSteeringAngleRate(static_cast<float>(_max_steer_angle_rate));
    gettimeofday(&tpend,NULL);

    qDebug() << (tpend.tv_sec-tpstart.tv_sec)*1000.0 + (tpend.tv_usec-tpstart.tv_usec)/1000.0 << "ms";
//    qDebug() << _m_time.elapsed() << "ms";
}

void LatControl_LQR::Work(MessageManager *msg, GeometricTrack *a_track, TrajectoryAnalyzer track, VehicleController *ctl)
{
    double nerest_distance;
    switch(_lat_control_lqr_status)
    {
        case lqr_init_status:
            if(ctl->getAPAEnable() && (msg->getSteeringAngle() < 5.0f) && (msg->getSteeringAngle() > -5.0f))
            {
                a_track->Init(); // 跟踪位置初始化，从坐标零点开始控制
                _lat_control_lqr_status = lqr_process_status;
            }
            else
            {
                ctl->SteeringAngle 		= 0;
                ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
            }
            break;

        case lqr_process_status:
            if(ctl->getAPAEnable())
            {
                ComputeControlCommand(a_track,msg,track,ctl);
                nerest_distance = track.DistanceToEnd(static_cast<double>(a_track->getPosition().getX()),
                                                      static_cast<double>(a_track->getPosition().getY()));
                if( nerest_distance < 0.1 )
                {
                    ctl->setDistance(0);
                    ctl->setVelocity(0.0);
                    ctl->setAPAEnable(0);
                    ctl->setGear(Parking);
                    _lat_control_lqr_status = lqr_init_status;
                }
                else
                {

                }
            }
            else
            {
                _lat_control_lqr_status = lqr_init_status;
            }
            break;

        default:
            a_track->Init(); // 跟踪位置初始化，从坐标零点开始控制
            ctl->SteeringAngle 		= 0;
            ctl->SteeringAngleRate 	= MAX_STEERING_ANGLE_RATE;
            _lat_control_lqr_status = lqr_init_status;
            break;
    }
}

double LatControl_LQR::getLr() {return _lr;}

void   LateralErr::setRefHeading(double value)  {_ref_heading = value;}
double LateralErr::getRefHeading()              {return  _ref_heading;}

void   LateralErr::setRefHeadingRate(double value)  {_ref_heading_rate = value;}
double LateralErr::getRefHeadingRate()              {return  _ref_heading_rate;}

void   LateralErr::setHeading(double value) {_heading = value;}
double LateralErr::getHeading()             {return  _heading;}

void   LateralErr::setHeadingRate(double value) {_heading_rate = value;}
double LateralErr::getHeadingRate()             {return  _heading_rate;}

void   LateralErr::setCurvature(double value)   {_curvature = value;}
double LateralErr::getCurvature()               {return  _curvature;}

void   LateralErr::setLateralError(double value){_lateral_error = value;}
double LateralErr::getLateralError()            {return  _lateral_error;}

void   LateralErr::setLateralErrorRate(double value){_lateral_error_rate = value;}
double LateralErr::getLateralErrorRate()            {return  _lateral_error_rate;}

void   LateralErr::setHeadingError(double value){_heading_error = value;}
double LateralErr::getHeadingError()            {return  _heading_error;}

void   LateralErr::setHeadingErrorRate(double value){_heading_error_rate = value;}
double LateralErr::getHeadingErrorRate()            {return  _heading_error_rate;}
