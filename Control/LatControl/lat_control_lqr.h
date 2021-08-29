#ifndef LATCONTROL_LQR_H
#define LATCONTROL_LQR_H

#include "QDebug"
#include "QTime"
#include "sys/time.h"

#include "Control/Interface/controller.h"

#include "math.h"
#include "Common/Math/vector_2d.h"
#include "Common/Utils/Inc/link_list.h"
#include "Common/VehicleState/GeometricTrack/geometric_track.h"
#include "Common/Configure/Configs/vehilce_config.h"

#include "Planning/Curvature/curvature.h"
#include "Common/Math/linear_quadratic_regulator.h"
#include "Control/Common/trajectory_analyzer.h"
#include "Common/Math/math_utils.h"

typedef enum _LatControl_LQR_Status
{
    lqr_init_status = 0,
    lqr_process_status
}LatControl_LQR_Status;

class LateralErr
{
public:
    void setRefHeading(double value);
    double getRefHeading();

    void setRefHeadingRate(double value);
    double getRefHeadingRate();

    void setHeading(double value);
    double getHeading();

    void setHeadingRate(double value);
    double getHeadingRate();

    void setCurvature(double value);
    double getCurvature();

    void setLateralError(double value);
    double getLateralError();

    void setLateralErrorRate(double value);
    double getLateralErrorRate();

    void setHeadingError(double value);
    double getHeadingError();

    void setHeadingErrorRate(double value);
    double getHeadingErrorRate();

private:
    double _ref_heading;
    double _ref_heading_rate;
    double _heading;
    double _heading_rate;
    double _curvature;


    // error
    double _lateral_error;
    double _lateral_error_rate;
    double _heading_error;
    double _heading_error_rate;
};

class LatControl_LQR
{
public:
    LatControl_LQR();

    void Init(VehilceConfig *vehicle_conf);
    bool LoadControlConf(VehilceConfig *vehicle_conf);

    void KinematicsModuleInit();

    void DynamicsModuleInit();

    void KinematicsModuleUpdate();

    void DynamicsModuleUpdate();
    /**
     * @brief UpdateState:更新状态矩阵
     * @param err：更新误差状态
     */
    void UpdateState(LateralErr *err);

    void KinematicsUpdateMatrix(double v, double k);

    void DynamicsUpdateMatrix(double v);

    /**
     * @brief UpdateMatrix:更新矩阵
     * @param k：曲率
     */
    void UpdateMatrix(const double k);

    /**
     * @brief Compute FeedForward Steering Angle
     * @param ref_curvature:current target curvature
     * @return
     */
    double ComputeFeedForward(const double ref_curvature)const;

    /**
     * @brief Compute Lateral Errors
     * @param x:vehicle actual x axis position
     * @param y:vehicle actual y axis position
     * @param yaw:vehicle actual yaw orientation
     * @param linear_v: actual vehicle velocity
     * @param angular_v: actual vehicle yaw rate
     * @param trajectory_analyzer: trajectory function
     * @param error : return the relation error
     */
    void ComputeLateralErrors(const double x, const double y, const double yaw,
                              const double linear_v, const double angular_v,
                              TrajectoryAnalyzer &trajectory_analyzer,LateralErr *error);


    void ComputeControlCommand(GeometricTrack *act_track, MessageManager *msg, TrajectoryAnalyzer track, VehicleController *ctl);

    void Work(MessageManager *msg, GeometricTrack *a_track, TrajectoryAnalyzer track, VehicleController *ctl);

    //
    double getLr();
private:
    MessageManager *_message_manager;
    GeometricTrack *_vehicle_track;

    TrajectoryAnalyzer _trajectory_analyzer;
    // the following parameters are vehicle physics related.
    // control time interval
    double _ts = 0.0;
    // corner stiffness; front
    double _cf = 0.0;
    // corner stiffness; rear
    double _cr = 0.0;
    // distance between front and rear wheel center
    double _wheelbase = 0.0;
    // mass of the vehicle
    double _mass = 0.0;
    // distance from front wheel center to COM
    double _lf = 0.0;
    // distance from rear wheel center to COM
    double _lr = 0.0;
    // rotational inertia
    double _iz = 0.0;
    // the ratio between the turn of the steering wheel and the turn of the wheels
    double _steer_ratio = 0.0;

    double _max_steer_angle = 0.0;
    double _max_steer_angle_rate = 0.0;

    // parameters for lqr solver; number of iterations
    uint16_t _lqr_max_iteration = 0;
    // parameters for lqr solver; threshold for computation
    double _lqr_eps = 0.0;

    // number of states without previews, includes
    // lateral error, lateral error rate, heading error, heading error rate
    const int _basic_state_size = 4;

    // vehicle state matrix
    Eigen::MatrixXd _matrix_a;
    // vehicle state matrix (discrete-time)
    Eigen::MatrixXd _matrix_ad;
    // vehicle state matrix compound; related to preview
    Eigen::MatrixXd _matrix_adc;
    // control matrix
    Eigen::MatrixXd _matrix_b;
    // control matrix (discrete-time)
    Eigen::MatrixXd _matrix_bd;
    // control matrix compound
    Eigen::MatrixXd _matrix_bdc;
    // gain matrix
    Eigen::MatrixXd _matrix_k;
    // control authority weighting matrix
    Eigen::MatrixXd _matrix_r;
    // state weighting matrix
    Eigen::MatrixXd _matrix_q;
    // updated state weighting matrix
    Eigen::MatrixXd _matrix_q_updated;
    // vehicle state matrix coefficients
    Eigen::MatrixXd _matrix_a_coeff;
    Eigen::MatrixXd _matrix_b_coeff;
    // 4 by 1 matrix; state matrix
    Eigen::MatrixXd _matrix_state;

    double _previous_lateral_error;
    double _previous_heading_error;

    double _driving_orientation_ = 0.0;

    double _min_speed_protection = 0.1;

    LatControl_LQR_Status _lat_control_lqr_status;
    float last_cross_err;

    QTime _m_time;

    struct timeval tpstart,tpend;
};

#endif // LATCONTROL_LQR_H
