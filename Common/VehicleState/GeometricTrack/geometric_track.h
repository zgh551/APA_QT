/*****************************************************************************/
/* FILE NAME: geometric_track.h                 COPYRIGHT (c) Henry Zhu 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: track the vehilce position        					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      January 9 2019      Initial Version                  */
/*****************************************************************************/

#ifndef VEHICLESTATE_GEOMETRICTRACK_GEOMETRIC_TRACK_H_
#define VEHICLESTATE_GEOMETRICTRACK_GEOMETRIC_TRACK_H_


#include "../Interface/vehicle_state.h"

typedef struct _fit_ratio
{
	float a;
	float b;
}FitRatio;

class GeometricTrack :public VehicleState
{
public:
    GeometricTrack();
    virtual ~GeometricTrack();

    void Init(void);
    void Init(float x,float y,float yaw);

    void VelocityUpdate(MessageManager *msg,float dt) override;
    void PulseUpdate(MessageManager *msg) override;
    void PulseTrackUpdate(MessageManager *msg) override;

    void VelocityPulseUpdate(MessageManager *msg);

    void DynamicsUpdate(MessageManager *msg);

    Vector2d ComputeCOMPosition(double rear_to_com_distance);

    float pi2pi(float angle);

    int32_t getSumRearLeftPulse();
    void    setSumRearLeftPulse(int32_t value);
    Property<GeometricTrack,int32_t,READ_WRITE> SumRearLeftPulse;

    int32_t getSumRearRightPulse();
    void    setSumRearRightPulse(int32_t value);
    Property<GeometricTrack,int32_t,READ_WRITE> SumRearRightPulse;
private:
    float _last_yaw,_delta_yaw;

    uint16_t _last_rear_left_pulse;
    uint16_t _last_rear_right_pulse;

    int32_t _delta_rear_left_pulse;
    int32_t _delta_rear_right_pulse;

    int16_t _cumulation_rear_left_pulse;
    int16_t _cumulation_rear_right_pulse;

    float _cumulation_middle_displacement;
    float _velocity_line_rate;
    float _last_velocity_line_rate;
    float _err_velocity;
    uint8_t _wait_time_cnt;
    uint8_t _velocity_lock;

    int32_t _sum_rear_left_pulse;
    int32_t _sum_rear_right_pulse;
};

#endif /* VEHICLESTATE_GEOMETRICTRACK_GEOMETRIC_TRACK_H_ */
