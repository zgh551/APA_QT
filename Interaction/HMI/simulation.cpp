#include "simulation.h"

Simulation::Simulation()
{

    _max_position = MAX_POSITION;// 速度控制上限点
    _min_position = MIN_POSITION;// 速度控制下限点
    _max_velocity = MAX_VELOCITY;// 直线段的速度
    _min_velocity = MIN_VELOCITY;// 曲线段的速度

    _steering_angle_set = 0;
}

void Simulation::Update(VehicleController *c,MessageManager *m)
{
    if(c->APAEnable)
    {
        m->WheelSpeedRearLeft  = VelocityControl(c->Distance,c->Velocity);//c->Velocity+0.5f;
        m->WheelSpeedRearRight = VelocityControl(c->Distance,c->Velocity);//c->Velocity+0.5f;

        m->SteeringAngle     = SteeringAngleControl(c->getSteeringAngle(),c->getSteeringAngleRate(),0.02);
        m->SteeringAngleRate = c->getSteeringAngleRate();

        m->Gear = static_cast<GearStatus>(c->getGear());

        if(m->getWheelSpeedRearLeft() < 1.0e-6f)
        {
            m->WheelSpeedDirection = StandStill;
        }
        else {
            if(Drive == c->getGear()){
                m->WheelSpeedDirection = Forward;
            }
            else if (Reverse == c->getGear()) {
                m->WheelSpeedDirection = Backward;
            }
            else {
                m->WheelSpeedDirection = StandStill;
            }
        }
    }
    else
    {
        m->WheelSpeedRearLeft  = 0.0f;
        m->WheelSpeedRearRight = 0.0f;

        m->SteeringAngle     = 0;
        m->SteeringAngleRate = c->getSteeringAngleRate();
        m->WheelSpeedDirection = StandStill;
    }
}

//根据距离规划速度
float Simulation::VelocityPlanningControl(float distance)
{
    if(distance > _max_position)
    {
        return _max_velocity;
    }
    else if(distance > _min_position)
    {
        return _min_velocity + (_max_velocity - _min_velocity)*(distance - _min_position)/(_max_position - _min_position);
    }
    else if(distance > 0.05)
    {
        return distance * _min_velocity / _min_position ;
    }
    else
    {
        return 0;
    }
}


float Simulation::VelocityControl(float distance,float velocity)
{
    float temp_v;
    temp_v = VelocityPlanningControl(distance);
    return velocity < temp_v ? velocity :temp_v;
}

float Simulation::SteeringAngleControl(float angle,float angle_rate,float dt)
{
    float da = angle_rate * dt;
    float left_target_angle = angle - da;
    float right_target_angle = angle + da;

    if(_steering_angle_set < left_target_angle)
    {
        _steering_angle_set += da;
    }
    else if(_steering_angle_set > right_target_angle)
    {
        _steering_angle_set -= da;
    }
    else
    {
        _steering_angle_set = angle;
    }
    return _steering_angle_set;
}
