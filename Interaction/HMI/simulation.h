#ifndef SIMULATION_H
#define SIMULATION_H

#include <QMainWindow>
#include "Interaction/CANBUS/BoRui/bo_rui_message.h"
#include "Interaction/CANBUS/BoRui/bo_rui_controller.h"

/**************************速度控制******************************/
#define MAX_POSITION            ( 0.9 ) // 速度控制上限点
#define MIN_POSITION            ( 0.4 ) // 速度控制下限点
#define MAX_VELOCITY	  		( 1.0 ) // 直线段的速度
#define MIN_VELOCITY	      	( 0.3 ) // 曲线段的速度

class Simulation
{
public:
    Simulation();

    void Update(VehicleController *c,MessageManager *m);

    float VelocityPlanningControl(float distance);
    float VelocityControl(float distance,float velocity);

    float SteeringAngleControl(float angle,float angle_rate,float dt);

private:
    float _max_position,_min_position;
    float _max_velocity,_min_velocity;

    float _steering_angle_set;
};

#endif // SIMULATION_H
