/*
 * bo_rui_controller.cpp
 *
 *  Created on: 2019年3月15日
 *      Author: Henry Zhu
 */

#include "./Interaction/CANBUS/BoRui/bo_rui_controller.h"

BoRuiController::BoRuiController() {
    // TODO Auto-generated constructor stub

}

BoRuiController::~BoRuiController() {
    // TODO Auto-generated destructor stub
}

void BoRuiController::Init()
{

}
/**
 * @brief start the vehicle controller.
 * @return true if successfully started.
 */
void BoRuiController::Start()
{

}

/**
 * @brief stop the vehicle controller.
 */
void BoRuiController::Stop()
{

}
  /**
   * @brief update the vehicle controller.
   * @param command the control command
   * @return error_code
   */
void BoRuiController::Update(ControlCommand cmd)
{

}

void BoRuiController::Update(APAControlCommand cmd)
{
    APAEnable         = cmd.ControlEnable.B.APAEnable;

    Gear              = cmd.Gear;

    SteeringAngle 	  = cmd.SteeringAngle;
    SteeringAngleRate = cmd.SteeringAngleRate;

    Velocity          = cmd.Velocity;
    Distance          = cmd.Distance;
}


void BoRuiController::VehicleContorl()
{

}

void BoRuiController::VehicleContorlPri()
{

}

void BoRuiController::SteeringAngleControl(float dt)
{
//    float da = SteeringAngleRate * dt;
//    float left_target_angle = SteeringAngle - da;
//    float right_target_angle = SteeringAngle + da;
//
//    if(SteeringAngleSet < left_target_angle)
//    {
//    	SteeringAngleSet = SteeringAngleSet + da;
//    }
//    else if(SteeringAngleSet > right_target_angle)
//    {
//    	SteeringAngleSet = SteeringAngleSet - da;
//    }
//    else
//    {
//    	SteeringAngleSet = SteeringAngle;
//    }
}

void BoRuiController::SteeringAngleControl(float dt,float actual_steering)
{

}

void BoRuiController::Push(float dt)
{
//	SteeringAngleControl(dt);
//	VehicleContorl();
}

void BoRuiController::Push(float dt,float actual_steering)
{
    SteeringAngleControl(dt,actual_steering);
    VehicleContorl();
}
