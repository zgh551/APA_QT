/*****************************************************************************/
/* FILE NAME: message_manager.c                 COPYRIGHT (c) Henry Zhu 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: Messege manage module 								         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      December 28 2018    Initial Version                  */
/*****************************************************************************/

#include "./Interaction/CANBUS/Interface/message_manager.h"

MessageManager::MessageManager() {
    // wheel speed
    WheelSpeedFrontLeft.setContainer(this);
    WheelSpeedFrontLeft.getter(&MessageManager::getWheelSpeedFrontLeft);
    WheelSpeedFrontLeft.setter(&MessageManager::setWheelSpeedFrontLeft);

    WheelSpeedFrontRight.setContainer(this);
    WheelSpeedFrontRight.getter(&MessageManager::getWheelSpeedFrontRight);
    WheelSpeedFrontRight.setter(&MessageManager::setWheelSpeedFrontRight);

    WheelSpeedRearRight.setContainer(this);
    WheelSpeedRearRight.getter(&MessageManager::getWheelSpeedRearRight);
    WheelSpeedRearRight.setter(&MessageManager::setWheelSpeedRearRight);

    WheelSpeedRearLeft.setContainer(this);
    WheelSpeedRearLeft.getter(&MessageManager::getWheelSpeedRearLeft);
    WheelSpeedRearLeft.setter(&MessageManager::setWheelSpeedRearLeft);

    VehicleMiddleSpeed.setContainer(this);
    VehicleMiddleSpeed.getter(&MessageManager::getVehicleMiddleSpeed);
    VehicleMiddleSpeed.setter(&MessageManager::setVehicleMiddleSpeed);

    VehicleMiddleSpeedValid.setContainer(this);
    VehicleMiddleSpeedValid.getter(&MessageManager::getVehicleMiddleSpeedValid);
    VehicleMiddleSpeedValid.setter(&MessageManager::setVehicleMiddleSpeedValid);

    // wheel speed direction
    WheelSpeedDirection.setContainer(this);
    WheelSpeedDirection.getter(&MessageManager::getWheelSpeedDirection);
    WheelSpeedDirection.setter(&MessageManager::setWheelSpeedDirection);
    // wheel pulse
    WheelPulseFrontLeft.setContainer(this);
    WheelPulseFrontLeft.getter(&MessageManager::getWheelPulseFrontLeft);
    WheelPulseFrontLeft.setter(&MessageManager::setWheelPulseFrontLeft);

    WheelPulseFrontRight.setContainer(this);
    WheelPulseFrontRight.getter(&MessageManager::getWheelPulseFrontRight);
    WheelPulseFrontRight.setter(&MessageManager::setWheelPulseFrontRight);

    WheelPulseRearRight.setContainer(this);
    WheelPulseRearRight.getter(&MessageManager::getWheelPulseRearRight);
    WheelPulseRearRight.setter(&MessageManager::setWheelPulseRearRight);

    WheelPulseRearLeft.setContainer(this);
    WheelPulseRearLeft.getter(&MessageManager::getWheelPulseRearLeft);
    WheelPulseRearLeft.setter(&MessageManager::setWheelPulseRearLeft);

    WheelSumPulse.setContainer(this);
    WheelSumPulse.getter(&MessageManager::getWheelSumPulse);
    WheelSumPulse.setter(&MessageManager::setWheelSumPulse);
    // wheel pulse dirction
    WheelPulseDirection.setContainer(this);
    WheelPulseDirection.getter(&MessageManager::getWheelPulseDirection);
    WheelPulseDirection.setter(&MessageManager::setWheelPulseDirection);

    // SAS Steering angle
    SteeringAngle.setContainer(this);
    SteeringAngle.getter(&MessageManager::getSteeringAngle);
    SteeringAngle.setter(&MessageManager::setSteeringAngle);

    SteeringAngleRate.setContainer(this);
    SteeringAngleRate.getter(&MessageManager::getSteeringAngleRate);
    SteeringAngleRate.setter(&MessageManager::setSteeringAngleRate);

    // TCU
    Gear.setContainer(this);
    Gear.getter(&MessageManager::getGear);
    Gear.setter(&MessageManager::setGear);

    // ESP Sensor
    YawRate.setContainer(this);
    YawRate.getter(&MessageManager::getYawRate);
    YawRate.setter(&MessageManager::setYawRate);

    LonAcc.setContainer(this);
    LonAcc.getter(&MessageManager::getLonAcc);
    LonAcc.setter(&MessageManager::setLonAcc);

    LatAcc.setContainer(this);
    LatAcc.getter(&MessageManager::getLatAcc);
    LatAcc.setter(&MessageManager::setLatAcc);

    BrakePressure.setContainer(this);
    BrakePressure.getter(&MessageManager::getBrakePressure);
    BrakePressure.setter(&MessageManager::setBrakePressure);
}

MessageManager::~MessageManager() {

}


// wheel speed
float MessageManager::getWheelSpeedFrontLeft()           { return _wheel_speed_front_left;}
void  MessageManager::setWheelSpeedFrontLeft(float value){_wheel_speed_front_left = value;}

float MessageManager::getWheelSpeedFrontRight()           { return _wheel_speed_front_right;}
void  MessageManager::setWheelSpeedFrontRight(float value){_wheel_speed_front_right = value;}

float MessageManager::getWheelSpeedRearRight()           { return _wheel_speed_rear_right;}
void  MessageManager::setWheelSpeedRearRight(float value){_wheel_speed_rear_right = value;}

float MessageManager::getWheelSpeedRearLeft()           { return _wheel_speed_rear_left;}
void  MessageManager::setWheelSpeedRearLeft(float value){_wheel_speed_rear_left = value;}

float MessageManager::getVehicleMiddleSpeed()           { return _vehicle_middle_speed;}
void  MessageManager::setVehicleMiddleSpeed(float value){_vehicle_middle_speed = value;}

uint8_t MessageManager::getVehicleMiddleSpeedValid()             { return _vehicle_middle_speed_valid;}
void    MessageManager::setVehicleMiddleSpeedValid(uint8_t value){_vehicle_middle_speed_valid = value;}

SpeedStatus MessageManager::getVehicleMiddleSpeedAbnormal()             { return _vehicle_middle_speed_abnormal;}
void    MessageManager::setVehicleMiddleSpeedAbnormal(SpeedStatus value){_vehicle_middle_speed_abnormal = value;}
// wheel speed direction
DirectStatus MessageManager::getWheelSpeedDirection()                  { return _wheel_speed_direction;}
void         MessageManager::setWheelSpeedDirection(DirectStatus value){_wheel_speed_direction = value;}

// wheel pulse
uint16_t MessageManager::getWheelPulseFrontLeft()              { return _wheel_pulse_front_left;}
void     MessageManager::setWheelPulseFrontLeft(uint16_t value){_wheel_pulse_front_left = value;}

uint16_t MessageManager::getWheelPulseFrontRight()              { return _wheel_pulse_front_right;}
void     MessageManager::setWheelPulseFrontRight(uint16_t value){_wheel_pulse_front_right = value;}

uint16_t MessageManager::getWheelPulseRearRight()              { return _wheel_pulse_rear_right;}
void     MessageManager::setWheelPulseRearRight(uint16_t value){_wheel_pulse_rear_right = value;}

uint16_t MessageManager::getWheelPulseRearLeft()              { return _wheel_pulse_rear_left;}
void     MessageManager::setWheelPulseRearLeft(uint16_t value){_wheel_pulse_rear_left = value;}

int32_t MessageManager::getWheelSumPulse()              { return _wheel_sum_pulse;}
void    MessageManager::setWheelSumPulse(int32_t value){_wheel_sum_pulse = value;}
// wheel pulse dirction
DirectStatus MessageManager::getWheelPulseDirection()                  { return _wheel_pulse_direction;}
void         MessageManager::setWheelPulseDirection(DirectStatus value){_wheel_pulse_direction = value;}

// SAS Steering angle
float MessageManager::getSteeringAngle()           { return _steering_angle;}
void  MessageManager::setSteeringAngle(float value){_steering_angle = value;}

float MessageManager::getSteeringAngleRate()           { return _steering_angle_rate;}
void  MessageManager::setSteeringAngleRate(float value){_steering_angle_rate = value;}

// TCU gear
GearStatus MessageManager::getGear()                { return _gear;}
void       MessageManager::setGear(GearStatus value){_gear = value;}

// ESP Sensor
float MessageManager::getYawRate()           { return _yaw_rate;}
void  MessageManager::setYawRate(float value){_yaw_rate = value;}

float MessageManager::getLonAcc()           { return _lon_acc;}
void  MessageManager::setLonAcc(float value){_lon_acc = value;}

float MessageManager::getLatAcc()           { return _lat_acc;}
void  MessageManager::setLatAcc(float value){_lat_acc = value;}

uint8_t MessageManager::getBrakePressure()             { return _brake_pressure;}
void    MessageManager::setBrakePressure(uint8_t value){_brake_pressure = value;}

ActuatorStatus MessageManager::getESC_Status()                    { return _esc_status;}
void           MessageManager::setESC_Status(ActuatorStatus value){_esc_status = value;}

ActuatorStatus MessageManager::getEPS_Status()                    { return _eps_status;}
void           MessageManager::setEPS_Status(ActuatorStatus value){_eps_status = value;}
