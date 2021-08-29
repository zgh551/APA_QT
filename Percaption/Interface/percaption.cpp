/*
 * percaption_information.cpp
 *
 *  Created on: 2019年1月9日
 *      Author: Henry Zhu
 */

#include <./Percaption/Interface/percaption.h>

Percaption::Percaption() {
    PositionX.setContainer(this);
    PositionX.getter(&Percaption::getPositionX);
    PositionX.setter(&Percaption::setPositionX);

    PositionY.setContainer(this);
    PositionY.getter(&Percaption::getPositionY);
    PositionY.setter(&Percaption::setPositionY);

    AttitudeYaw.setContainer(this);
    AttitudeYaw.getter(&Percaption::getAttitudeYaw);
    AttitudeYaw.setter(&Percaption::setAttitudeYaw);

    ParkingLength.setContainer(this);
    ParkingLength.getter(&Percaption::getParkingLength);
    ParkingLength.setter(&Percaption::setParkingLength);

    ParkingWidth.setContainer(this);
    ParkingWidth.getter(&Percaption::getParkingWidth);
    ParkingWidth.setter(&Percaption::setParkingWidth);

    DetectParkingStatus.setContainer(this);
    DetectParkingStatus.getter(&Percaption::getDetectParkingStatus);
    DetectParkingStatus.setter(&Percaption::setDetectParkingStatus);

    Command.setContainer(this);
    Command.getter(&Percaption::getCommand);
    Command.setter(&Percaption::setCommand);

    ValidParkingEdgePosition.setContainer(this);
    ValidParkingEdgePosition.getter(&Percaption::getValidParkingEdgePosition);
    ValidParkingEdgePosition.setter(&Percaption::setValidParkingEdgePosition);

    ValidParkingCenterPosition.setContainer(this);
    ValidParkingCenterPosition.getter(&Percaption::getValidParkingCenterPosition);
    ValidParkingCenterPosition.setter(&Percaption::setValidParkingCenterPosition);

    ParallelParkingSlotPosition.setContainer(this);
    ParallelParkingSlotPosition.getter(&Percaption::getParallelParkingSlotPosition);
    ParallelParkingSlotPosition.setter(&Percaption::setParallelParkingSlotPosition);

    ObstacleDistance.setContainer(this);
    ObstacleDistance.getter(&Percaption::getObstacleDistance);
    ObstacleDistance.setter(&Percaption::setObstacleDistance);

    FrontObstacleDistance.setContainer(this);
    FrontObstacleDistance.getter(&Percaption::getFrontObstacleDistance);
    FrontObstacleDistance.setter(&Percaption::setFrontObstacleDistance);

    RearObstacleDistance.setContainer(this);
    RearObstacleDistance.getter(&Percaption::getRearObstacleDistance);
    RearObstacleDistance.setter(&Percaption::setRearObstacleDistance);

    FrontEdgeFitLinePacket.setContainer(this);
    FrontEdgeFitLinePacket.getter(&Percaption::getFrontEdgeFitLinePacket);
    FrontEdgeFitLinePacket.setter(&Percaption::setFrontEdgeFitLinePacket);

    LeftFitLinePacket.setContainer(this);
    LeftFitLinePacket.getter(&Percaption::getLeftFitLinePacket);
    LeftFitLinePacket.setter(&Percaption::setLeftFitLinePacket);

    RightFitLinePacket.setContainer(this);
    RightFitLinePacket.getter(&Percaption::getRightFitLinePacket);
    RightFitLinePacket.setter(&Percaption::setRightFitLinePacket);

    CenterFitLinePacket.setContainer(this);
    CenterFitLinePacket.getter(&Percaption::getCenterFitLinePacket);
    CenterFitLinePacket.setter(&Percaption::setCenterFitLinePacket);

    Init();
}

Percaption::~Percaption() {

}
void Percaption::Init(void)
{
    _position_x = 0.0f;
    _position_y = 0.0f;
    _attitude_yaw = 0.0f;
    _parking_length = 0.0f;
    _parking_width  = 0.0f;
    _detect_parking_status = false;
    _command = 0;

    _front_obstacle_distance.region = CenterRegion;
    _front_obstacle_distance.status = OverDetection;
    _front_obstacle_distance.distance = 3;

    _rear_obstacle_distance.region = CenterRegion;
    _rear_obstacle_distance.status = OverDetection;
    _rear_obstacle_distance.distance = 3;

}

float Percaption::getPositionX()           { return  _position_x;}
void  Percaption::setPositionX(float value){ _position_x = value;}

float Percaption::getPositionY()           { return  _position_y;}
void  Percaption::setPositionY(float value){ _position_y = value;}

float Percaption::getAttitudeYaw()           { return  _attitude_yaw;}
void  Percaption::setAttitudeYaw(float value){ _attitude_yaw = value;}

float Percaption::getParkingLength()           { return  _parking_length;}
void  Percaption::setParkingLength(float value){ _parking_length = value;}

float Percaption::getParkingWidth()           { return  _parking_width;}
void  Percaption::setParkingWidth(float value){ _parking_width = value;}

bool Percaption::getDetectParkingStatus()           { return  _detect_parking_status;}
void Percaption::setDetectParkingStatus(bool value) { _detect_parking_status = value;}

uint8_t Percaption::getCommand()              { return  _command;}
void    Percaption::setCommand(uint8_t value) { _command = value;}

ObstacleInformationPacket Percaption::getValidParkingEdgePosition()           { return  _valid_parking_edge_position;}
void  Percaption::setValidParkingEdgePosition(ObstacleInformationPacket value){ _valid_parking_edge_position = value;}

EdgeInformationPacket Percaption::getValidParkingCenterPosition()           { return  _valid_parking_center_position;}
void  Percaption::setValidParkingCenterPosition(EdgeInformationPacket value){ _valid_parking_center_position = value;}

ObstacleDistancePacket Percaption::getObstacleDistance()           { return  _obstacle_distance;}
void  Percaption::setObstacleDistance(ObstacleDistancePacket value){ _obstacle_distance = value;}

ObstacleDistancePacket Percaption::getFrontObstacleDistance()           { return  _front_obstacle_distance;}
void  Percaption::setFrontObstacleDistance(ObstacleDistancePacket value){ _front_obstacle_distance = value;}

ObstacleDistancePacket Percaption::getRearObstacleDistance()           { return  _rear_obstacle_distance;}
void  Percaption::setRearObstacleDistance(ObstacleDistancePacket value){ _rear_obstacle_distance = value;}

ParallelParkingInformationPacket Percaption::getParallelParkingSlotPosition()           { return  _parallel_parking_slot_position;}
void  Percaption::setParallelParkingSlotPosition(ParallelParkingInformationPacket value){ _parallel_parking_slot_position = value;}

LineFitInformationPacket Percaption::getFrontEdgeFitLinePacket()           { return  _front_edge_fit_line_packet;}
void  Percaption::setFrontEdgeFitLinePacket(LineFitInformationPacket value){ _front_edge_fit_line_packet = value;}

LineFitInformationPacket Percaption::getLeftFitLinePacket()           { return  _left_fit_line_packet;}
void  Percaption::setLeftFitLinePacket(LineFitInformationPacket value){ _left_fit_line_packet = value;}

LineFitInformationPacket Percaption::getRightFitLinePacket()           { return  _right_fit_line_packet;}
void  Percaption::setRightFitLinePacket(LineFitInformationPacket value){ _right_fit_line_packet = value;}

LineFitInformationPacket Percaption::getCenterFitLinePacket()           { return  _center_fit_line_packet;}
void  Percaption::setCenterFitLinePacket(LineFitInformationPacket value){ _center_fit_line_packet = value;}
