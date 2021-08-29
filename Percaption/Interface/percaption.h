/*
 * percaption_information.h
 *
 *  Created on: 2019年1月9日
 *      Author: Henry Zhu
 */

#ifndef INTERFACE_PERCAPTION_INFORMATION_H_
#define INTERFACE_PERCAPTION_INFORMATION_H_

#include <QMainWindow>
#include "Common/Utils/Inc/property.h"
#include "Common/Configure/Configs/vehilce_config.h"
#include "Common/VehicleState/Interface/vehicle_state.h"

#include "Interaction/Ultrasonic/Ultrasonic.h"

// math
#include "math.h"
#include "Common/Math/vector_2d.h"
#include "Common/Utils/Inc/link_list.h"
#include "Common/Math/curve_fitting.h"
#include "Common/Math/huogh.h"

typedef enum _ObstacleRegion
{
    LeftRegion = 0,
    LeftCenterRegion,
    CenterRegion,
    RightCenterRegion,
    RightRegion
}ObstacleRegion;

typedef struct _LineFitInformationPacket
{
    float angle;
    float offset;
    uint8_t valid_flag;
    float variance;
}LineFitInformationPacket;

typedef struct _EdgeInformationPacket
{
    Vector2d position;
    float    angle;
    uint8_t  valid_flag;
}EdgeInformationPacket;

typedef struct _ObstacleInformationPacket
{
    EdgeInformationPacket FrontOutSide;
    EdgeInformationPacket RearOutSide;
}ObstacleInformationPacket;

typedef struct _ParallelParkingInformationPacket
{
    Vector2d InSideRearPoint_Position;
    Vector2d InSideFrontPoint_Position;
    Vector2d OutSideFrontPoint_Position;
    Vector2d OutSideRearPoint_Position;
}ParallelParkingInformationPacket;

typedef struct _ObstacleDistancePacket
{
    float            distance;
    ObstacleRegion   region;
    UltrasonicStatus status;
}ObstacleDistancePacket;

class Percaption {
public:
    Percaption();
    virtual ~Percaption();

    void Init(void);

//	virtual uint8_t Work(Ultrasonic *u,VehicleState *v) = 0;
    // wheel speed
    float getPositionX();
    void  setPositionX(float value);
    Property<Percaption,float,READ_WRITE> PositionX;

    float getPositionY();
    void  setPositionY(float value);
    Property<Percaption,float,READ_WRITE> PositionY;

    float getAttitudeYaw();
    void  setAttitudeYaw(float value);
    Property<Percaption,float,READ_WRITE> AttitudeYaw;

    float getParkingLength();
    void  setParkingLength(float value);
    Property<Percaption,float,READ_WRITE> ParkingLength;

    float getParkingWidth();
    void  setParkingWidth(float value);
    Property<Percaption,float,READ_WRITE> ParkingWidth;

    bool getDetectParkingStatus();
    void setDetectParkingStatus(bool value);
    Property<Percaption,bool,READ_WRITE> DetectParkingStatus;

    uint8_t getCommand();
    void setCommand(uint8_t value);
    Property<Percaption,uint8_t,READ_WRITE> Command;

    ObstacleInformationPacket getValidParkingEdgePosition();
    void    setValidParkingEdgePosition(ObstacleInformationPacket value);
    Property<Percaption,ObstacleInformationPacket,READ_WRITE> ValidParkingEdgePosition;

    EdgeInformationPacket getValidParkingCenterPosition();
    void    setValidParkingCenterPosition(EdgeInformationPacket value);
    Property<Percaption,EdgeInformationPacket,READ_WRITE> ValidParkingCenterPosition;

    ObstacleDistancePacket getObstacleDistance();
    void    setObstacleDistance(ObstacleDistancePacket value);
    Property<Percaption,ObstacleDistancePacket,READ_WRITE> ObstacleDistance;

    ObstacleDistancePacket getFrontObstacleDistance();
    void    setFrontObstacleDistance(ObstacleDistancePacket value);
    Property<Percaption,ObstacleDistancePacket,READ_WRITE> FrontObstacleDistance;

    ObstacleDistancePacket getRearObstacleDistance();
    void    setRearObstacleDistance(ObstacleDistancePacket value);
    Property<Percaption,ObstacleDistancePacket,READ_WRITE> RearObstacleDistance;

    ParallelParkingInformationPacket getParallelParkingSlotPosition();
    void    setParallelParkingSlotPosition(ParallelParkingInformationPacket value);
    Property<Percaption,ParallelParkingInformationPacket,READ_WRITE> ParallelParkingSlotPosition;

    LineFitInformationPacket getFrontEdgeFitLinePacket();
    void setFrontEdgeFitLinePacket(LineFitInformationPacket value);
    Property<Percaption,LineFitInformationPacket,READ_WRITE> FrontEdgeFitLinePacket;

    LineFitInformationPacket getLeftFitLinePacket();
    void setLeftFitLinePacket(LineFitInformationPacket value);
    Property<Percaption,LineFitInformationPacket,READ_WRITE> LeftFitLinePacket;

    LineFitInformationPacket getRightFitLinePacket();
    void setRightFitLinePacket(LineFitInformationPacket value);
    Property<Percaption,LineFitInformationPacket,READ_WRITE> RightFitLinePacket;

    LineFitInformationPacket getCenterFitLinePacket();
    void setCenterFitLinePacket(LineFitInformationPacket value);
    Property<Percaption,LineFitInformationPacket,READ_WRITE> CenterFitLinePacket;
protected:
    // 最终输出的障碍物重新定位的库位信息
    ObstacleInformationPacket _valid_parking_edge_position;
    // 输出最终的库位边沿定位库位中心位置
    EdgeInformationPacket _valid_parking_center_position;
    // 输出障碍物离车辆边沿的距离
    ObstacleDistancePacket _obstacle_distance;
    ObstacleDistancePacket _front_obstacle_distance;
    ObstacleDistancePacket _rear_obstacle_distance;

    ParallelParkingInformationPacket _parallel_parking_slot_position;
    // 库边沿拟合
    LineFitInformationPacket _front_edge_fit_line_packet;
    LineFitInformationPacket _left_fit_line_packet;
    LineFitInformationPacket _right_fit_line_packet;
    LineFitInformationPacket _center_fit_line_packet;
private:
    float _position_x;
    float _position_y;
    float _attitude_yaw;
    float _parking_length;
    float _parking_width;
    bool  _detect_parking_status;
    uint8_t _command;
};

#endif /* INTERFACE_PERCAPTION_INFORMATION_H_ */
