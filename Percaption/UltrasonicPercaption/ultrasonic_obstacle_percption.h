/*****************************************************************************/
/* FILE NAME: ultrasonic_abstacle_percption.cpp COPYRIGHT (c) Henry Zhu 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the vector property             					         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      January 2 2019      Initial Version                  */
/* 1.1	 Henry Zhu      November 16 2019    Update huogh filter              */
/*****************************************************************************/

#ifndef ULTRASONICPERCAPTION_ULTRASONIC_OBSTACLE_PERCPTION_H_
#define ULTRASONICPERCAPTION_ULTRASONIC_OBSTACLE_PERCPTION_H_

#include "./Percaption/Interface/percaption.h"

// 选择运行的平台：嵌入式 -> 0 ; PC -> 1;
#define EMBEDDED_PLATFORM              (0)
#define PC_PLATFORM                    (1)
#define RUNNING_PLATFORM               PC_PLATFORM
// 选择车辆入库的方向，可选的是x轴还是y轴
#define X_AXIS_ENTER				   (0)
#define Y_AXIS_ENTER				   (1)
#define ENTER_PARK_DIRECTION           (X_AXIS_ENTER)

// 泊车控制命令
#define RIGHT_PARKING_START_CMD        (0x50)
#define LEFT_PARKING_START_CMD         (0x51)
#define ENTER_PARKING_START_CMD        (0x52)

#define RIGHT_PARKING_STOP_CMD         (0x58)
#define LEFT_PARKING_STOP_CMD          (0x59)
#define ENTER_PARKING_STOP_CMD         (0x5A)

#define LEFT_RIGHT_PARKING_CAL_CMD     (0x60)
#define ENTER_PARKING_CAL_CMD          (0x62)
// 入库过程库位调整相关参数
#define DISTANCE_ERR_THRESHOLD        (0.6f)
#define LEVEL_THRESHOLD               (2.2f)
#define WIDTH_THRESHOLD               (30.0f)
#define DISTANCE_THRESHOLD            (0.6f)
#define STEP_DISTANCE                 (0.05f)
#define MIN_EDGE_FIT_NUM              (10)
#define MIN_EDGE_FIT_DISTANCE         (0.6f)
#define MIN_LOCATION_NUM              (30)
// 进库后的相关参数调整
#define FIT_LINE_STEP_DISTANCE        (0.05f)
#define FIT_LINE_STEP_LEVEL_THRESHOLD (3.0f)
#define MIN_FIT_NUM                   (10)
#define MIN_FIT_DISTANCE              (0.6f)

// 霍夫滤波参数
#define DELTA_RHO                     (0.2f)
#define DELTA_THETA                   (0.5f)
#define DISTRIBUTION_THRESHOLD        (5)
#define LINES_MAX                     (1)
#define MIN_THETA                     (0.0f)
#define MAX_THETA                     (static_cast<float>(M_PI))

// 库位边沿查找算法
typedef enum _EdgeFindingState
{
    ObstacleWaitEdge= 0,
    VehicleEdgeWaitstate,
    WaitEnterDenseArea,
    JudgeParkingValid,
    waitParking
}EdgeFindingState;

typedef enum _EdgeFindingStateV1_0
{
    EdgeJudge= 0,
    EdgeLooking
}EdgeFindingStateV1_0;

typedef enum _LocationStatus
{
    LocationReady= 0,
    LocationDataPush,
    LocationCalculate,
    LocationFinish
}LocationStatus;

typedef enum _UltrasonicLocationPushState
{
    WaitPushStart= 0,
    ParkingLeftEdgeUltrasonicDataPush,
    ParkingRightEdgeUltrasonicDataPush,
    ParkingCenterUltrasonicDataPush
}UltrasonicLocationPushState;

typedef enum _UltrasonicLocationCalculateState
{
    WaitCommandForCalculate = 0,
    FrontRearEdgeCalculate,
    ParkingCenterCalculate
}UltrasonicLocationCalculateState;

typedef struct _para_edge_list
{
    int length=0;
    float X[10];
    float Y[10];
}_para_edge_list;

class ObstacleLinkList : public LinkList<ObstacleLocationPacket>{};

class UltrasonicObstaclePercption : public Percaption
{
public:
    UltrasonicObstaclePercption();
    virtual ~UltrasonicObstaclePercption();

    void Init();
    /******************************库位重新定位的状态机***************************************/
    // 库位重新定位
    /*
     * 数据推送状态机，负责有效超声数据的推送；
     * PC端需50ms调用一次该函数,嵌入式直接放入5ms定时器即可
     * */
    void DataPushStateMachine(Ultrasonic_Packet* p_dat,ObstacleLocationPacket *ug_dat,uint16_t index);

    // PC端数据注入测试使用该函数
    void DataPushStateMachine(Ultrasonic_Packet p_dat,ObstacleLocationPacket ug_dat,uint16_t index);
    /*
     * 库位位置和中心点位置，重新计算的状态机
     * 使用时无时序要求，放入最低优先级任务中即可，比如While(1)死循环中
     * 该状态机有返回值
     * 如果返回值为:-1 -> 计算未完成；0 -> 计算完成
     * */
    int8_t ParkingCalculateStateMachine(void);
    /*********************************************基础函数***************************************/
    uint32_t getFrontEdgeListLength();
    uint32_t getRearEdgeListLength();
    uint32_t getLeftEdgeListLength();
    uint32_t getRightEdgeListLength();
    LocationStatus getUltrasonicLocationStatus();
private:
    /*** the function of private ***/
    void Push(ObstacleLinkList *list,Ultrasonic_Packet u_dat,ObstacleLocationPacket p_dat);
    /*
     * 有序数据的推送
     * */
    void OrderedPush(ObstacleLinkList *list,ObstacleLocationPacket p_dat);
    /*
     * 无序数据的推送
     * */
    void DisorderPush(ObstacleLinkList *list,ObstacleLocationPacket p_dat);
    /*
     * Parallel Parking Date Push
     * */
    void ParaStructDataPush(_para_edge_list &list,ObstacleLocationPacket p_dat);

    void ParkingCenterPush(ObstacleLinkList *list,Ultrasonic_Packet u_dat,ObstacleLocationPacket p_dat);
    /*******************************************************************************************/
    /**************************************** 边沿提取算法 ***************************************/
    /*******************************************************************************************/
    void EdgeFinding_V1_0(ObstacleLinkList *list);
    void EdgeFinding_V1_1(ObstacleLinkList *list,ObstacleLinkList *vehicle_list);//增加车辆链表
    /*******************************************************************************************/
    /**************************************** 数据分布算法 ***************************************/
    /*******************************************************************************************/
    // 求取最高分布值的最优解
    float HighestDistribution(uint16_t group_number,uint16_t* group_value_array,float min_value);
    // 求取最高分布的索引值
    uint8_t HighestDistributionBase(uint8_t group_number,uint16_t* group_value_array);

    void FindPositionListMinValue(ObstacleLinkList *valid_list,float *min_x,uint16_t *num_x,float *min_y,uint16_t *num_y);
    void PositionPointDistributed(ObstacleLinkList *valid_list,
                                  float min_x,uint16_t x_array_len,uint16_t *x_value_array,
                                  float min_y,uint16_t y_array_len,uint16_t *y_value_array);
    void EdgePositionValueDistributed(ObstacleLinkList *valid_list);
    /*******************************************************************************************/
    /**************************************** 霍夫变换算法 ***************************************/
    /*******************************************************************************************/
    // 基于霍夫变换的边沿线滤波
    void EdgeLineFit_HoughFilter(ObstacleLinkList *line_point,LineFitInformationPacket *line);
    /*******************************************************************************************/
    /**************************************** 重定位算法 *****************************************/
    /*******************************************************************************************/
    // 入库过程中的前后边界拟合
    void FrontRearEdgeLineFit();
    // 进库后的库位边沿线拟合
    void ParkingCenterEdgeLineFit();
    /*******************************************************************************************/
    /**************************************** 私有变量 ******************************************/
    /*******************************************************************************************/
    uint8_t _calculate_command;
    LocationStatus _ultrasonic_location_sts;

    EdgeFindingState _edge_finding_state;
    EdgeFindingStateV1_0 _edge_finding_state_v1_0;

    UltrasonicLocationPushState      _data_push_state;
    UltrasonicLocationCalculateState _parking_calculate_state;

    // 入库过程中相关数据缓存
    ObstacleLinkList *_ultrasonic_front_edge_list;
    ObstacleLinkList *_ultrasonic_front_edge_list_first_vehicle;
    ObstacleLinkList *_ultrasonic_rear_edge_list_triangle_location;

    // 进库后相关数据缓存
    ObstacleLinkList *_left_edge_position_list;
    ObstacleLinkList *_right_edge_position_list;

    ObstacleLinkList *_left_fit_edge_list;
    ObstacleLinkList *_right_fit_edge_list;

    ObstacleInformationPacket _parking_position;
    ObstacleInformationPacket _vehicle_position;

    uint8_t _push_cnt;
    /******************************************/
    /////////////////////////////////////////////
    CurveFitting _line_fit;
    Huogh        _huogh;
};

#endif /* ULTRASONICPERCAPTION_ULTRASONIC_ABSTACLE_PERCPTION_H_ */
