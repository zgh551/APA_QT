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

#include "ultrasonic_obstacle_percption.h"

UltrasonicObstaclePercption::UltrasonicObstaclePercption() {
    Init();
}
UltrasonicObstaclePercption::~UltrasonicObstaclePercption() {
}

void UltrasonicObstaclePercption::Init()
{
    _ultrasonic_location_sts = LocationReady;

    _data_push_state = WaitPushStart;
    _parking_calculate_state = WaitCommandForCalculate;

    _edge_finding_state = ObstacleWaitEdge;
    _edge_finding_state_v1_0 = EdgeJudge;
    // 最终输出的库位信息
    _valid_parking_edge_position.FrontOutSide.position = Vector2d(0,0);
    _valid_parking_edge_position.FrontOutSide.angle = 0.0f;
    _valid_parking_edge_position.FrontOutSide.valid_flag = 0;

    _valid_parking_edge_position.RearOutSide.position  = Vector2d(0,0);
    _valid_parking_edge_position.RearOutSide.angle = 0.0f;
    _valid_parking_edge_position.RearOutSide.valid_flag = 0;
    // 进库后的库位中心调整
    _valid_parking_center_position.position = Vector2d(0,0);
    _valid_parking_center_position.angle    = 0;
    _push_cnt = 0;
    /******************************************/
    _ultrasonic_front_edge_list                  = new ObstacleLinkList;
    _ultrasonic_front_edge_list_first_vehicle    = new ObstacleLinkList;
    _ultrasonic_rear_edge_list_triangle_location = new ObstacleLinkList;

    _left_edge_position_list  = new ObstacleLinkList;
    _right_edge_position_list = new ObstacleLinkList;

    _left_fit_edge_list   = new ObstacleLinkList;
    _right_fit_edge_list  = new ObstacleLinkList;

}

/******************************************************************************************************************************/
/***                                                    数据推送                                                             ***/
/******************************************************************************************************************************/
void UltrasonicObstaclePercption::Push(ObstacleLinkList *list,Ultrasonic_Packet u_dat,ObstacleLocationPacket p_dat)
{
    // 时间序列的数据
    if(!list->getHeadNode())
    {
        if (u_dat.Distance2 > u_dat.Distance1)
        {
            if( (u_dat.Distance2 - u_dat.Distance1) < DISTANCE_ERR_THRESHOLD )
            {
                list->Add(p_dat);
            }
        }
        else
        {
            if((u_dat.Level > LEVEL_THRESHOLD) && (u_dat.Width > WIDTH_THRESHOLD))
            {
                list->Add(p_dat);
            }
        }
    }
    else
    {
        if(p_dat.Position.getX() < list->getEndNode()->data.Position.getX())
        {
            if (u_dat.Distance2 > u_dat.Distance1)
            {
                if( (u_dat.Distance2 - u_dat.Distance1) < DISTANCE_ERR_THRESHOLD )
                {
                    list->Add(p_dat);
                }
            }
            else
            {
                if((u_dat.Level > LEVEL_THRESHOLD) && (u_dat.Width > WIDTH_THRESHOLD))
                {
                    list->Add(p_dat);
                }
            }
        }
    }
}

// 有序列的数据
void UltrasonicObstaclePercption::OrderedPush(ObstacleLinkList *list,ObstacleLocationPacket p_dat)
{
    // 时间序列的数据
    if(!list->getHeadNode())
    {
        if(p_dat.Status == 0)
        {
            list->Add(p_dat);
        }
    }
    else
    {
        if( (p_dat.Status == 0) && (p_dat.Position.getX() < list->getEndNode()->data.Position.getX()))
        {
            list->Add(p_dat);
        }
    }
}

// 无序列的数据
void UltrasonicObstaclePercption::DisorderPush(ObstacleLinkList *list,ObstacleLocationPacket p_dat)
{
    // 无序数据
    if(!list->getHeadNode())
    {
        if(p_dat.Status == 0)
        {
            list->Add(p_dat);
        }
    }
    else
    {
        if( (p_dat.Status == 0) &&
            (fabs(p_dat.Position.LengthSquare() - list->getEndNode()->data.Position.LengthSquare()) > 1.0e-6f))
        {
            list->Add(p_dat);
        }
    }
}

void UltrasonicObstaclePercption::ParkingCenterPush(ObstacleLinkList *list,Ultrasonic_Packet u_dat,ObstacleLocationPacket p_dat)
{
    ObstacleLocationPacket temp_packet;
    // 有序数据集
    if(!list->getEndNode())
    {
        if( u_dat.Level > FIT_LINE_STEP_LEVEL_THRESHOLD )
        {
#if ENTER_PARK_DIRECTION == X_AXIS_ENTER
            list->Add(p_dat);
#elif ENTER_PARK_DIRECTION == Y_AXIS_ENTER
            temp_packet.Position = Vector2d(p_dat.Position.getY(),-p_dat.Position.getX());
            temp_packet.Status = p_dat.Status;
            list->Add(temp_packet);
#endif
        }
    }
    else
    {
#if ENTER_PARK_DIRECTION == X_AXIS_ENTER
        if((u_dat.Level > FIT_LINE_STEP_LEVEL_THRESHOLD) && (p_dat.Position.getX() < list->getEndNode()->data.Position.getX()))
        {
            list->Add(p_dat);
        }
#elif ENTER_PARK_DIRECTION == Y_AXIS_ENTER
        if((u_dat.Level > FIT_LINE_STEP_LEVEL_THRESHOLD) && (p_dat.Position.getY() < list->getEndNode()->data.Position.getX()))
        {
            temp_packet.Position = Vector2d(p_dat.Position.getY(),-p_dat.Position.getX());
            temp_packet.Status = p_dat.Status;
            list->Add(temp_packet);
        }
#endif
    }
}
/******************************************************************************************************************************/
/***                                                    边沿查找                                                             ***/
/******************************************************************************************************************************/
void UltrasonicObstaclePercption::EdgeFinding_V1_0(ObstacleLinkList *list)
{
    Node<ObstacleLocationPacket>* _current_node;//当前节点
    Node<ObstacleLocationPacket>* _last_node;//上一节点
    ObstacleInformationPacket vehicle_position_temp;
    float max_y_axis_value;
    float _err_distance;

    if(!list->getHeadNode())
    {
        return;
    }

    _last_node    = list->getHeadNode();//上一节点
    _current_node = _last_node->next;//当前节点

    while(_current_node->next != NULL)
    {
        _err_distance = (_current_node->data.Position - _last_node->data.Position).Length();
        switch(_edge_finding_state_v1_0)
        {
            case EdgeJudge:
                if(_err_distance > DISTANCE_THRESHOLD)
                {
                    vehicle_position_temp.FrontOutSide.position = _current_node->data.Position;
                }
                else
                {
                    vehicle_position_temp.FrontOutSide.position = _last_node->data.Position;
                }
                max_y_axis_value = vehicle_position_temp.FrontOutSide.position.getY();
                _edge_finding_state_v1_0 = EdgeLooking;
                break;

            case EdgeLooking:
                if(_err_distance > DISTANCE_THRESHOLD)
                {
                    vehicle_position_temp.RearOutSide.position = _last_node->data.Position;

                    _valid_parking_edge_position.FrontOutSide.position.setX( vehicle_position_temp.RearOutSide.position.getX() );
                    _valid_parking_edge_position.FrontOutSide.position.setY( max_y_axis_value );

                    vehicle_position_temp.FrontOutSide.position = _current_node->data.Position;
                    max_y_axis_value = vehicle_position_temp.FrontOutSide.position.getY();
                }
                else
                {
                    vehicle_position_temp.RearOutSide.position = _current_node->data.Position;
                    max_y_axis_value = _current_node->data.Position.getY() > max_y_axis_value ? _current_node->data.Position.getY() : max_y_axis_value;
                }
                break;

            default:
                break;
        }
        if(_err_distance < DISTANCE_THRESHOLD)
        {
            _valid_parking_edge_position.FrontOutSide.position.setX( vehicle_position_temp.RearOutSide.position.getX() );
            _valid_parking_edge_position.FrontOutSide.position.setY( max_y_axis_value );
        }
        _last_node = _current_node;
        _current_node = _current_node->next;
    }
    if(list->Length() != 0){list->Delete();}
}

void UltrasonicObstaclePercption::EdgeFinding_V1_1(ObstacleLinkList *list,ObstacleLinkList *vehicle_list)
{
    Node<ObstacleLocationPacket>* _current_node;//当前节点
    Node<ObstacleLocationPacket>* _last_node;//上一节点
    ObstacleInformationPacket vehicle_position_temp;
    float max_y_axis_value = -10;
    float _err_distance;

    if(0 == list->Length())
    {
        return;
    }

    _last_node    = list->getHeadNode();//上一节点
    _current_node = _last_node->next;//当前节点
    if(vehicle_list->Length() != 0){vehicle_list->Delete();}
    _edge_finding_state_v1_0 = EdgeJudge;
    while(_current_node->next != NULL)
    {
        _err_distance = (_current_node->data.Position - _last_node->data.Position).Length();
        switch(_edge_finding_state_v1_0)
        {
            case EdgeJudge:
                if(_err_distance > DISTANCE_THRESHOLD)
                {
                    vehicle_position_temp.FrontOutSide.position = _current_node->data.Position;
                    vehicle_list->Add(_current_node->data);
                }
                else
                {
                    vehicle_position_temp.FrontOutSide.position = _last_node->data.Position;
                    vehicle_list->Add(_last_node->data);
                }
                max_y_axis_value = vehicle_position_temp.FrontOutSide.position.getY();
                _edge_finding_state_v1_0 = EdgeLooking;
                break;

            case EdgeLooking:
                if(_err_distance > DISTANCE_THRESHOLD)
                {
                    vehicle_position_temp.RearOutSide.position = _last_node->data.Position;
                    _valid_parking_edge_position.FrontOutSide.position.setX( vehicle_position_temp.RearOutSide.position.getX() );
                    _valid_parking_edge_position.FrontOutSide.position.setY( max_y_axis_value );
                    _valid_parking_edge_position.FrontOutSide.valid_flag = 0xA5;
                    vehicle_position_temp.FrontOutSide.position = _current_node->data.Position;
                    max_y_axis_value = vehicle_position_temp.FrontOutSide.position.getY();
                    if(vehicle_list->Length() != 0){vehicle_list->Delete();}
                }
                else
                {
                    vehicle_position_temp.RearOutSide.position = _current_node->data.Position;
                    vehicle_list->Add(_current_node->data);
                    max_y_axis_value = _current_node->data.Position.getY() > max_y_axis_value ? _current_node->data.Position.getY() : max_y_axis_value;
                }
                break;

            default:

                break;
        }
        if(_err_distance < DISTANCE_THRESHOLD)
        {
            _valid_parking_edge_position.FrontOutSide.position.setX( vehicle_position_temp.RearOutSide.position.getX() );
            _valid_parking_edge_position.FrontOutSide.position.setY( max_y_axis_value );
            _valid_parking_edge_position.FrontOutSide.valid_flag = 0xA5;
        }
        _last_node = _current_node;
        _current_node = _current_node->next;
    }
//    if(list->Length() != 0){list->Delete();}
}

/******************************************************************************************************************************/
/***                                                    数值分布                                                             ***/
/******************************************************************************************************************************/
void UltrasonicObstaclePercption::FindPositionListMinValue(ObstacleLinkList *valid_list,float *min_x,uint16_t *num_x,float *min_y,uint16_t *num_y)
{
    float max_x,max_y;

    if(0 == valid_list->Length()){return;}
    //当前节点零时变量
    Node<ObstacleLocationPacket> *_current_node_triangle = valid_list->getHeadNode();
    *min_x = _current_node_triangle->data.Position.getX();
    *min_y = _current_node_triangle->data.Position.getY();
     max_x = _current_node_triangle->data.Position.getX();
     max_y = _current_node_triangle->data.Position.getY();

    while(_current_node_triangle->next != NULL)
    {
        *min_x = _current_node_triangle->data.Position.getX() < *min_x ? _current_node_triangle->data.Position.getX() : *min_x;
        *min_y = _current_node_triangle->data.Position.getY() < *min_y ? _current_node_triangle->data.Position.getY() : *min_y;
         max_x = _current_node_triangle->data.Position.getX() >  max_x ? _current_node_triangle->data.Position.getX() :  max_x;
         max_y = _current_node_triangle->data.Position.getY() >  max_y ? _current_node_triangle->data.Position.getY() :  max_y;
        _current_node_triangle = _current_node_triangle->next;
    }
    *num_x = static_cast<uint16_t>((max_x - *min_x)/STEP_DISTANCE) + 1;
    *num_y = static_cast<uint16_t>((max_y - *min_y)/STEP_DISTANCE) + 1;
}

void UltrasonicObstaclePercption::PositionPointDistributed(ObstacleLinkList *valid_list,
                                                           float min_x,uint16_t x_array_len,uint16_t *x_value_array,
                                                           float min_y,uint16_t y_array_len,uint16_t *y_value_array)
{
    uint16_t i;
    if(0 == valid_list->Length()){return;}
    for(i = 0;i<x_array_len;i++)
    {
        x_value_array[i] = 0;
    }
    for(i = 0;i<y_array_len;i++)
    {
        y_value_array[i] = 0;
    }
    //当前节点零时变量
    Node<ObstacleLocationPacket> *_current_node_triangle = valid_list->getHeadNode();
    while(_current_node_triangle->next != NULL)
    {
        for(i = 0;i<x_array_len;i++)
        {
            if( (_current_node_triangle->data.Position.getX() >= (min_x + STEP_DISTANCE*i)) && (_current_node_triangle->data.Position.getX() < (min_x + STEP_DISTANCE*(i + 1))))
            {
                x_value_array[i]++;
                break;
            }
        }
        for(i = 0;i<y_array_len;i++)
        {
            if( (_current_node_triangle->data.Position.getY() >= (min_y + STEP_DISTANCE*i)) && (_current_node_triangle->data.Position.getY() < (min_y + STEP_DISTANCE*(i + 1))))
            {
                y_value_array[i]++;
                break;
            }
        }
        _current_node_triangle = _current_node_triangle->next;
    }
}

float UltrasonicObstaclePercption::HighestDistribution(uint16_t group_number,uint16_t* group_value_array,float min_value)
{
    uint8_t i;
    uint8_t max_distribute_number_id = 0;
    uint16_t max_distribute_number_value = 0;

    uint16_t sum_value;
    float master_ratio,slave_ratio;

    for(i = 0; i < group_number; i++)
    {
        if(i == 0)
        {
            max_distribute_number_id = i;
            max_distribute_number_value = group_value_array[i];
        }
        else
        {
            if(group_value_array[i] > max_distribute_number_value)
            {
                max_distribute_number_value = group_value_array[i];
                max_distribute_number_id    = i;
            }
        }
    }
    //
    if((0 == max_distribute_number_id) && (group_number > 1))
    {
        if(group_value_array[max_distribute_number_id] < 2.0f * group_value_array[max_distribute_number_id + 1])
        {
            sum_value    = group_value_array[max_distribute_number_id] + group_value_array[max_distribute_number_id + 1];
            master_ratio = 1.0f * group_value_array[ max_distribute_number_id ]     / sum_value;
            slave_ratio  = 1.0f * group_value_array[ max_distribute_number_id + 1 ] / sum_value;
            return min_value + STEP_DISTANCE * ( (max_distribute_number_id + 0.5f) * master_ratio + (max_distribute_number_id + 1.5f) * slave_ratio);
        }
        else
        {
            return min_value + STEP_DISTANCE * (max_distribute_number_id + 0.5f);
        }
    }
    else if( (group_number - 1) == max_distribute_number_id )
    {
        if(group_value_array[max_distribute_number_id] < 2.0f * group_value_array[max_distribute_number_id - 1])
        {
            sum_value    = group_value_array[max_distribute_number_id] + group_value_array[max_distribute_number_id - 1];
            master_ratio = 1.0f * group_value_array[ max_distribute_number_id ]     / sum_value;
            slave_ratio  = 1.0f * group_value_array[ max_distribute_number_id - 1 ] / sum_value;
            return min_value + STEP_DISTANCE * ( (max_distribute_number_id + 0.5f) * master_ratio + (max_distribute_number_id - 0.5f) * slave_ratio);
        }
        else
        {
            return min_value + STEP_DISTANCE * (max_distribute_number_id + 0.5f);
        }
    }
    else
    {
        if(group_value_array[max_distribute_number_id - 1] > group_value_array[max_distribute_number_id + 1])
        {
            if(group_value_array[max_distribute_number_id] < 2.0f * group_value_array[max_distribute_number_id - 1])
            {
                sum_value    = group_value_array[max_distribute_number_id] + group_value_array[max_distribute_number_id - 1];
                master_ratio = 1.0f * group_value_array[ max_distribute_number_id ]     / sum_value;
                slave_ratio  = 1.0f * group_value_array[ max_distribute_number_id - 1 ] / sum_value;
                return min_value + STEP_DISTANCE * ( (max_distribute_number_id + 0.5f) * master_ratio + (max_distribute_number_id - 0.5f) * slave_ratio);
            }
            else
            {
                return min_value + STEP_DISTANCE * (max_distribute_number_id + 0.5f);
            }
        }
        else
        {
            if(group_value_array[max_distribute_number_id] < 2.0f * group_value_array[max_distribute_number_id + 1])
            {
                sum_value    = group_value_array[max_distribute_number_id] + group_value_array[max_distribute_number_id + 1];
                master_ratio = 1.0f * group_value_array[ max_distribute_number_id ]     / sum_value;
                slave_ratio  = 1.0f * group_value_array[ max_distribute_number_id + 1 ] / sum_value;
                return min_value + STEP_DISTANCE * ( (max_distribute_number_id + 0.5f) * master_ratio + (max_distribute_number_id + 1.5f) * slave_ratio);
            }
            else
            {
                return min_value + STEP_DISTANCE * (max_distribute_number_id + 0.5f);
            }
        }
    }
}

void  UltrasonicObstaclePercption::EdgePositionValueDistributed(ObstacleLinkList *valid_list)
{
    float min_x,min_y;
    uint16_t  x_group_number,y_group_number;

    FindPositionListMinValue(valid_list,&min_x,&x_group_number,&min_y,&y_group_number);

    uint16_t distribute_number_x[x_group_number];
    uint16_t distribute_number_y[y_group_number];

    PositionPointDistributed(valid_list,
                             min_x,x_group_number,distribute_number_x,
                             min_y,y_group_number,distribute_number_y);

    _valid_parking_edge_position.RearOutSide.position.setX(HighestDistribution(x_group_number,distribute_number_x,min_x));
    _valid_parking_edge_position.RearOutSide.position.setY(HighestDistribution(y_group_number,distribute_number_y,min_y));
    _valid_parking_edge_position.RearOutSide.valid_flag = 0xA5;

//    valid_list->Delete();
}
/******************************************************************************************************************************/
/***                                                    数据滤波                                                             ***/
/******************************************************************************************************************************/
void UltrasonicObstaclePercption::EdgeLineFit_HoughFilter(ObstacleLinkList *line_point,LineFitInformationPacket *line)
{
    if(( line_point->Length() > MIN_FIT_NUM) &&
       ((line_point->getHeadNode()->data.Position - line_point->getEndNode()->data.Position).Length() > MIN_FIT_DISTANCE))
    {
        ObstacleLinkList *_fit_edge_list = new ObstacleLinkList();
        _huogh.HoughLinesStandard(line_point,_fit_edge_list,DELTA_RHO,DELTA_THETA,DISTRIBUTION_THRESHOLD,LINES_MAX,MIN_THETA,MAX_THETA);
        line->variance = _line_fit.LineFitting(_fit_edge_list,&line->angle,&line->offset);
        line->valid_flag = 0xAA;
        qDebug("USS STYPE Slot Angle:%f Offset:%f",static_cast<double>(line->angle),static_cast<double>(line->offset));
    }
    else {
        line->angle = 0.0f;
        line->offset = 0.0f;
        line->variance = 0.0f;
        line->valid_flag = 0x55;
        qDebug("Parking Edge too Small");
    }
}
/******************************************************************************************************************************/
/***                                                    线性拟合                                                             ***/
/******************************************************************************************************************************/
// 入库过程中前后库边沿定位函数
void UltrasonicObstaclePercption::FrontRearEdgeLineFit()
{
    // 库位边界点计算
    if(_ultrasonic_front_edge_list->Length() == 0){return;}
    EdgeFinding_V1_1(_ultrasonic_front_edge_list,_ultrasonic_front_edge_list_first_vehicle);
    //车辆斜率计算
    EdgeLineFit_HoughFilter(_ultrasonic_front_edge_list_first_vehicle,&_front_edge_fit_line_packet);
    // 车辆尾部数值分布
    EdgePositionValueDistributed(_ultrasonic_rear_edge_list_triangle_location);
}


// 车辆进库，通过库位边界，测定库位的中心位置，准确的前提是车辆基本垂直进入
void UltrasonicObstaclePercption::ParkingCenterEdgeLineFit()
{
    EdgeLineFit_HoughFilter(_left_edge_position_list,&_left_fit_line_packet);
    EdgeLineFit_HoughFilter(_right_edge_position_list,&_right_fit_line_packet);
    if(_left_edge_position_list->Length() != 0){_left_edge_position_list->Delete();}
    if(_right_edge_position_list->Length() != 0){_right_edge_position_list->Delete();}
}

/******************************************************************************************************************************/
/***                                                    数据推送任务                                                          ***/
/******************************************************************************************************************************/
void  UltrasonicObstaclePercption::DataPushStateMachine(Ultrasonic_Packet* p_dat,ObstacleLocationPacket *ug_dat,uint16_t index)
{
    switch(_data_push_state)
    {
        case WaitPushStart:
            if(RIGHT_PARKING_START_CMD == Command)//泊车右侧入库
            {
                if(0 != _ultrasonic_front_edge_list->Length()){_ultrasonic_front_edge_list->Delete();}
                if(0 != _ultrasonic_rear_edge_list_triangle_location->Length()){_ultrasonic_rear_edge_list_triangle_location->Delete();}
                _ultrasonic_location_sts = LocationDataPush;
                _data_push_state = ParkingRightEdgeUltrasonicDataPush;
            }
            else if(LEFT_PARKING_START_CMD == Command)//泊车左侧入库
            {
                if(0 != _ultrasonic_front_edge_list->Length()){_ultrasonic_front_edge_list->Delete();}
                if(0 != _ultrasonic_rear_edge_list_triangle_location->Length()){_ultrasonic_rear_edge_list_triangle_location->Delete();}
                _ultrasonic_location_sts = LocationDataPush;
                _data_push_state = ParkingLeftEdgeUltrasonicDataPush;
            }
            else if(ENTER_PARKING_START_CMD == Command)//泊车进库
            {
                if(0 != _left_edge_position_list->Length()){_left_edge_position_list->Delete();}
                if(0 != _right_edge_position_list->Length()){_right_edge_position_list->Delete();}
                _ultrasonic_location_sts = LocationDataPush;
                _data_push_state = ParkingCenterUltrasonicDataPush;
            }
            break;

        case ParkingRightEdgeUltrasonicDataPush:
#if RUNNING_PLATFORM == Embedded_PLATFORM
            if( (0 == u_dat->ScheduleTimeCnt) || (14 == u_dat->ScheduleTimeCnt))
            {
                Push(_ultrasonic_front_edge_list,p_dat[11],ug_dat[11]);
            }
            if(23 == u_dat->ScheduleTimeCnt)
            {
                Push(_ultrasonic_rear_edge_list_triangle_location,ug_dat[5]);
                Push(_ultrasonic_rear_edge_list_triangle_location,ug_dat[6]);
            }
#elif RUNNING_PLATFORM == PC_PLATFORM
            switch(index)
            {
                case 5:
                    DisorderPush(_ultrasonic_rear_edge_list_triangle_location,ug_dat[5]);
                break;

                case 6:
                    DisorderPush(_ultrasonic_rear_edge_list_triangle_location,ug_dat[6]);
                break;

                case 11:
                    Push(_ultrasonic_front_edge_list,p_dat[11],ug_dat[11]);
                break;

                default:
                break;
            }
#endif
            if(RIGHT_PARKING_STOP_CMD == Command)//垂直结束
            {
                _calculate_command = LEFT_RIGHT_PARKING_CAL_CMD;//库位计算命令
                _data_push_state = WaitPushStart;
            }
            break;

        case ParkingLeftEdgeUltrasonicDataPush:
    #if RUNNING_PLATFORM == Embedded_PLATFORM
            if((12 == u_dat->ScheduleTimeCnt) || (26 == u_dat->ScheduleTimeCnt))
            {
                Push(_ultrasonic_front_edge_list,p_dat[10],ug_dat[10]);
            }
            if(23 == u_dat->ScheduleTimeCnt)
            {
                Push(_ultrasonic_rear_edge_list_triangle_location,ug_dat[5]);
                Push(_ultrasonic_rear_edge_list_triangle_location,ug_dat[6]);
            }
    #elif RUNNING_PLATFORM == PC_PLATFORM
            switch(index)
            {
                case 5:
                    DisorderPush(_ultrasonic_rear_edge_list_triangle_location,ug_dat[5]);
                break;

                case 6:
                    DisorderPush(_ultrasonic_rear_edge_list_triangle_location,ug_dat[6]);
                break;

                case 10:
                    Push(_ultrasonic_front_edge_list,p_dat[10],ug_dat[10]);
                break;
                default:
                break;
            }
    #endif
            if(LEFT_PARKING_STOP_CMD == Command)
            {
                _calculate_command = LEFT_RIGHT_PARKING_CAL_CMD;//库位计算命令
                _data_push_state = WaitPushStart;
            }
            break;

        case ParkingCenterUltrasonicDataPush:
#if RUNNING_PLATFORM == Embedded_PLATFORM
            if((12 == u_dat->ScheduleTimeCnt) || (26 == u_dat->ScheduleTimeCnt))
            {
                ParkingCenterPush(_left_edge_position_list,p_dat[10],ug_dat[10]);
            }
            if((0 == u_dat->ScheduleTimeCnt) || (14 == u_dat->ScheduleTimeCnt))
            {
                ParkingCenterPush(_right_edge_position_list,p_dat[11],ug_dat[11]);
            }
#elif RUNNING_PLATFORM == PC_PLATFORM
            switch(index)
            {
                case 10:
                    ParkingCenterPush(_left_edge_position_list,p_dat[10],ug_dat[10]);
                break;

                case 11:
                    ParkingCenterPush(_right_edge_position_list,p_dat[11],ug_dat[11]);
                break;

                default:
                break;
            }
#endif
            if(ENTER_PARKING_STOP_CMD == Command)//进库完成，开始计算
            {
                _calculate_command = ENTER_PARKING_CAL_CMD;//进库计算命令
                _data_push_state = WaitPushStart;
            }
            break;

        default:
            break;
    }
}

void  UltrasonicObstaclePercption::DataPushStateMachine(Ultrasonic_Packet p_dat,ObstacleLocationPacket ug_dat,uint16_t index)
{
    switch(_data_push_state)
    {
        case WaitPushStart:
            if(RIGHT_PARKING_START_CMD == Command)//垂直泊车右侧入库
            {
                if(0 != _ultrasonic_front_edge_list->Length()){_ultrasonic_front_edge_list->Delete();}
                if(0 != _ultrasonic_rear_edge_list_triangle_location->Length()){_ultrasonic_rear_edge_list_triangle_location->Delete();}
                _ultrasonic_location_sts = LocationDataPush;
                _data_push_state = ParkingRightEdgeUltrasonicDataPush;
            }
            else if(LEFT_PARKING_START_CMD == Command)//垂直泊车左侧入库
            {
                if(0 != _ultrasonic_front_edge_list->Length()){_ultrasonic_front_edge_list->Delete();}
                if(0 != _ultrasonic_rear_edge_list_triangle_location->Length()){_ultrasonic_rear_edge_list_triangle_location->Delete();}
                _ultrasonic_location_sts = LocationDataPush;
                _data_push_state = ParkingLeftEdgeUltrasonicDataPush;
            }
            else if(ENTER_PARKING_START_CMD == Command)//垂直泊车进库
            {
                if(0 != _left_edge_position_list->Length()){_left_edge_position_list->Delete();}
                if(0 != _right_edge_position_list->Length()){_right_edge_position_list->Delete();}
                _ultrasonic_location_sts = LocationDataPush;
                _data_push_state = ParkingCenterUltrasonicDataPush;
            }
            break;

        case ParkingRightEdgeUltrasonicDataPush:
#if RUNNING_PLATFORM == Embedded_PLATFORM
            if( (12 == u_dat->ScheduleTimeCnt) || (26 == u_dat->ScheduleTimeCnt))
            {
                Push(_ultrasonic_front_edge_list,p_dat[11],ug_dat[11]);
            }
            if(23 == u_dat->ScheduleTimeCnt)
            {
                Push(_ultrasonic_rear_edge_list_triangle_location,ug_dat[5]);
                Push(_ultrasonic_rear_edge_list_triangle_location,ug_dat[6]);
            }
#elif RUNNING_PLATFORM == PC_PLATFORM
            switch(index)
            {
                case 5:
                    DisorderPush(_ultrasonic_rear_edge_list_triangle_location,ug_dat);
                break;

                case 6:
                    DisorderPush(_ultrasonic_rear_edge_list_triangle_location,ug_dat);
                break;

                case 11:
                    Push(_ultrasonic_front_edge_list,p_dat,ug_dat);
                break;

                default:

                break;
            }
#endif
            if(RIGHT_PARKING_STOP_CMD == Command)//数据推送结束
            {
                _calculate_command = LEFT_RIGHT_PARKING_CAL_CMD;//库位计算命令
                _data_push_state = WaitPushStart;
            }
            break;

        case ParkingLeftEdgeUltrasonicDataPush:
#if RUNNING_PLATFORM == Embedded_PLATFORM
            if((0 == u_dat->ScheduleTimeCnt) || (14 == u_dat->ScheduleTimeCnt))
            {
              Push(_ultrasonic_front_edge_list,p_dat[10],ug_dat[10]);
            }
            if(23 == u_dat->ScheduleTimeCnt)
            {
              Push(_ultrasonic_rear_edge_list_triangle_location,ug_dat[5]);
              Push(_ultrasonic_rear_edge_list_triangle_location,ug_dat[6]);
            }
#elif RUNNING_PLATFORM == PC_PLATFORM
            switch(index)
            {
                case 5:
                    DisorderPush(_ultrasonic_rear_edge_list_triangle_location,ug_dat);
                break;

                case 6:
                    DisorderPush(_ultrasonic_rear_edge_list_triangle_location,ug_dat);
                break;

                case 10:
                    Push(_ultrasonic_front_edge_list,p_dat,ug_dat);
                break;

                default:
                break;
            }
#endif
            if(LEFT_PARKING_STOP_CMD == Command)
            {
                _calculate_command = LEFT_RIGHT_PARKING_CAL_CMD;//库位计算命令
                _data_push_state = WaitPushStart;
            }
        break;

        case ParkingCenterUltrasonicDataPush:
#if RUNNING_PLATFORM == Embedded_PLATFORM
            if((26 == u_dat->ScheduleTimeCnt) || (12 == u_dat->ScheduleTimeCnt))
            {
                ParkingCenterPush(_left_edge_position_list,p_dat[10],ug_dat[10]);
            }
            if((0 == u_dat->ScheduleTimeCnt) || (14 == u_dat->ScheduleTimeCnt))
            {
                ParkingCenterPush(_right_edge_position_list,p_dat[11],ug_dat[11]);
            }
#elif RUNNING_PLATFORM == PC_PLATFORM
            switch(index)
            {
                case 10:
                    ParkingCenterPush(_left_edge_position_list,p_dat,ug_dat);
                break;

                case 11:
                    ParkingCenterPush(_right_edge_position_list,p_dat,ug_dat);
                break;

                default:
                break;
            }
#endif
            if(ENTER_PARKING_STOP_CMD == Command)//进库完成，开始计算
            {
                _calculate_command = ENTER_PARKING_CAL_CMD;//进库计算命令
                _data_push_state = WaitPushStart;
            }
            break;

        default:
            break;
    }
}

int8_t UltrasonicObstaclePercption::ParkingCalculateStateMachine(void)
{
    switch(_parking_calculate_state)
    {
        case WaitCommandForCalculate:
            if(LEFT_RIGHT_PARKING_CAL_CMD == _calculate_command)
            {
                _ultrasonic_location_sts = LocationCalculate;
                _parking_calculate_state = FrontRearEdgeCalculate;
            }
            else if(ENTER_PARKING_CAL_CMD == _calculate_command)
            {
                _ultrasonic_location_sts = LocationCalculate;
                _parking_calculate_state = ParkingCenterCalculate;
            }
            break;

        case FrontRearEdgeCalculate:
            FrontRearEdgeLineFit();
            _calculate_command = 0;
            _ultrasonic_location_sts = LocationFinish;
            _parking_calculate_state = WaitCommandForCalculate;
            return SUCCESS;
            break;

        case ParkingCenterCalculate:
            ParkingCenterEdgeLineFit();
            _calculate_command = 0;
            _ultrasonic_location_sts = LocationFinish;
            _parking_calculate_state = WaitCommandForCalculate;
            return SUCCESS;
            break;

        default:
            break;
    }
    return FAIL;
}

/******************************************************************************************************************************/
/***                                                    属性值获取                                                            ***/
/******************************************************************************************************************************/
uint32_t UltrasonicObstaclePercption::getFrontEdgeListLength(){return _ultrasonic_front_edge_list->Length();}

uint32_t UltrasonicObstaclePercption::getRearEdgeListLength(){return _ultrasonic_rear_edge_list_triangle_location->Length();}

uint32_t UltrasonicObstaclePercption::getLeftEdgeListLength(){return _left_edge_position_list->Length();}

uint32_t UltrasonicObstaclePercption::getRightEdgeListLength(){return _right_edge_position_list->Length();}

LocationStatus UltrasonicObstaclePercption::getUltrasonicLocationStatus() { return  _ultrasonic_location_sts;}

