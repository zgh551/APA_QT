#include "trajectory_analyzer.h"

TrajectoryAnalyzer::TrajectoryAnalyzer()
{
    _trajectory_points_vector = new std::vector<TargetTrack>;
}

void TrajectoryAnalyzer::Init(TrackLinkList *list)
{
    _trajectory_points_list = list;
}

void TrajectoryAnalyzer::Init(std::vector<TargetTrack> *vec)
{
    _trajectory_points_vector = vec;
    _target_point_index = 0;
}
/**
 * @brief CalculateNearestPointByPosition 基于位置信息计算目标曲线上最近的点
 * @param x 当前位置x轴坐标
 * @param y 当前位置y轴坐标
 * @return 返回最近目标点信息
 */
TargetTrack TrajectoryAnalyzer::CalculateNearestPointByPosition(const double x, const double y)
{
    Vector2d target_point,current_point;
    uint16_t start_id,end_id;
    double min_value,current_value;
//    start_id = 0.0 > _target_point_index - 10 ? 0 : _target_point_index - 10;
//    end_id   = _trajectory_points_vector->size() < _target_point_index + 20 ? _trajectory_points_vector->size() : _target_point_index + 20;
    target_point = _trajectory_points_vector->at(0).point;
    current_point = Vector2d(x,y);
    min_value = (target_point - current_point).Length();
    for(uint16_t i = 0;i < _trajectory_points_vector->size();i++)
    {
        current_value = (_trajectory_points_vector->at(i).point - current_point).Length();
        if(current_value < min_value)
        {
            _target_point_index = i;
            min_value = current_value;
        }
    }
    return _trajectory_points_vector->at(_target_point_index);
}

/**
 * @brief DistanceToEnd 计算离最后目标曲线点的距离
 * @param x 当前位置x轴坐标
 * @param y 当前位置y轴坐标
 * @return 离最后点的距离值
 */
double TrajectoryAnalyzer::DistanceToEnd(const double x, const double y)const
{
    return (_trajectory_points_vector->back().point - Vector2d(x,y)).Length();
}

void TrajectoryAnalyzer::TrajectoryTransformToCOM(double rear_to_com_distance)
{
    for(uint16_t i = 0;i < _trajectory_points_vector->size();i++)
    {
        _trajectory_points_vector->at(i).point = ComputeCOMPosition(rear_to_com_distance,_trajectory_points_vector->at(i));
    }
}

Vector2d TrajectoryAnalyzer::ComputeCOMPosition(double rear_to_com_distance,TargetTrack path_point)
{
    return path_point.point + Vector2d(rear_to_com_distance,0.0).rotate(path_point.yaw);
}
//TargetTrack TrajectoryAnalyzer::CalculateNearestPointByPosition(const double x, const double y)const
//{
//    Node<TargetTrack>* track_index_node;
//    Vector2d temp_track;
//    TargetTrack index_node;
//    float min_value;

//    temp_track.setX(static_cast<float>(x));
//    temp_track.setY(static_cast<float>(y));
//    index_node.point.setX(0.0);
//    index_node.point.setY(0.0);
//    index_node.yaw = 0.0f;
//    index_node.curvature = 0.0f;
//    if(!_trajectory_points_list)
//    {
//        return index_node;
//    }
//    if(_trajectory_points_list->Length() < 1)
//    {
//        return index_node;
//    }

//    track_index_node = _trajectory_points_list->getHeadNode();
//    min_value = (track_index_node->data.point - temp_track).Length();
//    index_node = track_index_node->data;
//    while(track_index_node->next != NULL)
//    {
//        if((track_index_node->data.point - temp_track).Length() < min_value)
//        {
//            min_value = (track_index_node->data.point - temp_track).Length();
//            index_node = track_index_node->data;
//        }
//        track_index_node = track_index_node->next;
//    }
//    return index_node;
//}


//double TrajectoryAnalyzer::DistanceToEnd(const double x, const double y)const
//{
//    return static_cast<double>((_trajectory_points_list->getEndNode()->data.point - Vector2d(static_cast<float>(x),static_cast<float>(y))).Length());
//}
