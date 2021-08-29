#ifndef TRAJECTORYANALYZER_H
#define TRAJECTORYANALYZER_H

#include <QMainWindow>
#include <vector>
#include <stdlib.h>
#include <algorithm>

#include "math.h"
#include "Common/Math/vector_2d.h"
#include "Common/Utils/Inc/link_list.h"
#include "Common/VehicleState/GeometricTrack/geometric_track.h"
#include "Common/Configure/Configs/vehilce_config.h"

typedef struct _TargetTrack
{
    Vector2d point;
    double    yaw;
    double    curvature;
    double    velocity;
}TargetTrack;

/**
 * @brief The TrackLinkList class：曲线跟踪目标曲线链表
 */
class TrackLinkList : public LinkList<TargetTrack>{};

class TrajectoryAnalyzer
{
public:
    TrajectoryAnalyzer();

    void Init(TrackLinkList *list);

    /**
     * @brief Init 初始化容器
     * @param vec 待更新的容器
     */
    void Init(std::vector<TargetTrack> *vec);

    /**
     * @brief CalculateNearestPointByPosition 基于位置信息计算目标曲线上最近的点
     * @param x 当前位置x轴坐标
     * @param y 当前位置y轴坐标
     * @return 返回最近目标点信息
     */
    TargetTrack CalculateNearestPointByPosition(const double x, const double y);

    /**
     * @brief DistanceToEnd 计算离最后目标曲线点的距离
     * @param x 当前位置x轴坐标
     * @param y 当前位置y轴坐标
     * @return 离最后点的距离值
     */
    double DistanceToEnd(const double x, const double y)const;

    void TrajectoryTransformToCOM(double rear_to_com_distance);

    Vector2d ComputeCOMPosition(double rear_to_com_distance,TargetTrack path_point);
private:
    Node<TargetTrack>* _track_node;
    TrackLinkList    * _track_list;
    TrackLinkList    * _trajectory_points_list;

    std::vector<TargetTrack> *_trajectory_points_vector;
    std::vector<TargetTrack>::iterator _current_it;
    uint16_t _target_point_index;
};

#endif // TRAJECTORYANALYZER_H
