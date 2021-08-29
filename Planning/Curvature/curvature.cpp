#include "curvature.h"

Curvature::Curvature()
{
    Init();
}

void Curvature::Init()
{

}

float Curvature::TargetLine(float x)
{
    return cosf(COEFFICIENT_TLS*x)-1.0;
}

float Curvature::TargetLineFirstDerivative(float x)
{
    return -COEFFICIENT_TLS*sinf(COEFFICIENT_TLS*x);
}

float Curvature::TargetLineSecondDerivative(float x)
{
    return -COEFFICIENT_TLS*COEFFICIENT_TLS*cosf(COEFFICIENT_TLS*x);
}
/**
 * @brief LatControl::TrackingCurve 生成目标曲线
 * @param x：输入x轴数据
 * @return 返回目标跟踪路径数据集
 */
TargetTrack Curvature::TrackingCurve(float x)
{
    TargetTrack _temp_track;
    float _temp_cos_psi_r;

    _temp_track.point.setX(x);
    _temp_track.point.setY(TargetLine(x));
    _temp_track.yaw = atanf(TargetLineFirstDerivative(x)); // (-90,90)°
    _temp_cos_psi_r = cosf(_temp_track.yaw);
    _temp_track.curvature = _temp_cos_psi_r*_temp_cos_psi_r*_temp_cos_psi_r*TargetLineSecondDerivative(x);
    return _temp_track;
}

/**
 * @brief LatControl::GenerateCurvatureSets 产生曲线数据集
 * @param list：存储数据的链表
 */
void Curvature::GenerateCurvatureSets(TrackLinkList *list,uint16_t type)
{
    TargetTrack temp_track;
    float index_x,rho,gain;
    Node<TargetTrack>* last_track_node;
    Node<TargetTrack>* current_track_node;
    Node<TargetTrack>* next_track_node;
    float first_derivative_x,first_derivative_y;
    float second_derivative_x,second_derivative_y;
    /// 变量初始化
    list->Init();
    temp_track.point.setX(0.0f);
    temp_track.point.setY(0.0f);
    temp_track.yaw =0.0f;
    temp_track.curvature = 0.0f;
    index_x = 0.0f;

    switch(type)
    {
    case 1:
        // 三角函数曲线
        while(index_x < (MV_PI/COEFFICIENT_TLS))
        {
            temp_track.point.setX(index_x);
            temp_track.point.setY(1.0f*TargetLine(index_x));
            list->Add(temp_track);
            index_x = index_x + 0.1f;
        }
        break;

    case 2:
        // 8形曲线
        index_x = -MV_PI;
        while(index_x < MV_PI)
        {
            rho = sqrtf(cosf(2*index_x));
            gain = rho*10;
            temp_track.point.setX(gain*cosf(index_x));
            temp_track.point.setY(gain*sinf(index_x));
            list->Add(temp_track);
            index_x = index_x + 0.01f;
        }
        break;

    case 3:
        // 8字圆
        // 后退
//        index_x = MV_PI2;
//        while(index_x < MV_PI)
//        {
//            gain = 5;
//            temp_track.point.setX(gain*cosf(index_x));
//            temp_track.point.setY(gain*sinf(index_x)-gain);
//            list->Add(temp_track);
//            index_x = index_x + 0.01f;
//        }
//        index_x = 0;
//        while(index_x > -(MV_PI+MV_PI2))
//        {
//            gain = 5;
//            temp_track.point.setX(gain*cosf(index_x)-2*gain);
//            temp_track.point.setY(gain*sinf(index_x)-gain);
//            list->Add(temp_track);
//            index_x = index_x - 0.01f;
//        }

        // 前进
        index_x = MV_PI2;
        while(index_x > -MV_PI)
        {
            gain = 5;
            temp_track.point.setX(gain*cosf(index_x));
            temp_track.point.setY(gain*sinf(index_x)-gain);
            list->Add(temp_track);
            index_x = index_x - 0.01f;
        }
        index_x = 0;
        while(index_x < (MV_PI + MV_PI2))
        {
            gain = 5;
            temp_track.point.setX(gain*cosf(index_x)-2*gain);
            temp_track.point.setY(gain*sinf(index_x)-gain);
            list->Add(temp_track);
            index_x = index_x + 0.01f;
        }
        break;

    default:
        break;
    }

    last_track_node = list->getHeadNode();
    current_track_node = last_track_node->next;
    next_track_node = current_track_node->next;
    do
    {
        first_derivative_x = next_track_node->data.point.getX() - current_track_node->data.point.getX();
        first_derivative_y = next_track_node->data.point.getY() - current_track_node->data.point.getY();
        second_derivative_x = next_track_node->data.point.getX() + last_track_node->data.point.getX() - 2*current_track_node->data.point.getX();
        second_derivative_y = next_track_node->data.point.getY() + last_track_node->data.point.getY() - 2*current_track_node->data.point.getY();
        current_track_node->data.yaw = atan2f(first_derivative_y,first_derivative_x);

        current_track_node->data.curvature = (second_derivative_y*first_derivative_x - second_derivative_x*first_derivative_y)
                                           / powf(powf(first_derivative_x,2)+powf(first_derivative_y,2),1.5);

        current_track_node->data.velocity = 0.5;

        last_track_node = last_track_node->next;
        current_track_node = last_track_node->next;
        next_track_node = current_track_node->next;
    }while(next_track_node->next != NULL);
}

void Curvature::GenerateCurvaturePointSets(std::vector<TargetTrack> *vec,uint16_t type)
{
    TargetTrack temp_track;
    float index_x,gain;
    index_x = 0.0;
    if(vec->size() > 0)
    {
        vec->clear();
    }

    switch(type)
    {
    case 1:// 三角函数曲线
        index_x = -35.0f;
        while(index_x < 0.0f)
        {
            temp_track.point.setX(index_x);
            temp_track.point.setY(15.0);
            vec->push_back(temp_track);
            index_x = index_x + SAMPLE_STEP;
        }
        while(index_x < (MV_PI/COEFFICIENT_TLS))
        {

            temp_track.point.setX(index_x);
            temp_track.point.setY(15.0f + 5.0f*TargetLine(index_x));
            vec->push_back(temp_track);
            index_x = index_x + SAMPLE_STEP;
        }
        while(index_x < 40.0f)
        {
            temp_track.point.setX(index_x);
            temp_track.point.setY(5.0);
            vec->push_back(temp_track);
            index_x = index_x + SAMPLE_STEP;
        }
        break;

    case 3:// 圆
//        index_x = 0.0;
//        gain = 5;
//        while(index_x < MV_PI)
//        {
//            temp_track.point.setX(gain*cosf(index_x));
//            temp_track.point.setY(gain*sinf(index_x)-gain);
//            vec->push_back(temp_track);
//            index_x = index_x + SAMPLE_STEP;
//        }
        gain = 5;
        index_x = MV_PI2;
        while(index_x > -MV_PI)
        {
            temp_track.point.setX(gain*cosf(index_x));
            temp_track.point.setY(gain*sinf(index_x)-gain);
            vec->push_back(temp_track);
            index_x = index_x - SAMPLE_STEP;
        }
        index_x = 0;
        while(index_x < (MV_PI + MV_PI2))
        {
            temp_track.point.setX(gain*cosf(index_x)-2*gain);
            temp_track.point.setY(gain*sinf(index_x)-gain);
            vec->push_back(temp_track);
            index_x = index_x + SAMPLE_STEP;
        }
        break;

    default:
        break;
    }
    CurvatureCalculate(vec);
}

void Curvature::CurvatureCalculate(std::vector<TargetTrack> *vec)
{
    float first_derivative_x,first_derivative_y;
    float second_derivative_x,second_derivative_y;
    for(std::vector<TargetTrack>::iterator it = vec->begin();it != vec->end();it++)
    {
        if(it == vec->begin())
        {
            first_derivative_x = (it + 1)->point.getX() - it->point.getX();
            first_derivative_y = (it + 1)->point.getY() - it->point.getY();
            second_derivative_x = (it + 2)->point.getX() + it->point.getX() - 2.0f * (it + 1)->point.getX();
            second_derivative_y = (it + 2)->point.getY() + it->point.getY() - 2.0f * (it + 1)->point.getY();
        }
        else if(it == vec->end())
        {
            first_derivative_x = it->point.getX() - (it-1)->point.getX();
            first_derivative_y = it->point.getY() - (it-1)->point.getY();
            second_derivative_x = (it - 2)->point.getX() + it->point.getX() - 2.0f * (it - 1)->point.getX();
            second_derivative_y = (it - 2)->point.getY() + it->point.getY() - 2.0f * (it - 1)->point.getY();
        }
        else
        {
            first_derivative_x = (it + 1)->point.getX() - it->point.getX();
            first_derivative_y = (it + 1)->point.getY() - it->point.getY();
            second_derivative_x = (it + 1)->point.getX() + (it - 1)->point.getX() - 2.0f * it->point.getX();
            second_derivative_y = (it + 1)->point.getY() + (it - 1)->point.getY() - 2.0f * it->point.getY();
        }
        it->yaw = atan2f(first_derivative_y,first_derivative_x);

        it->curvature = (second_derivative_y*first_derivative_x - second_derivative_x*first_derivative_y)
                                           / powf(powf(first_derivative_x,2)+powf(first_derivative_y,2),1.5);
    }
}
