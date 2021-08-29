#ifndef HUOGH_H
#define HUOGH_H

#include "math.h"
#include "Common/Utils/Inc/link_list.h"

class SortLinkList : public LinkList<uint32_t>{};

typedef struct _LinePolar
{
    float rho;
    float angle;
}LinePolar;

class Huogh
{
public:
    Huogh();

    void HoughLinesStandard(LinkList<ObstacleLocationPacket> *list,LinkList<ObstacleLocationPacket> *lines_array,
                            float rho,float theta,
                            uint16_t threshold,uint16_t lines_max,
                            float min_theta,float max_theta);

private:
    void createTrigTable( uint32_t numangle, float min_theta, float theta_step,float irho, float *tabSin, float *tabCos );
    void findLocalMaximums( uint32_t numrho, uint32_t numangle, uint16_t threshold,const uint16_t *accum,SortLinkList *sort_buf);
    void sortList(SortLinkList *sort_buf,uint16_t *accum,uint32_t *sort_array);


};

#endif // HUOGH_H
