#include "huogh.h"

Huogh::Huogh()
{

}

void Huogh::createTrigTable( uint32_t numangle, float min_theta,float theta_step,
                             float irho,float *tabSin, float *tabCos )
{
    float ang = min_theta;
    for(uint32_t n = 0; n < numangle; ang += theta_step, n++ )
    {
        tabSin[n] = sinf(ang) * irho;
        tabCos[n] = cosf(ang) * irho;
    }
}

void Huogh::findLocalMaximums( uint32_t numrho, uint32_t numangle, uint16_t threshold,const uint16_t *accum,SortLinkList *sort_buf)
{
    for(uint32_t r=0;r < numrho;r++)
    {
        for(uint32_t n =0;n < numangle;n++)
        {
            uint32_t base = (n + 1)*(numrho + 2) + r + 1;
            if( (accum[base] >  threshold) &&
                (accum[base] >  accum[base - 1]) &&
                (accum[base] >= accum[base + 1]) &&
                (accum[base] >  accum[base - numrho - 2]) &&
                (accum[base] >= accum[base + numrho + 2]) )
            {
                sort_buf->Add(base);
            }
        }
    }
}

void Huogh::sortList(SortLinkList *sort_buf,uint16_t *accum,uint32_t *sort_array)
{
    uint32_t list_len = sort_buf->Length();
//    sort_array = new uint32_t[list_len];

    Node<uint32_t> *head_node = sort_buf->getHeadNode();

    for(uint32_t i = 0;i < list_len;i++)
    {
        sort_array[i] = head_node->data;
        for(uint32_t j = i;j > 0;j--)
        {
            if(accum[sort_array[j]] > accum[sort_array[j - 1]])
            {
                uint32_t temp = sort_array[j];
                sort_array[j] = sort_array[j - 1];
                sort_array[j - 1] = temp;
            }
            else
            {
                break;
            }
        }
        head_node = head_node->next;
    }
}


void Huogh::HoughLinesStandard(LinkList<ObstacleLocationPacket> *list,LinkList<ObstacleLocationPacket> *lines_array,
                               float rho,float theta,
                               uint16_t threshold,uint16_t lines_max,
                               float min_theta,float max_theta)
{
    uint32_t numangle,numrho;

    float irho = 1/ rho;

    float max_rho = fmax(list->getHeadNode()->data.Position.Length(),list->getEndNode()->data.Position.Length());
    float min_rho = -max_rho;

    if((max_theta > min_theta) && (max_rho > min_rho))
    {
        numangle = static_cast<uint32_t>((max_theta - min_theta) / theta);
        numrho = static_cast<uint32_t>((max_rho - min_rho) / rho);
    }
    else
    {
        return;
    }

//    float tabSin[numangle];
//    float tabCos[numangle];
    float *tabSin = new float[numangle];
    float *tabCos = new float[numangle];
    for(uint32_t i=0;i < numangle;i++)
    {
        tabSin[i] = 0.0f;
        tabCos[i] = 0.0f;
    }
    // create the sin and cos table
    this->createTrigTable(numangle,min_theta,theta,irho,tabSin,tabCos);
    uint32_t array_demension = (numangle + 2)*(numrho + 2);
//    uint16_t accum[array_demension];
    uint16_t *accum = new uint16_t[array_demension];
    for(uint32_t i=0;i < array_demension;i++)
    {
        accum[i] = 0;
    }
    Node<ObstacleLocationPacket> *head_list;
    head_list = list->getHeadNode();
    while(head_list != NULL)
    {
        for(uint32_t n=0;n < numangle;n++)
        {
            int32_t r = static_cast<int32_t>(head_list->data.Position.getX() * tabCos[n] +
                                             head_list->data.Position.getY() * tabSin[n] );
            r += (numrho - 1) / 2;
            accum[(n + 1) * (numrho + 2) + r + 1]++;
        }
        head_list = head_list->next;
    }

    SortLinkList *sort_buf = new SortLinkList();
    this->findLocalMaximums(numrho,numangle,threshold,accum,sort_buf);
    uint32_t sort_array[sort_buf->Length()];
    if(sort_buf->Length() > 0)
    {
        this->sortList(sort_buf,accum,sort_array);
    }
    else
    {
        return;
    }
    if(0 != sort_buf->Length()){sort_buf->Delete();}

    // 获取特征提取后的直线点
//    uint16_t LinesMax = lines_max < sort_buf->Length() ? lines_max : sort_buf->Length();
    head_list = list->getHeadNode();
    while(NULL != head_list)
    {
        for(uint32_t n=0;n < numangle;n++)
        {
            int32_t r = static_cast<int32_t>(head_list->data.Position.getX() * tabCos[n] +
                                             head_list->data.Position.getY() * tabSin[n] );
            r += (numrho - 1) / 2;
            uint32_t point_index = (n + 1) * (numrho + 2) + r + 1;
            if(point_index == sort_array[0])
            {
                lines_array->Add(head_list->data);
                break;
            }
        }
        head_list = head_list->next;
    }
    delete[] accum;

    delete[] tabSin;
    delete[] tabCos;
}
