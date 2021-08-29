/*****************************************************************************/
/* FILE NAME: curve_fitting.cpp                 COPYRIGHT (c) Henry Zhu 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: this class using to fit the curve line  			         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu     May 20 2019         Initial Version                  */
/*****************************************************************************/
#include "./Common/Math/curve_fitting.h"

CurveFitting::CurveFitting() {
	// TODO Auto-generated constructor stub

}

CurveFitting::~CurveFitting() {
	// TODO Auto-generated destructor stub
}

float CurveFitting::LineFitting(LinkList<ObstacleLocationPacket> *l,float *a,float *b)
{
    float sum_x,sum_y,sum_xy,sum_x2,sum_n;
    float sum_variance;
    float denominator,molecule_a,molecule_b;
    float K,offset;

    Node<ObstacleLocationPacket> *current_node;
    current_node = l->getHeadNode();

    sum_x = 0;
    sum_y = 0;
    sum_xy = 0;
    sum_x2 = 0;
    sum_n  = 0;
    while(current_node->next != NULL)
    {
        sum_x += current_node->data.Position.getX();
        sum_y += current_node->data.Position.getY();
        sum_xy += current_node->data.Position.getX()*current_node->data.Position.getY();
        sum_x2 += current_node->data.Position.getX()*current_node->data.Position.getX();
        sum_n++;
        current_node = current_node->next;
    }
    denominator = sum_n * sum_x2 - sum_x * sum_x;
    if(denominator == 0)
    {
        *a = PI_2;
    }
    else
    {
        molecule_a  = sum_n * sum_xy - sum_x * sum_y;
        molecule_b  = sum_x2 * sum_y - sum_x * sum_xy;
        K = molecule_a/denominator;
        offset = molecule_b/denominator;
        *a = atanf(K);
        *b = offset;
    }

    current_node = l->getHeadNode();
    sum_variance = 0.0f;
    while(current_node->next != NULL)
    {
        if(fabsf(*a - PI_2) < 1.0e-6)
        {
            sum_variance += powf(current_node->data.Position.getX() - sum_x/sum_n,2);
        }
        else
        {
            sum_variance += powf(K * current_node->data.Position.getX() + offset - current_node->data.Position.getY(),2);
        }
        current_node = current_node->next;
    }
    return sum_variance;
}
