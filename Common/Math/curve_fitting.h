/*****************************************************************************/
/* FILE NAME: curve_fitting.h                   COPYRIGHT (c) Henry Zhu 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: this class using to fit the curve line  			         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      May 20 2019         Initial Version                  */
/*****************************************************************************/
#ifndef MATH_CURVE_FITTING_H_
#define MATH_CURVE_FITTING_H_

#include "./Common/Utils/Inc/link_list.h"


class CurveFitting {
public:
	CurveFitting();
	virtual ~CurveFitting();

        float LineFitting(LinkList<ObstacleLocationPacket> *l,float *a,float *b);
};

#endif /* MATH_CURVE_FITTING_H_ */
