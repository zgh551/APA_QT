/*****************************************************************************/
/* FILE NAME: interpolation.h                   COPYRIGHT (c) Henry Zhu 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: this class using to interpolation data  			         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      July 26 2019        Initial Version                  */
/*****************************************************************************/
#ifndef MATH_INTERPOLATION_H_
#define MATH_INTERPOLATION_H_

#include <QMainWindow>
#include "math.h"
#include "./Common/Utils/Inc/property.h"
#include "./Common/Configure/Configs/vehilce_config.h"

#define kDoubleEpsilon (1.0e-6)

class Interpolation {
public:
	Interpolation();
	virtual ~Interpolation();

	void ArrayIndexFind(float *array,uint16_t num,float input,uint16_t *before,uint16_t *after);

	float InterpolateValue(float x,float x0,float y0,float x1,float y1);

	float InterpolationYZ(float y,float *y_tb,uint16_t y_num,float *z_tb);

	float Interpolation2D(float x,float y,float *x_tb,uint16_t x_num,float *y_tb,uint16_t y_num,float *z_tb);

private:

};

#endif /* MATH_INTERPOLATION_H_ */
