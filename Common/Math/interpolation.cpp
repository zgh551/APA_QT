/*****************************************************************************/
/* FILE NAME: interpolation.cpp                 COPYRIGHT (c) Henry Zhu 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: this class using to interpolation data  			         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      July 26 2019        Initial Version                  */
/*****************************************************************************/
#include "./Common/Math/interpolation.h"

Interpolation::Interpolation() {
	// TODO Auto-generated constructor stub

}

Interpolation::~Interpolation() {
	// TODO Auto-generated destructor stub
}

/*
 * 查找输入值所对应的ID
 * */
void Interpolation::ArrayIndexFind(float *array,uint16_t num,float input,uint16_t *before,uint16_t *after)
{
	uint16_t min_index,max_index;
	uint16_t middle_index;
	min_index = 0;
	max_index = num - 1;

	while( (max_index - min_index) > 1)
	{
		middle_index = (min_index + max_index) / 2 ;
		if(input < array[middle_index])
		{
			max_index = middle_index;
		}
		else
		{
			min_index = middle_index;
		}
	}
	*before = min_index;
	*after  = max_index;
}

float Interpolation::InterpolateValue(float x,float x0,float y0,float x1,float y1)
{
	if(fabs(x - x0) < kDoubleEpsilon)
	{
		return y0;
	}
	else if(fabs(x - x1) < kDoubleEpsilon)
	{
		return y1;
	}
	else
	{
		return y0 + (y1 - y0)*(x - x0)/ (x1 - x0);
	}
}

float Interpolation::InterpolationYZ(float y,float *y_tb,uint16_t y_num,float *z_tb)
{
	float max_y = y_tb[y_num - 1];
	float min_y = y_tb[0];
	uint16_t y_before,y_after;

	if(y >= (max_y - kDoubleEpsilon))
	{
		return z_tb[y_num - 1];
	}
	if(y <= (min_y + kDoubleEpsilon))
	{
		return z_tb[0];
	}

	ArrayIndexFind(y_tb,y_num,y,&y_before,&y_after);
	return InterpolateValue(y,y_tb[y_before],z_tb[y_before],y_tb[y_after],z_tb[y_after]);
}

float Interpolation::Interpolation2D(float x,float y,float *x_tb,uint16_t x_num,float *y_tb,uint16_t y_num,float *z_tb)
{
	float max_x = x_tb[x_num - 1];
	float min_x = x_tb[0];

	uint16_t x_before,x_after;
	uint16_t z_before_value,z_after_value;
	if(x > (max_x - kDoubleEpsilon))
	{
		return InterpolationYZ(y,y_tb,y_num, (z_tb + (x_num - 1)*y_num));
	}
	if(x < (min_x + kDoubleEpsilon))
	{
		return InterpolationYZ(y,y_tb,y_num, z_tb);
	}

	ArrayIndexFind(x_tb,x_num,x,&x_before,&x_after);

	z_before_value = InterpolationYZ(y,y_tb,y_num, (z_tb + x_before * y_num));
	z_after_value  = InterpolationYZ(y,y_tb,y_num, (z_tb + x_after  * y_num));
	return InterpolateValue(x,x_tb[x_before],z_before_value,x_tb[x_after],z_after_value);
}

