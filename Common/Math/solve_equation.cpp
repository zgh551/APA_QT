/*
 * solve_equation.cpp
 *
 *  Created on: 2019年3月18日
 *      Author: Henry Zhu
 */

#include "./Common/Math/solve_equation.h"

SolveEquation::SolveEquation() {
	// TODO Auto-generated constructor stub

}

SolveEquation::~SolveEquation() {
	// TODO Auto-generated destructor stub
}

// A*cos + B*sin = C
int8_t SolveEquation::TrigonometricEquation(float a,float b,float c,float *theta1,float *theta2)
{
	float temp_cmp,temp,t1,t2;
	temp_cmp = b*b + (a - c)*(a + c);
	if(temp_cmp < 0)
	{
		return FAIL;
	}
	else
	{
		temp = sqrtf(temp_cmp);
		t1 = (b + temp)/(a + c);
		t2 = (b - temp)/(a + c);
		*theta1 = 2 * atanf(t1);
		*theta2 = 2 * atanf(t2);
		return SUCCESS;
	}
}

// a*x^2 + b*x + c = 0
int8_t SolveEquation::QuadraticEquation(float a,float b,float c,float *x1,float *x2)
{
	float temp_cmp,temp;
	temp_cmp = b*b - 4*a*c;
	if(temp_cmp < 0)
	{
		return FAIL;
	}
	else
	{
		temp = sqrtf(temp_cmp);

		*x1 = 0.5 * (-b + temp) / a;
		*x2 = 0.5 * (-b - temp) / a;
		return SUCCESS;
	}
}
