/*****************************************************************************/
/* FILE NAME: algebraic_geometry.h              COPYRIGHT (c) Henry Zhu 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the algebraic geometry aclculate method  			         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      January 16 2019      Initial Version                 */
/*****************************************************************************/

#ifndef MATH_ALGEBRAIC_GEOMETRY_H_
#define MATH_ALGEBRAIC_GEOMETRY_H_

#include <QMainWindow>
#include "math.h"
#include "./Common/Utils/Inc/property.h"
#include "./Common/Math/vector_2d.h"
#include "./Common/Math/solve_equation.h"
#include "./Common/Configure/Configs/vehilce_config.h"

typedef struct _Line
{
	Vector2d Point;
	float Angle;
}Line;

typedef struct _Circle
{
	Vector2d Center;
	float Radius;
}Circle;

class AlgebraicGeometry {
public:
	AlgebraicGeometry();
	virtual ~AlgebraicGeometry();

	//二次方程求解
	float QuadraticEquation(float a,float b,float c);
	float QuadraticEquationMin(float a,float b,float c);

	//过一点的直线方程
	float LinearAlgebra(Line l,float x);
	//已知系数的直线方程
	float LinearAlgebraCoefficient(float a,float b,float x);
	// 已知圆上两点和半径，算两点间的弧长
	float ArcLength(Vector2d a,Vector2d b,float r);

	/************************************************************************/
	// 已知圆的半径，求与圆和直线相切圆的坐标位置
	void Tangent_CCL(Line l,Circle cl,Circle *cr);

	void Tangent_CCL_Up(Line l,Circle cl,Circle *cr);
	// 已知竖直直线和圆 ，以及相切圆的半径，求与它们相切圆的坐标
	void Tangent_CCL_VerticalLine(Line l,Circle cl,Circle *cr,Vector2d *lp,Vector2d *rp);

	// 已知线段和圆心，求相切圆的半径
	void Tangent_CL(Line l,Circle *c,Vector2d *p);
	// 已知两圆心位置，求与两圆相切直线的切点位置
	void Tangent_CLC(Circle cl,Circle cr,Line *lm,Vector2d *ll,Vector2d *lr);
	void Tangent_CLC_Up(Circle cl,Circle cr,Line *lm,Vector2d *ll,Vector2d *lr);
	// 已知两条直线，求与两直线相切的圆的坐标和半径,以及切点的坐标
	void Tangent_LLC(Line l_1,Line l_2,Circle *c,Vector2d *lp,Vector2d *rp);

	// 已知直线和圆的半径，求解与直线相切的圆的圆心，并保证圆上的某点与固定点的距离为余量值(垂直泊车)
	void Tanget_LC_Margin(Line l,Vector2d p,float margin,Circle *c,Vector2d *t_p);

	// 已知库边沿，左圆，求与左圆相切且留有余量的圆
	void Tanget_CC_Margin(Circle cl,Vector2d p,float margin,Circle *cr,Line *t_l);

	// 两圆相交求交点
	void Intersection_CC(Circle c1,Circle c2,Vector2d *p1,Vector2d *p2);


private:
	SolveEquation _solve_equation;

};

#endif /* MATH_ALGEBRAIC_GEOMETRY_H_ */
