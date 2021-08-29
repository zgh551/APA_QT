/*****************************************************************************/
/* FILE NAME: algebraic_geometry.cpp            COPYRIGHT (c) Henry Zhu 2019 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: the algebraic geometry aclculate method  			         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Henry Zhu      January 16 2019      Initial Version                 */
/*****************************************************************************/
#include "./Common/Math/algebraic_geometry.h"

AlgebraicGeometry::AlgebraicGeometry() {


}

AlgebraicGeometry::~AlgebraicGeometry() {

}

//二次方程求解
float AlgebraicGeometry::QuadraticEquation(float a,float b,float c)
{
	return 0.5 * (-b + sqrtf(b*b - 4*a*c)) / a;
}

float AlgebraicGeometry::QuadraticEquationMin(float a,float b,float c)
{
	return 0.5 * (-b - sqrtf(b*b - 4*a*c)) / a;
}

//过一点的直线方程
float AlgebraicGeometry::LinearAlgebra(Line l,float x)
{
	return tanf(l.Angle)*(x - l.Point.getX()) + l.Point.getY();
}

float AlgebraicGeometry::LinearAlgebraCoefficient(float a,float b,float x)
{
	return a * x + b;
}
// 已知圆上两点和半径，算两点间的弧长
float AlgebraicGeometry::ArcLength(Vector2d a,Vector2d b,float r)
{
	return 2 * asinf((a-b).Length()*0.5/r)*r;
}


//已知圆的半径，求与圆和直线相切圆的坐标位置
void AlgebraicGeometry::Tangent_CCL(Line l,Circle cl,Circle *cr)
{
	float a,b,c;
	float sec_v,tan_v;
	float Value_temp;
	sec_v = 1/cosf(l.Angle);
	tan_v = tanf(l.Angle);

	Value_temp = l.Point.getY() - cl.Center.getY() - tan_v*l.Point.getX() - sec_v*cr->Radius;

	a = 1 + tan_v * tan_v;
	b = 2*(tan_v * Value_temp - cl.Center.getX());
	c = powf(Value_temp,2) - powf(cl.Radius + cr->Radius ,2);

    cr->Center.setX(QuadraticEquation(a,b,c));

    cr->Center.setY(LinearAlgebra(l,cr->Center.getX()) - sec_v*cr->Radius);
}

void AlgebraicGeometry::Tangent_CCL_Up(Line l,Circle cl,Circle *cr)
{
	float a,b,c;
	float sec_v,tan_v;
	float Value_temp;
	sec_v = 1/cosf(l.Angle);
	tan_v = tanf(l.Angle);

	Value_temp = l.Point.getY() - cl.Center.getY() - tan_v*l.Point.getX() + sec_v*cr->Radius;

	a = 1 + tan_v * tan_v;
	b = 2*(tan_v * Value_temp - cl.Center.getX());
	c = powf(Value_temp,2) - powf(cl.Radius + cr->Radius ,2);

    cr->Center.setX(QuadraticEquation(a,b,c));

    cr->Center.setY(LinearAlgebra(l,cr->Center.getX()) + sec_v*cr->Radius);
}

void AlgebraicGeometry::Tangent_CCL_VerticalLine(Line l,Circle cl,Circle *cr,Vector2d *lp,Vector2d *rp)
{
	float a,b,c;
	Vector2d temp_v;

    cr->Center.setX(l.Point.getX() + cr->Radius);

	a = 1;
	b = -2 * cl.Center.getY();
	c = cl.Center.getY() * cl.Center.getY() + (cl.Center.getX() - cr->Center.getX()) * (cl.Center.getX() - cr->Center.getX()) - (cl.Radius + cr->Radius) * (cl.Radius + cr->Radius);

    cr->Center.setY( QuadraticEquationMin(a,b,c) );

    lp->setX( l.Point.getX());
    lp->setY( cr->Center.getY());

	temp_v = Vector2d(cr->Radius,0);
	*rp = temp_v.rotate( (cl.Center - cr->Center).Angle() ) + cr->Center;
//	rp->X = (cl.Radius/(cl.Radius + cr->Radius))*(cr->Center.getX() - cl.Center.getX()) + cl.Center.getX();
//	rp->Y = (cr->Radius/(cl.Radius + cr->Radius))*(cl.Center.getY() - cr->Center.getY()) + cr->Center.getY();
}
// 求与直线相切圆的半径
void AlgebraicGeometry::Tangent_CL(Line l,Circle *c,Vector2d *p)
{
	Vector2d temp_difference;
	*p = (c->Center-l.Point).Orthogonal(l.Angle) + l.Point;

	temp_difference = c->Center - l.Point;
	c->Radius = fabsf(sinf(l.Angle)*temp_difference.getX() - cosf(l.Angle)*temp_difference.getY());
}

// 已知两圆心位置，求与两圆相切直线的切点位置
void AlgebraicGeometry::Tangent_CLC(Circle cl,Circle cr,Line *lm,Vector2d *ll,Vector2d *lr)
{
	float K1;
	float d_cl_lm;
	float alpha,beta;
	Vector2d v_ll_lm,v_lr_lm;

	K1 = cr.Radius / (cl.Radius + cr.Radius);
	lm->Point = (cl.Center - cr.Center)*K1 + cr.Center;

	d_cl_lm = (lm->Point - cl.Center).Length();
	alpha = (lm->Point - cl.Center).Angle();
	beta  = acosf(cl.Radius / d_cl_lm);
	lm->Angle = alpha - beta + PI_2;

	v_ll_lm = Vector2d(-cl.Radius * tanf(beta) ,0);
	v_lr_lm = Vector2d( cr.Radius * tanf(beta) ,0);

	*ll = lm->Point + v_ll_lm.rotate(lm->Angle);
	*lr = lm->Point + v_lr_lm.rotate(lm->Angle);
}
void AlgebraicGeometry::Tangent_CLC_Up(Circle cl,Circle cr,Line *lm,Vector2d *ll,Vector2d *lr)
{
	float K1;
	float d_cl_lm;
	float alpha,beta;
	Vector2d v_ll_lm,v_lr_lm;

	K1 = cr.Radius / (cl.Radius + cr.Radius);
	lm->Point = (cl.Center - cr.Center)*K1 + cr.Center;

	d_cl_lm = (lm->Point - cl.Center).Length();
	alpha = (lm->Point - cl.Center).Angle();
	beta  = acosf(cl.Radius / d_cl_lm);
	lm->Angle = alpha + beta - PI_2;

	v_ll_lm = Vector2d(-cl.Radius * tanf(beta) ,0);
	v_lr_lm = Vector2d( cr.Radius * tanf(beta) ,0);

	*ll = lm->Point + v_ll_lm.rotate(lm->Angle);
	*lr = lm->Point + v_lr_lm.rotate(lm->Angle);
}
// 已知两条直线，求与两直线相切的圆的坐标和半径,以及切点的坐标
void AlgebraicGeometry::Tangent_LLC(Line l_1,Line l_2,Circle *c,Vector2d *lp,Vector2d *rp)
{
	float l_a,l_b;
	float sec_v,tan_v;
	Vector2d temp_v;

	if( 0 == l_2.Angle)
	{
        c->Center.setY( l_2.Point.getY() - c->Radius );

        lp->setX( l_1.Point.getX() );
        lp->setY( c->Center.getY() );

        rp->setX( c->Center.getX() );
        rp->setY( l_2.Point.getY() );
	}
	else
	{
		sec_v = 1/cosf(l_2.Angle);
		tan_v = tanf(l_2.Angle);

		l_a = tan_v;
		l_b = l_2.Point.getY() - l_2.Point.getX() * tan_v - sec_v * c->Radius;

        c->Center.setX( l_1.Point.getX() + c->Radius );
        c->Center.setY( LinearAlgebraCoefficient(l_a,l_b,c->Center.getX()) );

		temp_v = Vector2d(c->Radius,0);
		*rp = temp_v.rotate(l_2.Angle + PI_2) + c->Center;

        lp->setX( l_1.Point.getX() );
        lp->setY( c->Center.getY() );
	}
}

// 已知直线和圆的半径，求解与直线相切的圆的圆心，并保证圆上的某点与固定点的距离为余量值
void AlgebraicGeometry::Tanget_LC_Margin(Line l,Vector2d p,float margin,Circle *c,Vector2d *t_p)
{
	float l_a,l_b;
	float c_A,c_B,c_C;
	float sec_v,tan_v;
	Vector2d temp_v;

	sec_v = 1/cosf(l.Angle);
	tan_v = tanf(l.Angle);
	l_a = tan_v;
	l_b = l.Point.getY() - l.Point.getX() * tan_v - sec_v * c->Radius;

	c_A = 1 + l_a * l_a;
	c_B = 2 * l_a * (l_b - p.getY()) - 2 * p.getX();
	c_C = p.getX() * p.getX() + (l_b - p.getY())*(l_b - p.getY()) - (c->Radius - margin - RIGHT_EDGE_TO_CENTER) * (c->Radius - margin - RIGHT_EDGE_TO_CENTER);

    c->Center.setX( QuadraticEquation(c_A,c_B,c_C) );

    c->Center.setY( LinearAlgebraCoefficient(l_a,l_b,c->Center.getX()) );

	temp_v = Vector2d(c->Radius,0);
	*t_p = temp_v.rotate(l.Angle + PI_2) + c->Center;
}

void AlgebraicGeometry::Tanget_CC_Margin(Circle cl,Vector2d p,float margin,Circle *cr,Line *t_l)
{
	float A,B,C;
//	float ea,eb,ec;
	float temp_c;
	float alpha,theta1,theta2,theta_min;
	Vector2d temp_v;

	A = 2 * (cl.Radius + cr->Radius) * (cl.Center.getY() - p.getY());
	B = 2 * (cl.Radius + cr->Radius) * (cl.Center.getX() - p.getX());
	C = (cr->Radius - margin - RIGHT_EDGE_TO_CENTER) * (cr->Radius - margin - RIGHT_EDGE_TO_CENTER) +
		2 * ( p.getX() * cl.Center.getX() + p.getY() * cl.Center.getY()) -
		( cl.Center.getX() * cl.Center.getX() + cl.Center.getY() * cl.Center.getY()) -
		(cl.Radius + cr->Radius)*(cl.Radius + cr->Radius);

//	ea = A*A + B*B;
//	eb = -2 * A *C;
//	ec = C*C - B*B;
//
//	temp_s = QuadraticEquation(ea,eb,ec);
//	theta = asinf(temp_s);

	alpha = atan2f(A,B);
	temp_c = C/sqrtf(A*A+B*B);
	theta1 = -acosf(temp_c) + alpha;
	theta2 =  acosf(temp_c) + alpha;
	theta_min = fabs(theta1) < fabs(theta2) ? theta1 : theta2;


	temp_v = Vector2d(cl.Radius,0);
	t_l->Point = temp_v.rotate(theta_min) + cl.Center;
	t_l->Angle = theta_min + PI_2;
//	float l_a,l_b;
//	float c_A,c_B,c_C;
//	Vector2d temp_v;
//
//	l_a = tanf((p - cl.Center).Angle());
//	l_b = -l_a * p.getX() + p.getY();
//
//	c_A = 1 + l_a * l_a;
//	c_B = 2 * l_a * (l_b - p.getY()) - 2 * p.getX();
//	c_C = p.getX() * p.getX() + (l_b - p.getY())*(l_b - p.getY()) - (cr->Radius - margin - RIGHT_EDGE_TO_CENTER) * (cr->Radius - margin - RIGHT_EDGE_TO_CENTER);
//
//	cr->Center.X = QuadraticEquation(c_A,c_B,c_C);
//	cr->Center.Y = LinearAlgebraCoefficient(l_a,l_b,cr->Center.getX());
//

}


// 两圆相交求交点
void AlgebraicGeometry::Intersection_CC(Circle c1,Circle c2,Vector2d *p1,Vector2d *p2)
{
	float A,B,C;
	float theta1,theta2;
	A = c1.Center.getX() - c2.Center.getX();
	B = c1.Center.getY() - c2.Center.getY();
	C = 0.5*(c2.Radius*c2.Radius - c1.Radius*c1.Radius - A*A - B*B)/c1.Radius;

	_solve_equation.TrigonometricEquation(A, B, C, &theta1, &theta2);

    p1->setX( c1.Center.getX() + c1.Radius * cosf(theta1) );
    p1->setY( c1.Center.getY() + c1.Radius * sinf(theta1) );

    p2->setX( c1.Center.getX() + c1.Radius * cosf(theta2) );
    p2->setY( c1.Center.getY() + c1.Radius * sinf(theta2) );
}
