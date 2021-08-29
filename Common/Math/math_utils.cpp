#include "math_utils.h"

namespace math{

/**
 * @brief Normallize Angle to [-PI,PI)
 * @param angle the original value of the angle
 * @return The normalized value of the angle
 */
double NormallizeAngle(const double angle)
{
    double a = fmod(angle + M_PI,2.0 * M_PI);
    if(a < 0.0)
    {
        a += 2.0 * M_PI;
    }
    return  a - M_PI;
}

/**
 * @brief Normallize Angle to [0 , 2PI)
 * @param angle: the original value of the angle
 * @return The normalized value of the angle
 */
double TwoPiNormallizeAngle(const double angle)
{
    double a = fmod(angle,2.0 * M_PI);
    if(a < 0.0)
    {
        a += 2.0 * M_PI;
    }
    return  a;
}

/**
 * @brief 获取变量的符号
 * @param x: 求取符号的变量
 * @return 符号值: (-1.0, 1.0)
 */
double sgn(double x)
{
    return ((x < 0.0) ? -1.0 : 1.0);
}

/**
 * @brief Computation of the distance between two points
 * @param x1,y1 : one point
 * @param x2,y2 : another point
 * @return the distance of two point
 */
double PointDistance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

/**
 * @brief Transformation of (local_x, local_y) from local coordinate system to global one
 * @param (x, y, psi): the local body point in global coordinate system
 * @param (local_x,local_y): the local coordinate system
 * @param (global_x,global_y): the global coordinate system
 */
void change_to_global_frame(double x, double y, double psi,
                            double   local_x, double   local_y,
                            double *global_x, double *global_y)
{
    double sin_psi = sin(psi);
    double cos_psi = cos(psi);

    *global_x = x + local_x * cos_psi - local_y * sin_psi ;
    *global_y = y + local_x * sin_psi + local_y * cos_psi ;
}

/**
 * @brief Transformation of (global_x, global_y) from global coordinate system to local one
 * @param (x, y, psi): the local body point in global coordinate system
 * @param (global_x,global_y): the global coordinate system
 * @param (local_x,local_y): the local coordinate system
 */
void change_to_local_frame(double x, double y, double psi,
                           double global_x, double global_y,
                           double *local_x, double *local_y)
{
    double sin_psi = sin(psi);
    double cos_psi = cos(psi);

    *local_x = (global_x - x) * cos_psi - (global_y - y) * sin_psi ;
    *local_y = (global_x - x) * sin_psi + (global_y - y) * cos_psi ;
}

/**
 * @brief Initialize an array with a given value
 * @param array :init array
 * @param size :size of the array
 * @param value :the init value
 */
void DoubleArrayInit(double array[],uint16_t size, double value)
{
    for(uint16_t i = 0; i < size; i++)
    {
        array[i] = value;
    }
}

/**
 * @brief Initialize an array with nullptr
 * @param array :the init array
 * @param size :the size fo array
 */
void PointerArrayInit(void *array[], uint16_t size)
{
    for(uint16_t i = 0; i < size; i++)
    {
        array[i] = nullptr;
    }
}

/**
 * @brief Find index with minimal value in double array
 * @param array :the finding array
 * @param size :the size of array
 * @return
 */
uint16_t ArrayIndexMin(double array[], uint16_t size)
{
    double min_value = array[0];
    uint16_t index_min = 0;
    for (uint16_t i = 1; i < size; i++)
    {
        if( array[i] < min_value )
        {
            index_min = i;
            min_value = array[i];
        }
    }
    return index_min;
}

/**
 * @brief get the epsilon value
 * @return  the epsilon value
 */
double getEpsilon(void){return _epsilon; }

}


