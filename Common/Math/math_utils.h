#ifndef MATHUTILS_H
#define MATHUTILS_H

#include <QMainWindow>
#include "math.h"
#include "Common/Configure/Configs/system_config.h"

namespace math{
    const double _epsilon = 1.0e-15;
    /**
     * @brief Normallize Angle to [-PI,PI)
     * @param angle:the original value of the angle
     * @return The normalized value of the angle
     */
    double NormallizeAngle(const double angle);

    /**
     * @brief Normallize Angle to [0 , 2PI)
     * @param angle: the original value of the angle
     * @return The normalized value of the angle
     */
    double TwoPiNormallizeAngle(const double angle);

    /**
     * @brief 获取变量的符号
     * @param x: 求取符号的变量
     * @return 符号值: (-1.0, 1.0)
     */
    double sgn(double x);

    /**
     * @brief Computation of the distance between two points
     * @param x1,y1 : one point
     * @param x2,y2 : another point
     * @return the distance of two point
     */
    double PointDistance(double x1, double y1, double x2, double y2);

    /**
     * @brief Transformation of (local_x, local_y) from local coordinate system to global one
     * @param (x, y, psi): the local body point in global coordinate system
     * @param (local_x,local_y): the local coordinate system
     * @param (global_x,global_y): the global coordinate system
     */
    void change_to_global_frame(double x, double y, double psi,
                                double   local_x, double   local_y,
                                double *global_x, double *global_y);

    /**
     * @brief Transformation of (global_x, global_y) from global coordinate system to local one
     * @param (x, y, psi): the local body point in global coordinate system
     * @param (global_x,global_y): the global coordinate system
     * @param (local_x,local_y): the local coordinate system
     */
    void change_to_local_frame(double x, double y, double psi,
                               double global_x, double global_y,
                               double *local_x, double *local_y);

    /**
     * @brief Initialize an array with a given value
     * @param array :init array
     * @param size :size of the array
     * @param value :the init value
     */
    void DoubleArrayInit(double array[],uint16_t size, double value);

    /**
     * @brief Initialize an array with nullptr
     * @param array :the init array
     * @param size :the size fo array
     */
    void PointerArrayInit(void *array[], uint16_t size);

    /**
     * @brief Find index with minimal value in double array
     * @param array :the finding array
     * @param size :the size of array
     * @return
     */
    uint16_t ArrayIndexMin(double array[], uint16_t size);

    /**
     * @brief get the epsilon value
     * @return  the epsilon value
     */
    double getEpsilon(void);

    /**
     * @brief Clamp a value between two bounds.
     *        If the value goes beyond the bounds, return one of the bounds,
     *        otherwise, return the original value.
     * @param value The original value to be clamped.
     * @param bound1 One bound to clamp the value.
     * @param bound2 The other bound to clamp the value.
     * @return The clamped value.
     */
    template <typename T>
    T Clamp(const T value, T bound1, T bound2) {
      if (bound1 > bound2) {
        std::swap(bound1, bound2);
      }

      if (value < bound1) {
        return bound1;
      } else if (value > bound2) {
        return bound2;
      }
      return value;
    }
}

#endif // MATHUTILS_H
