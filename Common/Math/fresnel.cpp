#include "fresnel.h"

namespace  math{

void fresnel_0_8(double x, double &S_f, double &C_f)
{
  // T2n(x/8) = Tn(2*(x/8)*(x/8)-1.0)
  // T2n_p1(x/8) = 2*(x/8)*T2n(x/8)-T2n_m1(x/8)
  double quarter_x = 0.25 * x;
  double arg = 0.03125 * x * x - 1.0;
  double T0 = 1;
  double T1 = 0.125 * x;
  double T2 = arg;
  double T3 = quarter_x * T2 - T1;
  double A = chebev_a[0] * T0 + chebev_a[1] * T2;
  double B = chebev_b[0] * T1 + chebev_b[1] * T3;
  double T2n_m4 = T0;
  double T2n_m2 = T2;
  double T2n_m1 = T3;
  for (int n = 2; n < 17; n++)
  {
    double T2n = 2.0 * arg * T2n_m2 - T2n_m4;
    double T2n_p1 = quarter_x * T2n - T2n_m1;
    A += chebev_a[n] * T2n;
    B += chebev_b[n] * T2n_p1;
    T2n_m4 = T2n_m2;
    T2n_m2 = T2n;
    T2n_m1 = T2n_p1;
  }
  double T34 = 2.0 * arg * T2n_m2 - T2n_m4;
  A += chebev_a[17] * T34;

  double sqrt_x = sqrt(x);
  C_f = MV_2PI_SQRT_INV * sqrt_x * A;
  S_f = MV_2PI_SQRT_INV * sqrt_x * B;
  return;
}

void fresnel_8_inf(double x, double &S_f, double &C_f)
{
  // T2n(8/x) = Tn(2*(8/x)*(8/x)-1.0)
  double arg = 128.0 / (x * x) - 1.0;
  double T0 = 1;
  double T2 = arg;
  double E = chebev_e[0] * T0 + chebev_e[1] * T2;
  double F = chebev_f[0] * T0 + chebev_f[1] * T2;
  double T2n_m4 = T0;
  double T2n_m2 = T2;
  for (int n = 2; n < 35; n++)
  {
    double T2n = 2.0 * arg * T2n_m2 - T2n_m4;
    E += chebev_e[n] * T2n;
    F += chebev_f[n] * T2n;
    T2n_m4 = T2n_m2;
    T2n_m2 = T2n;
  }
  for (int n = 35; n < 41; n++)
  {
    double T2n = 2.0 * arg * T2n_m2 - T2n_m4;
    E += chebev_e[n] * T2n;
    T2n_m4 = T2n_m2;
    T2n_m2 = T2n;
  }

  double sin_x = sin(x);
  double cos_x = cos(x);
  double sqrt_x = sqrt(x);
  C_f = 0.5 - MV_2PI_SQRT_INV * (E * cos_x / (2 * x) - F * sin_x) / sqrt_x;
  S_f = 0.5 - MV_2PI_SQRT_INV * (E * sin_x / (2 * x) + F * cos_x) / sqrt_x;
  return;
}

void Fresnel(double s,double &S_f,double &C_f)
{
    double x = MV_PI2 * s * s;
    if (x <= 8.0)
    {
        fresnel_0_8(x, S_f, C_f);
    }
    else
    {
        fresnel_8_inf(x, S_f, C_f);
    }

    if (s < 0)
    {
      S_f = -S_f;
      C_f = -C_f;
    }
    return;
}

}

