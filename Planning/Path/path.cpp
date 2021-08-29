#include "path.h"

Path::Path(const Configuration &start, const Configuration &end, double kappa, double sigma, double length)
{
    start_ = start;
    end_   = end;
    kappa_ = kappa;
    sigma_ = sigma;
    lenght_ = length;
}


