#ifndef PATH_H
#define PATH_H

#include "Planning/Common/configuration.h"
#include "Planning/Common/steering_common.h"

class Path
{
public:
    /**
     * @brief Path Constructor
     * @param start : the start configuration
     * @param end : the end configuration
     * @param kappa : the curvature of the path
     * @param sigma : the sharpness of the path
     * @param length : the lenght of the path
     */
    Path(const Configuration &start, const Configuration &end, double kappa, double sigma, double length);

    Configuration getStart(void) { return  start_; }
    Configuration getEnd(void) { return end_; }
    double getKappa(void) { return kappa_; }
    double getSigma(void) { return sigma_; }
    double getLength(void) { return lenght_; }
protected:

    /**
     * @brief the start configuration
     */
    Configuration start_;
    /**
     * @brief the end configuration
     */
    Configuration end_;
    /**
     * @brief the curvature of the path
     */
    double kappa_;
    /**
     * @brief the sharpness of the path
     */
    double sigma_;
    /**
     * @brief the lenght of the path
     */
    double lenght_;
};



#endif // PATH_H
