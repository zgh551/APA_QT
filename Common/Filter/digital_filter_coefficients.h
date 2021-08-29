#ifndef DIGITALFILTERCOEFFICIENTS_H
#define DIGITALFILTERCOEFFICIENTS_H

#include <QMainWindow>
#include "Common/Configure/Configs/system_config.h"
#include "math.h"

namespace common {

/**
 * @brief Get low-pass coefficients for digital filter.
 * @param ts Time interval between signals.
 * @param cutoff_freq Cutoff of frequency to filter high-frequency signals out.
 * @param denominators Denominator coefficients for digital filter.
 * @param numerators Numerator coefficients for digital filter.
 */
void LpfCoefficients(const double ts, const double cutoff_freq,
                     std::vector<double> *denominators,
                     std::vector<double> *numerators);

}

#endif // DIGITALFILTERCOEFFICIENTS_H
