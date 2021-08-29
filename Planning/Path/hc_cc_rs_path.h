#ifndef HC_CC_RS_PATH_H
#define HC_CC_RS_PATH_H

#include "Planning/Path/path.h"
#include "Planning/Path/hc_cc_circle.h"

namespace hc_cc_rs {

/**
 * @brief hc-/cc-reeds-shepp path types
 * E :Empty
 * S :Straight
 * T :Turn
 * c :Cusp
 */
enum path_type
{
    E, // empty
    S, // Straight Line
    T, // one turn
    TT, // two circle not cusp
    TcT, // two circle with one cusp
    // Reeds-Shepp families:
    TcTcT,
    TcTT,
    TTcT,
    TST,
    TSTcT,
    TcTST,
    TcTSTcT,
    TTcTT,
    TcTTcT,
    // Additional Families:
    TTT,
    TcST,
    TScT,
    TcScT
};

const uint16_t nb_hc_cc_rs_paths = 18;
}

class HC_CC_RS_Path : public Path
{
public:
    HC_CC_RS_Path(const Configuration &start, const Configuration &end,
                  hc_cc_rs::path_type type, double kappa, double sigma, double length,
                  Configuration *qi1, Configuration *qi2, Configuration *qi3, Configuration *qi4,
                  HC_CC_Circle *cstart, HC_CC_Circle *cend, HC_CC_Circle *ci1, HC_CC_Circle *ci2);

    virtual ~HC_CC_RS_Path();

    /**
     * @brief getType
     * @return
     */
    hc_cc_rs::path_type getType();

    Configuration *getQi1();
    Configuration *getQi2();
    Configuration *getQi3();
    Configuration *getQi4();

    HC_CC_Circle *getCircleStart();
    HC_CC_Circle *getCircleEnd();
    HC_CC_Circle *getCi1();
    HC_CC_Circle *getCi2();

protected:

    /**
     * @brief Path type
     */
    hc_cc_rs::path_type type_;

    /**
     * @brief Intermediate configurations
     */
    Configuration *qi1_, *qi2_, *qi3_, *qi4_;

    /**
     * @brief Start, end and intermediate circles
     */
    HC_CC_Circle *c_start_, *c_end_, *ci1_, *ci2_;
};

#endif // HC_CC_RS_PATH_H
