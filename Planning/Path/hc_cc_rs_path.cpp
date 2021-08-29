#include "hc_cc_rs_path.h"

HC_CC_RS_Path::HC_CC_RS_Path(const Configuration &start, const Configuration &end,
                             hc_cc_rs::path_type type, double kappa, double sigma, double length,
                             Configuration *qi1, Configuration *qi2, Configuration *qi3, Configuration *qi4,
                             HC_CC_Circle *cstart, HC_CC_Circle *cend, HC_CC_Circle *ci1, HC_CC_Circle *ci2)
    : Path(start, end, kappa, sigma, length)
{
    type_ = type;

    qi1_ = qi1;
    qi2_ = qi2;
    qi3_ = qi3;
    qi4_ = qi4;

    c_start_ = cstart;
    c_end_   = cend;
    ci1_     = ci1;
    ci2_     = ci2;
}

HC_CC_RS_Path::~HC_CC_RS_Path()
{
    delete qi1_;
    delete qi2_;
    delete qi3_;
    delete qi4_;

    delete c_start_;
    delete c_end_;
    delete ci1_;
    delete ci2_;
}

hc_cc_rs::path_type HC_CC_RS_Path::getType(){ return type_; }

Configuration *HC_CC_RS_Path::getQi1(){ return qi1_; }
Configuration *HC_CC_RS_Path::getQi2(){ return qi2_; }
Configuration *HC_CC_RS_Path::getQi3(){ return qi3_; }
Configuration *HC_CC_RS_Path::getQi4(){ return qi4_; }

HC_CC_Circle *HC_CC_RS_Path::getCircleStart(){ return c_start_; }
HC_CC_Circle *HC_CC_RS_Path::getCircleEnd(){ return c_end_; }
HC_CC_Circle *HC_CC_RS_Path::getCi1(){ return ci1_; }
HC_CC_Circle *HC_CC_RS_Path::getCi2(){ return ci2_; }
