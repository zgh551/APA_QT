#ifndef FRESNEL_H
#define FRESNEL_H

#include <QMainWindow>
#include "Common/Configure/Configs/system_config.h"
#include "math.h"

namespace math {
    /**
     * @brief Fresnel Integrals approximated with Chebyshev polynomials
     * @param s: the lenght of arc
     * @param S_f: int_0_s(sin(pi/2 u^2)du)
     * @param C_f: int_0_s(cos(pi/s u^2)du)
     */
    void Fresnel(double s,double &S_f,double &C_f);


    /**
     * @brief Chebyshev coefficients for approximating Fresnel integrals
     */
    const static double chebev_a[18] = {
         0.76435138664186000189, -0.43135547547660179313,  0.43288199979726653054,
        -0.26973310338387111029,  0.08416045320876935378, -0.01546524484461381958,
         0.00187855423439822018, -0.00016264977618887547,  0.00001057397656383260,
        -0.00000053609339889243,  0.00000002181658454933, -0.00000000072901621186,
         0.00000000002037332548, -0.00000000000048344033,  0.00000000000000986533,
        -0.00000000000000017502,  0.00000000000000000272, -0.00000000000000000004
    };

    const static double chebev_b[17] = {
         0.63041404314570539241, -0.42344511405705333544,  0.37617172643343656625,
        -0.16249489154509567415,  0.03822255778633008694, -0.00564563477132190899,
         0.00057454951976897367, -0.00004287071532102004,  0.00000245120749923299,
        -0.00000011098841840868,  0.00000000408249731696, -0.00000000012449830219,
         0.00000000000320048425, -0.00000000000007032416,  0.00000000000000133638,
        -0.00000000000000002219,  0.00000000000000000032
    };

    const static double chebev_e[41] = {
        0.97462779093296822410, -0.02424701873969321371, 0.00103400906842977317, -0.00008052450246908016,
        0.00000905962481966582, -0.00000131016996757743, 0.00000022770820391497, -0.00000004558623552026,
        0.00000001021567537083, -0.00000000251114508133, 0.00000000066704761275, -0.00000000018931512852,
        0.00000000005689898935, -0.00000000001798219359, 0.00000000000594162963, -0.00000000000204285065,
        0.00000000000072797580, -0.00000000000026797428, 0.00000000000010160694, -0.00000000000003958559,
        0.00000000000001581262, -0.00000000000000646411, 0.00000000000000269981, -0.00000000000000115038,
        0.00000000000000049942, -0.00000000000000022064, 0.00000000000000009910, -0.00000000000000004520,
        0.00000000000000002092, -0.00000000000000000982, 0.00000000000000000467, -0.00000000000000000225,
        0.00000000000000000110, -0.00000000000000000054, 0.00000000000000000027, -0.00000000000000000014,
        0.00000000000000000007, -0.00000000000000000004, 0.00000000000000000002, -0.00000000000000000001,
        0.00000000000000000001
    };

    const static double chebev_f[35] = {
        0.99461545179407928910, -0.00524276766084297210, 0.00013325864229883909, -0.00000770856452642713,
        0.00000070848077032045, -0.00000008812517411602, 0.00000001359784717148, -0.00000000246858295747,
        0.00000000050925789921, -0.00000000011653400634, 0.00000000002906578309, -0.00000000000779847361,
        0.00000000000222802542, -0.00000000000067239338, 0.00000000000021296411, -0.00000000000007041482,
        0.00000000000002419805, -0.00000000000000861080, 0.00000000000000316287, -0.00000000000000119596,
        0.00000000000000046444, -0.00000000000000018485, 0.00000000000000007527, -0.00000000000000003131,
        0.00000000000000001328, -0.00000000000000000574, 0.00000000000000000252, -0.00000000000000000113,
        0.00000000000000000051, -0.00000000000000000024, 0.00000000000000000011, -0.00000000000000000005,
        0.00000000000000000002, -0.00000000000000000001, 0.00000000000000000001
    };
}
#endif // FRESNEL_H