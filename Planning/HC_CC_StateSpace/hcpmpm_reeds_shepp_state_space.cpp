#include "hcpmpm_reeds_shepp_state_space.h"

class HCPMPM_ReedsSheppStateSpace::HCPMPM_ReedsShepp
{
public:
    /************************ TT ************************/
    /**
     * @brief Judge TT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() == c2.getLeft()) // two circle tangent
        {
            return false;
        }
        else if(c1.getForward() == c2.getForward())
        {
            return false;
        }
        else
        {
            return fabs(_distance - 2 * _parent->_radius) < math::getEpsilon();
        }
    }

    /**
     * @brief Computation of the tangent point of outer circles
     * @param c1 :one circle
     * @param c2 :another circle
     * @param q :the tangent point
     */
    void TT_TangentCircles(HC_CC_Circle &c1, HC_CC_Circle &c2, Configuration **q) const
    {
        double x = 0.5 * (c1.getCenter_x() + c2.getCenter_x());
        double y = 0.5 * (c1.getCenter_y() + c2.getCenter_y());
        double angle = atan2(c2.getCenter_y() - c1.getCenter_y(),
                             c2.getCenter_x() - c1.getCenter_x());
        double q_psi;
        if(c1.getLeft())
        {
            if(c1.getForward())
            {
                q_psi = angle + MV_PI2 - _parent->_mu;
            }
            else
            {
                q_psi = angle + MV_PI2 + _parent->_mu;
            }
        }
        else
        {
            if(c1.getForward())
            {
                q_psi = angle - MV_PI2 + _parent->_mu;
            }
            else
            {
                q_psi = angle - MV_PI2 - _parent->_mu;
            }
        }
        *q = new Configuration(x, y, q_psi, 0.0);
    }

    /**
     * @brief Computation of the TT path
     * @param c1 :one circle
     * @param c2 :another circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q :the configuration tangent point
     * @return the length of path
     */
    double TT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                    HC_CC_Circle **cstart, HC_CC_Circle **cend,
                    Configuration **q1, Configuration **q2, Configuration **q3) const
    {
        TT_TangentCircles(c1, c2, q2);
        *cstart = new HC_CC_Circle(**q2, c1.getLeft(), !c1.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(**q2, c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *q1     = new Configuration(c1.getStart().getX(), c1.getStart().getY(), c1.getStart().getPsi(), c1.getKappa());
        *q3     = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());
        return (*cstart)->hc_turn_lenght(**q1) + (*cend)->hc_turn_lenght(**q3);
    }

    /************************ TcT ************************/
    /**
     * @brief Judge TcT path whether exist
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @return if exist return true,else return false
     */
    bool TcT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() == c2.getLeft()) // two circle tangent
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return fabs(_distance - 2 * fabs(c1.getKappaInv())) < math::getEpsilon();
        }
    }

    /**
     * @brief Computation of the tangent point of internal circles
     * @param c1 :one circle
     * @param c2 :another circle
     * @param q  :the tangent point
     */
    void TcT_TangentCircles(HC_CC_Circle &c1, HC_CC_Circle &c2, Configuration **q) const
    {
        double distance = c1.CenterDistance(c2);
        double angle    = atan2(c2.getCenter_y() - c1.getCenter_y(),
                                c2.getCenter_x() - c1.getCenter_x());
        double delta_x = 0.5 * distance;
        double delta_y = 0.0;
        double x,y,psi;
        if(c1.getLeft())
        {
            if(c1.getForward())
            {
                psi = angle + MV_PI2;
                math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), angle,
                                             delta_x,  delta_y, &x, &y);
            }
            else
            {
                psi = angle + MV_PI2;
                math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), angle,
                                             delta_x, -delta_y, &x, &y);
            }
        }
        else
        {
            if(c1.getForward())
            {
                psi = angle - MV_PI2;
                math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), angle,
                                             delta_x, -delta_y, &x, &y);
            }
            else
            {
                psi = angle - MV_PI2;
                math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), angle,
                                             delta_x,  delta_y, &x, &y);
            }
        }
        *q = new Configuration(x, y, psi, c1.getKappa());
    }

    /**
     * @brief Computation of the TcT path
     * @param c1 :one circle
     * @param c2 :another circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q :the configuration tangent point
     * @return the length of path
     */
    double TcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                     HC_CC_Circle **cstart, HC_CC_Circle **cend, Configuration **q) const
    {
        TcT_TangentCircles(c1, c2, q);
        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(c2);
        return (*cstart)->rs_turn_lenght(**q) + (*cend)->rs_turn_lenght(**q);
    }

    /************************ Reeds-Shepp families: ************************/

    /************************ TcTcT ************************/
    /**
     * @brief Judge TcTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTcT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() != c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() == c2.getForward())
        {
            return false;
        }
        else
        {
            return _distance <= 4 * fabs(c1.getKappaInv());
        }
    }

    /**
     * @brief Computation of the tangent point of circles
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param qa1 :the first tangent point of upper tangent circle
     * @param qa2 :the second tangent point of upper tangent circle
     * @param qb1 :the first tangent point of down tangent circle
     * @param qb2 :the second tangent point of down tangent circle
     */
    void TcTcT_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                               Configuration **qa1, Configuration **qa2,
                               Configuration **qb1, Configuration **qb2) const
    {
        double psi = _angle;
        double two_r = 2 * fabs(c1.getKappaInv());
        double delta_x = 0.5 * _distance;
        double delta_y = sqrt(pow(two_r,2) - pow(delta_x,2));
        double x,y;

        // the upper tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi, delta_x,  delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle1(x, y, !c1.getLeft(), !c1.getForward(), true, _parent->_rs_circle_param);

        // the down tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi, delta_x, -delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle2(x, y, !c1.getLeft(), !c1.getForward(), true, _parent->_rs_circle_param);

        TcT_TangentCircles(c1, middle_tangent_circle1, qa1);
        TcT_TangentCircles(middle_tangent_circle1, c2, qa2);

        TcT_TangentCircles(c1, middle_tangent_circle2, qb1);
        TcT_TangentCircles(middle_tangent_circle2, c2, qb2);
    }

    /**
     * @brief Computation of the length of TcTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param ci : the middle tangent circle
     * @param q1 : the tengent pint betwen start circle and middle circle
     * @param q2 : the tengent pint betwen end circle and middle circle
     * @return the length of the TcTcT Path
     */
    double TcTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2, HC_CC_Circle **ci) const
    {
        Configuration *qa1, *qa2, *qb1, *qb2;
        TcTcT_TangentCircles(c1, c2, &qa1, &qa2, &qb1, &qb2);

        HC_CC_Circle *middle_tangent_circle1, *middle_tangent_circle2;
        middle_tangent_circle1 = new HC_CC_Circle(*qa1, !c1.getLeft(), !c1.getForward(), true, _parent->_rs_circle_param);
        middle_tangent_circle2 = new HC_CC_Circle(*qb1, !c1.getLeft(), !c1.getForward(), true, _parent->_rs_circle_param);

        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(c2);

        // select shortest connection
        double length1 = (*cstart)->rs_turn_lenght(*qa1)
                       + middle_tangent_circle1->rs_turn_lenght(*qa2)
                       + (*cend)->rs_turn_lenght(*qa2);

        double length2 = (*cstart)->rs_turn_lenght(*qb1)
                       + middle_tangent_circle1->rs_turn_lenght(*qb2)
                       + (*cend)->rs_turn_lenght(*qb2);

        if(length1 < length2)
        {
            *q1 = qa1;
            *q2 = qa2;
            *ci = middle_tangent_circle1;
            delete qb1;
            delete qb2;
            delete middle_tangent_circle2;
            return length1;
        }
        else
        {
            *q1 = qb1;
            *q2 = qb2;
            *ci = middle_tangent_circle2;
            delete qa1;
            delete qa2;
            delete middle_tangent_circle1;
            return length2;
        }
    }

    /************************ TcTT ************************/
    /**
     * @brief Judge TcTT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() != c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance <= 2 * ( _parent->_radius + fabs(c1.getKappaInv()) ))
                && (_distance >= 2 * ( _parent->_radius - fabs(c1.getKappaInv()) ));
        }
    }

    /**
     * @brief Computation of the tangent point of TcTT circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param qa1 :the first tangent point of upper tangent circle
     * @param qa2 :the second tangent point of upper tangent circle
     * @param qb1 :the first tangent point of down tangent circle
     * @param qb2 :the second tangent point of down tangent circle
     */
    void TcTT_TangentCircles(HC_CC_Circle &c1, HC_CC_Circle &c2,
                             Configuration **qa1, Configuration **qa2,
                             Configuration **qb1, Configuration **qb2) const
    {
        double psi = _angle;
        double r1 = 2 * fabs(c1.getKappaInv());
        double r2 = 2 * _parent->_radius;
        double delta_x = (pow(r1, 2) + pow(_distance, 2) - pow(r2, 2)) / ( 2 * _distance );
        double delta_y = sqrt(pow(r1, 2) - pow(delta_x,2));
        double x,y;

        // the upper tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi, delta_x,  delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle1(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        // the down tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi, delta_x, -delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle2(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        TcT_TangentCircles(c1, middle_tangent_circle1, qa1);
        TT_TangentCircles(middle_tangent_circle1, c2, qa2);

        TcT_TangentCircles(c1, middle_tangent_circle2, qb1);
        TT_TangentCircles(middle_tangent_circle2, c2, qb2);
    }

    /**
     * @brief Computation of the length of TcTT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param ci : the middle tangent circle
     * @param q1 : the tengent pint betwen start circle and middle circle
     * @param q2 : the tengent pint betwen end circle and middle circle
     * @return the length of the TcTT Path
     */
    double TcTT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                      HC_CC_Circle **cstart, HC_CC_Circle **cend,
                      Configuration **q1, Configuration **q2, HC_CC_Circle **ci) const
    {
        Configuration *qa1, *qa2, *qb1, *qb2;
        TcTT_TangentCircles(c1, c2, &qa1, &qa2, &qb1, &qb2);

        HC_CC_Circle *middle_tangent_circle1, *middle_tangent_circle2;
        HC_CC_Circle *end_circle1, *end_circle2;
        middle_tangent_circle1 = new HC_CC_Circle(*qa2, !c1.getLeft(),  c1.getForward(), true, _parent->_hc_cc_circle_param);
        end_circle1            = new HC_CC_Circle(*qa2,  c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        middle_tangent_circle2 = new HC_CC_Circle(*qb2, !c1.getLeft(),  c1.getForward(), true, _parent->_hc_cc_circle_param);
        end_circle2            = new HC_CC_Circle(*qb2,  c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);

        *cstart = new HC_CC_Circle(c1);
        *q2     = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());

        // select shortest connection
        double length1 = (*cstart)->rs_turn_lenght(*qa1)
                       + middle_tangent_circle1->hc_turn_lenght(*qa1)
                       + end_circle1->hc_turn_lenght(**q2);

        double length2 = (*cstart)->rs_turn_lenght(*qb1)
                       + middle_tangent_circle2->hc_turn_lenght(*qb1)
                       + end_circle2->hc_turn_lenght(**q2);

        if(length1 < length2)
        {
            *cend = end_circle1;
            *q1 = qa1;
            *ci = middle_tangent_circle1;
            delete qa2;
            delete qb1;
            delete qb2;
            delete middle_tangent_circle2;
            delete end_circle2;
            return length1;
        }
        else
        {
            *cend = end_circle2;
            *q1 = qb1;
            *ci = middle_tangent_circle2;
            delete qa1;
            delete qa2;
            delete qb2;
            delete middle_tangent_circle1;
            delete end_circle1;
            return length2;
        }
    }

    /************************ TTcT ************************/
    /**
     * @brief Judge TTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TTcT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() != c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance <= 2 * ( _parent->_radius + fabs(c1.getKappaInv()) ))
                && (_distance >= 2 * ( _parent->_radius - fabs(c1.getKappaInv()) ));
        }
    }

    /**
     * @brief Computation of the tangent point of TTcT circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param qa1 :the first tangent point of upper tangent circle
     * @param qa2 :the second tangent point of upper tangent circle
     * @param qb1 :the first tangent point of down tangent circle
     * @param qb2 :the second tangent point of down tangent circle
     */
    void TTcT_TangentCircles(HC_CC_Circle &c1, HC_CC_Circle &c2,
                             Configuration **qa1, Configuration **qa2,
                             Configuration **qb1, Configuration **qb2) const
    {
        double psi = _angle;
        double r1 = 2 * _parent->_radius;
        double r2 = 2 * fabs(c1.getKappaInv());
        double delta_x = (pow(r1, 2) + pow(_distance, 2) - pow(r2, 2)) / ( 2 * _distance );
        double delta_y = sqrt(pow(r1, 2) - pow(delta_x,2));
        double x,y;

        // the upper tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi, delta_x,  delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle1(x, y, !c1.getLeft(), c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        // the down tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi, delta_x, -delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle2(x, y, !c1.getLeft(), c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        TT_TangentCircles(c1, middle_tangent_circle1, qa1);
        TcT_TangentCircles(middle_tangent_circle1, c2, qa2);

        TT_TangentCircles(c1, middle_tangent_circle2, qb1);
        TcT_TangentCircles(middle_tangent_circle2, c2, qb2);
    }

    /**
     * @brief Computation of the length of TTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the tengent pint betwen start circle and middle circle
     * @param q2 : the tengent pint betwen end circle and middle circle
     * @param ci : the middle tangent circle
     * @return the length of the TTcT Path
     */
    double TTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                      HC_CC_Circle **cstart, HC_CC_Circle **cend,
                      Configuration **q1, Configuration **q2, HC_CC_Circle **ci) const
    {
        Configuration *qa1, *qa2, *qb1, *qb2;
        TTcT_TangentCircles(c1, c2, &qa1, &qa2, &qb1, &qb2);

        HC_CC_Circle *middle_tangent_circle1, *middle_tangent_circle2;
        HC_CC_Circle *start_circle1, *start_circle2;
        start_circle1          = new HC_CC_Circle(*qa1,  c1.getLeft(), !c1.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        middle_tangent_circle1 = new HC_CC_Circle(*qa1, !c1.getLeft(),  c1.getForward(), true, _parent->_hc_cc_circle_param);
        start_circle2          = new HC_CC_Circle(*qb1,  c1.getLeft(), !c1.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        middle_tangent_circle2 = new HC_CC_Circle(*qb1, !c1.getLeft(),  c1.getForward(), true, _parent->_hc_cc_circle_param);

        *cend   = new HC_CC_Circle(c2);
        *q1     = new Configuration(c1.getStart().getX(), c1.getStart().getY(), c1.getStart().getPsi(), c1.getKappa());

        // select shortest connection
        double length1 = start_circle1->hc_turn_lenght(**q1)
                       + middle_tangent_circle1->hc_turn_lenght(*qa2)
                       + (*cend)->rs_turn_lenght(*qa2);

        double length2 = start_circle2->hc_turn_lenght(**q1)
                       + middle_tangent_circle2->hc_turn_lenght(*qb2)
                       + (*cend)->rs_turn_lenght(*qb2);

        if(length1 < length2)
        {
            *cstart = start_circle1;
            *q2 = qa2;
            *ci = middle_tangent_circle1;
            delete qa1;
            delete qb1;
            delete qb2;
            delete start_circle2;
            delete middle_tangent_circle2;
            return length1;
        }
        else
        {
            *cstart = start_circle2;
            *q2 = qb2;
            *ci = middle_tangent_circle2;
            delete qa1;
            delete qa2;
            delete qb1;
            delete middle_tangent_circle1;
            delete start_circle1;
            return length2;
        }
    }
    /************************ TST ************************/
    /**
     * @brief Judge TiST path whether exist
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @return if exist return true,else return false
     */
    bool TiST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() == c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() == c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance >= 2 * _parent->_radius);
        }
    }

    /**
     * @brief Judge TeST path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TeST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() != c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() == c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance >= 2 * _parent->_radius * _parent->_sin_mu);
        }
    }

    /**
     * @brief Judge TST path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        return TiST_Exist(c1, c2) || TeST_Exist(c1, c2);
    }

    /**
     * @brief Computation of the tangent point of TiST circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     */
    void TiST_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                              Configuration **q1, Configuration **q2) const
    {
        double distance = c1.CenterDistance(c2);
        double angle    = atan2( c2.getCenter_y() - c1.getCenter_y() ,
                                 c2.getCenter_x() - c1.getCenter_x() );
        double alpha = asin(2 * _parent->_radius * _parent->_cos_mu / distance);
        double delta_x = _parent->_radius * _parent->_sin_mu;
        double delta_y = _parent->_radius * _parent->_cos_mu;
        double x, y, psi;

        if(c1.getLeft() && c1.getForward())
        {
            psi = angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            psi = angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            psi = angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            psi = angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the tangent point of TiST circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 :the intersection point betwen start circle and straight line
     * @param q2 :the intersection point betwen end circle and straight line
     */
    void TeST_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                              Configuration **q1, Configuration **q2) const
    {
        double psi = atan2( c2.getCenter_y() - c1.getCenter_y() ,
                            c2.getCenter_x() - c1.getCenter_x() );
        double delta_x = _parent->_radius * _parent->_sin_mu;
        double delta_y = _parent->_radius * _parent->_cos_mu;
        double x, y;

        if(c1.getLeft() && c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the length of TiST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     * @return the length of the TiST Path
     */
    double TiST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                      HC_CC_Circle **cstart, HC_CC_Circle **cend,
                      Configuration **q1, Configuration **q2,
                      Configuration **q3, Configuration **q4) const
    {
        TiST_TangentCircles(c1, c2, q2, q3);
        *cstart = new HC_CC_Circle(**q2, c1.getLeft(), !c1.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(**q3, c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *q1     = new Configuration(c1.getStart().getX(), c1.getStart().getY(), c1.getStart().getPsi(), c1.getKappa());
        *q4     = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());

        return  (*cstart)->hc_turn_lenght(**q1) +
                (**q2).distance(**q3) +
                (*cend)->hc_turn_lenght(**q4);
    }

    /**
     * @brief Computation of the length of TeST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TeST Path
     */
    double TeST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                      HC_CC_Circle **cstart, HC_CC_Circle **cend,
                      Configuration **q1, Configuration **q2,
                      Configuration **q3, Configuration **q4) const
    {
        TeST_TangentCircles(c1, c2, q2, q3);
        *cstart = new HC_CC_Circle(**q2, c1.getLeft(), !c1.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(**q3, c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *q1     = new Configuration(c1.getStart().getX(), c1.getStart().getY(), c1.getStart().getPsi(), c1.getKappa());
        *q4     = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());
        return  (*cstart)->hc_turn_lenght(**q1) +
                (**q2).distance(**q3) +
                (*cend)->hc_turn_lenght(**q4);
    }

    /**
     * @brief Computation of the length of TST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TST Path
     */
    double TST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                     HC_CC_Circle **cstart, HC_CC_Circle **cend,
                     Configuration **q1, Configuration **q2,
                     Configuration **q3, Configuration **q4) const
    {
        if(TiST_Exist(c1, c2))
        {
            return TiST_Path(c1, c2, cstart, cend, q1, q2, q3, q4);
        }
        else if(TeST_Exist(c1, c2))
        {
            return TeST_Path(c1,c2, cstart, cend, q1, q2, q3, q4);
        }
        else
        {
            return numeric_limits<double>::max();
        }
    }

    /************************ TSTcT ************************/
    /**
     * @brief Judge TiSTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TiSTcT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() != c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance >= sqrt(pow(2 * _parent->_radius * _parent->_sin_mu + 2 *fabs(c1.getKappaInv()), 2) +
                                      pow(2 * _parent->_radius * _parent->_cos_mu, 2)));
        }
    }

    /**
     * @brief Judge TeSTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TeSTcT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() == c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance >= 2 * (fabs(c1.getKappaInv()) + _parent->_radius * _parent->_sin_mu));
        }
    }

    /**
     * @brief Judge TSTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TSTcT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        return TiSTcT_Exist(c1, c2) || TeSTcT_Exist(c1, c2);
    }

    /**
     * @brief Computation of the length of TiSTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the tengent pint betwen start circle and straight line
     * @param q2 : the tengent pint betwen middle circle and straight line
     * @param q3 : the tengent pint betwen middle circle and end circle
     * @param c1 : the middle circle
     * @return the length of the TiSTcT Path
     */
    double TiSTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                        HC_CC_Circle **cstart, HC_CC_Circle **cend,
                        Configuration **q1, Configuration **q2,
                        Configuration **q3, Configuration **q4,
                        HC_CC_Circle **ci) const
    {
        double psi = _angle;
        double delta_y = (4 * _parent->_radius * _parent->_cos_mu) / (fabs(c2.getKappa()) * _distance);
        double delta_x = sqrt(pow(2 * c2.getKappaInv(), 2) - pow(delta_y, 2));
        double x, y;

        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, delta_y, &x, &y);
        HC_CC_Circle middle_circle(x, y, !c2.getLeft(), c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        TiST_TangentCircles(c1, middle_circle, q2, q3);
        TcT_TangentCircles(middle_circle, c2, q4);

        *cstart = new HC_CC_Circle(**q2, c1.getLeft(), !c1.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(c2);
        *q1     = new Configuration(c1.getStart().getX(), c1.getStart().getY(), c1.getStart().getPsi(), c1.getKappa());
        *ci     = new HC_CC_Circle(**q3, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);
        return  (*cstart)->hc_turn_lenght(**q1) +
                (**q2).distance(**q3) +
                (*ci)->hc_turn_lenght(**q4) +
                (*cend)->rs_turn_lenght(**q4);

    }

    /**
     * @brief Computation of the length of TeSTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the tengent pint betwen start circle and straight line
     * @param q2 : the tengent pint betwen middle circle and straight line
     * @param q3 : the tengent pint betwen middle circle and end circle
     * @param c1 : the middle circle
     * @return the length of the TeSTcT Path
     */
    double TeSTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                        HC_CC_Circle **cstart, HC_CC_Circle **cend,
                        Configuration **q1, Configuration **q2,
                        Configuration **q3, Configuration **q4,
                        HC_CC_Circle **ci) const
    {
        double psi = _angle;
        double delta_x = 2 * fabs(c2.getKappaInv());
        double delta_y = 0;
        double x, y;

        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, delta_y, &x, &y);
        HC_CC_Circle middle_circle(x, y, !c2.getLeft(), c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        TeST_TangentCircles(c1, middle_circle, q2, q3);
        TcT_TangentCircles(middle_circle, c2, q4);

        *cstart = new HC_CC_Circle(**q2, c1.getLeft(), !c1.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(c2);
        *q1     = new Configuration(c1.getStart().getX(), c1.getStart().getY(), c1.getStart().getPsi(), c1.getKappa());
        *ci     = new HC_CC_Circle(**q3, c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);
        return  (*cstart)->hc_turn_lenght(**q1) +
                (**q2).distance(**q3) +
                (*ci)->hc_turn_lenght(**q4) +
                (*cend)->rs_turn_lenght(**q4);
    }

    /**
     * @brief Computation of the length of TeSTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the tengent pint betwen start circle and straight line
     * @param q2 : the tengent pint betwen middle circle and straight line
     * @param q3 : the tengent pint betwen middle circle and end circle
     * @param c1 : the middle circle
     * @return the length of the TeSTcT Path
     */
    double TSTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2,
                       Configuration **q3, Configuration **q4,
                       HC_CC_Circle **ci) const
    {
        if( TiSTcT_Exist(c1, c2) )
        {
            return TiSTcT_Path(c1, c2, cstart, cend, q1, q2, q3, q4, ci);
        }
        else if( TeSTcT_Exist(c1, c2) )
        {
            return TeSTcT_Path(c1, c2, cstart, cend, q1, q2, q3, q4, ci);
        }
        else
        {
            return numeric_limits<double>::max();
        }
    }
    /************************ TcTST ************************/
    /**
     * @brief Judge TcTiST path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTiST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() != c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance >= sqrt(pow(2 * _parent->_radius * _parent->_sin_mu + 2 *fabs(c1.getKappaInv()), 2) +
                                      pow(2 * _parent->_radius * _parent->_cos_mu, 2)));
        }
    }

    /**
     * @brief Judge TcTeST path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTeST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() == c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance >= 2 * (fabs(c1.getKappaInv()) + _parent->_radius * _parent->_sin_mu));
        }
    }

    /**
     * @brief Judge TcTST path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTST_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        return TcTiST_Exist(c1, c2) || TcTeST_Exist(c1, c2);
    }

    /**
     * @brief Computation of the length of TcTiST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the tengent point betwen start circle and straight line
     * @param q2 : the tengent point betwen middle circle and straight line
     * @param q3 : the tengent point betwen middle circle and end circle
     * @param q4 : the configuration point in the end
     * @param c1 : the middle circle
     * @return the length of the TcTiST Path
     */
    double TcTiST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                        HC_CC_Circle **cstart, HC_CC_Circle **cend,
                        Configuration **q1, Configuration **q2,
                        Configuration **q3, Configuration **q4,
                        HC_CC_Circle **ci) const
    {
        double psi = _angle;
        double delta_y = (4 * _parent->_radius * _parent->_cos_mu) / (fabs(c2.getKappa()) * _distance);
        double delta_x = sqrt(pow(2 * c2.getKappaInv(), 2) - pow(delta_y, 2));
        double x, y;

        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, -delta_y, &x, &y);
        HC_CC_Circle middle_circle(x, y, !c2.getLeft(), !c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        TcT_TangentCircles(c1, middle_circle, q1);
        TiST_TangentCircles(middle_circle, c2, q2, q3);

        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(**q3,  c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *ci     = new HC_CC_Circle(**q2, !c1.getLeft(),  c1.getForward(), true, _parent->_hc_cc_circle_param);
        *q4     = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());

        return  (*cstart)->rs_turn_lenght(**q1) +
                (*ci)->hc_turn_lenght(**q1) +
                (**q2).distance(**q3) +
                (*cend)->hc_turn_lenght(**q4);
    }

    /**
     * @brief Computation of the length of TcTeST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the cross point betwen start circle and straight line
     * @param q2 : the tengent point betwen middle circle and straight line
     * @param q3 : the tengent point betwen middle circle and end circle
     * @param q4 : the tengent point betwen middle circle and end circle
     * @param c1 : the middle circle
     * @return the length of the TcTeST Path
     */
    double TcTeST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                        HC_CC_Circle **cstart, HC_CC_Circle **cend,
                        Configuration **q1, Configuration **q2,
                        Configuration **q3, Configuration **q4,
                        HC_CC_Circle **ci) const
    {
        double psi = _angle;
        double delta_x = 2 * fabs(c2.getKappaInv());
        double delta_y = 0;
        double x, y;

        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, delta_y, &x, &y);
        HC_CC_Circle middle_circle(x, y,  c2.getLeft(), !c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        TcT_TangentCircles(c1, middle_circle, q1);
        TeST_TangentCircles(middle_circle, c2, q2, q3);


        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(**q3,  c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *ci     = new HC_CC_Circle(**q2, !c1.getLeft(),  c1.getForward(), true, _parent->_hc_cc_circle_param);
        *q4     = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());

        return  (*cstart)->rs_turn_lenght(**q1) +
                (*ci)->hc_turn_lenght(**q1) +
                (**q2).distance(**q3) +
                (*cend)->hc_turn_lenght(**q4);
    }

    /**
     * @brief Computation of the length of TcTST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the tengent pint betwen start circle and straight line
     * @param q2 : the tengent pint betwen middle circle and straight line
     * @param q3 : the tengent pint betwen middle circle and end circle
     * @param c1 : the middle circle
     * @return the length of the TcTST Path
     */
    double TcTST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2,
                       Configuration **q3, Configuration **q4,
                       HC_CC_Circle **ci) const
    {
        if( TcTiST_Exist(c1, c2) )
        {
            return TcTiST_Path(c1, c2, cstart, cend, q1, q2, q3, q4, ci);
        }
        else if( TcTeST_Exist(c1, c2) )
        {
            return TcTeST_Path(c1, c2, cstart, cend, q1, q2, q3, q4, ci);
        }
        else
        {
            return numeric_limits<double>::max();
        }
    }
    /************************ TcTSTcT ************************/
    /**
     * @brief Judge TcTiSTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTiSTcT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() == c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() == c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance >= sqrt(pow(2 * _parent->_radius, 2) +
                                      16 * _parent->_radius * _parent->_sin_mu * fabs(c1.getKappaInv()) +
                                      pow(4 * c1.getKappaInv(), 2)));
        }
    }

    /**
     * @brief Judge TcTeSTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTeSTcT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() != c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() == c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance >= 4 * fabs(c1.getKappaInv()) + 2 * _parent->_radius * _parent->_sin_mu );
        }
    }

    /**
     * @brief Judge TcTSTcT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcTSTcT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        return TcTiSTcT_Exist(c1, c2) || TcTeSTcT_Exist(c1, c2);
    }

    /**
     * @brief Computation of the length of TcTiSTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the intersection point betwen start circle and left middle circle
     * @param q2 : the intersection pint betwen left middle circle and straight line
     * @param q3 : the intersection pint betwen right middle circle and straight line
     * @param q3 : the intersection pint betwen right middle circle and end circle
     * @param ci1 : the left middle circle
     * @param ci2 : the right middle circle
     * @return the length of the TcTiSTcT Path
     */
    double TcTiSTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                          HC_CC_Circle **cstart, HC_CC_Circle **cend,
                          Configuration **q1, Configuration **q2,
                          Configuration **q3, Configuration **q4,
                          HC_CC_Circle **ci1, HC_CC_Circle **ci2) const
    {
        double psi = _angle;
        double delta_y = ( 4 * _parent->_radius * _parent->_cos_mu ) / ( fabs(c1.getKappa()) * _distance );
        double delta_x = sqrt(pow(2 * c1.getKappaInv(), 2) - pow(delta_y, 2));
        double x, y;

        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, delta_y, &x, &y);
        HC_CC_Circle left_middle_circle(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, -delta_y, &x, &y);
        HC_CC_Circle right_middle_circle(x, y, !c2.getLeft(), c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        TcT_TangentCircles(c1, left_middle_circle, q1);
        TiST_TangentCircles(left_middle_circle, right_middle_circle, q2, q3);
        TcT_TangentCircles(right_middle_circle, c2, q4);

        *cstart = new HC_CC_Circle(c1);
        *cend = new HC_CC_Circle(c2);
        *ci1 = new HC_CC_Circle(**q2, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);
        *ci2 = new HC_CC_Circle(**q3, !c2.getLeft(), c2.getForward(), true, _parent->_hc_cc_circle_param);

        return  (*cstart)->rs_turn_lenght(**q1) +
                (*ci1)->hc_turn_lenght(**q1) +
                (**q2).distance(**q3) +
                (*ci2)->hc_turn_lenght(**q4) +
                (*cend)->rs_turn_lenght(**q4);
    }

    /**
     * @brief Computation of the length of TcTeSTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the intersection point betwen start circle and left middle circle
     * @param q2 : the intersection pint betwen left middle circle and straight line
     * @param q3 : the intersection pint betwen right middle circle and straight line
     * @param q3 : the intersection pint betwen right middle circle and end circle
     * @param ci1 : the left middle circle
     * @param ci2 : the right middle circle
     * @return the length of the TcTeSTcT Path
     */
    double TcTeSTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                          HC_CC_Circle **cstart, HC_CC_Circle **cend,
                          Configuration **q1, Configuration **q2,
                          Configuration **q3, Configuration **q4,
                          HC_CC_Circle **ci1, HC_CC_Circle **ci2) const
    {
        double psi = _angle;
        double delta_x = 2 * fabs(c1.getKappaInv());
        double delta_y = 0;
        double x, y;

        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, delta_y, &x, &y);
        HC_CC_Circle left_middle_circle(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, delta_y, &x, &y);
        HC_CC_Circle right_middle_circle(x, y, !c2.getLeft(), c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        TcT_TangentCircles(c1, left_middle_circle, q1);
        TeST_TangentCircles(left_middle_circle, right_middle_circle, q2, q3);
        TcT_TangentCircles(right_middle_circle, c2, q4);

        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(c2);
        *ci1    = new HC_CC_Circle(**q2, !c1.getLeft(), c1.getForward(), true, _parent->_hc_cc_circle_param);
        *ci2    = new HC_CC_Circle(**q3, !c2.getLeft(), c2.getForward(), true, _parent->_hc_cc_circle_param);
        return  (*cstart)->rs_turn_lenght(**q1) +
                (*ci1)->hc_turn_lenght(**q1) +
                (**q2).distance(**q3) +
                (*ci2)->hc_turn_lenght(**q4) +
                (*cend)->rs_turn_lenght(**q4);
    }

    /**
     * @brief Computation of the length of TcTSTcT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the start circle
     * @param cend :the end circle
     * @param q1 : the intersection point betwen start circle and left middle circle
     * @param q2 : the intersection pint betwen left middle circle and straight line
     * @param q3 : the intersection pint betwen right middle circle and straight line
     * @param q3 : the intersection pint betwen right middle circle and end circle
     * @param ci1 : the left middle circle
     * @param ci2 : the right middle circle
     * @return the length of the TcTSTcT Path
     */
    double TcTSTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                         HC_CC_Circle **cstart, HC_CC_Circle **cend,
                         Configuration **q1, Configuration **q2,
                         Configuration **q3, Configuration **q4,
                         HC_CC_Circle **ci1, HC_CC_Circle **ci2) const
    {
        if( TcTiSTcT_Exist(c1, c2) )
        {
            return TcTiSTcT_Path(c1, c2, cstart, cend, q1, q2, q3, q4, ci1, ci2);
        }
        else if( TcTeSTcT_Exist(c1, c2) )
        {
            return TcTeSTcT_Path(c1, c2, cstart, cend, q1, q2, q3, q4, ci1, ci2);
        }
        else
        {
            return numeric_limits<double>::max();
        }
    }

    /************************ TTcTT ************************/
    /**
     * @brief Judge TTcTT path whether exist
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @return if exist return true,else return false
     */
    bool TTcTT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() == c2.getLeft()) // two circle tangent
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance <= 4 * _parent->_radius + 2 * fabs(c1.getKappaInv()));
        }
    }

    /**
     * @brief Computation of the tangent point of internal circles
     * @param c1 :start circle
     * @param c2 :end circle
     * @param qa1 :the tangent point betwen start circle and left middle circle
     * @param qa2 :the inersection point betwen left middle circle and right middle circle
     * @param qa3 :the tangent point betwen right middle circle and end circle
     * @param qb1 :the tangent point betwen start circle and left middle circle
     * @param qb2 :the inersection point betwen left middle circle and right middle circle
     * @param qb3 :the tangent point betwen right middle circle and end circle
     */
    void TTcTT_TangentCircles(HC_CC_Circle &c1, HC_CC_Circle &c2,
                              Configuration **qa1, Configuration **qa2, Configuration **qa3,
                              Configuration **qb1, Configuration **qb2, Configuration **qb3) const
    {
        double psi = _angle;
        double r1, r2, delta_x, delta_y, x, y;

        r1 = 2 * fabs(c1.getKappaInv());
        r2 = 2 * _parent->_radius;

        if( _distance < 4 * _parent->_radius - 2 * fabs(c1.getKappaInv()) )
        {
            delta_x = (_distance + r1) / 2;
            delta_y = sqrt(pow(r2, 2) - pow(delta_x, 2));
        }
        else
        {
            delta_x = (_distance - r1) / 2;
            delta_y = sqrt(pow(r2, 2) - pow(delta_x, 2));
        }

        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, delta_y, &x, &y);
        HC_CC_Circle left_middle_circle1(x, y, !c1.getLeft(), c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);
        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, delta_y, &x, &y);
        HC_CC_Circle right_middle_circle1(x, y, !c2.getLeft(), !c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, -delta_y, &x, &y);
        HC_CC_Circle left_middle_circle2(x, y, !c1.getLeft(), c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);
        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, -delta_y, &x, &y);
        HC_CC_Circle right_middle_circle2(x, y, !c2.getLeft(), !c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        TT_TangentCircles(c1, left_middle_circle1, qa1);
        TcT_TangentCircles(left_middle_circle1, right_middle_circle1, qa2);
        TT_TangentCircles(right_middle_circle1, c2, qa3);

        TT_TangentCircles(c1, left_middle_circle2, qb1);
        TcT_TangentCircles(left_middle_circle2, right_middle_circle2, qb2);
        TT_TangentCircles(right_middle_circle2, c2, qb3);
    }

    /**
     * @brief Computation of the TTcTT path
     * @param c1 :Configuration Start circle
     * @param c2 :Configuration end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 :the tangent point betwen start circle and left middle circle
     * @param q2 :the inersection point betwen left middle circle and right middle circle
     * @param q3 :the tangent point betwen right middle circle and end circle
     * @param ci1 :the left middle circle
     * @param ci2 :the right middle circle
     * @return the length of path
     */
    double TTcTT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2, Configuration **q3,
                       HC_CC_Circle **ci1, HC_CC_Circle **ci2) const
    {
        Configuration *qa1, *qa2, *qa3;
        Configuration *qb1, *qb2, *qb3;
        TTcTT_TangentCircles(c1, c2, &qa1, &qa2, &qa3, &qb1, &qb2, &qb3);

        HC_CC_Circle *left_middle_cricle1, *right_middle_circle1;
        HC_CC_Circle *left_middle_cricle2, *right_middle_circle2;
        HC_CC_Circle *start_circle1, *start_circle2;
        HC_CC_Circle *end_circle1, *end_circle2;
        start_circle1        = new HC_CC_Circle(*qa1,  c1.getLeft(), !c1.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        left_middle_cricle1  = new HC_CC_Circle(*qa1, !c1.getLeft(),  c1.getForward(), true, _parent->_hc_cc_circle_param);
        right_middle_circle1 = new HC_CC_Circle(*qa3, !c2.getLeft(),  c2.getForward(), true, _parent->_hc_cc_circle_param);
        end_circle1          = new HC_CC_Circle(*qa3,  c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);

        start_circle2        = new HC_CC_Circle(*qb1,  c1.getLeft(), !c1.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        left_middle_cricle2  = new HC_CC_Circle(*qb1, !c1.getLeft(),  c1.getForward(), true, _parent->_hc_cc_circle_param);
        right_middle_circle2 = new HC_CC_Circle(*qb3, !c2.getLeft(),  c2.getForward(), true, _parent->_hc_cc_circle_param);
        end_circle2          = new HC_CC_Circle(*qb3,  c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);

        *q1   = new Configuration(c1.getStart().getX(), c1.getStart().getY(), c1.getStart().getPsi(), c1.getKappa());
        *q3   = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());

        // select shortest connection path
        double lenght1 = start_circle1->hc_turn_lenght(**q1) +
                         left_middle_cricle1->hc_turn_lenght(*qa2) +
                         right_middle_circle1->hc_turn_lenght(*qa2) +
                         end_circle1->hc_turn_lenght(**q3);

        double lenght2 = start_circle2->hc_turn_lenght(**q1) +
                         left_middle_cricle2->hc_turn_lenght(*qb2) +
                         right_middle_circle2->hc_turn_lenght(*qb2) +
                         end_circle2->hc_turn_lenght(**q3);

        if( lenght1 < lenght2 )
        {
            *cstart = start_circle1;
            *cend   = end_circle1;
            *q2  = qa2;
            *ci1 = left_middle_cricle1;
            *ci2 = right_middle_circle1;
            delete qa1;
            delete qa3;
            delete qb1;
            delete qb2;
            delete qb3;
            delete left_middle_cricle2;
            delete right_middle_circle2;
            delete start_circle2;
            delete end_circle2;
            return lenght1;
        }
        else
        {
            *cstart = start_circle2;
            *cend   = end_circle2;
            *q2  = qb2;
            *ci1 = left_middle_cricle2;
            *ci2 = right_middle_circle2;
            delete qa1;
            delete qa2;
            delete qa3;
            delete qb1;
            delete qb3;
            delete left_middle_cricle1;
            delete right_middle_circle1;
            delete start_circle1;
            delete end_circle1;
            return lenght2;
        }
    }

    /************************ TcTTcT ************************/
    /**
     * @brief Judge TcTTcT path whether exist
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @return if exist return true,else return false
     */
    bool TcTTcT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() == c2.getLeft()) // two circle tangent
        {
            return false;
        }
        else if(c1.getForward() == c2.getForward())
        {
            return false;
        }
        else
        {
            return (_distance <= 4 * fabs(c1.getKappaInv()) + 2 * _parent->_radius) &&
                   (_distance >= 4 * fabs(c1.getKappaInv()) - 2 * _parent->_radius);
        }
    }

    /**
     * @brief Computation of the tangent point of internal circles
     * @param c1 :start circle
     * @param c2 :end circle
     * @param qa1 :the inersection point betwen start circle and left middle circle
     * @param qa2 :the tangent point betwen left middle circle and right middle circle
     * @param qa3 :the inersection point betwen right middle circle and end circle
     * @param qb1 :the inersection point betwen start circle and left middle circle
     * @param qb2 :the tangent point betwen left middle circle and right middle circle
     * @param qb3 :the inersection point betwen right middle circle and end circle
     */
    void TcTTcT_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                                Configuration **qa1, Configuration **qa2, Configuration **qa3,
                                Configuration **qb1, Configuration **qb2, Configuration **qb3) const
    {
        double psi = _angle;
        double r1, r2, delta_x, delta_y, x, y;

        r1 = 2 * fabs(c1.getKappaInv());
        r2 = _parent->_radius;

        delta_x = (pow(r1, 2) + pow(_distance / 2, 2) - pow(r2, 2)) / _distance;
        delta_y = sqrt(pow(r1, 2) - pow(delta_x, 2));

        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, delta_y, &x, &y);
        HC_CC_Circle left_middle_circle1(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);
        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, -delta_y, &x, &y);
        HC_CC_Circle right_middle_circle1(x, y, !c2.getLeft(), c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, -delta_y, &x, &y);
        HC_CC_Circle left_middle_circle2(x, y, !c1.getLeft(), !c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);
        math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                     -delta_x, delta_y, &x, &y);
        HC_CC_Circle right_middle_circle2(x, y, !c2.getLeft(), c2.getForward(), c2.getRegular(), _parent->_hc_cc_circle_param);

        TcT_TangentCircles(c1, left_middle_circle1, qa1);
        TT_TangentCircles(left_middle_circle1, right_middle_circle1, qa2);
        TcT_TangentCircles(right_middle_circle1, c2, qa3);

        TcT_TangentCircles(c1, left_middle_circle2, qb1);
        TT_TangentCircles(left_middle_circle2, right_middle_circle2, qb2);
        TcT_TangentCircles(right_middle_circle2, c2, qb3);
    }

    /**
     * @brief Computation of the TcTTcT path
     * @param c1 :Configuration Start circle
     * @param c2 :Configuration end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 :the inersection point betwen start circle and left middle circle
     * @param q2 :the tangent point betwen left middle circle and right middle circle
     * @param ci1 :the left middle circle
     * @param ci2 :the right middle circle
     * @return the length of path
     */
    double TcTTcT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                        HC_CC_Circle **cstart, HC_CC_Circle **cend,
                        Configuration **q1, Configuration **q2,
                        HC_CC_Circle **ci1, HC_CC_Circle **ci2) const
    {
        Configuration *qa1, *qa2, *qa3;
        Configuration *qb1, *qb2, *qb3;
        TcTTcT_TangentCircles(c1, c2, &qa1, &qa2, &qa3, &qb1, &qb2, &qb3);

        HC_CC_Circle *left_middle_cricle1, *right_middle_circle1;
        HC_CC_Circle *left_middle_cricle2, *right_middle_circle2;
        left_middle_cricle1  = new HC_CC_Circle(*qa2, !c1.getLeft(),  c1.getForward(), true, _parent->_hc_cc_circle_param);
        right_middle_circle1 = new HC_CC_Circle(*qa2,  c1.getLeft(), !c1.getForward(), true, _parent->_hc_cc_circle_param);
        left_middle_cricle2  = new HC_CC_Circle(*qb2, !c1.getLeft(),  c1.getForward(), true, _parent->_hc_cc_circle_param);
        right_middle_circle2 = new HC_CC_Circle(*qb2,  c1.getLeft(), !c1.getForward(), true, _parent->_hc_cc_circle_param);

        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(c2);

        // select shortest connection path
        double lenght1 = (*cstart)->rs_turn_lenght(*qa1) +
                         left_middle_cricle1->hc_turn_lenght(*qa1) +
                         right_middle_circle1->hc_turn_lenght(*qa3) +
                         (*cend)->rs_turn_lenght(*qa3);

        double lenght2 = (*cstart)->rs_turn_lenght(*qb1) +
                         left_middle_cricle2->hc_turn_lenght(*qb1) +
                         right_middle_circle2->hc_turn_lenght(*qb3) +
                         (*cend)->rs_turn_lenght(*qb3);

        if( lenght1 < lenght2 )
        {
            *q1 = qa1;
            *q2 = qa3;
            *ci1 = left_middle_cricle1;
            *ci2 = right_middle_circle1;
            delete qa2;
            delete qb1;
            delete qb2;
            delete qb3;
            delete left_middle_cricle2;
            delete right_middle_circle2;
            return lenght1;
        }
        else
        {
            *q1 = qb1;
            *q2 = qb3;
            *ci1 = left_middle_cricle2;
            *ci2 = right_middle_circle2;
            delete qa1;
            delete qa2;
            delete qa3;
            delete qb2;
            delete left_middle_cricle1;
            delete right_middle_circle1;
            return lenght2;
        }
    }
    /************************ Additional Families ************************/
    /************************ TTT ************************/
    /**
     * @brief Judge TTT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TTT_Exist(HC_CC_Circle &c1, HC_CC_Circle &c2) const
    {
        if(c1.getLeft() != c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() == c2.getForward())
        {
            return false;
        }
        else
        {
            return _distance <= 4 * _parent->_radius;
        }
    }

    /**
     * @brief Computation of the tangent point of TTT circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param qa1 :the first tangent point of upper tangent circle
     * @param qa2 :the second tangent point of upper tangent circle
     * @param qb1 :the first tangent point of down tangent circle
     * @param qb2 :the second tangent point of down tangent circle
     */
    void TTT_TangentCircles(HC_CC_Circle &c1, HC_CC_Circle &c2,
                             Configuration **qa1, Configuration **qa2,
                             Configuration **qb1, Configuration **qb2) const
    {
        double psi = _angle;
        double r = 2 * _parent->_radius;
        double delta_x = 0.5 * _distance;
        double delta_y = sqrt(pow(r, 2) - pow(delta_x,2));
        double x,y;

        // the upper tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x,  delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle1(x, y, !c1.getLeft(), c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        // the down tangent circle
        math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                     delta_x, -delta_y, &x, &y);
        HC_CC_Circle middle_tangent_circle2(x, y, !c1.getLeft(), c1.getForward(), c1.getRegular(), _parent->_hc_cc_circle_param);

        TT_TangentCircles(c1, middle_tangent_circle1, qa1);
        TT_TangentCircles(middle_tangent_circle1, c2, qa2);

        TT_TangentCircles(c1, middle_tangent_circle2, qb1);
        TT_TangentCircles(middle_tangent_circle2, c2, qb2);
    }

    /**
     * @brief Computation of the length of TTT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the tengent pint betwen start circle and middle circle
     * @param q2 : the tengent pint betwen end circle and middle circle
     * @param ci : the middle tangent circle
     * @return the length of the TcTcT Path
     */
    double TTT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                     HC_CC_Circle **cstart, HC_CC_Circle **cend,
                     Configuration **q1, Configuration **q2, Configuration **q3,
                     HC_CC_Circle **ci) const
    {
        Configuration *qa1, *qa2, *qb1, *qb2;
        TTT_TangentCircles(c1, c2, &qa1, &qa2, &qb1, &qb2);

        HC_CC_Circle *middle_tangent_circle1, *middle_tangent_circle2;
        HC_CC_Circle *start_circle1, *start_circle2;
        HC_CC_Circle *end_circle1, *end_circle2;

        start_circle1          = new HC_CC_Circle(*qa1,  c1.getLeft(), !c1.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        middle_tangent_circle1 = new HC_CC_Circle(*qa1, !c1.getLeft(),  c1.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        end_circle1            = new HC_CC_Circle(*qa2,  c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        start_circle2          = new HC_CC_Circle(*qb1,  c1.getLeft(), !c1.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        middle_tangent_circle2 = new HC_CC_Circle(*qb1, !c1.getLeft(),  c1.getForward(), CC_REGULAR, _parent->_hc_cc_circle_param);
        end_circle2            = new HC_CC_Circle(*qb2,  c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);

        *q1   = new Configuration(c1.getStart().getX(), c1.getStart().getY(), c1.getStart().getPsi(), c1.getKappa());
        *q3   = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());
        // select shortest connection
        double length1 = start_circle1->hc_turn_lenght(**q1)
                       + middle_tangent_circle1->cc_turn_lenght(*qa2)
                       + end_circle1->hc_turn_lenght(**q3);

        double length2 = start_circle2->hc_turn_lenght(**q1)
                       + middle_tangent_circle2->cc_turn_lenght(*qb2)
                       + end_circle2->hc_turn_lenght(**q3);

        if(length1 < length2)
        {
            *cstart = start_circle1;
            *cend   = end_circle1;
            *q2 = qa2;
            *ci = middle_tangent_circle1;
            delete qa1;
            delete qb1;
            delete qb2;
            delete middle_tangent_circle2;
            delete start_circle2;
            delete end_circle2;
            return length1;
        }
        else
        {
            *cstart = start_circle2;
            *cend   = end_circle2;
            *q2 = qb2;
            *ci = middle_tangent_circle2;
            delete qa1;
            delete qa2;
            delete qb1;
            delete middle_tangent_circle1;
            delete start_circle1;
            delete end_circle1;
            return length2;
        }
    }

    /************************ TcST ************************/
    /**
     * @brief Judge TciST path whether exist
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @return if exist return true,else return false
     */
    bool TciST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() == c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return _distance >= sqrt( pow(_parent->_radius * _parent->_sin_mu, 2) +
                                      pow(_parent->_radius * _parent->_cos_mu + fabs(c1.getKappaInv()), 2));
        }
    }

    /**
     * @brief Judge TceST path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TceST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() != c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return _distance >= sqrt( pow(_parent->_radius * _parent->_sin_mu, 2) +
                                      pow(_parent->_radius * _parent->_cos_mu - fabs(c1.getKappaInv()), 2));
        }
    }

    /**
     * @brief Judge TcST path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcST_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        return TciST_Exist(c1, c2) || TceST_Exist(c1, c2);
    }

    /**
     * @brief Computation of the tangent point of TciST circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     */
    void TciST_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                               Configuration **q1, Configuration **q2) const
    {
        double alpha = asin( (_parent->_radius * _parent->_cos_mu + fabs(c1.getKappaInv())) / _distance);
        double delta_x1 = 0.0;
        double delta_y1 = fabs(c1.getKappaInv());
        double delta_x2 = _parent->_radius * _parent->_sin_mu;
        double delta_y2 = _parent->_radius * _parent->_cos_mu;
        double x, y, psi;

        if(c1.getLeft() && c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the tangent point of TceST circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 :the intersection point betwen start circle and straight line
     * @param q2 :the intersection point betwen end circle and straight line
     */
    void TceST_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                               Configuration **q1, Configuration **q2) const
    {
        double alpha = asin( (_parent->_radius * _parent->_cos_mu - fabs(c1.getKappaInv())) / _distance);
        double delta_x1 = 0.0;
        double delta_y1 = fabs(c1.getKappaInv());
        double delta_x2 = _parent->_radius * _parent->_sin_mu;
        double delta_y2 = _parent->_radius * _parent->_cos_mu;
        double x, y, psi;

        if(c1.getLeft() && c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, 0);
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                         -delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, 0);
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the length of TciST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     * @return the length of the TciST Path
     */
    double TciST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2, Configuration **q3) const
    {
        TciST_TangentCircles(c1, c2, q1, q2);
        *q3 = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());
        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(**q2, c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        return  (*cstart)->rs_turn_lenght(**q1) +
                (**q1).distance(**q2) +
                (*cend)->hc_turn_lenght(**q3);

    }

    /**
     * @brief Computation of the length of TceST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TeST Path
     */
    double TceST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2, Configuration **q3) const
    {
        TceST_TangentCircles(c1, c2, q1, q2);
        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(**q2, c2.getLeft(), !c2.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *q3 = new Configuration(c2.getStart().getX(), c2.getStart().getY(), c2.getStart().getPsi(), c2.getKappa());
        return  (*cstart)->rs_turn_lenght(**q1) +
                (**q1).distance(**q2) +
                (*cend)->hc_turn_lenght(**q3);
    }

    /**
     * @brief Computation of the length of TcST path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TST Path
     */
    double TcST_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                      HC_CC_Circle **cstart, HC_CC_Circle **cend,
                      Configuration **q1, Configuration **q2, Configuration **q3) const
    {
        if(TciST_Exist(c1, c2))
        {
            return TciST_Path(c1, c2, cstart, cend, q1, q2, q3);
        }
        else if(TceST_Exist(c1, c2))
        {
            return TceST_Path(c1, c2, cstart, cend, q1, q2, q3);
        }
        else
        {
            return numeric_limits<double>::max();
        }
    }
    /************************ TScT ************************/
    /**
     * @brief Judge TiScT path whether exist
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @return if exist return true,else return false
     */
    bool TiScT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() == c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return _distance >= sqrt( pow(_parent->_radius * _parent->_sin_mu, 2) +
                                      pow(_parent->_radius * _parent->_cos_mu + fabs(c1.getKappaInv()), 2));
        }
    }

    /**
     * @brief Judge TeScT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TeScT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() != c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() != c2.getForward())
        {
            return false;
        }
        else
        {
            return _distance >= sqrt( pow(_parent->_radius * _parent->_sin_mu, 2) +
                                      pow(_parent->_radius * _parent->_cos_mu - fabs(c1.getKappaInv()), 2));
        }
    }

    /**
     * @brief Judge TScT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TScT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        return TiScT_Exist(c1, c2) || TeScT_Exist(c1, c2);
    }

    /**
     * @brief Computation of the tangent point of TiScT circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     */
    void TiScT_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                               Configuration **q1, Configuration **q2) const
    {
        double alpha = asin( (_parent->_radius * _parent->_cos_mu + fabs(c1.getKappaInv())) / _distance);
        double delta_x1 = _parent->_radius * _parent->_sin_mu;
        double delta_y1 = _parent->_radius * _parent->_cos_mu;
        double delta_x2 = 0.0;
        double delta_y2 = fabs(c1.getKappaInv());
        double x, y, psi;

        if(c1.getLeft() && c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the tangent point of TeScT circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 :the intersection point betwen start circle and straight line
     * @param q2 :the intersection point betwen end circle and straight line
     */
    void TeScT_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                               Configuration **q1, Configuration **q2) const
    {
        double alpha = asin( (_parent->_radius * _parent->_cos_mu - fabs(c1.getKappaInv())) / _distance);
        double delta_x1 = _parent->_radius * _parent->_sin_mu;
        double delta_y1 = _parent->_radius * _parent->_cos_mu;
        double delta_x2 = 0.0;
        double delta_y2 = fabs(c1.getKappaInv());
        double x, y, psi;

        if(c1.getLeft() && c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1,  delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2,  delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                          delta_x1, -delta_y1, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, 0.0);

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x2, -delta_y2, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the length of TiScT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     * @return the length of the TiScT Path
     */
    double TiScT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2, Configuration **q3) const
    {
        TiScT_TangentCircles(c1, c2, q2, q3);
        *cstart = new HC_CC_Circle(**q2, c1.getLeft(), !c1.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(c2);
        *q1 = new Configuration(c1.getStart().getX(), c1.getStart().getY(), c1.getStart().getPsi(), c1.getKappa());
        return  (*cstart)->hc_turn_lenght(**q1) +
                (**q2).distance(**q3) +
                (*cend)->rs_turn_lenght(**q3);

    }

    /**
     * @brief Computation of the length of TeScT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TeScT Path
     */
    double TeScT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2, Configuration **q3) const
    {
        TeScT_TangentCircles(c1, c2, q2, q3);
        *q1     = new Configuration(c1.getStart().getX(), c1.getStart().getY(), c1.getStart().getPsi(), c1.getKappa());
        *cstart = new HC_CC_Circle(**q2, c1.getLeft(), !c1.getForward(), HC_REGULAR, _parent->_hc_cc_circle_param);
        *cend   = new HC_CC_Circle(c2);
        return  (*cstart)->hc_turn_lenght(**q1) +
                (**q2).distance(**q3) +
                (*cend)->rs_turn_lenght(**q3);
    }

    /**
     * @brief Computation of the length of TScT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TScT Path
     */
    double TScT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                      HC_CC_Circle **cstart, HC_CC_Circle **cend,
                      Configuration **q1, Configuration **q2, Configuration **q3) const
    {
        if(TiScT_Exist(c1, c2))
        {
            return TiScT_Path(c1, c2, cstart, cend, q1, q2, q3);
        }
        else if(TeScT_Exist(c1, c2))
        {
            return TeScT_Path(c1, c2, cstart, cend, q1, q2, q3);
        }
        else
        {
            return numeric_limits<double>::max();
        }
    }
    /************************ TcScT ************************/
    /**
     * @brief Judge TciScT path whether exist
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @return if exist return true,else return false
     */
    bool TciScT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() == c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() == c2.getForward())
        {
            return false;
        }
        else
        {
            return _distance >= 2 * fabs(c1.getKappaInv());
        }
    }

    /**
     * @brief Judge TceScT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TceScT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        if(c1.getLeft() != c2.getLeft())
        {
            return false;
        }
        else if(c1.getForward() == c2.getForward())
        {
            return false;
        }
        else
        {
            return _distance >= math::getEpsilon();
        }
    }

    /**
     * @brief Judge TcScT path whether exist
     * @param c1 :the start path
     * @param c2 :the end path
     * @return if exist return true,else return false
     */
    bool TcScT_Exist( HC_CC_Circle &c1, HC_CC_Circle &c2 ) const
    {
        return TciScT_Exist(c1, c2) || TceScT_Exist(c1, c2);
    }

    /**
     * @brief Computation of the tangent point of TciScTs circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     */
    void TciScT_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                                Configuration **q1, Configuration **q2) const
    {
        double alpha = asin( 2 / (fabs(c1.getKappa()) * _distance));
        double delta_x = 0.0;
        double delta_y = fabs(c1.getKappaInv());
        double x, y, psi;

        if(c1.getLeft() && c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x,  delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x,  delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            psi = _angle + alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x,  delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            psi = _angle - alpha;
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x,  delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the tangent point of TceScT circles path
     * @param c1 :the start circle
     * @param c2 :the end circle
     * @param q1 :the intersection point betwen start circle and straight line
     * @param q2 :the intersection point betwen end circle and straight line
     */
    void TceScT_TangentCircles( HC_CC_Circle &c1, HC_CC_Circle &c2,
                                Configuration **q1, Configuration **q2) const
    {
        double psi = _angle;
        double delta_x = 0.0;
        double delta_y = fabs(c1.getKappaInv());
        double x, y;

        if(c1.getLeft() && c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x,  delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x,  delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else if(c1.getLeft() && !c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else if(!c1.getLeft() && c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x, -delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi + MV_PI, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x, -delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi + MV_PI, c2.getKappa());
        }
        else if(!c1.getLeft() && !c1.getForward())
        {
            math::change_to_global_frame(c1.getCenter_x(), c1.getCenter_y(), psi,
                                         -delta_x,  delta_y, &x, &y);
            *q1 = new Configuration(x, y, psi, c1.getKappa());

            math::change_to_global_frame(c2.getCenter_x(), c2.getCenter_y(), psi,
                                          delta_x,  delta_y, &x, &y);
            *q2 = new Configuration(x, y, psi, c2.getKappa());
        }
        else
        {

        }
    }

    /**
     * @brief Computation of the length of TciScT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the intersection pint betwen start circle and straight line
     * @param q2 : the intersection pint betwen end circle and straight line
     * @return the length of the TciScT Path
     */
    double TciScT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                        HC_CC_Circle **cstart, HC_CC_Circle **cend,
                        Configuration **q1, Configuration **q2) const
    {
        TciScT_TangentCircles(c1, c2, q1, q2);
        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(c2);
        return  (*cstart)->rs_turn_lenght(**q1) +
                (**q1).distance(**q2) +
                (*cend)->rs_turn_lenght(**q2);

    }

    /**
     * @brief Computation of the length of TceScT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TceScT Path
     */
    double TceScT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                        HC_CC_Circle **cstart, HC_CC_Circle **cend,
                        Configuration **q1, Configuration **q2) const
    {
        TceScT_TangentCircles(c1, c2, q1, q2);
        *cstart = new HC_CC_Circle(c1);
        *cend   = new HC_CC_Circle(c2);
        return  (*cstart)->rs_turn_lenght(**q1) +
                (**q1).distance(**q2) +
                (*cend)->rs_turn_lenght(**q2);
    }

    /**
     * @brief Computation of the length of TcScT path
     * @param c1 :the start cricle
     * @param c2 :the end circle
     * @param cstart :the output start circle
     * @param cend :the output end circle
     * @param q1 : the Intersection pint betwen start circle and straight line
     * @param q2 : the Intersection pint betwen end circle and straight line
     * @return the length of the TcScT Path
     */
    double TcScT_Path( HC_CC_Circle &c1, HC_CC_Circle &c2,
                       HC_CC_Circle **cstart, HC_CC_Circle **cend,
                       Configuration **q1, Configuration **q2) const
    {
        if(TciScT_Exist(c1, c2))
        {
            return TciScT_Path(c1, c2, cstart, cend, q1, q2);
        }
        else if(TceScT_Exist(c1, c2))
        {
            return TceScT_Path(c1,c2, cstart, cend, q1, q2);
        }
        else
        {
            return numeric_limits<double>::max();
        }
    }

    /************************ class display conversion ************************/
    explicit HCPMPM_ReedsShepp(HCPMPM_ReedsSheppStateSpace *parent)
    {
        _parent = parent;
    }
    /************************ private variable interface ************************/
    void   setDistance(double value){ _distance = value; }
    double getDistance(void)        { return _distance; }

    void   setAngle(double value) { _angle = value; }
    double getAngle(void)         { return _angle; }
private:
    HCPMPM_ReedsSheppStateSpace *_parent;

    double _distance = 0.0;
    double _angle = 0.0;
};// end of HCPMPM_ReedsShepp class function define

/**
 * @brief HC00_ReedsSheppStateSpace Constructor
 * @param kappa :the max curvature of the path
 * @param sigma :the max sharpness of the path
 * @param discretization :the discretization step
 */
HCPMPM_ReedsSheppStateSpace::HCPMPM_ReedsSheppStateSpace(double kappa, double sigma, double discretization)
    : HC_CC_StateSpace(kappa, sigma, discretization)
    , _hcpmpm_reeds_shepp{ unique_ptr<HCPMPM_ReedsShepp>(new HCPMPM_ReedsShepp(this))}
{
    _rs_circle_param.setParam(_kappa, numeric_limits<double>::max(), 1 / _kappa, 0.0);
    _radius = _hc_cc_circle_param.getRadius();
    _mu = _hc_cc_circle_param.getMu();
    _sin_mu = _hc_cc_circle_param.getSinMu();
    _cos_mu = _hc_cc_circle_param.getCosMu();
}

/**
 * @brief Destructor
 */
HCPMPM_ReedsSheppStateSpace::~HCPMPM_ReedsSheppStateSpace() = default;

/**
 * @brief Returns a sequence of turns and straight lines connecting the two circles c1 and c2
 * @param c1 :start circle
 * @param c2 :end circle
 * @return a sequence of turns and straight line
 */
HC_CC_RS_Path* HCPMPM_ReedsSheppStateSpace::HCPMPM_CirclesReedsSheppPath(HC_CC_Circle &c1, HC_CC_Circle &c2) const
{
    // table containing the lengths of the paths, the intermediate configurations and circles
    double length[nb_hc_cc_rs_paths];
    math::DoubleArrayInit(length, nb_hc_cc_rs_paths, numeric_limits<double>::max());

    Configuration *qi1[nb_hc_cc_rs_paths];
    math::PointerArrayInit(reinterpret_cast<void **>(qi1), nb_hc_cc_rs_paths);
    Configuration *qi2[nb_hc_cc_rs_paths];
    math::PointerArrayInit(reinterpret_cast<void **>(qi2), nb_hc_cc_rs_paths);
    Configuration *qi3[nb_hc_cc_rs_paths];
    math::PointerArrayInit(reinterpret_cast<void **>(qi3), nb_hc_cc_rs_paths);
    Configuration *qi4[nb_hc_cc_rs_paths];
    math::PointerArrayInit(reinterpret_cast<void **>(qi4), nb_hc_cc_rs_paths);

    HC_CC_Circle *cstart[nb_hc_cc_rs_paths];
    math::PointerArrayInit(reinterpret_cast<void **>(cstart), nb_hc_cc_rs_paths);
    HC_CC_Circle *cend[nb_hc_cc_rs_paths];
    math::PointerArrayInit(reinterpret_cast<void **>(cend), nb_hc_cc_rs_paths);
    HC_CC_Circle *ci1[nb_hc_cc_rs_paths];
    math::PointerArrayInit(reinterpret_cast<void **>(ci1), nb_hc_cc_rs_paths);
    HC_CC_Circle *ci2[nb_hc_cc_rs_paths];
    math::PointerArrayInit(reinterpret_cast<void **>(ci2), nb_hc_cc_rs_paths);

    // precomputations
    _hcpmpm_reeds_shepp->setDistance( c1.CenterDistance(c2) );
    _hcpmpm_reeds_shepp->setAngle( atan2(c2.getCenter_y() - c1.getCenter_y(),
                                         c2.getCenter_x() - c1.getCenter_x()));

    if(c1.getStart().equal(c2.getStart()))// case E
    {
        length[hc_cc_rs::E] = 0.0;
    }
    else if(c1.isConfigurationOn_HC_CC_Circle(c2.getStart())) // case T
    {
        cstart[hc_cc_rs::T] = new HC_CC_Circle(c1.getStart(), c1.getLeft(), c1.getForward(), true, _rs_circle_param);
        length[hc_cc_rs::T] = cstart[hc_cc_rs::T]->rs_turn_lenght(c2.getStart());
    }
    else
    {
        if( _hcpmpm_reeds_shepp->TT_Exist(c1, c2)) //case TT
        {
            length[hc_cc_rs::TT] =
                _hcpmpm_reeds_shepp->TT_Path(
                    c1, c2,
                    &cstart[hc_cc_rs::TT], &cend[hc_cc_rs::TT],
                    &qi1[hc_cc_rs::TT], &qi2[hc_cc_rs::TT], &qi3[hc_cc_rs::TT]);
        }
        if( _hcpmpm_reeds_shepp->TcT_Exist(c1, c2) ) // case TcT
        {
            length[hc_cc_rs::TcT] =
                _hcpmpm_reeds_shepp->TcT_Path(
                    c1, c2,
                    &cstart[hc_cc_rs::TcT], &cend[hc_cc_rs::TcT],
                    &qi1[hc_cc_rs::TcT]);
        }
        /************************ Reeds-Shepp families: ************************/
        if( _hcpmpm_reeds_shepp->TcTcT_Exist(c1, c2) ) // case TcTcT
        {
            length[hc_cc_rs::TcTcT] =
                _hcpmpm_reeds_shepp->TcTcT_Path(
                    c1, c2,
                    &cstart[hc_cc_rs::TcTcT], &cend[hc_cc_rs::TcTcT],
                    &qi1[hc_cc_rs::TcTcT], &qi2[hc_cc_rs::TcTcT],
                    &ci1[hc_cc_rs::TcTcT]);
        }
        if( _hcpmpm_reeds_shepp->TcTT_Exist(c1, c2) ) // case TcTT
        {
            length[hc_cc_rs::TcTT] =
                _hcpmpm_reeds_shepp->TcTT_Path(
                    c1, c2,
                    &cstart[hc_cc_rs::TcTT], &cend[hc_cc_rs::TcTT],
                    &qi1[hc_cc_rs::TcTT], &qi2[hc_cc_rs::TcTT],
                    &ci1[hc_cc_rs::TcTT]);
        }
        if( _hcpmpm_reeds_shepp->TTcT_Exist(c1, c2) ) // case TTcT
        {
            length[hc_cc_rs::TTcT] =
                _hcpmpm_reeds_shepp->TTcT_Path(
                    c1, c2,
                    &cstart[hc_cc_rs::TTcT], &cend[hc_cc_rs::TTcT],
                    &qi1[hc_cc_rs::TTcT], &qi2[hc_cc_rs::TTcT],
                    &ci1[hc_cc_rs::TTcT]);
        }
        if( _hcpmpm_reeds_shepp->TST_Exist(c1, c2) ) // case TST
        {
            length[hc_cc_rs::TST] =
                _hcpmpm_reeds_shepp->TST_Path(
                    c1, c2,
                    &cstart[hc_cc_rs::TST], &cend[hc_cc_rs::TST],
                    &qi1[hc_cc_rs::TST], &qi2[hc_cc_rs::TST],
                    &qi3[hc_cc_rs::TST], &qi4[hc_cc_rs::TST]);
        }
        if( _hcpmpm_reeds_shepp->TSTcT_Exist(c1, c2) ) // case TSTcT
        {
            length[hc_cc_rs::TSTcT] =
                _hcpmpm_reeds_shepp->TSTcT_Path(
                    c1, c2,
                    &cstart[hc_cc_rs::TSTcT], &cend[hc_cc_rs::TSTcT],
                    &qi1[hc_cc_rs::TSTcT], &qi2[hc_cc_rs::TSTcT],
                    &qi3[hc_cc_rs::TSTcT], &qi4[hc_cc_rs::TSTcT],
                    &ci1[hc_cc_rs::TSTcT]);
        }
        if( _hcpmpm_reeds_shepp->TcTST_Exist(c1, c2) ) // case TcTST
        {
            length[hc_cc_rs::TcTST] =
                _hcpmpm_reeds_shepp->TcTST_Path(
                    c1, c2,
                    &cstart[hc_cc_rs::TcTST], &cend[hc_cc_rs::TcTST],
                    &qi1[hc_cc_rs::TcTST], &qi2[hc_cc_rs::TcTST],
                    &qi3[hc_cc_rs::TcTST], &qi4[hc_cc_rs::TcTST],
                    &ci1[hc_cc_rs::TcTST]);
        }
        if( _hcpmpm_reeds_shepp->TcTSTcT_Exist(c1, c2) ) // case TcTSTcT
        {
            length[hc_cc_rs::TcTSTcT] =
                _hcpmpm_reeds_shepp->TcTSTcT_Path(
                    c1, c2,
                    &cstart[hc_cc_rs::TcTSTcT], &cend[hc_cc_rs::TcTSTcT],
                    &qi1[hc_cc_rs::TcTSTcT], &qi2[hc_cc_rs::TcTSTcT],
                    &qi3[hc_cc_rs::TcTSTcT], &qi4[hc_cc_rs::TcTSTcT],
                    &ci1[hc_cc_rs::TcTSTcT], &ci2[hc_cc_rs::TcTSTcT]);
        }
        if( _hcpmpm_reeds_shepp->TTcTT_Exist(c1, c2) ) // case TTcTT
        {
            length[hc_cc_rs::TTcTT] =
                _hcpmpm_reeds_shepp->TTcTT_Path(
                    c1, c2,
                    &cstart[hc_cc_rs::TTcTT], &cend[hc_cc_rs::TTcTT],
                    &qi1[hc_cc_rs::TTcTT], &qi2[hc_cc_rs::TTcTT], &qi3[hc_cc_rs::TTcTT],
                    &ci1[hc_cc_rs::TTcTT], &ci2[hc_cc_rs::TTcTT]);
        }
        if( _hcpmpm_reeds_shepp->TcTTcT_Exist(c1, c2) ) // case TcTTcT
        {
            length[hc_cc_rs::TcTTcT] =
                _hcpmpm_reeds_shepp->TcTTcT_Path(
                    c1, c2,
                    &cstart[hc_cc_rs::TcTTcT], &cend[hc_cc_rs::TcTTcT],
                    &qi1[hc_cc_rs::TcTTcT], &qi2[hc_cc_rs::TcTTcT],
                    &ci1[hc_cc_rs::TcTTcT], &ci2[hc_cc_rs::TcTTcT]);
        }
        /************************ Additional Families: ************************/
        if( _hcpmpm_reeds_shepp->TTT_Exist(c1, c2) ) // case TTT
        {
            length[hc_cc_rs::TTT] =
                _hcpmpm_reeds_shepp->TTT_Path(
                    c1, c2,
                    &cstart[hc_cc_rs::TTT], &cend[hc_cc_rs::TTT],
                    &qi1[hc_cc_rs::TTT], &qi2[hc_cc_rs::TTT], &qi3[hc_cc_rs::TTT],
                    &ci1[hc_cc_rs::TTT]);
        }
        if( _hcpmpm_reeds_shepp->TcST_Exist(c1, c2) ) // case TcST
        {
            length[hc_cc_rs::TcST] =
                _hcpmpm_reeds_shepp->TcST_Path(
                    c1, c2,
                    &cstart[hc_cc_rs::TcST], &cend[hc_cc_rs::TcST],
                    &qi1[hc_cc_rs::TcST], &qi2[hc_cc_rs::TcST], &qi3[hc_cc_rs::TcST]);
        }
        if( _hcpmpm_reeds_shepp->TScT_Exist(c1, c2) ) // case TScT
        {
            length[hc_cc_rs::TScT] =
                _hcpmpm_reeds_shepp->TScT_Path(
                    c1, c2,
                    &cstart[hc_cc_rs::TScT], &cend[hc_cc_rs::TScT],
                    &qi1[hc_cc_rs::TScT], &qi2[hc_cc_rs::TScT], &qi3[hc_cc_rs::TScT]);
        }
        if( _hcpmpm_reeds_shepp->TcScT_Exist(c1, c2) ) // case TcScT
        {
            length[hc_cc_rs::TcScT] =
                _hcpmpm_reeds_shepp->TcScT_Path(
                    c1, c2,
                    &cstart[hc_cc_rs::TcScT], &cend[hc_cc_rs::TcScT],
                    &qi1[hc_cc_rs::TcScT], &qi2[hc_cc_rs::TcScT]);
        }
    }// end of if

    // select shortest path
    hc_cc_rs::path_type best_path_type = static_cast<hc_cc_rs::path_type>(math::ArrayIndexMin(length, nb_hc_cc_rs_paths));
    HC_CC_RS_Path *path;
    path = new HC_CC_RS_Path(c1.getStart(), c2.getStart(),
                             best_path_type, _kappa, _sigma, length[best_path_type],
                             qi1[best_path_type], qi2[best_path_type],
                             qi3[best_path_type], qi4[best_path_type],
                             cstart[best_path_type], cend[best_path_type],
                             ci1[best_path_type], ci2[best_path_type]);
    for (uint16_t i = 0; i < nb_hc_cc_rs_paths; i++)
    {
        if(i != best_path_type)
        {
            delete qi1[i];
            delete qi2[i];
            delete qi3[i];
            delete qi4[i];
            delete cstart[i];
            delete cend[i];
            delete ci1[i];
            delete ci2[i];
        }
    }
    return path;
}

/**
 * @brief Returns a sequence of turns and straight lines connecting a start and an end configuration
 * @param state1 :the start state
 * @param state2 :the end state
 * @return a sequence of turns and straight lines
 */
HC_CC_RS_Path* HCPMPM_ReedsSheppStateSpace::HCPMPM_ReedsSheppPath(const State &state1, const State &state2) const
{
    // define the start and end configuration point
    Configuration start1(state1.x, state1.y, state1.psi, _kappa);
    Configuration start2(state1.x, state1.y, state1.psi, -_kappa);
    Configuration end1(state2.x, state2.y, state2.psi, _kappa);
    Configuration end2(state2.x, state2.y, state2.psi, -_kappa);

    // compute the four circles at the start and end configuration
    HC_CC_Circle *start_circle[4];
    HC_CC_Circle *end_circle[4];

    start_circle[0] = new HC_CC_Circle(start1, true,  true,  true, _rs_circle_param);
    start_circle[1] = new HC_CC_Circle(start2, false, true,  true, _rs_circle_param);
    start_circle[2] = new HC_CC_Circle(start1, true,  false, true, _rs_circle_param);
    start_circle[3] = new HC_CC_Circle(start2, false, false, true, _rs_circle_param);

    end_circle[0] = new HC_CC_Circle(end1, true,  true,  true, _rs_circle_param);
    end_circle[1] = new HC_CC_Circle(end2, false, true,  true, _rs_circle_param);
    end_circle[2] = new HC_CC_Circle(end1, true,  false, true, _rs_circle_param);
    end_circle[3] = new HC_CC_Circle(end2, false, false, true, _rs_circle_param);

    // compute the shortest path for the 16 combinations (4 circles at the beginning and 4 at the end)
    HC_CC_RS_Path *path[] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
                              nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };

    double lg[] = { numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(),
                    numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(),
                    numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(),
                    numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(),
                    numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(),
                    numeric_limits<double>::max() };

    for (uint16_t i = 0; i < 4; i++)
    {
        // skip circle at the end for curvature continuity
        if( i == 0 && state1.kappa < 0 )
        {
            continue;
        }
        else if( i == 1 && state1.kappa > 0 )
        {
            continue;
        }
        else if( i == 2 && state1.kappa < 0 )
        {
            continue;
        }
        else if( i == 3 && state1.kappa > 0 )
        {
            continue;
        }
        else
        {
            for (uint16_t j = 0; j < 4; j++)
            {
                // skip circle at the end for curvature continuity
                if( j == 0 && state2.kappa < 0 )
                {
                    continue;
                }
                else if( j == 1 && state2.kappa > 0 )
                {
                    continue;
                }
                else if( j == 2 && state2.kappa < 0 )
                {
                    continue;
                }
                else if( j == 3 && state2.kappa > 0 )
                {
                    continue;
                }
                else
                {
                    path[4 * i + j] = this->HCPMPM_CirclesReedsSheppPath(*start_circle[i], *end_circle[j]);
                    if (path[4 * i + j])
                    {
                        lg[4 * i + j] = path[4 * i + j]->getLength();
                    }
                }
            }
        }
    }

    // select shortest path
    uint16_t best_path_index = math::ArrayIndexMin(lg, 16);

    // clean up
    for (uint16_t i = 0; i < 4; i++)
    {
        delete start_circle[i];
        delete end_circle[i];
    }
    for (uint16_t i = 0; i < 16; i++)
    {
        if( i != best_path_index )
        {
            delete path[i];
        }
    }
    return path[best_path_index];
}

/**
 * @brief Returns shortest path length from state1 to state2
 * @param state1 :the start state
 * @param state2 :the end state
 * @return
 */
double HCPMPM_ReedsSheppStateSpace::getDistance(const State &state1, const State &state2) const
{
    HC_CC_RS_Path *path = this->HCPMPM_ReedsSheppPath(state1 , state2);
    double length = path->getLength();
    delete path;
    return length;
}

/**
 * @brief Returns controls of the shortest path from state1 to state2
 * @param state1 :the start state
 * @param state2 :the end state
 * @return
 */
vector<Control> HCPMPM_ReedsSheppStateSpace::getControls(const State &state1, const State &state2) const
{
    vector<Control> hc_rs_controls;
    hc_rs_controls.reserve(8);
    HC_CC_RS_Path *path = this->HCPMPM_ReedsSheppPath(state1, state2);
    switch (path->getType())
    {
        case hc_cc_rs::E:
            steering::EmptyControls(hc_rs_controls);
            break;

        case hc_cc_rs::T:
            steering::RS_TurnControls(*(path->getCircleStart()), path->getEnd(), true, hc_rs_controls);
            break;

        case hc_cc_rs::TT:
            steering::HC_TurnControls(*(path->getCircleStart()), *(path->getQi1()), false, hc_rs_controls);
            steering::HC_TurnControls(*(path->getCircleEnd()), *(path->getQi3()), true, hc_rs_controls);
            break;

        case hc_cc_rs::TcT:
            steering::RS_TurnControls(*(path->getCircleStart()), *(path->getQi1()), true, hc_rs_controls);
            steering::RS_TurnControls(*(path->getCircleEnd()), *(path->getQi1()), false, hc_rs_controls);
            break;
        /************************ Reeds-Shepp families: ************************/
        case hc_cc_rs::TcTcT:
            steering::RS_TurnControls(*(path->getCircleStart()), *(path->getQi1()), true, hc_rs_controls);
            steering::RS_TurnControls(*(path->getCi1()), *(path->getQi2()), true, hc_rs_controls);
            steering::RS_TurnControls(*(path->getCircleEnd()), *(path->getQi2()), false, hc_rs_controls);
            break;

        case hc_cc_rs::TcTT:
            steering::RS_TurnControls(*(path->getCircleStart()), *(path->getQi1()), true, hc_rs_controls);
            steering::HC_TurnControls(*(path->getCi1()), *(path->getQi1()), false, hc_rs_controls);
            steering::HC_TurnControls(*(path->getCircleEnd()), *(path->getQi2()), true, hc_rs_controls);
            break;

        case hc_cc_rs::TTcT:
            steering::HC_TurnControls(*(path->getCircleStart()), *(path->getQi1()), false, hc_rs_controls);
            steering::HC_TurnControls(*(path->getCi1()), *(path->getQi2()), true, hc_rs_controls);
            steering::RS_TurnControls(*(path->getCircleEnd()), *(path->getQi2()), false, hc_rs_controls);
            break;

        case hc_cc_rs::TST:
            steering::HC_TurnControls(*(path->getCircleStart()), *(path->getQi1()), false, hc_rs_controls);
            steering::StraightControls(*(path->getQi2()), *(path->getQi3()), hc_rs_controls);
            steering::HC_TurnControls(*(path->getCircleEnd()), *(path->getQi4()), true, hc_rs_controls);
            break;

        case hc_cc_rs::TSTcT:
            steering::HC_TurnControls(*(path->getCircleStart()), *(path->getQi1()), false, hc_rs_controls);
            steering::StraightControls(*(path->getQi2()), *(path->getQi3()), hc_rs_controls);
            steering::HC_TurnControls(*(path->getCi1()), *(path->getQi4()), true, hc_rs_controls);
            steering::RS_TurnControls(*(path->getCircleEnd()), *(path->getQi4()), false, hc_rs_controls);
            break;

        case hc_cc_rs::TcTST:
            steering::RS_TurnControls(*(path->getCircleStart()), *(path->getQi1()), true, hc_rs_controls);
            steering::HC_TurnControls(*(path->getCi1()), *(path->getQi1()), false, hc_rs_controls);
            steering::StraightControls(*(path->getQi2()), *(path->getQi3()), hc_rs_controls);
            steering::HC_TurnControls(*(path->getCircleEnd()), *(path->getQi4()), true, hc_rs_controls);
            break;

        case hc_cc_rs::TcTSTcT:
            steering::RS_TurnControls(*(path->getCircleStart()), *(path->getQi1()), true, hc_rs_controls);
            steering::HC_TurnControls(*(path->getCi1()), *(path->getQi1()), false, hc_rs_controls);
            steering::StraightControls(*(path->getQi2()), *(path->getQi3()), hc_rs_controls);
            steering::HC_TurnControls(*(path->getCi2()), *(path->getQi4()), true, hc_rs_controls);
            steering::RS_TurnControls(*(path->getCircleEnd()), *(path->getQi4()), false, hc_rs_controls);
            break;

        case hc_cc_rs::TTcTT:
            steering::HC_TurnControls(*(path->getCircleStart()), *(path->getQi1()), false, hc_rs_controls);
            steering::HC_TurnControls(*(path->getCi1()), *(path->getQi2()), true, hc_rs_controls);
            steering::HC_TurnControls(*(path->getCi2()), *(path->getQi2()), false, hc_rs_controls);
            steering::HC_TurnControls(*(path->getCircleEnd()), *(path->getQi3()), false, hc_rs_controls);
            break;

        case hc_cc_rs::TcTTcT:
            steering::RS_TurnControls(*(path->getCircleStart()), *(path->getQi1()), true, hc_rs_controls);
            steering::HC_TurnControls(*(path->getCi1()), *(path->getQi1()), false, hc_rs_controls);
            steering::HC_TurnControls(*(path->getCi2()), *(path->getQi2()), true, hc_rs_controls);
            steering::RS_TurnControls(*(path->getCircleEnd()), *(path->getQi2()), false, hc_rs_controls);
            break;
        /************************ Additional Families: ************************/
        case hc_cc_rs::TTT:
            steering::HC_TurnControls(*(path->getCircleStart()), *(path->getQi1()), false, hc_rs_controls);
            steering::CC_TurnControls(*(path->getCi1()), *(path->getQi2()), true, hc_rs_controls);
            steering::HC_TurnControls(*(path->getCircleEnd()), *(path->getQi3()), true, hc_rs_controls);
            break;

        case hc_cc_rs::TcST:
            steering::RS_TurnControls(*(path->getCircleStart()), *(path->getQi1()), true, hc_rs_controls);
            steering::StraightControls(*(path->getQi1()), *(path->getQi2()), hc_rs_controls);
            steering::HC_TurnControls(*(path->getCircleEnd()), *(path->getQi3()), true, hc_rs_controls);
            break;

        case hc_cc_rs::TScT:
            steering::HC_TurnControls(*(path->getCircleStart()), *(path->getQi1()), false, hc_rs_controls);
            steering::StraightControls(*(path->getQi2()), *(path->getQi3()), hc_rs_controls);
            steering::RS_TurnControls(*(path->getCircleEnd()), *(path->getQi3()), false, hc_rs_controls);
            break;

        case hc_cc_rs::TcScT:
            steering::RS_TurnControls(*(path->getCircleStart()), *(path->getQi1()), true, hc_rs_controls);
            steering::StraightControls(*(path->getQi1()), *(path->getQi2()), hc_rs_controls);
            steering::RS_TurnControls(*(path->getCircleEnd()), *(path->getQi2()), false, hc_rs_controls);
            break;

        default:
            break;
    }
    delete path;
    return hc_rs_controls;
}
