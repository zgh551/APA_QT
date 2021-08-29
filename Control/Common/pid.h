/*
 * PID.h
 *
 *  Created on: 2018年11月28日
 *      Author: Henry Zhu
 */

#ifndef PID_H_
#define PID_H_

//#include "derivative.h"
//#include "property.h"
#include <QMainWindow>
#include <math.h>

class PID {
public:
	PID();
	PID(float dt,float kp,float ki,float kd,float i_lim,float out_lim);
	PID(float dt,float kp,float ki,float kd,float i_lim,float out_lim,float threshold);
	virtual ~PID();
	float pidUpdate(float measured);
	float pidUpdateIntegralSeparation(float measured);
	float pidUpdateIntegralSeparation_V1_0(float measured);
	float pidUpdateIntegralSeparation_V1_1(float measured);
    /*** Variabel Property ***/
    /* KP */
    float getKP();
    void  setKP(float value);
//    Property<PID,float,READ_WRITE> KP;

    /* KI */
    float getKI();
    void  setKI(float value);
//    Property<PID,float,READ_WRITE> KI;

    /* KD */
    float getKD();
    void  setKD(float value);
//    Property<PID,float,READ_WRITE> KD;

    /* Desired */
    float getDesired();
    void  setDesired(float value);
//    Property<PID,float,READ_WRITE> Desired;

    /* ILimit */
    float getILimit();
    void  setILimit(float value);
//    Property<PID,float,READ_WRITE> ILimit;

    /* OutputLimit */
    float getOutputLimit();
    void  setOutputLimit(float value);
//    Property<PID,float,READ_WRITE> OutputLimit;

    /* Dt */
    float getDt();
    void  setDt(float value);
//    Property<PID,float,READ_WRITE> Dt;

    /* OutputOffset */
    float getThreshold();
    void  setThreshold(float value);
//    Property<PID,float,READ_WRITE> Threshold;
private:
    float _desired;      //< set point
    float _error;        //< error
    float _prevError;    //< previous error
    float _integ;        //< integral
    float _deriv;        //< derivative
    float _kp;           //< proportional gain
    float _ki;           //< integral gain
    float _kd;           //< derivative gain
    float _outP;         //< proportional output (debugging)
    float _outI;         //< integral output (debugging)
    float _outD;         //< derivative output (debugging)
    float _iLimit;       //< integral limit, absolute value. '0' means no limit.
    float _outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
    float _threshold; //< the output offset set
    float _dt;           //< delta-time dt
};

#endif /* PID_H_ */
