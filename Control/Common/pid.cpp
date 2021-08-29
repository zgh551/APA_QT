/*
 * PID.cpp
 *
 *  Created on: 2018年11月28日
 *      Author: Henry Zhu
 */

#include "Control/Common/pid.h"

PID::PID() {
//    KP.setContainer(this);
//    KP.getter(&PID::getKP);
//    KP.setter(&PID::setKP);

//    KI.setContainer(this);
//    KI.getter(&PID::getKI);
//    KI.setter(&PID::setKI);

//    KD.setContainer(this);
//    KD.getter(&PID::getKD);
//    KD.setter(&PID::setKD);

//    Desired.setContainer(this);
//    Desired.getter(&PID::getDesired);
//    Desired.setter(&PID::setDesired);

//    ILimit.setContainer(this);
//    ILimit.getter(&PID::getILimit);
//    ILimit.setter(&PID::setILimit);

//    OutputLimit.setContainer(this);
//    OutputLimit.getter(&PID::getOutputLimit);
//    OutputLimit.setter(&PID::setOutputLimit);

//    Dt.setContainer(this);
//    Dt.getter(&PID::getDt);
//    Dt.setter(&PID::setDt);

//    Threshold.setContainer(this);
//    Threshold.getter(&PID::getThreshold);
//    Threshold.setter(&PID::setThreshold);

    _desired = 0.0f;      //< set point
    _error = 0.0f;        //< error
    _prevError = 0.0f;    //< previous error
    _integ = 0.0f;        //< integral
    _deriv = 0.0f;        //< derivative
    _kp = 0.0f;           //< proportional gain
    _ki = 0.0f;           //< integral gain
    _kd = 0.0f;           //< derivative gain
    _iLimit = 3;       //< integral limit, absolute value. '0' means no limit.
    _outputLimit = 5;  //< total PID output limit, absolute value. '0' means no limit.
    _threshold = 0.0f;
    _dt = 0.0f;           //< delta-time dt
}

PID::PID(float dt,float kp,float ki,float kd,float i_lim,float out_lim) {
//    KP.setContainer(this);
//    KP.getter(&PID::getKP);
//    KP.setter(&PID::setKP);

//    KI.setContainer(this);
//    KI.getter(&PID::getKI);
//    KI.setter(&PID::setKI);

//    KD.setContainer(this);
//    KD.getter(&PID::getKD);
//    KD.setter(&PID::setKD);

//    Desired.setContainer(this);
//    Desired.getter(&PID::getDesired);
//    Desired.setter(&PID::setDesired);

//    ILimit.setContainer(this);
//    ILimit.getter(&PID::getILimit);
//    ILimit.setter(&PID::setILimit);

//    OutputLimit.setContainer(this);
//    OutputLimit.getter(&PID::getOutputLimit);
//    OutputLimit.setter(&PID::setOutputLimit);

//    Dt.setContainer(this);
//    Dt.getter(&PID::getDt);
//    Dt.setter(&PID::setDt);

//    Threshold.setContainer(this);
//    Threshold.getter(&PID::getThreshold);
//    Threshold.setter(&PID::setThreshold);

    _desired = 0.0f;      //< set point
    _error = 0.0f;        //< error
    _prevError = 0.0f;    //< previous error
    _integ = 0.0f;        //< integral
    _deriv = 0.0f;        //< derivative
    _kp = kp;           //< proportional gain
    _ki = ki;           //< integral gain
    _kd = kd;           //< derivative gain
    _iLimit = i_lim;       //< integral limit, absolute value. '0' means no limit.
    _outputLimit = out_lim;  //< total PID output limit, absolute value. '0' means no limit.
    _threshold = 0.0f;
    _dt = dt;           //< delta-time dt
}

PID::PID(float dt,float kp,float ki,float kd,float i_lim,float out_lim,float threshold) {
//    KP.setContainer(this);
//    KP.getter(&PID::getKP);
//    KP.setter(&PID::setKP);

//    KI.setContainer(this);
//    KI.getter(&PID::getKI);
//    KI.setter(&PID::setKI);

//    KD.setContainer(this);
//    KD.getter(&PID::getKD);
//    KD.setter(&PID::setKD);

//    Desired.setContainer(this);
//    Desired.getter(&PID::getDesired);
//    Desired.setter(&PID::setDesired);

//    ILimit.setContainer(this);
//    ILimit.getter(&PID::getILimit);
//    ILimit.setter(&PID::setILimit);

//    OutputLimit.setContainer(this);
//    OutputLimit.getter(&PID::getOutputLimit);
//    OutputLimit.setter(&PID::setOutputLimit);

//    Dt.setContainer(this);
//    Dt.getter(&PID::getDt);
//    Dt.setter(&PID::setDt);

//    Threshold.setContainer(this);
//    Threshold.getter(&PID::getThreshold);
//    Threshold.setter(&PID::setThreshold);

    _desired = 0.0f;      //< set point
    _error = 0.0f;        //< error
    _prevError = 0.0f;    //< previous error
    _integ = 0.0f;        //< integral
    _deriv = 0.0f;        //< derivative
    _kp = kp;           //< proportional gain
    _ki = ki;           //< integral gain
    _kd = kd;           //< derivative gain
    _iLimit = i_lim;       //< integral limit, absolute value. '0' means no limit.
    _outputLimit = out_lim;  //< total PID output limit, absolute value. '0' means no limit.
    _threshold = threshold;
    _dt = dt;           //< delta-time dt
}

PID::~PID() {

}

/****** Property ******/
/// KP
float PID::getKP()
{
    return _kp;
}
void PID::setKP(float value)
{
    _kp = value;
}

/// KI
float PID::getKI()
{
    return _ki;
}
void PID::setKI(float value)
{
    _ki = value;
}

/// KD
float PID::getKD()
{
    return _kd;
}
void PID::setKD(float value)
{
    _kd = value;
}

/// Desired
float PID::getDesired()
{
    return _desired;
}
void PID::setDesired(float value)
{
    _desired = value;
}

/// ILimit
float PID::getILimit()
{
    return _iLimit;
}
void PID::setILimit(float value)
{
    _iLimit = value;
}

/// OutputLimit
float PID::getOutputLimit()
{
    return _outputLimit;
}
void PID::setOutputLimit(float value)
{
    _outputLimit = value;
}

/// Dt
float PID::getDt()
{
    return _dt;
}
void PID::setDt(float value)
{
    _dt = value;
}

/// OutputOffset
float PID::getThreshold()
{
    return _threshold;
}
void PID::setThreshold(float value)
{
	_threshold = value;
}

/****** Function ******/
float PID::pidUpdate(float measured)
{
    float output = 0.0f;
    _error = _desired - measured;

    _outP = _kp * _error;
    output += _outP;

    _deriv = (_error - _prevError) / _dt;
    _outD = _kd * _deriv;
    output += _outD;

    _integ += _error * _dt;
    // Constrain the integral (unless the iLimit is zero)
    if(_iLimit != 0)
    {
    	_integ = fmin(_iLimit,fmax(-_iLimit,_integ));
    }
    _outI = _ki * _integ;
    output += _outI;

    // Constrain the total PID output (unless the outputLimit is zero)
    if(_outputLimit != 0)
    {
    	output = fmin(_outputLimit,fmax(-_outputLimit,output));
    }

    _prevError = _error;

    return output;
}

float PID::pidUpdateIntegralSeparation(float measured)
{
    float output = 0.0f;
    uint8_t index = 0;
    _error = _desired - measured;

    _outP = _kp * _error;
    output += _outP;

    _deriv = (_error - _prevError) / _dt;
    _outD = _kd * _deriv;
    output += _outD;

    //Integral separation
    if(_error < _threshold)
    {
    	index = 0.2;
    	_integ += _error * _dt;
    }
    else if(_error > _threshold)
    {
    	index = 1;
    	_integ += _error * _dt;
    }
    else
    {
    	index = 0;
    	_integ += _error * _dt;
    }

    // Constrain the integral (unless the iLimit is zero)
    if(_iLimit != 0)
    {
    	_integ = fmin(_iLimit,fmax(-_iLimit,_integ));
    }
    _outI = _ki * _integ * index;
    output += _outI;

    // Constrain the total PID output (unless the outputLimit is zero)
    if(_outputLimit != 0)
    {
    	output = fmin(_outputLimit,fmax(-_outputLimit,output));
    }

    _prevError = _error;

    return output;
}

float PID::pidUpdateIntegralSeparation_V1_0(float measured)
{
    float output = 0.0f;
    uint8_t index = 0;
    _error = _desired - measured;

    _outP = _kp * _error;
    output += _outP;

    _deriv = (_error - _prevError) / _dt;
    _outD = _kd * _deriv;
    output += _outD;

    //Integral separation
    if(_error > _threshold)
    {
    	index = 0;
    }
    else
    {
    	index = 1;
    	_integ += _error * _dt;
    }

    // Constrain the integral (unless the iLimit is zero)
    if(_iLimit != 0)
    {
    	_integ = fmin(_iLimit,fmax(-_iLimit,_integ));
    }
    _outI = _ki * _integ * index;
    output += _outI;

    // Constrain the total PID output (unless the outputLimit is zero)
    if(_outputLimit != 0)
    {
    	output = fmin(_outputLimit,fmax(-_outputLimit,output));
    }

    _prevError = _error;

    return output;
}

float PID::pidUpdateIntegralSeparation_V1_1(float measured)
{
    float output = 0.0f;

    _error = _desired - measured;

    _outP = _kp * _error;
    output += _outP;

    _deriv = (_error - _prevError) / _dt;
    _outD = _kd * _deriv;
    output += _outD;

    //Integral separation
    _integ += _ki * _error * _dt;

    // Constrain the integral (unless the iLimit is zero)
    if(_iLimit != 0)
    {
    	_integ = fmin(_iLimit,fmax(-_iLimit,_integ));
    }
    _outI = _integ;
    output += _outI;

    // Constrain the total PID output (unless the outputLimit is zero)
    if(_outputLimit != 0)
    {
    	output = fmin(_outputLimit,fmax(-_outputLimit,output));
    }

    _prevError = _error;

    return output;
}
