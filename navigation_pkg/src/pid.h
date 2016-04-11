#ifndef _PID_H_
#define _PID_H_

//NOTE: CODE extracted from GITHUB (Bradley Snyder, bradley219)

/*******************PID Implementation Class*****************************************/

#include <iostream>
#include <cmath>
#include <ctime>

typedef struct{
    float PID_out;
    float PID_dt;
    float PID_err;
} PID_OUTPUT_TYPE;

class PIDImpl
{
    public:
        PIDImpl( float max, float min, float Kp, float Kd, float Ki );
        ~PIDImpl();
        PID_OUTPUT_TYPE calculate( float setpoint, float pv );

    private:
        std::clock_t _prevt;
        float       _max;
        float       _min;
        float       _Kp;
        float       _Kd;
        float       _Ki;
        float       _pre_error;
        float       _integral;
};

/*******************PID Interface Class*****************************************/

class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( float max, float min, float Kp, float Kd, float Ki );

        // Returns the manipulated variable given a setpoint and current process value
        PID_OUTPUT_TYPE calculate( float setpoint, float pv );
        ~PID();

    private:
        PIDImpl *pimpl;
};


/**
 * Implementation of PID
 */

PID::PID(float max, float min, float Kp, float Kd, float Ki )
{
    pimpl = new PIDImpl(max,min,Kp,Kd,Ki);
}
PID_OUTPUT_TYPE PID::calculate( float setpoint, float pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID() 
{
    delete pimpl;
}


PIDImpl::PIDImpl( float max, float min, float Kp, float Kd, float Ki ) :
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
    //store time_0
    _prevt = std::clock();
}

PID_OUTPUT_TYPE PIDImpl::calculate( float setpoint, float pv )
{
    // Calculate error
    float error = setpoint - pv;

    // Proportional term
    float Pout = _Kp * error;

    //time right now
    std::clock_t time_now = std::clock();
    //elapsed time from last PID loop
    float _dt = float(time_now - _prevt) / CLOCKS_PER_SEC; 

    // Integral term
    _integral += error * _dt;
    float Iout = _Ki * _integral;

    // Derivative term
    float derivative = (error - _pre_error) / _dt;
    float Dout = _Kd * derivative;

    // Calculate total output
    float output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;
    _prevt = time_now;

    //create return package
    PID_OUTPUT_TYPE output_;
    output_.PID_dt = _dt;
    output_.PID_out = output;
    output_.PID_err = error;
    return output_;
}

PIDImpl::~PIDImpl()
{
}

#endif