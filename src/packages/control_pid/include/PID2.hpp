/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <float.h>

class PID2
{
private:
	//---------POSITION LOOP -------//
    float _i;
    float _d;
    float _setpoint;
    float _feedfw;
    float _k;
    float _ki;
    float _kd;
    float _min;
    float _max;
    float _k_ff;
    float _saturator;
    float _saturator_s;
    //---------VELOCITY LOOP ------//
    float _iv;
    float _dv;
    float _feedfwv;
    float _kv;
    float _kiv;
    float _kdv;
    float _minv;
    float _maxv;
    float _k_ffv;
    float _saturatorv;
    bool _feed_fw_vel_enable;
    //----------TORQUE LOOP ------------//
    float _it;
    float _dt;
    float _feedfwt;
    float _kt;
    float _kit;
    float _kdt;
    float _mint;
    float _maxt;
    float _k_fft;
    float _saturatort;
	float _ktorque;


public:
    inline
    PID2(
        void
    );
    inline void config(float k,float ti,float td,float ts,float min,float max,float sat,float kff,float k2,float ti2,float td2,float ts2,float min2,
					   float max2,float sat2,float kff2,float k3,float ti3,float td3,float ts3,float min3,float max3,float sat3,float kff3,float ktorque);

 inline void
    config_unsafe(
        float k,
        float ti,
        float td,
        float ts,
        float min,
        float max,
		float sat,
		float kff,
	    float k2,
	    float ti2,
	    float td2,
	    float ts2,
		float min2,
		float max2,
		float sat2,
		float kff2,
		float k3,
	    float ti3,
	    float td3,
	    float ts3,
		float min3,
		float max3,
		float sat3,
		float kff3

    );

    inline void
    set(
        float setpoint,
		float feedforward,
		float feedforwardt
    );

    inline float
    update(
        float measure,
		float velocity,
		float current
    );

    enum select_i { ALL_I, POS_I_ONLY };

    inline void
    reset_i(
    	select_i sel_i = ALL_I
    );

    inline void
    hold_pos(
        bool enable
    );
};


PID2::PID2(
    void
)
{
    _i = 0;
    _d = 0;
    _setpoint = 0;
    _k   = 0;
    _k_ff = 0;
    _ki  = 0;
    _kd  = 0;
    _min = -FLT_MAX;
    _max = FLT_MAX;
    _saturator = 0;
    _saturator_s = FLT_MAX;
    _feed_fw_vel_enable = true;
    _iv=0;
    _dv=0;
    _feedfwv=0;
    _kv=0;
    _kiv=0;
    _kdv=0;
    _minv=-FLT_MAX;
    _maxv=-FLT_MAX;
    _k_ffv=0;
    _saturatorv=0;
    _it=0;
    _dt=0;
    _feedfwt=0;
    _kt=0;
    _kit=0;
    _kdt=0;
    _mint=-FLT_MAX;
    _maxt=-FLT_MAX;
    _k_fft=0;
    _saturatort=0;
	_ktorque = 0.00853;
}


void PID2::config(float k,float ti,float td,float ts,float min,float max,float sat,float kff,float k2,float ti2,float td2,float ts2,float min2,float max2,float sat2,float kff2,
				  float k3,float ti3,float td3,float ts3, float min3, float max3, float sat3, float kff3, float ktorque)
{
    chSysLock();
    _k   = k;
    _ki  = (ti == 0) ? 0 : k * (ts / ti);
    _kd  = k * (td / ts);
    _min = min;
    _max = max;
    _i   = 0;
    _d   = 0;
    _k_ff = kff;
    //_saturator = sat;
    _saturator_s = sat;
    _kv   = k2;
    _kiv  = (ti2 == 0) ? 0 : k2 * (ts / ti2);
    _kdv  = k2 * (td2 / ts);
    _minv = min2;
    _maxv = max2;
    _iv   = 0;
    _dv   = 0;
    _k_ffv = kff2;
    _saturatorv = sat2;
    _kt   = k3;
    _kit  = (ti3 == 0) ? 0 : k3 * (ts / ti3);
    _kdt  = k3 * (td3 / ts);
    _mint = min3;
    _maxt = max3;
    _it   = 0;
    _dt   = 0;
    _k_fft = kff3;
    _saturatort = sat3;
	_ktorque=ktorque;
    chSysUnlock();
}


void
PID2::config_unsafe(
    float k,
    float ti,
    float td,
    float ts,
	float min,
	float max,
	float sat,
	float kff,
    float k2,
    float ti2,
    float td2,
    float ts2,
	float min2,
	float max2,
	float sat2,
	float kff2,
	float k3,
    float ti3,
    float td3,
    float ts3,
	float min3,
	float max3,
	float sat3,
	float kff3
)
{
    _k   = k;
    _ki  = (ti == 0) ? 0 : k * (ts / ti);
    _kd  = k * (td / ts);
    _min = min;
    _max = max;
    _i   = 0;
    _d   = 0;
    _k_ff = kff;
    //_saturator = sat;
    _saturator_s = sat;
    _kv   = k2;
    _kiv  = (ti2 == 0) ? 0 : k2 * (ts / ti2);
    _kdv  = k2 * (td2 / ts);
    _minv = min2;
    _maxv = max2;
    _iv   = 0;
    _dv   = 0;
    _k_ffv = kff2;
    _saturatorv = sat2;
    _kt   = k3;
    _kit  = (ti3 == 0) ? 0 : k3 * (ts / ti3);
    _kdt  = k3 * (td3 / ts);
    _mint = min3;
    _maxt = max3;
    _it   = 0;
    _dt   = 0;
    _k_fft = kff3;
    _saturatort = sat3;
}



void
PID2::set(
    float setpoint,
	float feedforward,
	float feedforwardt
)
{
    chSysLock();

    _setpoint = setpoint;
    _feedfw = feedforward;
    _feedfwv = feedforwardt;
    chSysUnlock();
}

// Reset control integral terms
void
PID2::reset_i(
    select_i sel_i
)
{
	if (sel_i == POS_I_ONLY)
	{
		_i = 0.0f;
	}
	else
	{
		_i = _iv = _it = 0.0f;
	}
}

// Open/close the position loop (hold current position/restore position control)
void
PID2::hold_pos(
	bool enable
)
{
	if (enable) { // Open the position loop
		_saturator = 0;
		_feed_fw_vel_enable = false;
	} else { // Close the position loop
		_saturator = _saturator_s;
		_feed_fw_vel_enable = true;
	}
}

float
PID2::update(
    float meas_angle,
	float meas_vel,
	float meas_curr
)
{
    float error;
    float output;

    /* calculate error */
    error = _setpoint - meas_angle;

    /* proportional term */
    output = (_k * error);

    /* integral term */
    _i     += _ki * error;
    output += _i;

    /* derivative term */
    output += _kd * (error - _d);
    _d      = error;

    /* saturation filter */
    if (output > _saturator) {
        output = _saturator;
    } else if (output < -_saturator) {
        output = -_saturator;
    }


    /* feed forward velocity contribution after the position loop */
    if(_feed_fw_vel_enable)
	    output += _k_ff*_feedfw;

    if (output > _max) {
            output = _max;
            /* anti windup: cancel error integration */
            _i -= _ki * error;
        } else if (output < _min) {
            output = _min;
            /* anti windup: cancel error integration */
            _i -= _ki * error;
        }



    //-------------------------------- VELOCITY LOOP---------------------------//

    float errorv;

    /* calculate error */
    errorv = output - meas_vel;

    /* proportional term */
    output = (_kv * errorv);

    /* integral term */
    _iv     += _kiv * errorv;
    output += _iv;

    /* derivative term */
    output += _kdv * (errorv - _dv);
    _dv      = errorv;

    /* saturation filter */
    if (output > _saturatorv) {
        output = _saturatorv;
    } else if (output < -_saturatorv) {
        output = -_saturatorv;
    }



    output += _k_ffv*_feedfwv;

    if (output > _maxv) {
            output = _maxv;
            /* anti windup: cancel error integration */
            _iv -= _kiv * errorv;
        } else if (output < _minv) {
            output = _minv;
            /* anti windup: cancel error integration */
            _iv -= _kiv * errorv;
        }

    //-------------------------------- CURRENT LOOP---------------------------//

    float errort;

    // calculate error //
    errort = output - meas_curr * _ktorque;

    // proportional term //
    output = (_kt * errort);

    // integral term //
    _it     += _kit * errort;
    output += _it;

    // derivative term ù7/
    output += _kdt * (errorv - _dt);
    _dt      = errort;

    // saturation filter //
    if (output > _saturatort) {
        output = _saturatort;
    } else if (output < -_saturatort) {
        output = -_saturatort;
    }


    if (output > _maxt) {
            output = _maxt;
            // anti windup: cancel error integration
            _it -= _kit * errort;
        } else if (output < _mint) {
            output = _mint;
            //anti windup: cancel error integration //
            _it -= _kit * errort;
        }


    // Normalize between -1.0 and 1.0;
    output /= _maxt;


    return output;
} // PID::update
