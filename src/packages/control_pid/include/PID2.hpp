/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <float.h>

#define PI_GRECO        3.14159265358979f
#define ANGOLO_PIATTO   180.0f
#define ONE_FLOAT       1.0f
#define TWO_FLOAT       2.0f
#define ZERO_FLOAT      0.0f
#define QUASIZERO_FLOAT 0.00000001f
#define COUNTER_MAX     5000

class PID2
{

private:
  //---------TARGET FILTER -------//
    int   _trgfilt_cnt;
    float _trgfilt_Regr1;
    float _trgfilt_Regr2;
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
    uint32_t    _enable_cntr;
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
    //----------TORQUE CONSTANT --------//
    float _ktorque;

public:

    inline PID2(void);

    inline void config(float k, float ti, float td, float ts, float min, float max, float sat, float kff,
                       float k2,float ti2,float td2,float ts2,float min2,float max2,float sat2,float kff2,
                       float k3,float ti3,float td3,float ts3,float min3,float max3,float sat3,float kff3,
                       float ktorque);

    inline void config_unsafe(float k, float ti, float td, float ts, float min, float max, float sat, float kff,
                              float k2,float ti2,float td2,float ts2,float min2,float max2,float sat2,float kff2,
                              float k3,float ti3,float td3,float ts3,float min3,float max3,float sat3,float kff3);

    inline void set(float setpoint, float feedforward, float feedforwardt);

    inline float update(float measure, float velocity, float current, int brk_sts);

    enum select_i { ALL_I, POS_I_ONLY };

    inline void reset_i(select_i sel_i = ALL_I);

    inline void reset_trgFilter();

    inline void hold_pos(bool enable);

    inline void enable_pos(bool enable, uint32_t enable_time);

};


PID2::PID2(void)
{
  _trgfilt_cnt        = 0;
  _trgfilt_Regr1      = 0;
  _trgfilt_Regr2      = 0;
  _i                  = 0;
  _d                  = 0;
  _setpoint           = 0;
  _k                  = 0;
  _k_ff               = 0;
  _ki                 = 0;
  _kd                 = 0;
  _min                = -FLT_MAX;
  _max                = FLT_MAX;
  _saturator          = 0;
  _saturator_s        = FLT_MAX;
  _feed_fw_vel_enable = true;
  _iv                 = 0;
  _dv                 = 0;
  _feedfwv            = 0;
  _kv                 = 0;
  _kiv                = 0;
  _kdv                = 0;
  _minv               = -FLT_MAX;
  _maxv               = -FLT_MAX;
  _k_ffv              = 0;
  _saturatorv         = 0;
  _it                 = 0;
  _dt                 = 0;
  _feedfwt            = 0;
  _kt                 = 0;
  _kit                = 0;
  _kdt                = 0;
  _mint               = -FLT_MAX;
  _maxt               = -FLT_MAX;
  _k_fft              = 0;
  _saturatort         = 0;
  _ktorque            = 0.00853;
}


// Configure
void PID2::config(float k, float ti, float td, float ts,  float min,  float max,  float sat,  float kff,
                  float k2,float ti2,float td2,float ts2, float min2, float max2, float sat2, float kff2,
                  float k3,float ti3,float td3,float ts3, float min3, float max3, float sat3, float kff3,
                  float ktorque)
{
    chSysLock();

    _trgfilt_cnt   = 0;
    _trgfilt_Regr1 = 0;
    _trgfilt_Regr2 = 0;

    _k           = k;
    _ki          = ti;
    _kd          = td;
    _min         = min;
    _max         = max;
    _i           = 0;
    _d           = 0;
    _k_ff        = kff;
    //_saturator = sat;
    _saturator_s = sat;

    _kv          = k2;
    _kiv         = ti2;
    _kdv         = td2;
    _minv        = min2;
    _maxv        = max2;
    _iv          = 0;
    _dv          = 0;
    _k_ffv       = kff2;
    _saturatorv  = sat2;

    _kt          = k3;
    _kit         = ti3;
    _kdt         = td3;
    _mint        = min3;
    _maxt        = max3;
    _it          = 0;
    _dt          = 0;
    _k_fft       = kff3;
    _saturatort  = sat3;

    _ktorque     = ktorque;

    chSysUnlock();
}

// Configure Unsafe
void PID2::config_unsafe(float k, float ti, float td, float ts, float min, float max, float sat, float kff,
                         float k2,float ti2,float td2,float ts2,float min2,float max2,float sat2,float kff2,
                         float k3,float ti3,float td3,float ts3,float min3,float max3,float sat3,float kff3)
{
    _trgfilt_cnt   = 0;
    _trgfilt_Regr1 = 0;
    _trgfilt_Regr2 = 0;

    _k   = 7.5;              // k;
    _ki  = 0.0;              // ti;
    _kd  = 0.0;              // td;
    _min = -46816;           // min;
    _max = 46816;            // max;
    _i   = 0;
    _d   = 0;
    _k_ff = 0.0;             // kff;
    //_saturator = sat;
    _saturator_s = 10000000; // sat;

    _kv   = 0.01;            // k2;
    _kiv  = 0.005;           // ti2;
    _kdv  = 0.0;             // td2;
    _minv = -93631;          // min2;
    _maxv = 93631;           // max2;
    _iv   = 0;
    _dv   = 0;
    _k_ffv = 0.0;            // kff2;
    _saturatorv = 100000;    // sat2;

    _kt   = 0.0;             // k3;
    _kit  = 0.0;             // ti3;
    _kdt  = 0.0;             // td3;
    _mint = -12;             // min3;
    _maxt = 12;              // max3;
    _it   = 0;
    _dt   = 0;
    _k_fft = 3.0;
    _saturatort = 100000;    // sat3;

}

// Set references
void PID2::set(float setpoint, float feedforward, float feedforwardt)
{
    chSysLock();

    _setpoint = setpoint;
    _feedfw   = feedforward;
    _feedfwv  = feedforwardt;

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
    _trgfilt_cnt = 0;
  }
  else
  {
    _i = _iv = _it = 0.0f;
    _trgfilt_cnt = 0;
  }
}

// Reset target filter memory
void PID2::reset_trgFilter()
{

  _trgfilt_cnt = 0;

}

// Open/close the position loop (hold current position/restore position control)
void PID2::hold_pos(bool enable)
{
  if (enable)
  { // Open the position loop
    _saturator = 0;
    _feed_fw_vel_enable = false;
  }
  else
  { // Close the position loop
    _saturator = _saturator_s;
    _feed_fw_vel_enable = true;
  }
}

// Enable/disable position loop
void PID2::enable_pos(bool enable, uint32_t enable_time)
{
  _enable_cntr = enable_time;
  if (enable)
  { // Enable the position loop
    if (_enable_cntr > 0)
    {
      _enable_cntr --;
    }
    else
    {
      _saturator = _saturator_s;
      _feed_fw_vel_enable = true;
    }
  }
  else
  { // Disable the position loop
    _saturator = 0;
    _feed_fw_vel_enable = false;
  }
}

// Computation
float PID2::update(float meas_angle,float meas_vel,float meas_curr,int brk_sts)
{

    /* -------------------------------- DECLARATION --------------------------- */

    float vr_ConvDeg2Rad;

    float vr_FltTrg_Freq;
    float vr_TsLoop;
    float vr_FltTrg_ExpCoef,vr_FltTrgExpValue;
    float vr_FltTrg_a1;
    float vr_FltTrg_a2;

    float vr_PosTrg;
    float vr_PosRef;
    float vr_PosRef_1;
    float vr_PosRef_2;

    float vr_PosError;
    float vr_PosEn;
    float vr_PosKp,vr_PosKi,vr_PosKd;
    float vr_PosPID, vr_PosOutput;
    float vr_PosMis;
    float vr_PosFeedForward,vr_FFv,vr_FFa;

    float vr_VelError;
    float vr_VelEn;
    float vr_VelKp,vr_VelKi,vr_VelKd;
    float vr_VelPID, vr_VelOutput;
    float vr_VelMis;

    float vr_CurError;
    float vr_CurEn;
    float vr_CurKp,vr_CurKi,vr_CurKd;
    float vr_CurPID, vr_CurOutput;
    float vr_CurMis;

    float vr_CntrOutput;

    /* -------------------------------- SETTINGS --------------------------- */

      /* Conversion Factors */
        vr_ConvDeg2Rad = PI_GRECO / ANGOLO_PIATTO;

      /* Sample Time */
        vr_TsLoop = 0.001f;

      /* Target Filter */
        vr_FltTrg_Freq = _k_fft; // 3.0
        if (vr_FltTrg_Freq <= QUASIZERO_FLOAT)
        {
          vr_FltTrg_a1 = ZERO_FLOAT;
          vr_FltTrg_a2 = ZERO_FLOAT;

        }
        else
        {
          vr_FltTrg_ExpCoef = - TWO_FLOAT * PI_GRECO * vr_FltTrg_Freq * vr_TsLoop;
          vr_FltTrgExpValue =                     vr_FltTrg_ExpCoef/4.0 + 1.0;
          vr_FltTrgExpValue = vr_FltTrgExpValue * vr_FltTrg_ExpCoef/3.0 + 1.0;
          vr_FltTrgExpValue = vr_FltTrgExpValue * vr_FltTrg_ExpCoef/2.0 + 1.0;
          vr_FltTrgExpValue = vr_FltTrgExpValue * vr_FltTrg_ExpCoef/1.0 + 1.0;
          vr_FltTrg_a1 =        - TWO_FLOAT * vr_FltTrgExpValue;
          vr_FltTrg_a2 =  vr_FltTrgExpValue * vr_FltTrgExpValue;
          // vr_FltTrg_a1 = -1.96269027044188f;
          // vr_FltTrg_a2 =  0.96303895842604f;
        }

      /* Position Loop: enabled with P */
        if (_k <= QUASIZERO_FLOAT)
        {
          vr_PosEn = ZERO_FLOAT;
          vr_PosKp = ONE_FLOAT;
        }
        else
        {
          vr_PosEn = ONE_FLOAT;
          vr_PosKp = _k;
        }
        vr_PosKi = _ki;
        vr_PosKd = _kd;
        vr_FFv = _k_ff;
        vr_FFa = _k_ffv * vr_TsLoop;

      /* Velocity Loop: enabled with PI */
        if (_kv <= QUASIZERO_FLOAT)
        {
          vr_VelEn = ZERO_FLOAT;
          vr_VelKp = ONE_FLOAT;
        }
        else
        {
          vr_VelEn = ONE_FLOAT;
          vr_VelKp = _kv;
        }
        vr_VelKi = _kiv;
        vr_VelKd = _kdv;

      /* Current Loop: disabled */
        if (_kt <= QUASIZERO_FLOAT)
        {
          vr_CurEn = ZERO_FLOAT;
          vr_CurKp = ONE_FLOAT;
        }
        else
        {
          vr_CurEn = ONE_FLOAT;
          vr_CurKp = _kt;
        }
        vr_CurKi = _kit;
        vr_CurKd = _kdt;

    /* -------------------------------- TARGET FILTER --------------------------- */

      /* regression */
        vr_PosTrg = vr_ConvDeg2Rad * _setpoint;
        _trgfilt_cnt = _trgfilt_cnt + 1;
        if (_trgfilt_cnt == 1)
        {
          /* 1st time: */
            _trgfilt_Regr2 = vr_PosTrg;
            _trgfilt_Regr1 = vr_PosTrg;
            vr_PosRef_1    = vr_PosTrg;
            vr_PosRef_2    = vr_PosTrg;
            vr_PosRef      = vr_PosTrg;
            // palSetPad(GPIOA,11);
            // palSetPad(GPIOA,12);
        }
        else
        {
          /* next */
            vr_PosRef_1 = _trgfilt_Regr1;
            vr_PosRef_2 = _trgfilt_Regr2;
            vr_PosRef = (ONE_FLOAT + vr_FltTrg_a1 + vr_FltTrg_a2) * vr_PosTrg - vr_FltTrg_a1 * vr_PosRef_1 - vr_FltTrg_a2 * vr_PosRef_2;
            _trgfilt_Regr2 = _trgfilt_Regr1;
            _trgfilt_Regr1 = vr_PosRef;
          if (_trgfilt_cnt > COUNTER_MAX)
          {
            _trgfilt_cnt = COUNTER_MAX;
            // palClearPad(GPIOA,11);
            // palClearPad(GPIOA,12);
          }
        }

    /* -------------------------------- POSITION LOOP --------------------------- */

      /* measurement */
        vr_PosMis = vr_ConvDeg2Rad * meas_angle;
      /* calculate error */
        vr_PosError = vr_PosRef - vr_PosEn * vr_PosMis;
      /* PID */
        /* proportional term */
          vr_PosPID = vr_PosKp * vr_PosError;
        /* integral term */
          _i = _i + vr_PosError;
          vr_PosPID = vr_PosPID + vr_PosKi * _i;
        /* derivative term */
          vr_PosPID = vr_PosPID + vr_PosKd * (vr_PosError - _d);
          _d = vr_PosError;
      /* saturation filter (before feedforward) */
        if (vr_PosPID > _saturator)
        {
          vr_PosPID = _saturator;
        }
        else if (vr_PosPID < -_saturator)
        {
          vr_PosPID = -_saturator;
        }
      /* feed forward velocity contribution after the position loop */
        if (_feed_fw_vel_enable)
        {
          vr_PosFeedForward = vr_FFv * (vr_PosRef-vr_PosRef_1)/vr_TsLoop + vr_FFa * (vr_PosRef-2*vr_PosRef_1+vr_PosRef_2)/(vr_TsLoop*vr_TsLoop);
          vr_PosOutput = vr_PosPID + 0.0f * vr_PosFeedForward;
        }
        else
        {
          vr_PosOutput = vr_PosPID;
        }
      /* saturation filter (after feedforward) */
        if (vr_PosOutput > _max)
        {
          vr_PosOutput = _max;
          _i = _i - vr_PosError; /* anti windup: cancel error integration */
        }
        else if (vr_PosOutput < _min)
        {
          vr_PosOutput = _min;
          _i = _i - vr_PosError; /* anti windup: cancel error integration */
        }

    /* -------------------------------- VELOCITY LOOP--------------------------- */

      /* measurement */
        vr_VelMis = vr_ConvDeg2Rad * meas_vel;
      /* calculate error */
        if (brk_sts > 0)
        { /* il sistema o e' gia' sfrenato o si ta sfrenando */
          vr_VelError = vr_PosOutput - vr_VelEn * vr_VelMis;
        }
        else
        { /* il sistema non e' ancora sfrenato */
          vr_VelError = 0.0f - vr_VelEn * vr_VelMis;
        }
      /* PID */
        /* proportional term */
          vr_VelPID = vr_VelKp * vr_VelError;
        /* integral term */
          _iv = _iv + vr_VelError;
          vr_VelPID = vr_VelPID + vr_VelKi * _iv;
        /* derivative term */
          vr_VelPID = vr_VelPID + vr_VelKd * (vr_VelError - _dv);
          _dv = vr_VelError;
      /* saturation filter (before feedforward) */
        if (vr_VelPID > _saturatorv)
        {
          vr_VelPID = _saturatorv;
        }
        else if (vr_VelPID < -_saturatorv)
        {
          vr_VelPID = -_saturatorv;
        }
      /* feed forward contribution after the velocity loop */
        vr_VelOutput = vr_VelPID;
      /* saturation filter (after feedforward) */
        if (vr_VelOutput > _maxv)
        {
          vr_VelOutput = _maxv;
          _iv = _iv - vr_VelError; /* anti windup: cancel error integration */
        }
        else if (vr_VelOutput < _minv)
        {
          vr_VelOutput = _minv;
          _iv = _iv - vr_VelError; /* anti windup: cancel error integration */
        }

    /* -------------------------------- CURRENT LOOP--------------------------- */

      /* measurement */
        vr_CurMis = meas_curr;
      /* calculate error */
        vr_CurError = vr_VelOutput - vr_CurEn * vr_CurMis;
      /* PID */
        /* proportional term */
          vr_CurPID = vr_CurKp * vr_CurError;
        /* integral term */
          _it = _it + vr_CurError;
          vr_CurPID = vr_CurPID + vr_CurKi * _it;
        /* derivative term */
          vr_CurPID = vr_CurPID + vr_CurKd * (vr_CurError - _dt);
          _dt = vr_CurError;
      /* saturation filter (before feedforward) */
        if (vr_CurPID > _saturatort)
        {
          vr_CurPID = _saturatort;
        }
        else if (vr_CurPID < -_saturatort)
        {
          vr_CurPID = -_saturatort;
        }
      /* saturation filter (after feedforward) */
        vr_CurOutput = vr_CurPID;
        if (vr_CurOutput > _maxt)
        {
          vr_CurOutput = _maxt;
          _it = _it - vr_CurError; /* anti windup: cancel error integration */
        }
        else if (vr_CurOutput < _mint)
        {
          vr_CurOutput = _mint;
          _it = _it - vr_CurError; /* anti windup: cancel error integration */
        }

      /* -------------------------------- OUTPUT --------------------------- */

        /* Normalize between -1.0 and 1.0 */
          vr_CntrOutput = vr_CurOutput / _maxt;

        /* output */
          return vr_CntrOutput;

} // PID::update
