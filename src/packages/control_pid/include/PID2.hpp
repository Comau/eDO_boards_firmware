/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <float.h>
#include <math.h>

#define PI_GRECO        3.14159265358979f
#define ANGOLO_PIATTO   180.0f
#define ONE_FLOAT       1.0f
#define TWO_FLOAT       2.0f
#define ZERO_FLOAT      0.0f
#define QUASIZERO_FLOAT 0.00000001f
#define COUNTER_MAX     5000
#define KF_THRS_IST1    0.7f
#define KF_THRS_IST2    1.3f
#define KF_THRS_IST3    1.0f
#define KF_THRS_IST4    0.7f
#define KF_THRS_IST5    0.7f
#define KF_THRS_IST6    0.7f
#define MAX_THRS        1000.0f

#define TH_FE               186.0f   // (SP_ATTENTION: Changing the max velocity will change these parameters)
#define TH_VELOCITY_ARM     590.00f  // Max_Velocity + 20% //[Rad/s Motor]
#define TH_VELOCITY_WRIST   700.00f  // Max_Velocity + 20% //[Rad/s Motor]
#define TX_RATE_SELECTION   1231.0f

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
    //------- COLLISION DETECTION ------//
    int   _curfilt_cnt      ;
    float _FiltCurDyn_Regr2 ;
    float _FiltCurDyn_Regr1 ;
    float _FiltCurMis_Regr2 ;
    float _FiltCurMis_Regr1 ;   // DA INIZIALIZZARE
    
    

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

    inline float update(float measure, float velocity, float current, float reduction_ratio, int brk_sts, bool *ErrFlag);

    enum select_i { ALL_I, POS_I_ONLY };

    inline void reset_i(select_i sel_i = ALL_I);

    inline void reset_trgFilter();
    
    inline void reset_collDetFilter();

    inline void hold_pos(bool enable);

    inline void enable_pos(bool enable, uint32_t enable_time);
    
    inline float collDetect(float vr_CurMis, float vr_CurDyn, bool *flag, float *vr_CurMisFilt, float *vr_CurDynFilt, int vr_identity, float vr_curr_limit);

};


PID2::PID2(void)
{ _curfilt_cnt        = 0;
  _FiltCurDyn_Regr2   = 0.0f;
  _FiltCurDyn_Regr1   = 0.0f;
  _FiltCurMis_Regr2   = 0.0f;
  _FiltCurMis_Regr1   = 0.0f;
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
  _ktorque            = 0;
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
    _min = -4902;           // min;  %    _min = -46816;           // min;
    _max = 4902;            // max;  %    _max = 46816;            // max;
    _i   = 0;
    _d   = 0;
    _k_ff = 0.0;             // kff;
    //_saturator = sat;
    _saturator_s = 10000000; // sat;

    _kv   = 0.01;            // k2;
    _kiv  = 0.005;           // ti2;
    _kdv  = 0.0;             // td2;
    _minv = -12;          // min2;  %    _minv = -93631;          // min2
    _maxv = 12;           // max2;  %    _maxv = 93631;           // max2;
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

// Reset collision detection filter memory
void PID2::reset_collDetFilter()
{

  _curfilt_cnt = 0;

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
float PID2::update(float meas_angle,float meas_vel,float meas_curr,float reduction_ratio,int brk_sts,bool *ErrFlag)
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
	float vr_FollowingError;
	float vr_velocityCheck;
	float vr_ThresholdVelocity;
	bool  vb_Err;

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
		
      /* --------------------- SAFETYCHECK VELOCITY & FOLLOWING ERROR ----------------- */ 
	      vr_velocityCheck = (vr_PosRef-vr_PosRef_1)/vr_TsLoop;  //[Rad/s motore]
		  vr_FollowingError = vr_PosRef - vr_PosMis; //[Rad_mot]
		  
		  
		  if(fabs(reduction_ratio)>= TX_RATE_SELECTION)
		  {
		    vr_ThresholdVelocity = TH_VELOCITY_ARM;
		  }
		  else
		  {
		    vr_ThresholdVelocity = TH_VELOCITY_WRIST;
		  }
		  
		  if(fabs(vr_velocityCheck) > vr_ThresholdVelocity  || fabs(vr_FollowingError) > TH_FE)
		  {
		    vb_Err = true;
		  }
	      else
		  {
		    vb_Err = false;
		  }
		  
      /* -------------------------------- OUTPUT --------------------------- */

        /* Normalize between -1.0 and 1.0 */
          vr_CntrOutput = vr_CurOutput / _maxt;

        /* output */
		  *ErrFlag  = vb_Err;
          return vr_CntrOutput;

} // PID::update

float PID2::collDetect(float vr_CurMis, float vr_CurDyn, bool *flag, float *vr_CurMisFilt, float *vr_CurDynFilt, int vr_identity, float vr_curr_limit)
{

    /* -------------------------------- DECLARATION --------------------------- */
    
    float vr_TsMis;
    float vr_FiltCurDyn_Freq;
    float vr_FiltCurMis_Freq;
    float vr_FiltCurDyn_a1;
    float vr_FiltCurDyn_a2;
    float vr_FiltCurDyn_ExpCoef ;
    float vr_FiltCurDyn_ExpValue;
    float vr_FiltCurMis_a1;
    float vr_FiltCurMis_a2;
    float vr_FiltCurMis_ExpCoef ;
    float vr_FiltCurMis_ExpValue;   
    float vr_FiltCurDyn_1;
    float vr_FiltCurDyn_2;
    float vr_CurFiltDyn  ;
    float vr_FiltCurMis_1;
    float vr_FiltCurMis_2;
    float vr_CurFiltMis  ;
    float vr_CurRes;   
    float vr_IstError;
    bool  vb_ContFlag;
    bool  vb_IstFlag;
    bool  vb_CollisionFlag;
	float vr_Kf_Thrs_Ist;
    
    
    /* -------------------------------- SETTINGS --------------------------- */
      /* Sample Time */
        vr_TsMis = 0.010f;

      /* Cur Dyn Freq */
        vr_FiltCurDyn_Freq = 0.78f; // definito in questa funzione
        
      /* Cur Mis Freq */
        vr_FiltCurMis_Freq = 2.0f; // definito in questa funzione
        
	  if(vr_curr_limit == 0)
	  {
		if(_ktorque > 0){
			vr_Kf_Thrs_Ist = _ktorque;
		}
		else if(_ktorque == 0){
			vr_Kf_Thrs_Ist = MAX_THRS;
		}
		else{
			*flag=false;
			return 0.0f;
		}
	  }
	  else if(vr_curr_limit < 0){
		vr_Kf_Thrs_Ist = MAX_THRS;  
	  }
		  
	  else{
		vr_Kf_Thrs_Ist = vr_curr_limit;
	  }
	  
#if 0  
	  else{
	  /* CUSTOMIZZAZIONE SOGLIE    */
		if (vr_identity!=-1)
		{
			switch (vr_identity){
				
				case (1):
				vr_Kf_Thrs_Ist = KF_THRS_IST1;
				break;
				case (2):
				vr_Kf_Thrs_Ist = KF_THRS_IST2;
				break;
				case (3):
				vr_Kf_Thrs_Ist = KF_THRS_IST3;
				break;
				case (4):
				vr_Kf_Thrs_Ist = KF_THRS_IST4;
				break;
				case (5):
				vr_Kf_Thrs_Ist = KF_THRS_IST5;
				break;
				case (6):
				vr_Kf_Thrs_Ist = KF_THRS_IST6;
				break;
			}
		}
		else
		{
			*flag=false;
			return 0.0f;
		}
	  }
#endif
      /* IMPLEMETAZIONE DEI FILTRI    */
        /* Implementazione filtro su corrente modello dinamico       */   
        if (vr_FiltCurDyn_Freq <= QUASIZERO_FLOAT)
        {
          vr_FiltCurDyn_a1 = ZERO_FLOAT;
          vr_FiltCurDyn_a2 = ZERO_FLOAT;

        }
        else
        {
          vr_FiltCurDyn_ExpCoef  =     - TWO_FLOAT * PI_GRECO * vr_FiltCurDyn_Freq * vr_TsMis;
          vr_FiltCurDyn_ExpValue =                          vr_FiltCurDyn_ExpCoef/4.0 + 1.0;
          vr_FiltCurDyn_ExpValue = vr_FiltCurDyn_ExpValue * vr_FiltCurDyn_ExpCoef/3.0 + 1.0;
          vr_FiltCurDyn_ExpValue = vr_FiltCurDyn_ExpValue * vr_FiltCurDyn_ExpCoef/2.0 + 1.0;
          vr_FiltCurDyn_ExpValue = vr_FiltCurDyn_ExpValue * vr_FiltCurDyn_ExpCoef/1.0 + 1.0;
          vr_FiltCurDyn_a1       =                        - TWO_FLOAT * vr_FiltCurDyn_ExpValue;
          vr_FiltCurDyn_a2       =          vr_FiltCurDyn_ExpValue * vr_FiltCurDyn_ExpValue; 
        }
      
       /* Implementazione filtro su corrente misurata*/
       
        if (vr_FiltCurMis_Freq <= QUASIZERO_FLOAT)
        {
          vr_FiltCurMis_a1 = ZERO_FLOAT;
          vr_FiltCurMis_a2 = ZERO_FLOAT;

        }
        else
        {          
          vr_FiltCurMis_ExpCoef  =     - TWO_FLOAT * PI_GRECO * vr_FiltCurMis_Freq * vr_TsMis;
          vr_FiltCurMis_ExpValue =                          vr_FiltCurMis_ExpCoef/4.0 + 1.0;
          vr_FiltCurMis_ExpValue = vr_FiltCurMis_ExpValue * vr_FiltCurMis_ExpCoef/3.0 + 1.0;
          vr_FiltCurMis_ExpValue = vr_FiltCurMis_ExpValue * vr_FiltCurMis_ExpCoef/2.0 + 1.0;
          vr_FiltCurMis_ExpValue = vr_FiltCurMis_ExpValue * vr_FiltCurMis_ExpCoef/1.0 + 1.0;
          vr_FiltCurMis_a1       =                        - TWO_FLOAT * vr_FiltCurMis_ExpValue;
          vr_FiltCurMis_a2       =          vr_FiltCurMis_ExpValue * vr_FiltCurMis_ExpValue;          
        }

        _curfilt_cnt ++;
        if (_curfilt_cnt == 1)
        {
         /* 1st time: */ 
          _FiltCurDyn_Regr2  = vr_CurDyn;
          _FiltCurDyn_Regr1  = vr_CurDyn;
          vr_FiltCurDyn_1    = vr_CurDyn;
          vr_FiltCurDyn_2    = vr_CurDyn;
          vr_CurFiltDyn      = vr_CurDyn; 

          _FiltCurMis_Regr2  = vr_CurMis;
          _FiltCurMis_Regr1  = vr_CurMis;
          vr_FiltCurMis_1    = vr_CurMis;
          vr_FiltCurMis_2    = vr_CurMis;
          vr_CurFiltMis      = vr_CurMis;   

            // palSetPad(GPIOA,11);
            // palSetPad(GPIOA,12);
        }
        else
        {
          /* next */
           
         vr_FiltCurDyn_1   = _FiltCurDyn_Regr1;
         vr_FiltCurDyn_2   = _FiltCurDyn_Regr2;
         vr_CurFiltDyn     = (ONE_FLOAT + vr_FiltCurDyn_a1 + vr_FiltCurDyn_a2) * vr_CurDyn - vr_FiltCurDyn_a1 * vr_FiltCurDyn_1 - vr_FiltCurDyn_a2 * vr_FiltCurDyn_2;
         _FiltCurDyn_Regr2 = _FiltCurDyn_Regr1;
         _FiltCurDyn_Regr1 = vr_CurFiltDyn; 


         vr_FiltCurMis_1   = _FiltCurMis_Regr1;
         vr_FiltCurMis_2   = _FiltCurMis_Regr2;
         vr_CurFiltMis     = (ONE_FLOAT + vr_FiltCurMis_a1 + vr_FiltCurMis_a2) * vr_CurMis - vr_FiltCurMis_a1 * vr_FiltCurMis_1 - vr_FiltCurMis_a2 * vr_FiltCurMis_2;
         _FiltCurMis_Regr2 = _FiltCurMis_Regr1;
         _FiltCurMis_Regr1 = vr_CurFiltMis;          
            
            
            
    //    if (_curfilt_cnt > COUNTER_MAX)    GA_ATTENTION: va inserito???
    //    {
    //      _curfilt_cnt = COUNTER_MAX;
    //      // palClearPad(GPIOA,11);
    //      // palClearPad(GPIOA,12);
    //    }
        }
        
     // CALCOLO RESIDUI   
        vr_CurRes     = vr_CurFiltDyn - vr_CurFiltMis; 
        vr_IstError   = fabs(vr_CurRes);     

   
     // APPLICAZIONE DELLE SOGLIE
    
     // soglia su valore istantaneo 
        if (vr_IstError > vr_Kf_Thrs_Ist)  
        {
          vb_IstFlag = true; 
        }
        else
        {
          vb_IstFlag = false;           
        } // if (vr_IstError > vr_Kf_Thrs_Ist)
     // soglia sul valore continuo
        vb_ContFlag = false;
         
     // OR sulle soglie
        if (vb_ContFlag || vb_IstFlag)
        {  
          vb_CollisionFlag = true;
        }
        else
        {
          vb_CollisionFlag = false;  
        } // if wb_ContFlag | wb_IstFlag    
  
  *flag = vb_CollisionFlag;
  
  *vr_CurMisFilt = vr_CurFiltMis;
  
  *vr_CurDynFilt = vr_CurFiltDyn;
  
  return vr_IstError;        

} // PID2::collDetect
