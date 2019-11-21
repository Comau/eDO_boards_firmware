/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <float.h>
#include <math.h>
#include <list>
#include <core/hw/GPIO.hpp>


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
#define TH_FE_FACTOR		1.0f     // 
#define TH_VELOCITY_ARM     492.00f  // Max_Velocity arm   //[Rad/s Motor]
#define TH_VELOCITY_WRIST   584.00f  // Max_Velocity wrist //[Rad/s Motor]
#define TH_VELOCITY_FACTOR  1.2f	 // Threshold exceeding max velocity, default +20% = 1.2
#define TX_RATE_SELECTION   1231.0f
#define SAMPLE_DELAY        4

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
    bool  _feed_fw_vel_enable;
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
    bool  _collisionFlag;
    float _curRes;
    //--------- SAFETY CHECK ------------//
	float _maxvel_limit;
	float _fllerr_limit;
	
    bool maxvelDetect(float vr_PosRef, float vr_PosRef_1, float vr_TsLoop, float reduction_ratio);
	bool fllerrDetect(float vr_PosRef, float vr_PosMis);

public:

    PID2(void);

    void config(float k, float ti, float td, float ts, float min, float max, float sat, float kff,
                       float k2,float ti2,float td2,float ts2,float min2,float max2,float sat2,float kff2,
                       float k3,float ti3,float td3,float ts3,float min3,float max3,float sat3,float kff3,
                       float ktorque);

    void config_unsafe(float k, float ti, float td, float ts, float min, float max, float sat, float kff,
                              float k2,float ti2,float td2,float ts2,float min2,float max2,float sat2,float kff2,
                              float k3,float ti3,float td3,float ts3,float min3,float max3,float sat3,float kff3);

    void set(float setpoint, float feedforward, float feedforwardt);
	
	float update(float meas_angle,float meas_vel,float meas_curr,float reduction_ratio,int brk_sts,bool *FllerrFlag,bool *MaxvelFlag);
    
    enum select_i { ALL_I, POS_I_ONLY };

    void reset_i(select_i sel_i = ALL_I);

    void reset_trgFilter();
    
    void reset_collDetFilter();

    void hold_pos(bool enable);

    void enable_pos(bool enable, uint32_t enable_time);
    
	void set_maxvel_threshold(float maxvel_limit);
	void set_fllerr_threshold(float fllerr_limit);
	
    float collDetect(float vr_CurMis, float vr_CurDyn, bool *flag, float *vr_CurMisFilt, float *vr_CurDynFilt, int vr_identity, float vr_curr_limit, float vr_res_diff_limit);
	
	/* Collision Double Check */
	float _cur_res_joint;
	float _cur_res_rasp;
	float _cur_diff_joint;
	std::list<float> _jntResList; 
};


