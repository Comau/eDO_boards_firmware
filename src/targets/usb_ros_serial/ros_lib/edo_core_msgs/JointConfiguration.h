#ifndef _ROS_edo_core_msgs_JointConfiguration_h
#define _ROS_edo_core_msgs_JointConfiguration_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace edo_core_msgs
{

  class JointConfiguration : public ros::Msg
  {
    public:
      typedef float _kp_type;
      _kp_type kp;
      typedef float _ti_type;
      _ti_type ti;
      typedef float _td_type;
      _td_type td;
      typedef float _sat_type;
      _sat_type sat;
      typedef float _kff_type;
      _kff_type kff;
      typedef float _max_type;
      _max_type max;
      typedef float _kpv_type;
      _kpv_type kpv;
      typedef float _tiv_type;
      _tiv_type tiv;
      typedef float _tdv_type;
      _tdv_type tdv;
      typedef float _satv_type;
      _satv_type satv;
      typedef float _kffv_type;
      _kffv_type kffv;
      typedef float _maxv_type;
      _maxv_type maxv;
      typedef float _kpt_type;
      _kpt_type kpt;
      typedef float _tit_type;
      _tit_type tit;
      typedef float _tdt_type;
      _tdt_type tdt;
      typedef float _satt_type;
      _satt_type satt;
      typedef float _kfft_type;
      _kfft_type kfft;
      typedef float _maxt_type;
      _maxt_type maxt;
      typedef float _kt_type;
      _kt_type kt;

    JointConfiguration():
      kp(0),
      ti(0),
      td(0),
      sat(0),
      kff(0),
      max(0),
      kpv(0),
      tiv(0),
      tdv(0),
      satv(0),
      kffv(0),
      maxv(0),
      kpt(0),
      tit(0),
      tdt(0),
      satt(0),
      kfft(0),
      maxt(0),
      kt(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_kp;
      u_kp.real = this->kp;
      *(outbuffer + offset + 0) = (u_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kp);
      union {
        float real;
        uint32_t base;
      } u_ti;
      u_ti.real = this->ti;
      *(outbuffer + offset + 0) = (u_ti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ti.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ti);
      union {
        float real;
        uint32_t base;
      } u_td;
      u_td.real = this->td;
      *(outbuffer + offset + 0) = (u_td.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_td.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_td.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_td.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->td);
      union {
        float real;
        uint32_t base;
      } u_sat;
      u_sat.real = this->sat;
      *(outbuffer + offset + 0) = (u_sat.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sat.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sat.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sat.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sat);
      union {
        float real;
        uint32_t base;
      } u_kff;
      u_kff.real = this->kff;
      *(outbuffer + offset + 0) = (u_kff.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kff.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kff.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kff.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kff);
      union {
        float real;
        uint32_t base;
      } u_max;
      u_max.real = this->max;
      *(outbuffer + offset + 0) = (u_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max);
      union {
        float real;
        uint32_t base;
      } u_kpv;
      u_kpv.real = this->kpv;
      *(outbuffer + offset + 0) = (u_kpv.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kpv.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kpv.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kpv.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kpv);
      union {
        float real;
        uint32_t base;
      } u_tiv;
      u_tiv.real = this->tiv;
      *(outbuffer + offset + 0) = (u_tiv.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tiv.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tiv.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tiv.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tiv);
      union {
        float real;
        uint32_t base;
      } u_tdv;
      u_tdv.real = this->tdv;
      *(outbuffer + offset + 0) = (u_tdv.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tdv.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tdv.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tdv.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tdv);
      union {
        float real;
        uint32_t base;
      } u_satv;
      u_satv.real = this->satv;
      *(outbuffer + offset + 0) = (u_satv.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_satv.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_satv.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_satv.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->satv);
      union {
        float real;
        uint32_t base;
      } u_kffv;
      u_kffv.real = this->kffv;
      *(outbuffer + offset + 0) = (u_kffv.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kffv.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kffv.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kffv.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kffv);
      union {
        float real;
        uint32_t base;
      } u_maxv;
      u_maxv.real = this->maxv;
      *(outbuffer + offset + 0) = (u_maxv.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_maxv.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_maxv.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_maxv.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->maxv);
      union {
        float real;
        uint32_t base;
      } u_kpt;
      u_kpt.real = this->kpt;
      *(outbuffer + offset + 0) = (u_kpt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kpt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kpt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kpt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kpt);
      union {
        float real;
        uint32_t base;
      } u_tit;
      u_tit.real = this->tit;
      *(outbuffer + offset + 0) = (u_tit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tit);
      union {
        float real;
        uint32_t base;
      } u_tdt;
      u_tdt.real = this->tdt;
      *(outbuffer + offset + 0) = (u_tdt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tdt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tdt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tdt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tdt);
      union {
        float real;
        uint32_t base;
      } u_satt;
      u_satt.real = this->satt;
      *(outbuffer + offset + 0) = (u_satt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_satt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_satt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_satt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->satt);
      union {
        float real;
        uint32_t base;
      } u_kfft;
      u_kfft.real = this->kfft;
      *(outbuffer + offset + 0) = (u_kfft.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kfft.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kfft.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kfft.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kfft);
      union {
        float real;
        uint32_t base;
      } u_maxt;
      u_maxt.real = this->maxt;
      *(outbuffer + offset + 0) = (u_maxt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_maxt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_maxt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_maxt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->maxt);
      union {
        float real;
        uint32_t base;
      } u_kt;
      u_kt.real = this->kt;
      *(outbuffer + offset + 0) = (u_kt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kt);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_kp;
      u_kp.base = 0;
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kp = u_kp.real;
      offset += sizeof(this->kp);
      union {
        float real;
        uint32_t base;
      } u_ti;
      u_ti.base = 0;
      u_ti.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ti.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ti.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ti.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ti = u_ti.real;
      offset += sizeof(this->ti);
      union {
        float real;
        uint32_t base;
      } u_td;
      u_td.base = 0;
      u_td.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_td.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_td.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_td.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->td = u_td.real;
      offset += sizeof(this->td);
      union {
        float real;
        uint32_t base;
      } u_sat;
      u_sat.base = 0;
      u_sat.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sat.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sat.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sat.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sat = u_sat.real;
      offset += sizeof(this->sat);
      union {
        float real;
        uint32_t base;
      } u_kff;
      u_kff.base = 0;
      u_kff.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kff.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kff.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kff.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kff = u_kff.real;
      offset += sizeof(this->kff);
      union {
        float real;
        uint32_t base;
      } u_max;
      u_max.base = 0;
      u_max.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max = u_max.real;
      offset += sizeof(this->max);
      union {
        float real;
        uint32_t base;
      } u_kpv;
      u_kpv.base = 0;
      u_kpv.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kpv.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kpv.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kpv.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kpv = u_kpv.real;
      offset += sizeof(this->kpv);
      union {
        float real;
        uint32_t base;
      } u_tiv;
      u_tiv.base = 0;
      u_tiv.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tiv.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tiv.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tiv.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tiv = u_tiv.real;
      offset += sizeof(this->tiv);
      union {
        float real;
        uint32_t base;
      } u_tdv;
      u_tdv.base = 0;
      u_tdv.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tdv.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tdv.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tdv.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tdv = u_tdv.real;
      offset += sizeof(this->tdv);
      union {
        float real;
        uint32_t base;
      } u_satv;
      u_satv.base = 0;
      u_satv.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_satv.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_satv.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_satv.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->satv = u_satv.real;
      offset += sizeof(this->satv);
      union {
        float real;
        uint32_t base;
      } u_kffv;
      u_kffv.base = 0;
      u_kffv.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kffv.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kffv.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kffv.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kffv = u_kffv.real;
      offset += sizeof(this->kffv);
      union {
        float real;
        uint32_t base;
      } u_maxv;
      u_maxv.base = 0;
      u_maxv.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_maxv.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_maxv.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_maxv.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->maxv = u_maxv.real;
      offset += sizeof(this->maxv);
      union {
        float real;
        uint32_t base;
      } u_kpt;
      u_kpt.base = 0;
      u_kpt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kpt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kpt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kpt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kpt = u_kpt.real;
      offset += sizeof(this->kpt);
      union {
        float real;
        uint32_t base;
      } u_tit;
      u_tit.base = 0;
      u_tit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tit = u_tit.real;
      offset += sizeof(this->tit);
      union {
        float real;
        uint32_t base;
      } u_tdt;
      u_tdt.base = 0;
      u_tdt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tdt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tdt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tdt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tdt = u_tdt.real;
      offset += sizeof(this->tdt);
      union {
        float real;
        uint32_t base;
      } u_satt;
      u_satt.base = 0;
      u_satt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_satt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_satt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_satt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->satt = u_satt.real;
      offset += sizeof(this->satt);
      union {
        float real;
        uint32_t base;
      } u_kfft;
      u_kfft.base = 0;
      u_kfft.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kfft.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kfft.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kfft.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kfft = u_kfft.real;
      offset += sizeof(this->kfft);
      union {
        float real;
        uint32_t base;
      } u_maxt;
      u_maxt.base = 0;
      u_maxt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_maxt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_maxt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_maxt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->maxt = u_maxt.real;
      offset += sizeof(this->maxt);
      union {
        float real;
        uint32_t base;
      } u_kt;
      u_kt.base = 0;
      u_kt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kt = u_kt.real;
      offset += sizeof(this->kt);
     return offset;
    }

    const char * getType(){ return "edo_core_msgs/JointConfiguration"; };
    const char * getMD5(){ return "53d67f707da52f98419b61a5961787e9"; };

  };

}
#endif