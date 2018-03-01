#ifndef _ROS_edo_core_msgs_JointInit_h
#define _ROS_edo_core_msgs_JointInit_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace edo_core_msgs
{

  class JointInit : public ros::Msg
  {
    public:
      typedef uint8_t _mode_type;
      _mode_type mode;
      typedef uint64_t _joints_mask_type;
      _joints_mask_type joints_mask;
      typedef float _reduction_factor_type;
      _reduction_factor_type reduction_factor;

    JointInit():
      mode(0),
      joints_mask(0),
      reduction_factor(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      union {
        uint64_t real;
        uint32_t base;
      } u_joints_mask;
      u_joints_mask.real = this->joints_mask;
      *(outbuffer + offset + 0) = (u_joints_mask.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joints_mask.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joints_mask.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joints_mask.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joints_mask);
      union {
        float real;
        uint32_t base;
      } u_reduction_factor;
      u_reduction_factor.real = this->reduction_factor;
      *(outbuffer + offset + 0) = (u_reduction_factor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_reduction_factor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_reduction_factor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_reduction_factor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reduction_factor);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mode);
      union {
        uint64_t real;
        uint32_t base;
      } u_joints_mask;
      u_joints_mask.base = 0;
      u_joints_mask.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joints_mask.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joints_mask.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joints_mask.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joints_mask = u_joints_mask.real;
      offset += sizeof(this->joints_mask);
      union {
        float real;
        uint32_t base;
      } u_reduction_factor;
      u_reduction_factor.base = 0;
      u_reduction_factor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_reduction_factor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_reduction_factor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_reduction_factor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->reduction_factor = u_reduction_factor.real;
      offset += sizeof(this->reduction_factor);
     return offset;
    }

    const char * getType(){ return "edo_core_msgs/JointInit"; };
    const char * getMD5(){ return "3e68c8959b591478083b4be2f20df5d8"; };

  };

}
#endif