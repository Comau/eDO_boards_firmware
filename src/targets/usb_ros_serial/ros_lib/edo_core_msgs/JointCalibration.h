#ifndef _ROS_edo_core_msgs_JointCalibration_h
#define _ROS_edo_core_msgs_JointCalibration_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace edo_core_msgs
{

  class JointCalibration : public ros::Msg
  {
    public:
      typedef uint64_t _joints_mask_type;
      _joints_mask_type joints_mask;

    JointCalibration():
      joints_mask(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
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
     return offset;
    }

    const char * getType(){ return "edo_core_msgs/JointCalibration"; };
    const char * getMD5(){ return "e3c733216f52667eed4e5d125e26029e"; };

  };

}
#endif