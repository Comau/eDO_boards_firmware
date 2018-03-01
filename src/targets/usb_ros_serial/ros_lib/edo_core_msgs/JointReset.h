#ifndef _ROS_edo_core_msgs_JointReset_h
#define _ROS_edo_core_msgs_JointReset_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace edo_core_msgs
{

  class JointReset : public ros::Msg
  {
    public:
      typedef uint64_t _joints_mask_type;
      _joints_mask_type joints_mask;
      typedef uint32_t _disengage_steps_type;
      _disengage_steps_type disengage_steps;
      typedef float _disengage_offset_type;
      _disengage_offset_type disengage_offset;

    JointReset():
      joints_mask(0),
      disengage_steps(0),
      disengage_offset(0)
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
      *(outbuffer + offset + 0) = (this->disengage_steps >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->disengage_steps >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->disengage_steps >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->disengage_steps >> (8 * 3)) & 0xFF;
      offset += sizeof(this->disengage_steps);
      union {
        float real;
        uint32_t base;
      } u_disengage_offset;
      u_disengage_offset.real = this->disengage_offset;
      *(outbuffer + offset + 0) = (u_disengage_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_disengage_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_disengage_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_disengage_offset.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->disengage_offset);
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
      this->disengage_steps =  ((uint32_t) (*(inbuffer + offset)));
      this->disengage_steps |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->disengage_steps |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->disengage_steps |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->disengage_steps);
      union {
        float real;
        uint32_t base;
      } u_disengage_offset;
      u_disengage_offset.base = 0;
      u_disengage_offset.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_disengage_offset.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_disengage_offset.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_disengage_offset.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->disengage_offset = u_disengage_offset.real;
      offset += sizeof(this->disengage_offset);
     return offset;
    }

    const char * getType(){ return "edo_core_msgs/JointReset"; };
    const char * getMD5(){ return "a9a93ce1fc39c419654c782e78ed4d90"; };

  };

}
#endif