#ifndef _ROS_edo_core_msgs_JointControl_h
#define _ROS_edo_core_msgs_JointControl_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace edo_core_msgs
{

  class JointControl : public ros::Msg
  {
    public:
      typedef float _position_type;
      _position_type position;
      typedef float _velocity_type;
      _velocity_type velocity;
      typedef float _current_type;
      _current_type current;
      typedef float _ff_velocity_type;
      _ff_velocity_type ff_velocity;
      typedef float _ff_current_type;
      _ff_current_type ff_current;
      typedef float _R_rasp_type;
      _R_rasp_type R_rasp;

    JointControl():
      position(0),
      velocity(0),
      current(0),
      ff_velocity(0),
      ff_current(0),
      R_rasp(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_position;
      u_position.real = this->position;
      *(outbuffer + offset + 0) = (u_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position);
      union {
        float real;
        uint32_t base;
      } u_velocity;
      u_velocity.real = this->velocity;
      *(outbuffer + offset + 0) = (u_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity);
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.real = this->current;
      *(outbuffer + offset + 0) = (u_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_ff_velocity;
      u_ff_velocity.real = this->ff_velocity;
      *(outbuffer + offset + 0) = (u_ff_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ff_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ff_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ff_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ff_velocity);
      union {
        float real;
        uint32_t base;
      } u_ff_current;
      u_ff_current.real = this->ff_current;
      *(outbuffer + offset + 0) = (u_ff_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ff_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ff_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ff_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ff_current);
      union {
        float real;
        uint32_t base;
      } u_R_rasp;
      u_R_rasp.real = this->R_rasp;
      *(outbuffer + offset + 0) = (u_R_rasp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_R_rasp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_R_rasp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_R_rasp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->R_rasp);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_position;
      u_position.base = 0;
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position = u_position.real;
      offset += sizeof(this->position);
      union {
        float real;
        uint32_t base;
      } u_velocity;
      u_velocity.base = 0;
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity = u_velocity.real;
      offset += sizeof(this->velocity);
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.base = 0;
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current = u_current.real;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_ff_velocity;
      u_ff_velocity.base = 0;
      u_ff_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ff_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ff_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ff_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ff_velocity = u_ff_velocity.real;
      offset += sizeof(this->ff_velocity);
      union {
        float real;
        uint32_t base;
      } u_ff_current;
      u_ff_current.base = 0;
      u_ff_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ff_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ff_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ff_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ff_current = u_ff_current.real;
      offset += sizeof(this->ff_current);
      union {
        float real;
        uint32_t base;
      } u_R_rasp;
      u_R_rasp.base = 0;
      u_R_rasp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_R_rasp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_R_rasp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_R_rasp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->R_rasp = u_R_rasp.real;
      offset += sizeof(this->R_rasp);
     return offset;
    }

    const char * getType(){ return "edo_core_msgs/JointControl"; };
    const char * getMD5(){ return "d7ea2ff52846d3da7658c7349ad8692b"; };

  };

}
#endif