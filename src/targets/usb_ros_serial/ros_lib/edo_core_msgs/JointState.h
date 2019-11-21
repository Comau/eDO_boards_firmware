#ifndef _ROS_edo_core_msgs_JointState_h
#define _ROS_edo_core_msgs_JointState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace edo_core_msgs
{

  class JointState : public ros::Msg
  {
    public:
      typedef float _position_type;
      _position_type position;
      typedef float _velocity_type;
      _velocity_type velocity;
      typedef float _current_type;
      _current_type current;
      typedef uint8_t _commandFlag_type;
      _commandFlag_type commandFlag;
      typedef float _R_jnt_type;
      _R_jnt_type R_jnt;

    JointState():
      position(0),
      velocity(0),
      current(0),
      commandFlag(0),
      R_jnt(0)
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
      *(outbuffer + offset + 0) = (this->commandFlag >> (8 * 0)) & 0xFF;
      offset += sizeof(this->commandFlag);
      union {
        float real;
        uint32_t base;
      } u_R_jnt;
      u_R_jnt.real = this->R_jnt;
      *(outbuffer + offset + 0) = (u_R_jnt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_R_jnt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_R_jnt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_R_jnt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->R_jnt);
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
      this->commandFlag =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->commandFlag);
      union {
        float real;
        uint32_t base;
      } u_R_jnt;
      u_R_jnt.base = 0;
      u_R_jnt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_R_jnt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_R_jnt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_R_jnt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->R_jnt = u_R_jnt.real;
      offset += sizeof(this->R_jnt);
     return offset;
    }

    const char * getType(){ return "edo_core_msgs/JointState"; };
    const char * getMD5(){ return "3cd6c44c6a24e5bc5cfdb4c30f51c3a1"; };

  };

}
#endif