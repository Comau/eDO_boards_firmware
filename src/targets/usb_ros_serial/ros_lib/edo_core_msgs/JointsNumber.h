#ifndef _ROS_SERVICE_JointsNumber_h
#define _ROS_SERVICE_JointsNumber_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace edo_core_msgs
{

static const char JOINTSNUMBER[] = "edo_core_msgs/JointsNumber";

  class JointsNumberRequest : public ros::Msg
  {
    public:

    JointsNumberRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return JOINTSNUMBER; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class JointsNumberResponse : public ros::Msg
  {
    public:
      typedef int8_t _counter_type;
      _counter_type counter;

    JointsNumberResponse():
      counter(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_counter;
      u_counter.real = this->counter;
      *(outbuffer + offset + 0) = (u_counter.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->counter);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_counter;
      u_counter.base = 0;
      u_counter.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->counter = u_counter.real;
      offset += sizeof(this->counter);
     return offset;
    }

    const char * getType(){ return JOINTSNUMBER; };
    const char * getMD5(){ return "7e601941327d71909b32e0a223b62a6e"; };

  };

  class JointsNumber {
    public:
    typedef JointsNumberRequest Request;
    typedef JointsNumberResponse Response;
  };

}
#endif
