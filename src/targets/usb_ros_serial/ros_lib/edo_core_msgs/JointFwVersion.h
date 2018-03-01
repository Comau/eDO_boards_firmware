#ifndef _ROS_edo_core_msgs_JointFwVersion_h
#define _ROS_edo_core_msgs_JointFwVersion_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace edo_core_msgs
{

  class JointFwVersion : public ros::Msg
  {
    public:
      typedef uint8_t _id_type;
      _id_type id;
      typedef uint8_t _majorRev_type;
      _majorRev_type majorRev;
      typedef uint16_t _minorRev_type;
      _minorRev_type minorRev;
      typedef uint16_t _revision_type;
      _revision_type revision;
      typedef uint16_t _svn_type;
      _svn_type svn;

    JointFwVersion():
      id(0),
      majorRev(0),
      minorRev(0),
      revision(0),
      svn(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      *(outbuffer + offset + 0) = (this->majorRev >> (8 * 0)) & 0xFF;
      offset += sizeof(this->majorRev);
      *(outbuffer + offset + 0) = (this->minorRev >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->minorRev >> (8 * 1)) & 0xFF;
      offset += sizeof(this->minorRev);
      *(outbuffer + offset + 0) = (this->revision >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->revision >> (8 * 1)) & 0xFF;
      offset += sizeof(this->revision);
      *(outbuffer + offset + 0) = (this->svn >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->svn >> (8 * 1)) & 0xFF;
      offset += sizeof(this->svn);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
      this->majorRev =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->majorRev);
      this->minorRev =  ((uint16_t) (*(inbuffer + offset)));
      this->minorRev |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->minorRev);
      this->revision =  ((uint16_t) (*(inbuffer + offset)));
      this->revision |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->revision);
      this->svn =  ((uint16_t) (*(inbuffer + offset)));
      this->svn |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->svn);
     return offset;
    }

    const char * getType(){ return "edo_core_msgs/JointFwVersion"; };
    const char * getMD5(){ return "729b5dd99bc689dc95476cf527db5fa4"; };

  };

}
#endif