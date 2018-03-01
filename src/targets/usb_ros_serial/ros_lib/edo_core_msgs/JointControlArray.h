#ifndef _ROS_edo_core_msgs_JointControlArray_h
#define _ROS_edo_core_msgs_JointControlArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "edo_core_msgs/JointControl.h"

namespace edo_core_msgs
{

  class JointControlArray : public ros::Msg
  {
    public:
      typedef uint8_t _size_type;
      _size_type size;
      uint32_t joints_length;
      typedef edo_core_msgs::JointControl _joints_type;
      _joints_type st_joints;
      _joints_type * joints;

    JointControlArray():
      size(0),
      joints_length(0), joints(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->size >> (8 * 0)) & 0xFF;
      offset += sizeof(this->size);
      *(outbuffer + offset + 0) = (this->joints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joints_length);
      for( uint32_t i = 0; i < joints_length; i++){
      offset += this->joints[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->size =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->size);
      uint32_t joints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joints_length);
      if(joints_lengthT > joints_length)
        this->joints = (edo_core_msgs::JointControl*)realloc(this->joints, joints_lengthT * sizeof(edo_core_msgs::JointControl));
      joints_length = joints_lengthT;
      for( uint32_t i = 0; i < joints_length; i++){
      offset += this->st_joints.deserialize(inbuffer + offset);
        memcpy( &(this->joints[i]), &(this->st_joints), sizeof(edo_core_msgs::JointControl));
      }
     return offset;
    }

    const char * getType(){ return "edo_core_msgs/JointControlArray"; };
    const char * getMD5(){ return "fc08ec618ee557a2082dbd4320597984"; };

  };

}
#endif