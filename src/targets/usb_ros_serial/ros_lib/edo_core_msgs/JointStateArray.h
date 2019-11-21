#ifndef _ROS_edo_core_msgs_JointStateArray_h
#define _ROS_edo_core_msgs_JointStateArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "edo_core_msgs/JointState.h"

namespace edo_core_msgs
{

  class JointStateArray : public ros::Msg
  {
    public:
      typedef uint64_t _joints_mask_type;
      _joints_mask_type joints_mask;
      uint32_t joints_length;
      typedef edo_core_msgs::JointState _joints_type;
      _joints_type st_joints;
      _joints_type * joints;

    JointStateArray():
      joints_mask(0),
      joints_length(0), joints(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joints_mask >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joints_mask >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joints_mask >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joints_mask >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->joints_mask >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->joints_mask >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->joints_mask >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->joints_mask >> (8 * 7)) & 0xFF;
      offset += sizeof(this->joints_mask);
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
      this->joints_mask =  ((uint64_t) (*(inbuffer + offset)));
      this->joints_mask |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->joints_mask |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->joints_mask |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->joints_mask |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->joints_mask |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->joints_mask |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->joints_mask |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->joints_mask);
      uint32_t joints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joints_length);
      if(joints_lengthT > joints_length)
        this->joints = (edo_core_msgs::JointState*)realloc(this->joints, joints_lengthT * sizeof(edo_core_msgs::JointState));
      joints_length = joints_lengthT;
      for( uint32_t i = 0; i < joints_length; i++){
      offset += this->st_joints.deserialize(inbuffer + offset);
        memcpy( &(this->joints[i]), &(this->st_joints), sizeof(edo_core_msgs::JointState));
      }
     return offset;
    }

    const char * getType(){ return "edo_core_msgs/JointStateArray"; };
    const char * getMD5(){ return "dafd61bc0f5113200f8439cc763f9222"; };

  };

}
#endif