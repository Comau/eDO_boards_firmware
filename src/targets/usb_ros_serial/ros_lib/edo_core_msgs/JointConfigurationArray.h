#ifndef _ROS_edo_core_msgs_JointConfigurationArray_h
#define _ROS_edo_core_msgs_JointConfigurationArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "edo_core_msgs/JointConfiguration.h"

namespace edo_core_msgs
{

  class JointConfigurationArray : public ros::Msg
  {
    public:
      typedef uint64_t _joints_mask_type;
      _joints_mask_type joints_mask;
      uint32_t joints_length;
      typedef edo_core_msgs::JointConfiguration _joints_type;
      _joints_type st_joints;
      _joints_type * joints;

    JointConfigurationArray():
      joints_mask(0),
      joints_length(0), joints(NULL)
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
      uint32_t joints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joints_length);
      if(joints_lengthT > joints_length)
        this->joints = (edo_core_msgs::JointConfiguration*)realloc(this->joints, joints_lengthT * sizeof(edo_core_msgs::JointConfiguration));
      joints_length = joints_lengthT;
      for( uint32_t i = 0; i < joints_length; i++){
      offset += this->st_joints.deserialize(inbuffer + offset);
        memcpy( &(this->joints[i]), &(this->st_joints), sizeof(edo_core_msgs::JointConfiguration));
      }
     return offset;
    }

    const char * getType(){ return "edo_core_msgs/JointConfigurationArray"; };
    const char * getMD5(){ return "ecfe2e24742d3b217f7dea8ef1addc54"; };

  };

}
#endif