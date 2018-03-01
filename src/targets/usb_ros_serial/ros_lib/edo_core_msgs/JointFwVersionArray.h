#ifndef _ROS_edo_core_msgs_JointFwVersionArray_h
#define _ROS_edo_core_msgs_JointFwVersionArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "edo_core_msgs/JointFwVersion.h"

namespace edo_core_msgs
{

  class JointFwVersionArray : public ros::Msg
  {
    public:
      uint32_t nodes_length;
      typedef edo_core_msgs::JointFwVersion _nodes_type;
      _nodes_type st_nodes;
      _nodes_type * nodes;

    JointFwVersionArray():
      nodes_length(0), nodes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->nodes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nodes_length);
      for( uint32_t i = 0; i < nodes_length; i++){
      offset += this->nodes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t nodes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->nodes_length);
      if(nodes_lengthT > nodes_length)
        this->nodes = (edo_core_msgs::JointFwVersion*)realloc(this->nodes, nodes_lengthT * sizeof(edo_core_msgs::JointFwVersion));
      nodes_length = nodes_lengthT;
      for( uint32_t i = 0; i < nodes_length; i++){
      offset += this->st_nodes.deserialize(inbuffer + offset);
        memcpy( &(this->nodes[i]), &(this->st_nodes), sizeof(edo_core_msgs::JointFwVersion));
      }
     return offset;
    }

    const char * getType(){ return "edo_core_msgs/JointFwVersionArray"; };
    const char * getMD5(){ return "44042a6de27b1e8c96d0f61b42700899"; };

  };

}
#endif