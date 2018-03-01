#ifndef _ROS_edo_core_msgs_MovementCommand_h
#define _ROS_edo_core_msgs_MovementCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace edo_core_msgs
{

  class MovementCommand : public ros::Msg
  {
    public:
      typedef uint8_t _movement_type_type;
      _movement_type_type movement_type;
      typedef uint8_t _size_type;
      _size_type size;
      typedef uint8_t _ovr_type;
      _ovr_type ovr;
      uint32_t data_length;
      typedef float _data_type;
      _data_type st_data;
      _data_type * data;
      uint32_t movement_attributes_length;
      typedef uint8_t _movement_attributes_type;
      _movement_attributes_type st_movement_attributes;
      _movement_attributes_type * movement_attributes;

    MovementCommand():
      movement_type(0),
      size(0),
      ovr(0),
      data_length(0), data(NULL),
      movement_attributes_length(0), movement_attributes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->movement_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->movement_type);
      *(outbuffer + offset + 0) = (this->size >> (8 * 0)) & 0xFF;
      offset += sizeof(this->size);
      *(outbuffer + offset + 0) = (this->ovr >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ovr);
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      union {
        float real;
        uint32_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_datai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      *(outbuffer + offset + 0) = (this->movement_attributes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->movement_attributes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->movement_attributes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->movement_attributes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->movement_attributes_length);
      for( uint32_t i = 0; i < movement_attributes_length; i++){
      *(outbuffer + offset + 0) = (this->movement_attributes[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->movement_attributes[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->movement_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->movement_type);
      this->size =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->size);
      this->ovr =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->ovr);
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (float*)realloc(this->data, data_lengthT * sizeof(float));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(float));
      }
      uint32_t movement_attributes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      movement_attributes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      movement_attributes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      movement_attributes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->movement_attributes_length);
      if(movement_attributes_lengthT > movement_attributes_length)
        this->movement_attributes = (uint8_t*)realloc(this->movement_attributes, movement_attributes_lengthT * sizeof(uint8_t));
      movement_attributes_length = movement_attributes_lengthT;
      for( uint32_t i = 0; i < movement_attributes_length; i++){
      this->st_movement_attributes =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_movement_attributes);
        memcpy( &(this->movement_attributes[i]), &(this->st_movement_attributes), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "edo_core_msgs/MovementCommand"; };
    const char * getMD5(){ return "0aac7f5821019e6180fba9a9f6fc7963"; };

  };

}
#endif