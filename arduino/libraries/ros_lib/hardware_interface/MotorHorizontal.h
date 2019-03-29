#ifndef _ROS_hardware_interface_MotorHorizontal_h
#define _ROS_hardware_interface_MotorHorizontal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hardware_interface
{

  class MotorHorizontal : public ros::Msg
  {
    public:
      typedef uint16_t _t5_type;
      _t5_type t5;
      typedef uint16_t _t6_type;
      _t6_type t6;
      typedef uint16_t _t7_type;
      _t7_type t7;
      typedef uint16_t _t8_type;
      _t8_type t8;

    MotorHorizontal():
      t5(0),
      t6(0),
      t7(0),
      t8(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->t5 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->t5 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->t5);
      *(outbuffer + offset + 0) = (this->t6 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->t6 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->t6);
      *(outbuffer + offset + 0) = (this->t7 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->t7 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->t7);
      *(outbuffer + offset + 0) = (this->t8 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->t8 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->t8);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->t5 =  ((uint16_t) (*(inbuffer + offset)));
      this->t5 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->t5);
      this->t6 =  ((uint16_t) (*(inbuffer + offset)));
      this->t6 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->t6);
      this->t7 =  ((uint16_t) (*(inbuffer + offset)));
      this->t7 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->t7);
      this->t8 =  ((uint16_t) (*(inbuffer + offset)));
      this->t8 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->t8);
     return offset;
    }

    const char * getType(){ return "hardware_interface/MotorHorizontal"; };
    const char * getMD5(){ return "2377c7d3fdc5236758108dcb5cdaf388"; };

  };

}
#endif
