#ifndef _ROS_hardware_interface_MotorVertical_h
#define _ROS_hardware_interface_MotorVertical_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hardware_interface
{

  class MotorVertical : public ros::Msg
  {
    public:
      typedef uint16_t _t1_type;
      _t1_type t1;
      typedef uint16_t _t2_type;
      _t2_type t2;
      typedef uint16_t _t3_type;
      _t3_type t3;
      typedef uint16_t _t4_type;
      _t4_type t4;

    MotorVertical():
      t1(0),
      t2(0),
      t3(0),
      t4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->t1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->t1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->t1);
      *(outbuffer + offset + 0) = (this->t2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->t2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->t2);
      *(outbuffer + offset + 0) = (this->t3 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->t3 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->t3);
      *(outbuffer + offset + 0) = (this->t4 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->t4 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->t4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->t1 =  ((uint16_t) (*(inbuffer + offset)));
      this->t1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->t1);
      this->t2 =  ((uint16_t) (*(inbuffer + offset)));
      this->t2 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->t2);
      this->t3 =  ((uint16_t) (*(inbuffer + offset)));
      this->t3 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->t3);
      this->t4 =  ((uint16_t) (*(inbuffer + offset)));
      this->t4 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->t4);
     return offset;
    }

    const char * getType(){ return "hardware_interface/MotorVertical"; };
    const char * getMD5(){ return "603762be26aba25714445b9ae7e73749"; };

  };

}
#endif
