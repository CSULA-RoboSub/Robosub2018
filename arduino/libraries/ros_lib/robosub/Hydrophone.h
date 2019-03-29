#ifndef _ROS_robosub_Hydrophone_h
#define _ROS_robosub_Hydrophone_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robosub
{

  class Hydrophone : public ros::Msg
  {
    public:
      uint16_t times1[4];
      uint16_t times2[4];
      uint16_t times3[4];
      uint16_t times4[4];

    Hydrophone():
      times1(),
      times2(),
      times3(),
      times4()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->times1[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times1[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->times1[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->times2[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times2[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->times2[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->times3[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times3[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->times3[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->times4[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->times4[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->times4[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      this->times1[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->times1[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->times1[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      this->times2[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->times2[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->times2[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      this->times3[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->times3[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->times3[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      this->times4[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->times4[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->times4[i]);
      }
     return offset;
    }

    const char * getType(){ return "robosub/Hydrophone"; };
    const char * getMD5(){ return "bd1b01c5bfd977f39ed691d39664954d"; };

  };

}
#endif
