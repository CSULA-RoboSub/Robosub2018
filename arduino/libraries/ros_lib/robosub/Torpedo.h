#ifndef _ROS_robosub_Torpedo_h
#define _ROS_robosub_Torpedo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robosub
{

  class Torpedo : public ros::Msg
  {
    public:
      typedef int8_t _state_type;
      _state_type state;
      typedef int8_t _torpedo_number_type;
      _torpedo_number_type torpedo_number;

    Torpedo():
      state(0),
      torpedo_number(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      union {
        int8_t real;
        uint8_t base;
      } u_torpedo_number;
      u_torpedo_number.real = this->torpedo_number;
      *(outbuffer + offset + 0) = (u_torpedo_number.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->torpedo_number);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->state = u_state.real;
      offset += sizeof(this->state);
      union {
        int8_t real;
        uint8_t base;
      } u_torpedo_number;
      u_torpedo_number.base = 0;
      u_torpedo_number.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->torpedo_number = u_torpedo_number.real;
      offset += sizeof(this->torpedo_number);
     return offset;
    }

    const char * getType(){ return "robosub/Torpedo"; };
    const char * getMD5(){ return "9fcdfb282d4dcd54536fca1929ea85af"; };

  };

}
#endif
