#ifndef _ROS_robosub_MControl_h
#define _ROS_robosub_MControl_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robosub
{

  class MControl : public ros::Msg
  {
    public:
      typedef int8_t _state_type;
      _state_type state;
      typedef int8_t _mDirection_type;
      _mDirection_type mDirection;
      typedef uint16_t _power_type;
      _power_type power;
      typedef float _distance_type;
      _distance_type distance;
      typedef float _runningTime_type;
      _runningTime_type runningTime;

    MControl():
      state(0),
      mDirection(0),
      power(0),
      distance(0),
      runningTime(0)
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
      } u_mDirection;
      u_mDirection.real = this->mDirection;
      *(outbuffer + offset + 0) = (u_mDirection.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mDirection);
      *(outbuffer + offset + 0) = (this->power >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->power >> (8 * 1)) & 0xFF;
      offset += sizeof(this->power);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.real = this->distance;
      *(outbuffer + offset + 0) = (u_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance);
      union {
        float real;
        uint32_t base;
      } u_runningTime;
      u_runningTime.real = this->runningTime;
      *(outbuffer + offset + 0) = (u_runningTime.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_runningTime.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_runningTime.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_runningTime.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->runningTime);
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
      } u_mDirection;
      u_mDirection.base = 0;
      u_mDirection.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mDirection = u_mDirection.real;
      offset += sizeof(this->mDirection);
      this->power =  ((uint16_t) (*(inbuffer + offset)));
      this->power |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->power);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.base = 0;
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance = u_distance.real;
      offset += sizeof(this->distance);
      union {
        float real;
        uint32_t base;
      } u_runningTime;
      u_runningTime.base = 0;
      u_runningTime.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_runningTime.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_runningTime.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_runningTime.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->runningTime = u_runningTime.real;
      offset += sizeof(this->runningTime);
     return offset;
    }

    const char * getType(){ return "robosub/MControl"; };
    const char * getMD5(){ return "40881c3c32b00f7591c96988f8859368"; };

  };

}
#endif
