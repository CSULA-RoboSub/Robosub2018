#ifndef _ROS_auv_cal_state_la_2017_CVIn_h
#define _ROS_auv_cal_state_la_2017_CVIn_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace auv_cal_state_la_2017
{

  class CVIn : public ros::Msg
  {
    public:
      typedef bool _found_type;
      _found_type found;
      typedef bool _done_type;
      _done_type done;
      typedef int8_t _horizontal_type;
      _horizontal_type horizontal;
      typedef int8_t _vertical_type;
      _vertical_type vertical;
      typedef float _distance_type;
      _distance_type distance;
      typedef float _targetType_type;
      _targetType_type targetType;

    CVIn():
      found(0),
      done(0),
      horizontal(0),
      vertical(0),
      distance(0),
      targetType(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_found;
      u_found.real = this->found;
      *(outbuffer + offset + 0) = (u_found.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->found);
      union {
        bool real;
        uint8_t base;
      } u_done;
      u_done.real = this->done;
      *(outbuffer + offset + 0) = (u_done.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->done);
      union {
        int8_t real;
        uint8_t base;
      } u_horizontal;
      u_horizontal.real = this->horizontal;
      *(outbuffer + offset + 0) = (u_horizontal.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->horizontal);
      union {
        int8_t real;
        uint8_t base;
      } u_vertical;
      u_vertical.real = this->vertical;
      *(outbuffer + offset + 0) = (u_vertical.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->vertical);
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
      } u_targetType;
      u_targetType.real = this->targetType;
      *(outbuffer + offset + 0) = (u_targetType.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_targetType.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_targetType.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_targetType.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->targetType);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_found;
      u_found.base = 0;
      u_found.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->found = u_found.real;
      offset += sizeof(this->found);
      union {
        bool real;
        uint8_t base;
      } u_done;
      u_done.base = 0;
      u_done.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->done = u_done.real;
      offset += sizeof(this->done);
      union {
        int8_t real;
        uint8_t base;
      } u_horizontal;
      u_horizontal.base = 0;
      u_horizontal.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->horizontal = u_horizontal.real;
      offset += sizeof(this->horizontal);
      union {
        int8_t real;
        uint8_t base;
      } u_vertical;
      u_vertical.base = 0;
      u_vertical.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->vertical = u_vertical.real;
      offset += sizeof(this->vertical);
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
      } u_targetType;
      u_targetType.base = 0;
      u_targetType.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_targetType.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_targetType.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_targetType.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->targetType = u_targetType.real;
      offset += sizeof(this->targetType);
     return offset;
    }

    const char * getType(){ return "auv_cal_state_la_2017/CVIn"; };
    const char * getMD5(){ return "e19420d2d691415c977e10942cd9e2e9"; };

  };

}
#endif
