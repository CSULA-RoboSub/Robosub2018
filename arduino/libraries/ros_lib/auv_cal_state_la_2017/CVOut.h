#ifndef _ROS_auv_cal_state_la_2017_CVOut_h
#define _ROS_auv_cal_state_la_2017_CVOut_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace auv_cal_state_la_2017
{

  class CVOut : public ros::Msg
  {
    public:
      typedef float _targetType_type;
      _targetType_type targetType;

    CVOut():
      targetType(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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

    const char * getType(){ return "auv_cal_state_la_2017/CVOut"; };
    const char * getMD5(){ return "2dbb6639f74ba3f63faf0aba892a18b0"; };

  };

}
#endif
