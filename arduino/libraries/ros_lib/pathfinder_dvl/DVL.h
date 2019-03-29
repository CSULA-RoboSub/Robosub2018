#ifndef _ROS_pathfinder_dvl_DVL_h
#define _ROS_pathfinder_dvl_DVL_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pathfinder_dvl
{

  class DVL : public ros::Msg
  {
    public:
      typedef float _xpos_type;
      _xpos_type xpos;
      typedef float _xvel_type;
      _xvel_type xvel;
      typedef float _ypos_type;
      _ypos_type ypos;
      typedef float _yvel_type;
      _yvel_type yvel;
      typedef float _zpos_type;
      _zpos_type zpos;
      typedef float _zvel_type;
      _zvel_type zvel;

    DVL():
      xpos(0),
      xvel(0),
      ypos(0),
      yvel(0),
      zpos(0),
      zvel(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_xpos;
      u_xpos.real = this->xpos;
      *(outbuffer + offset + 0) = (u_xpos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xpos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xpos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xpos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xpos);
      union {
        float real;
        uint32_t base;
      } u_xvel;
      u_xvel.real = this->xvel;
      *(outbuffer + offset + 0) = (u_xvel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xvel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xvel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xvel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xvel);
      union {
        float real;
        uint32_t base;
      } u_ypos;
      u_ypos.real = this->ypos;
      *(outbuffer + offset + 0) = (u_ypos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ypos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ypos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ypos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ypos);
      union {
        float real;
        uint32_t base;
      } u_yvel;
      u_yvel.real = this->yvel;
      *(outbuffer + offset + 0) = (u_yvel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yvel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yvel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yvel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yvel);
      union {
        float real;
        uint32_t base;
      } u_zpos;
      u_zpos.real = this->zpos;
      *(outbuffer + offset + 0) = (u_zpos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zpos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zpos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zpos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->zpos);
      union {
        float real;
        uint32_t base;
      } u_zvel;
      u_zvel.real = this->zvel;
      *(outbuffer + offset + 0) = (u_zvel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zvel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zvel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zvel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->zvel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_xpos;
      u_xpos.base = 0;
      u_xpos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xpos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xpos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xpos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xpos = u_xpos.real;
      offset += sizeof(this->xpos);
      union {
        float real;
        uint32_t base;
      } u_xvel;
      u_xvel.base = 0;
      u_xvel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xvel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xvel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xvel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xvel = u_xvel.real;
      offset += sizeof(this->xvel);
      union {
        float real;
        uint32_t base;
      } u_ypos;
      u_ypos.base = 0;
      u_ypos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ypos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ypos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ypos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ypos = u_ypos.real;
      offset += sizeof(this->ypos);
      union {
        float real;
        uint32_t base;
      } u_yvel;
      u_yvel.base = 0;
      u_yvel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yvel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yvel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yvel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yvel = u_yvel.real;
      offset += sizeof(this->yvel);
      union {
        float real;
        uint32_t base;
      } u_zpos;
      u_zpos.base = 0;
      u_zpos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zpos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zpos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zpos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->zpos = u_zpos.real;
      offset += sizeof(this->zpos);
      union {
        float real;
        uint32_t base;
      } u_zvel;
      u_zvel.base = 0;
      u_zvel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zvel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zvel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zvel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->zvel = u_zvel.real;
      offset += sizeof(this->zvel);
     return offset;
    }

    const char * getType(){ return "pathfinder_dvl/DVL"; };
    const char * getMD5(){ return "c18887e1e6a0d80b3b4c69fc2e165d8d"; };

  };

}
#endif
