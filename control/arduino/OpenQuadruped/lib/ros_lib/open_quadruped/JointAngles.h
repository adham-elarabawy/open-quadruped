#ifndef _ROS_open_quadruped_JointAngles_h
#define _ROS_open_quadruped_JointAngles_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace open_quadruped
{

  class JointAngles : public ros::Msg
  {
    public:
      uint32_t fl_length;
      typedef float _fl_type;
      _fl_type st_fl;
      _fl_type * fl;
      uint32_t fr_length;
      typedef float _fr_type;
      _fr_type st_fr;
      _fr_type * fr;
      uint32_t bl_length;
      typedef float _bl_type;
      _bl_type st_bl;
      _bl_type * bl;
      uint32_t br_length;
      typedef float _br_type;
      _br_type st_br;
      _br_type * br;

    JointAngles():
      fl_length(0), fl(NULL),
      fr_length(0), fr(NULL),
      bl_length(0), bl(NULL),
      br_length(0), br(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->fl_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fl_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fl_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fl_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fl_length);
      for( uint32_t i = 0; i < fl_length; i++){
      union {
        float real;
        uint32_t base;
      } u_fli;
      u_fli.real = this->fl[i];
      *(outbuffer + offset + 0) = (u_fli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fli.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fl[i]);
      }
      *(outbuffer + offset + 0) = (this->fr_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fr_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fr_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fr_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fr_length);
      for( uint32_t i = 0; i < fr_length; i++){
      union {
        float real;
        uint32_t base;
      } u_fri;
      u_fri.real = this->fr[i];
      *(outbuffer + offset + 0) = (u_fri.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fri.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fri.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fri.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fr[i]);
      }
      *(outbuffer + offset + 0) = (this->bl_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->bl_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->bl_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->bl_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bl_length);
      for( uint32_t i = 0; i < bl_length; i++){
      union {
        float real;
        uint32_t base;
      } u_bli;
      u_bli.real = this->bl[i];
      *(outbuffer + offset + 0) = (u_bli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bli.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bl[i]);
      }
      *(outbuffer + offset + 0) = (this->br_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->br_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->br_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->br_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->br_length);
      for( uint32_t i = 0; i < br_length; i++){
      union {
        float real;
        uint32_t base;
      } u_bri;
      u_bri.real = this->br[i];
      *(outbuffer + offset + 0) = (u_bri.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bri.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bri.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bri.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->br[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t fl_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fl_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fl_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fl_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fl_length);
      if(fl_lengthT > fl_length)
        this->fl = (float*)realloc(this->fl, fl_lengthT * sizeof(float));
      fl_length = fl_lengthT;
      for( uint32_t i = 0; i < fl_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_fl;
      u_st_fl.base = 0;
      u_st_fl.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_fl.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_fl.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_fl.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_fl = u_st_fl.real;
      offset += sizeof(this->st_fl);
        memcpy( &(this->fl[i]), &(this->st_fl), sizeof(float));
      }
      uint32_t fr_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fr_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fr_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fr_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fr_length);
      if(fr_lengthT > fr_length)
        this->fr = (float*)realloc(this->fr, fr_lengthT * sizeof(float));
      fr_length = fr_lengthT;
      for( uint32_t i = 0; i < fr_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_fr;
      u_st_fr.base = 0;
      u_st_fr.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_fr.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_fr.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_fr.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_fr = u_st_fr.real;
      offset += sizeof(this->st_fr);
        memcpy( &(this->fr[i]), &(this->st_fr), sizeof(float));
      }
      uint32_t bl_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      bl_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      bl_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      bl_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->bl_length);
      if(bl_lengthT > bl_length)
        this->bl = (float*)realloc(this->bl, bl_lengthT * sizeof(float));
      bl_length = bl_lengthT;
      for( uint32_t i = 0; i < bl_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_bl;
      u_st_bl.base = 0;
      u_st_bl.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_bl.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_bl.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_bl.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_bl = u_st_bl.real;
      offset += sizeof(this->st_bl);
        memcpy( &(this->bl[i]), &(this->st_bl), sizeof(float));
      }
      uint32_t br_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      br_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      br_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      br_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->br_length);
      if(br_lengthT > br_length)
        this->br = (float*)realloc(this->br, br_lengthT * sizeof(float));
      br_length = br_lengthT;
      for( uint32_t i = 0; i < br_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_br;
      u_st_br.base = 0;
      u_st_br.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_br.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_br.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_br.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_br = u_st_br.real;
      offset += sizeof(this->st_br);
        memcpy( &(this->br[i]), &(this->st_br), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "open_quadruped/JointAngles"; };
    const char * getMD5(){ return "cc955e0566b06523084e350c65b2944e"; };

  };

}
#endif
