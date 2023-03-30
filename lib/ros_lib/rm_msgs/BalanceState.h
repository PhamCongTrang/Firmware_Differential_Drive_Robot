#ifndef _ROS_rm_msgs_BalanceState_h
#define _ROS_rm_msgs_BalanceState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace rm_msgs
{

  class BalanceState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _alpha_type;
      _alpha_type alpha;
      typedef float _alpha_dot_type;
      _alpha_dot_type alpha_dot;
      typedef float _vel_type;
      _vel_type vel;
      typedef float _theta_dot_type;
      _theta_dot_type theta_dot;
      typedef float _control_1_type;
      _control_1_type control_1;
      typedef float _control_2_type;
      _control_2_type control_2;

    BalanceState():
      header(),
      alpha(0),
      alpha_dot(0),
      vel(0),
      theta_dot(0),
      control_1(0),
      control_2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->alpha);
      offset += serializeAvrFloat64(outbuffer + offset, this->alpha_dot);
      offset += serializeAvrFloat64(outbuffer + offset, this->vel);
      offset += serializeAvrFloat64(outbuffer + offset, this->theta_dot);
      offset += serializeAvrFloat64(outbuffer + offset, this->control_1);
      offset += serializeAvrFloat64(outbuffer + offset, this->control_2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->alpha));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->alpha_dot));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vel));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->theta_dot));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->control_1));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->control_2));
     return offset;
    }

    virtual const char * getType() override { return "rm_msgs/BalanceState"; };
    virtual const char * getMD5() override { return "54fad2a9c502654a43a195c5bf42346b"; };

  };

}
#endif
