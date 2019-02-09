#ifndef _ROS_embedded_control_sensor_data_h
#define _ROS_embedded_control_sensor_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace embedded_control
{

  class sensor_data : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int64_t _sharp_ir_type;
      _sharp_ir_type sharp_ir;
      typedef int64_t _L_encoder_type;
      _L_encoder_type L_encoder;
      typedef int64_t _R_encoder_type;
      _R_encoder_type R_encoder;
      typedef int64_t _servo_angle_type;
      _servo_angle_type servo_angle;
      typedef int64_t _pressure_type;
      _pressure_type pressure;

    sensor_data():
      header(),
      sharp_ir(0),
      L_encoder(0),
      R_encoder(0),
      servo_angle(0),
      pressure(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int64_t real;
        uint64_t base;
      } u_sharp_ir;
      u_sharp_ir.real = this->sharp_ir;
      *(outbuffer + offset + 0) = (u_sharp_ir.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sharp_ir.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sharp_ir.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sharp_ir.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_sharp_ir.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_sharp_ir.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_sharp_ir.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_sharp_ir.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sharp_ir);
      union {
        int64_t real;
        uint64_t base;
      } u_L_encoder;
      u_L_encoder.real = this->L_encoder;
      *(outbuffer + offset + 0) = (u_L_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_L_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_L_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_L_encoder.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_L_encoder.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_L_encoder.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_L_encoder.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_L_encoder.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->L_encoder);
      union {
        int64_t real;
        uint64_t base;
      } u_R_encoder;
      u_R_encoder.real = this->R_encoder;
      *(outbuffer + offset + 0) = (u_R_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_R_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_R_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_R_encoder.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_R_encoder.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_R_encoder.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_R_encoder.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_R_encoder.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->R_encoder);
      union {
        int64_t real;
        uint64_t base;
      } u_servo_angle;
      u_servo_angle.real = this->servo_angle;
      *(outbuffer + offset + 0) = (u_servo_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo_angle.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_servo_angle.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_servo_angle.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_servo_angle.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_servo_angle.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->servo_angle);
      union {
        int64_t real;
        uint64_t base;
      } u_pressure;
      u_pressure.real = this->pressure;
      *(outbuffer + offset + 0) = (u_pressure.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pressure.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pressure.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pressure.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pressure.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pressure.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pressure.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pressure.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pressure);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int64_t real;
        uint64_t base;
      } u_sharp_ir;
      u_sharp_ir.base = 0;
      u_sharp_ir.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sharp_ir.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sharp_ir.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sharp_ir.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_sharp_ir.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_sharp_ir.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_sharp_ir.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_sharp_ir.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->sharp_ir = u_sharp_ir.real;
      offset += sizeof(this->sharp_ir);
      union {
        int64_t real;
        uint64_t base;
      } u_L_encoder;
      u_L_encoder.base = 0;
      u_L_encoder.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_L_encoder.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_L_encoder.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_L_encoder.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_L_encoder.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_L_encoder.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_L_encoder.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_L_encoder.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->L_encoder = u_L_encoder.real;
      offset += sizeof(this->L_encoder);
      union {
        int64_t real;
        uint64_t base;
      } u_R_encoder;
      u_R_encoder.base = 0;
      u_R_encoder.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_R_encoder.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_R_encoder.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_R_encoder.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_R_encoder.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_R_encoder.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_R_encoder.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_R_encoder.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->R_encoder = u_R_encoder.real;
      offset += sizeof(this->R_encoder);
      union {
        int64_t real;
        uint64_t base;
      } u_servo_angle;
      u_servo_angle.base = 0;
      u_servo_angle.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_angle.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo_angle.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo_angle.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_servo_angle.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_servo_angle.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_servo_angle.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_servo_angle.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->servo_angle = u_servo_angle.real;
      offset += sizeof(this->servo_angle);
      union {
        int64_t real;
        uint64_t base;
      } u_pressure;
      u_pressure.base = 0;
      u_pressure.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pressure.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pressure.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pressure.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_pressure.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_pressure.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_pressure.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_pressure.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->pressure = u_pressure.real;
      offset += sizeof(this->pressure);
     return offset;
    }

    const char * getType(){ return "embedded_control/sensor_data"; };
    const char * getMD5(){ return "8fbd662761b134c07d5fba2d118ea721"; };

  };

}
#endif