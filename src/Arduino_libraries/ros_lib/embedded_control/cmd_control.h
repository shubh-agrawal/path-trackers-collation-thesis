#ifndef _ROS_embedded_control_cmd_control_h
#define _ROS_embedded_control_cmd_control_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace embedded_control
{

  class cmd_control : public ros::Msg
  {
    public:
      typedef int64_t _servo_target_type;
      _servo_target_type servo_target;
      typedef int64_t _yaw_target_type;
      _yaw_target_type yaw_target;
      typedef int64_t _x_target_type;
      _x_target_type x_target;
      typedef int64_t _pitch_state_type;
      _pitch_state_type pitch_state;
      typedef int64_t _extend_state_type;
      _extend_state_type extend_state;
      typedef int64_t _gripper_state_type;
      _gripper_state_type gripper_state;

    cmd_control():
      servo_target(0),
      yaw_target(0),
      x_target(0),
      pitch_state(0),
      extend_state(0),
      gripper_state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_servo_target;
      u_servo_target.real = this->servo_target;
      *(outbuffer + offset + 0) = (u_servo_target.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_target.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo_target.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo_target.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_servo_target.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_servo_target.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_servo_target.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_servo_target.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->servo_target);
      union {
        int64_t real;
        uint64_t base;
      } u_yaw_target;
      u_yaw_target.real = this->yaw_target;
      *(outbuffer + offset + 0) = (u_yaw_target.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_target.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_target.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_target.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_yaw_target.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_yaw_target.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_yaw_target.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_yaw_target.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->yaw_target);
      union {
        int64_t real;
        uint64_t base;
      } u_x_target;
      u_x_target.real = this->x_target;
      *(outbuffer + offset + 0) = (u_x_target.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_target.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_target.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_target.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_x_target.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_x_target.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_x_target.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_x_target.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->x_target);
      union {
        int64_t real;
        uint64_t base;
      } u_pitch_state;
      u_pitch_state.real = this->pitch_state;
      *(outbuffer + offset + 0) = (u_pitch_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch_state.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch_state.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch_state.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pitch_state.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pitch_state.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pitch_state.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pitch_state.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pitch_state);
      union {
        int64_t real;
        uint64_t base;
      } u_extend_state;
      u_extend_state.real = this->extend_state;
      *(outbuffer + offset + 0) = (u_extend_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_extend_state.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_extend_state.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_extend_state.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_extend_state.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_extend_state.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_extend_state.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_extend_state.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->extend_state);
      union {
        int64_t real;
        uint64_t base;
      } u_gripper_state;
      u_gripper_state.real = this->gripper_state;
      *(outbuffer + offset + 0) = (u_gripper_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gripper_state.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gripper_state.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gripper_state.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_gripper_state.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_gripper_state.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_gripper_state.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_gripper_state.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->gripper_state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_servo_target;
      u_servo_target.base = 0;
      u_servo_target.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_target.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo_target.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo_target.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_servo_target.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_servo_target.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_servo_target.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_servo_target.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->servo_target = u_servo_target.real;
      offset += sizeof(this->servo_target);
      union {
        int64_t real;
        uint64_t base;
      } u_yaw_target;
      u_yaw_target.base = 0;
      u_yaw_target.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw_target.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw_target.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw_target.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_yaw_target.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_yaw_target.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_yaw_target.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_yaw_target.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->yaw_target = u_yaw_target.real;
      offset += sizeof(this->yaw_target);
      union {
        int64_t real;
        uint64_t base;
      } u_x_target;
      u_x_target.base = 0;
      u_x_target.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_target.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_target.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_target.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_x_target.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_x_target.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_x_target.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_x_target.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->x_target = u_x_target.real;
      offset += sizeof(this->x_target);
      union {
        int64_t real;
        uint64_t base;
      } u_pitch_state;
      u_pitch_state.base = 0;
      u_pitch_state.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch_state.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch_state.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch_state.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_pitch_state.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_pitch_state.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_pitch_state.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_pitch_state.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->pitch_state = u_pitch_state.real;
      offset += sizeof(this->pitch_state);
      union {
        int64_t real;
        uint64_t base;
      } u_extend_state;
      u_extend_state.base = 0;
      u_extend_state.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_extend_state.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_extend_state.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_extend_state.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_extend_state.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_extend_state.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_extend_state.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_extend_state.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->extend_state = u_extend_state.real;
      offset += sizeof(this->extend_state);
      union {
        int64_t real;
        uint64_t base;
      } u_gripper_state;
      u_gripper_state.base = 0;
      u_gripper_state.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gripper_state.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gripper_state.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gripper_state.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_gripper_state.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_gripper_state.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_gripper_state.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_gripper_state.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->gripper_state = u_gripper_state.real;
      offset += sizeof(this->gripper_state);
     return offset;
    }

    const char * getType(){ return "embedded_control/cmd_control"; };
    const char * getMD5(){ return "c9ad4ff7d2ee77d0840f59bfeeae7483"; };

  };

}
#endif