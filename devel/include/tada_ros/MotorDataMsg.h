// Generated by gencpp from file tada_ros/MotorDataMsg.msg
// DO NOT EDIT!


#ifndef TADA_ROS_MESSAGE_MOTORDATAMSG_H
#define TADA_ROS_MESSAGE_MOTORDATAMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace tada_ros
{
template <class ContainerAllocator>
struct MotorDataMsg_
{
  typedef MotorDataMsg_<ContainerAllocator> Type;

  MotorDataMsg_()
    : mode(0)
    , duration(0)
    , motor1_move(0)
    , motor2_move(0)
    , motor1_torque(0)
    , motor2_torque(0)
    , PF(0.0)
    , EV(0.0)
    , CPU0(0.0)
    , CPU1(0.0)
    , CPU2(0.0)
    , CPU3(0.0)
    , t(0.0)  {
    }
  MotorDataMsg_(const ContainerAllocator& _alloc)
    : mode(0)
    , duration(0)
    , motor1_move(0)
    , motor2_move(0)
    , motor1_torque(0)
    , motor2_torque(0)
    , PF(0.0)
    , EV(0.0)
    , CPU0(0.0)
    , CPU1(0.0)
    , CPU2(0.0)
    , CPU3(0.0)
    , t(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _mode_type;
  _mode_type mode;

   typedef int32_t _duration_type;
  _duration_type duration;

   typedef int32_t _motor1_move_type;
  _motor1_move_type motor1_move;

   typedef int32_t _motor2_move_type;
  _motor2_move_type motor2_move;

   typedef int32_t _motor1_torque_type;
  _motor1_torque_type motor1_torque;

   typedef int32_t _motor2_torque_type;
  _motor2_torque_type motor2_torque;

   typedef float _PF_type;
  _PF_type PF;

   typedef float _EV_type;
  _EV_type EV;

   typedef float _CPU0_type;
  _CPU0_type CPU0;

   typedef float _CPU1_type;
  _CPU1_type CPU1;

   typedef float _CPU2_type;
  _CPU2_type CPU2;

   typedef float _CPU3_type;
  _CPU3_type CPU3;

   typedef float _t_type;
  _t_type t;





  typedef boost::shared_ptr< ::tada_ros::MotorDataMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tada_ros::MotorDataMsg_<ContainerAllocator> const> ConstPtr;

}; // struct MotorDataMsg_

typedef ::tada_ros::MotorDataMsg_<std::allocator<void> > MotorDataMsg;

typedef boost::shared_ptr< ::tada_ros::MotorDataMsg > MotorDataMsgPtr;
typedef boost::shared_ptr< ::tada_ros::MotorDataMsg const> MotorDataMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tada_ros::MotorDataMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tada_ros::MotorDataMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tada_ros::MotorDataMsg_<ContainerAllocator1> & lhs, const ::tada_ros::MotorDataMsg_<ContainerAllocator2> & rhs)
{
  return lhs.mode == rhs.mode &&
    lhs.duration == rhs.duration &&
    lhs.motor1_move == rhs.motor1_move &&
    lhs.motor2_move == rhs.motor2_move &&
    lhs.motor1_torque == rhs.motor1_torque &&
    lhs.motor2_torque == rhs.motor2_torque &&
    lhs.PF == rhs.PF &&
    lhs.EV == rhs.EV &&
    lhs.CPU0 == rhs.CPU0 &&
    lhs.CPU1 == rhs.CPU1 &&
    lhs.CPU2 == rhs.CPU2 &&
    lhs.CPU3 == rhs.CPU3 &&
    lhs.t == rhs.t;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tada_ros::MotorDataMsg_<ContainerAllocator1> & lhs, const ::tada_ros::MotorDataMsg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tada_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::tada_ros::MotorDataMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tada_ros::MotorDataMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tada_ros::MotorDataMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tada_ros::MotorDataMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tada_ros::MotorDataMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tada_ros::MotorDataMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tada_ros::MotorDataMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9af5263dec48e390e62e66f239203e3c";
  }

  static const char* value(const ::tada_ros::MotorDataMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9af5263dec48e390ULL;
  static const uint64_t static_value2 = 0xe62e66f239203e3cULL;
};

template<class ContainerAllocator>
struct DataType< ::tada_ros::MotorDataMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tada_ros/MotorDataMsg";
  }

  static const char* value(const ::tada_ros::MotorDataMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tada_ros::MotorDataMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 mode\n"
"int32 duration\n"
"int32 motor1_move\n"
"int32 motor2_move\n"
"int32 motor1_torque\n"
"int32 motor2_torque\n"
"float32 PF\n"
"float32 EV\n"
"float32 CPU0\n"
"float32 CPU1\n"
"float32 CPU2\n"
"float32 CPU3\n"
"float32 t\n"
;
  }

  static const char* value(const ::tada_ros::MotorDataMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tada_ros::MotorDataMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mode);
      stream.next(m.duration);
      stream.next(m.motor1_move);
      stream.next(m.motor2_move);
      stream.next(m.motor1_torque);
      stream.next(m.motor2_torque);
      stream.next(m.PF);
      stream.next(m.EV);
      stream.next(m.CPU0);
      stream.next(m.CPU1);
      stream.next(m.CPU2);
      stream.next(m.CPU3);
      stream.next(m.t);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotorDataMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tada_ros::MotorDataMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tada_ros::MotorDataMsg_<ContainerAllocator>& v)
  {
    s << indent << "mode: ";
    Printer<int32_t>::stream(s, indent + "  ", v.mode);
    s << indent << "duration: ";
    Printer<int32_t>::stream(s, indent + "  ", v.duration);
    s << indent << "motor1_move: ";
    Printer<int32_t>::stream(s, indent + "  ", v.motor1_move);
    s << indent << "motor2_move: ";
    Printer<int32_t>::stream(s, indent + "  ", v.motor2_move);
    s << indent << "motor1_torque: ";
    Printer<int32_t>::stream(s, indent + "  ", v.motor1_torque);
    s << indent << "motor2_torque: ";
    Printer<int32_t>::stream(s, indent + "  ", v.motor2_torque);
    s << indent << "PF: ";
    Printer<float>::stream(s, indent + "  ", v.PF);
    s << indent << "EV: ";
    Printer<float>::stream(s, indent + "  ", v.EV);
    s << indent << "CPU0: ";
    Printer<float>::stream(s, indent + "  ", v.CPU0);
    s << indent << "CPU1: ";
    Printer<float>::stream(s, indent + "  ", v.CPU1);
    s << indent << "CPU2: ";
    Printer<float>::stream(s, indent + "  ", v.CPU2);
    s << indent << "CPU3: ";
    Printer<float>::stream(s, indent + "  ", v.CPU3);
    s << indent << "t: ";
    Printer<float>::stream(s, indent + "  ", v.t);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TADA_ROS_MESSAGE_MOTORDATAMSG_H
