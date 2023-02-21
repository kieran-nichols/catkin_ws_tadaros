// Generated by gencpp from file tada_ros/MotorListenMsg.msg
// DO NOT EDIT!


#ifndef TADA_ROS_MESSAGE_MOTORLISTENMSG_H
#define TADA_ROS_MESSAGE_MOTORLISTENMSG_H


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
struct MotorListenMsg_
{
  typedef MotorListenMsg_<ContainerAllocator> Type;

  MotorListenMsg_()
    : curr_pos1(0)
    , curr_pos2(0)  {
    }
  MotorListenMsg_(const ContainerAllocator& _alloc)
    : curr_pos1(0)
    , curr_pos2(0)  {
  (void)_alloc;
    }



   typedef int32_t _curr_pos1_type;
  _curr_pos1_type curr_pos1;

   typedef int32_t _curr_pos2_type;
  _curr_pos2_type curr_pos2;





  typedef boost::shared_ptr< ::tada_ros::MotorListenMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tada_ros::MotorListenMsg_<ContainerAllocator> const> ConstPtr;

}; // struct MotorListenMsg_

typedef ::tada_ros::MotorListenMsg_<std::allocator<void> > MotorListenMsg;

typedef boost::shared_ptr< ::tada_ros::MotorListenMsg > MotorListenMsgPtr;
typedef boost::shared_ptr< ::tada_ros::MotorListenMsg const> MotorListenMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tada_ros::MotorListenMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tada_ros::MotorListenMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tada_ros::MotorListenMsg_<ContainerAllocator1> & lhs, const ::tada_ros::MotorListenMsg_<ContainerAllocator2> & rhs)
{
  return lhs.curr_pos1 == rhs.curr_pos1 &&
    lhs.curr_pos2 == rhs.curr_pos2;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tada_ros::MotorListenMsg_<ContainerAllocator1> & lhs, const ::tada_ros::MotorListenMsg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tada_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::tada_ros::MotorListenMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tada_ros::MotorListenMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tada_ros::MotorListenMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tada_ros::MotorListenMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tada_ros::MotorListenMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tada_ros::MotorListenMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tada_ros::MotorListenMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b5e8d7932558c0150376d4a17f7d0f96";
  }

  static const char* value(const ::tada_ros::MotorListenMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb5e8d7932558c015ULL;
  static const uint64_t static_value2 = 0x0376d4a17f7d0f96ULL;
};

template<class ContainerAllocator>
struct DataType< ::tada_ros::MotorListenMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tada_ros/MotorListenMsg";
  }

  static const char* value(const ::tada_ros::MotorListenMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tada_ros::MotorListenMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 curr_pos1\n"
"int32 curr_pos2\n"
;
  }

  static const char* value(const ::tada_ros::MotorListenMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tada_ros::MotorListenMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.curr_pos1);
      stream.next(m.curr_pos2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotorListenMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tada_ros::MotorListenMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tada_ros::MotorListenMsg_<ContainerAllocator>& v)
  {
    s << indent << "curr_pos1: ";
    Printer<int32_t>::stream(s, indent + "  ", v.curr_pos1);
    s << indent << "curr_pos2: ";
    Printer<int32_t>::stream(s, indent + "  ", v.curr_pos2);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TADA_ROS_MESSAGE_MOTORLISTENMSG_H
