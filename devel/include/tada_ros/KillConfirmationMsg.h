// Generated by gencpp from file tada_ros/KillConfirmationMsg.msg
// DO NOT EDIT!


#ifndef TADA_ROS_MESSAGE_KILLCONFIRMATIONMSG_H
#define TADA_ROS_MESSAGE_KILLCONFIRMATIONMSG_H


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
struct KillConfirmationMsg_
{
  typedef KillConfirmationMsg_<ContainerAllocator> Type;

  KillConfirmationMsg_()
    : motors_killed(false)
    , sensors_killed(false)  {
    }
  KillConfirmationMsg_(const ContainerAllocator& _alloc)
    : motors_killed(false)
    , sensors_killed(false)  {
  (void)_alloc;
    }



   typedef uint8_t _motors_killed_type;
  _motors_killed_type motors_killed;

   typedef uint8_t _sensors_killed_type;
  _sensors_killed_type sensors_killed;





  typedef boost::shared_ptr< ::tada_ros::KillConfirmationMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tada_ros::KillConfirmationMsg_<ContainerAllocator> const> ConstPtr;

}; // struct KillConfirmationMsg_

typedef ::tada_ros::KillConfirmationMsg_<std::allocator<void> > KillConfirmationMsg;

typedef boost::shared_ptr< ::tada_ros::KillConfirmationMsg > KillConfirmationMsgPtr;
typedef boost::shared_ptr< ::tada_ros::KillConfirmationMsg const> KillConfirmationMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tada_ros::KillConfirmationMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tada_ros::KillConfirmationMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tada_ros::KillConfirmationMsg_<ContainerAllocator1> & lhs, const ::tada_ros::KillConfirmationMsg_<ContainerAllocator2> & rhs)
{
  return lhs.motors_killed == rhs.motors_killed &&
    lhs.sensors_killed == rhs.sensors_killed;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tada_ros::KillConfirmationMsg_<ContainerAllocator1> & lhs, const ::tada_ros::KillConfirmationMsg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tada_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::tada_ros::KillConfirmationMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tada_ros::KillConfirmationMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tada_ros::KillConfirmationMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tada_ros::KillConfirmationMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tada_ros::KillConfirmationMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tada_ros::KillConfirmationMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tada_ros::KillConfirmationMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8f0c1e581a5a8e60229fdfdefa9033aa";
  }

  static const char* value(const ::tada_ros::KillConfirmationMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8f0c1e581a5a8e60ULL;
  static const uint64_t static_value2 = 0x229fdfdefa9033aaULL;
};

template<class ContainerAllocator>
struct DataType< ::tada_ros::KillConfirmationMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tada_ros/KillConfirmationMsg";
  }

  static const char* value(const ::tada_ros::KillConfirmationMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tada_ros::KillConfirmationMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool motors_killed\n"
"bool sensors_killed\n"
;
  }

  static const char* value(const ::tada_ros::KillConfirmationMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tada_ros::KillConfirmationMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.motors_killed);
      stream.next(m.sensors_killed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct KillConfirmationMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tada_ros::KillConfirmationMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tada_ros::KillConfirmationMsg_<ContainerAllocator>& v)
  {
    s << indent << "motors_killed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.motors_killed);
    s << indent << "sensors_killed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.sensors_killed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TADA_ROS_MESSAGE_KILLCONFIRMATIONMSG_H
