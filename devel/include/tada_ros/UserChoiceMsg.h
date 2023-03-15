// Generated by gencpp from file tada_ros/UserChoiceMsg.msg
// DO NOT EDIT!


#ifndef TADA_ROS_MESSAGE_USERCHOICEMSG_H
#define TADA_ROS_MESSAGE_USERCHOICEMSG_H


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
struct UserChoiceMsg_
{
  typedef UserChoiceMsg_<ContainerAllocator> Type;

  UserChoiceMsg_()
    : choice(0)
    , angle(0)  {
    }
  UserChoiceMsg_(const ContainerAllocator& _alloc)
    : choice(0)
    , angle(0)  {
  (void)_alloc;
    }



   typedef uint8_t _choice_type;
  _choice_type choice;

   typedef int16_t _angle_type;
  _angle_type angle;





  typedef boost::shared_ptr< ::tada_ros::UserChoiceMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tada_ros::UserChoiceMsg_<ContainerAllocator> const> ConstPtr;

}; // struct UserChoiceMsg_

typedef ::tada_ros::UserChoiceMsg_<std::allocator<void> > UserChoiceMsg;

typedef boost::shared_ptr< ::tada_ros::UserChoiceMsg > UserChoiceMsgPtr;
typedef boost::shared_ptr< ::tada_ros::UserChoiceMsg const> UserChoiceMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tada_ros::UserChoiceMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tada_ros::UserChoiceMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tada_ros::UserChoiceMsg_<ContainerAllocator1> & lhs, const ::tada_ros::UserChoiceMsg_<ContainerAllocator2> & rhs)
{
  return lhs.choice == rhs.choice &&
    lhs.angle == rhs.angle;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tada_ros::UserChoiceMsg_<ContainerAllocator1> & lhs, const ::tada_ros::UserChoiceMsg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tada_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::tada_ros::UserChoiceMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tada_ros::UserChoiceMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tada_ros::UserChoiceMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tada_ros::UserChoiceMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tada_ros::UserChoiceMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tada_ros::UserChoiceMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tada_ros::UserChoiceMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "55503f586513b642e8ca2e4716095bc8";
  }

  static const char* value(const ::tada_ros::UserChoiceMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x55503f586513b642ULL;
  static const uint64_t static_value2 = 0xe8ca2e4716095bc8ULL;
};

template<class ContainerAllocator>
struct DataType< ::tada_ros::UserChoiceMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tada_ros/UserChoiceMsg";
  }

  static const char* value(const ::tada_ros::UserChoiceMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tada_ros::UserChoiceMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 choice\n"
"int16 angle\n"
;
  }

  static const char* value(const ::tada_ros::UserChoiceMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tada_ros::UserChoiceMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.choice);
      stream.next(m.angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UserChoiceMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tada_ros::UserChoiceMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tada_ros::UserChoiceMsg_<ContainerAllocator>& v)
  {
    s << indent << "choice: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.choice);
    s << indent << "angle: ";
    Printer<int16_t>::stream(s, indent + "  ", v.angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TADA_ROS_MESSAGE_USERCHOICEMSG_H