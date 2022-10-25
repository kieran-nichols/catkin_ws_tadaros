// Generated by gencpp from file tada_ros/ConfigMsg.msg
// DO NOT EDIT!


#ifndef TADA_ROS_MESSAGE_CONFIGMSG_H
#define TADA_ROS_MESSAGE_CONFIGMSG_H


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
struct ConfigMsg_
{
  typedef ConfigMsg_<ContainerAllocator> Type;

  ConfigMsg_()
    : x_loc(0.0)
    , y_loc(0.0)
    , z_loc(0.0)
    , angle_1(0.0)
    , angle_2(0.0)
    , angle_3(0.0)  {
    }
  ConfigMsg_(const ContainerAllocator& _alloc)
    : x_loc(0.0)
    , y_loc(0.0)
    , z_loc(0.0)
    , angle_1(0.0)
    , angle_2(0.0)
    , angle_3(0.0)  {
  (void)_alloc;
    }



   typedef double _x_loc_type;
  _x_loc_type x_loc;

   typedef double _y_loc_type;
  _y_loc_type y_loc;

   typedef double _z_loc_type;
  _z_loc_type z_loc;

   typedef double _angle_1_type;
  _angle_1_type angle_1;

   typedef double _angle_2_type;
  _angle_2_type angle_2;

   typedef double _angle_3_type;
  _angle_3_type angle_3;





  typedef boost::shared_ptr< ::tada_ros::ConfigMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tada_ros::ConfigMsg_<ContainerAllocator> const> ConstPtr;

}; // struct ConfigMsg_

typedef ::tada_ros::ConfigMsg_<std::allocator<void> > ConfigMsg;

typedef boost::shared_ptr< ::tada_ros::ConfigMsg > ConfigMsgPtr;
typedef boost::shared_ptr< ::tada_ros::ConfigMsg const> ConfigMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tada_ros::ConfigMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tada_ros::ConfigMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tada_ros::ConfigMsg_<ContainerAllocator1> & lhs, const ::tada_ros::ConfigMsg_<ContainerAllocator2> & rhs)
{
  return lhs.x_loc == rhs.x_loc &&
    lhs.y_loc == rhs.y_loc &&
    lhs.z_loc == rhs.z_loc &&
    lhs.angle_1 == rhs.angle_1 &&
    lhs.angle_2 == rhs.angle_2 &&
    lhs.angle_3 == rhs.angle_3;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tada_ros::ConfigMsg_<ContainerAllocator1> & lhs, const ::tada_ros::ConfigMsg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tada_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::tada_ros::ConfigMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tada_ros::ConfigMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tada_ros::ConfigMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tada_ros::ConfigMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tada_ros::ConfigMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tada_ros::ConfigMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tada_ros::ConfigMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9fbfd22fef180f2f17264d3d24989496";
  }

  static const char* value(const ::tada_ros::ConfigMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9fbfd22fef180f2fULL;
  static const uint64_t static_value2 = 0x17264d3d24989496ULL;
};

template<class ContainerAllocator>
struct DataType< ::tada_ros::ConfigMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tada_ros/ConfigMsg";
  }

  static const char* value(const ::tada_ros::ConfigMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tada_ros::ConfigMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x_loc\n"
"float64 y_loc\n"
"float64 z_loc\n"
"float64 angle_1\n"
"float64 angle_2\n"
"float64 angle_3\n"
;
  }

  static const char* value(const ::tada_ros::ConfigMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tada_ros::ConfigMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x_loc);
      stream.next(m.y_loc);
      stream.next(m.z_loc);
      stream.next(m.angle_1);
      stream.next(m.angle_2);
      stream.next(m.angle_3);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ConfigMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tada_ros::ConfigMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tada_ros::ConfigMsg_<ContainerAllocator>& v)
  {
    s << indent << "x_loc: ";
    Printer<double>::stream(s, indent + "  ", v.x_loc);
    s << indent << "y_loc: ";
    Printer<double>::stream(s, indent + "  ", v.y_loc);
    s << indent << "z_loc: ";
    Printer<double>::stream(s, indent + "  ", v.z_loc);
    s << indent << "angle_1: ";
    Printer<double>::stream(s, indent + "  ", v.angle_1);
    s << indent << "angle_2: ";
    Printer<double>::stream(s, indent + "  ", v.angle_2);
    s << indent << "angle_3: ";
    Printer<double>::stream(s, indent + "  ", v.angle_3);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TADA_ROS_MESSAGE_CONFIGMSG_H
