// Generated by gencpp from file tada_ros/ReconDataMsg.msg
// DO NOT EDIT!


#ifndef TADA_ROS_MESSAGE_RECONDATAMSG_H
#define TADA_ROS_MESSAGE_RECONDATAMSG_H


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
struct ReconDataMsg_
{
  typedef ReconDataMsg_<ContainerAllocator> Type;

  ReconDataMsg_()
    : timestamp(0.0)
    , pos_x(0.0)
    , pos_y(0.0)
    , pos_z(0.0)
    , vel_x(0.0)
    , vel_y(0.0)
    , vel_z(0.0)
    , accel_x(0.0)
    , accel_y(0.0)
    , accel_z(0.0)  {
    }
  ReconDataMsg_(const ContainerAllocator& _alloc)
    : timestamp(0.0)
    , pos_x(0.0)
    , pos_y(0.0)
    , pos_z(0.0)
    , vel_x(0.0)
    , vel_y(0.0)
    , vel_z(0.0)
    , accel_x(0.0)
    , accel_y(0.0)
    , accel_z(0.0)  {
  (void)_alloc;
    }



   typedef double _timestamp_type;
  _timestamp_type timestamp;

   typedef double _pos_x_type;
  _pos_x_type pos_x;

   typedef double _pos_y_type;
  _pos_y_type pos_y;

   typedef double _pos_z_type;
  _pos_z_type pos_z;

   typedef double _vel_x_type;
  _vel_x_type vel_x;

   typedef double _vel_y_type;
  _vel_y_type vel_y;

   typedef double _vel_z_type;
  _vel_z_type vel_z;

   typedef double _accel_x_type;
  _accel_x_type accel_x;

   typedef double _accel_y_type;
  _accel_y_type accel_y;

   typedef double _accel_z_type;
  _accel_z_type accel_z;





  typedef boost::shared_ptr< ::tada_ros::ReconDataMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tada_ros::ReconDataMsg_<ContainerAllocator> const> ConstPtr;

}; // struct ReconDataMsg_

typedef ::tada_ros::ReconDataMsg_<std::allocator<void> > ReconDataMsg;

typedef boost::shared_ptr< ::tada_ros::ReconDataMsg > ReconDataMsgPtr;
typedef boost::shared_ptr< ::tada_ros::ReconDataMsg const> ReconDataMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tada_ros::ReconDataMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tada_ros::ReconDataMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tada_ros::ReconDataMsg_<ContainerAllocator1> & lhs, const ::tada_ros::ReconDataMsg_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.pos_x == rhs.pos_x &&
    lhs.pos_y == rhs.pos_y &&
    lhs.pos_z == rhs.pos_z &&
    lhs.vel_x == rhs.vel_x &&
    lhs.vel_y == rhs.vel_y &&
    lhs.vel_z == rhs.vel_z &&
    lhs.accel_x == rhs.accel_x &&
    lhs.accel_y == rhs.accel_y &&
    lhs.accel_z == rhs.accel_z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tada_ros::ReconDataMsg_<ContainerAllocator1> & lhs, const ::tada_ros::ReconDataMsg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tada_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::tada_ros::ReconDataMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tada_ros::ReconDataMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tada_ros::ReconDataMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tada_ros::ReconDataMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tada_ros::ReconDataMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tada_ros::ReconDataMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tada_ros::ReconDataMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6efa2547ee2ac33067aa70230a2d9b97";
  }

  static const char* value(const ::tada_ros::ReconDataMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6efa2547ee2ac330ULL;
  static const uint64_t static_value2 = 0x67aa70230a2d9b97ULL;
};

template<class ContainerAllocator>
struct DataType< ::tada_ros::ReconDataMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tada_ros/ReconDataMsg";
  }

  static const char* value(const ::tada_ros::ReconDataMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tada_ros::ReconDataMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 timestamp\n"
"float64 pos_x\n"
"float64 pos_y\n"
"float64 pos_z\n"
"float64 vel_x\n"
"float64 vel_y\n"
"float64 vel_z\n"
"float64 accel_x\n"
"float64 accel_y\n"
"float64 accel_z\n"
;
  }

  static const char* value(const ::tada_ros::ReconDataMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tada_ros::ReconDataMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.pos_x);
      stream.next(m.pos_y);
      stream.next(m.pos_z);
      stream.next(m.vel_x);
      stream.next(m.vel_y);
      stream.next(m.vel_z);
      stream.next(m.accel_x);
      stream.next(m.accel_y);
      stream.next(m.accel_z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ReconDataMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tada_ros::ReconDataMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tada_ros::ReconDataMsg_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<double>::stream(s, indent + "  ", v.timestamp);
    s << indent << "pos_x: ";
    Printer<double>::stream(s, indent + "  ", v.pos_x);
    s << indent << "pos_y: ";
    Printer<double>::stream(s, indent + "  ", v.pos_y);
    s << indent << "pos_z: ";
    Printer<double>::stream(s, indent + "  ", v.pos_z);
    s << indent << "vel_x: ";
    Printer<double>::stream(s, indent + "  ", v.vel_x);
    s << indent << "vel_y: ";
    Printer<double>::stream(s, indent + "  ", v.vel_y);
    s << indent << "vel_z: ";
    Printer<double>::stream(s, indent + "  ", v.vel_z);
    s << indent << "accel_x: ";
    Printer<double>::stream(s, indent + "  ", v.accel_x);
    s << indent << "accel_y: ";
    Printer<double>::stream(s, indent + "  ", v.accel_y);
    s << indent << "accel_z: ";
    Printer<double>::stream(s, indent + "  ", v.accel_z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TADA_ROS_MESSAGE_RECONDATAMSG_H
