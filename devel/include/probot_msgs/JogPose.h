// Generated by gencpp from file probot_msgs/JogPose.msg
// DO NOT EDIT!


#ifndef PROBOT_MSGS_MESSAGE_JOGPOSE_H
#define PROBOT_MSGS_MESSAGE_JOGPOSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace probot_msgs
{
template <class ContainerAllocator>
struct JogPose_
{
  typedef JogPose_<ContainerAllocator> Type;

  JogPose_()
    : mask(0)
    , direction(0)
    , mode(0)
    , velocity_scale(0.0)
    , frame_id(0)  {
    }
  JogPose_(const ContainerAllocator& _alloc)
    : mask(0)
    , direction(0)
    , mode(0)
    , velocity_scale(0.0)
    , frame_id(0)  {
  (void)_alloc;
    }



   typedef int8_t _mask_type;
  _mask_type mask;

   typedef int8_t _direction_type;
  _direction_type direction;

   typedef int8_t _mode_type;
  _mode_type mode;

   typedef float _velocity_scale_type;
  _velocity_scale_type velocity_scale;

   typedef int8_t _frame_id_type;
  _frame_id_type frame_id;



  enum {
    DIR_PLUS = 0,
    DIR_MINUS = 1,
    JOGGING_STOP = 0,
    JOGGING_X = 1,
    JOGGING_Y = 2,
    JOGGING_Z = 4,
    JOGGING_ROLL = 8,
    JOGGING_PITCH = 16,
    JOGGING_YAW = 32,
    MOD_CONTINUOUS = 0,
    MOD_INCREMENT_10_MM = 1,
    MOD_INCREMENT_1_MM = 2,
    MOD_INCREMENT_01_MM = 3,
  };


  typedef boost::shared_ptr< ::probot_msgs::JogPose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::probot_msgs::JogPose_<ContainerAllocator> const> ConstPtr;

}; // struct JogPose_

typedef ::probot_msgs::JogPose_<std::allocator<void> > JogPose;

typedef boost::shared_ptr< ::probot_msgs::JogPose > JogPosePtr;
typedef boost::shared_ptr< ::probot_msgs::JogPose const> JogPoseConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::probot_msgs::JogPose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::probot_msgs::JogPose_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace probot_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'probot_msgs': ['/home/abstract/catkin_ws/src/probot_anno/probot_msgs/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::probot_msgs::JogPose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::probot_msgs::JogPose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::probot_msgs::JogPose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::probot_msgs::JogPose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::probot_msgs::JogPose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::probot_msgs::JogPose_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::probot_msgs::JogPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d193764a9f7e2d9ca8e4aaaa3600bff9";
  }

  static const char* value(const ::probot_msgs::JogPose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd193764a9f7e2d9cULL;
  static const uint64_t static_value2 = 0xa8e4aaaa3600bff9ULL;
};

template<class ContainerAllocator>
struct DataType< ::probot_msgs::JogPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "probot_msgs/JogPose";
  }

  static const char* value(const ::probot_msgs::JogPose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::probot_msgs::JogPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# The Jogging pose message contains control information for jogging\n"
"# end-effector to a target pose\n"
"# It means stopping jogging if mask is JOGGING_STOP\n"
"\n"
"# The bit map mask to indicate which pose data is jogged\n"
"int8 mask\n"
"\n"
"# The jogging direction\n"
"int8 direction\n"
"\n"
"# The jogging mode\n"
"int8 mode\n"
"\n"
"# The velocity scale for move_group interface\n"
"float32 velocity_scale\n"
"\n"
"# The jogging frame id\n"
"int8 frame_id\n"
"\n"
"int8 DIR_PLUS = 0\n"
"int8 DIR_MINUS = 1\n"
"\n"
"# bit map of jogging\n"
"int8 JOGGING_STOP = 0\n"
"int8 JOGGING_X = 1\n"
"int8 JOGGING_Y = 2\n"
"int8 JOGGING_Z = 4\n"
"int8 JOGGING_ROLL = 8\n"
"int8 JOGGING_PITCH = 16\n"
"int8 JOGGING_YAW = 32\n"
"\n"
"int8 MOD_CONTINUOUS            = 0\n"
"int8 MOD_INCREMENT_10_MM       = 1\n"
"int8 MOD_INCREMENT_1_MM        = 2\n"
"int8 MOD_INCREMENT_01_MM       = 3\n"
;
  }

  static const char* value(const ::probot_msgs::JogPose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::probot_msgs::JogPose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mask);
      stream.next(m.direction);
      stream.next(m.mode);
      stream.next(m.velocity_scale);
      stream.next(m.frame_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JogPose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::probot_msgs::JogPose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::probot_msgs::JogPose_<ContainerAllocator>& v)
  {
    s << indent << "mask: ";
    Printer<int8_t>::stream(s, indent + "  ", v.mask);
    s << indent << "direction: ";
    Printer<int8_t>::stream(s, indent + "  ", v.direction);
    s << indent << "mode: ";
    Printer<int8_t>::stream(s, indent + "  ", v.mode);
    s << indent << "velocity_scale: ";
    Printer<float>::stream(s, indent + "  ", v.velocity_scale);
    s << indent << "frame_id: ";
    Printer<int8_t>::stream(s, indent + "  ", v.frame_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PROBOT_MSGS_MESSAGE_JOGPOSE_H
