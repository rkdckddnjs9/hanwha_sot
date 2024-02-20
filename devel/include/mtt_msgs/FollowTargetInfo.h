// Generated by gencpp from file mtt_msgs/FollowTargetInfo.msg
// DO NOT EDIT!


#ifndef MTT_MSGS_MESSAGE_FOLLOWTARGETINFO_H
#define MTT_MSGS_MESSAGE_FOLLOWTARGETINFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>

namespace mtt_msgs
{
template <class ContainerAllocator>
struct FollowTargetInfo_
{
  typedef FollowTargetInfo_<ContainerAllocator> Type;

  FollowTargetInfo_()
    : header()
    , id()
    , object_class()
    , score()
    , pose()
    , size()
    , velocity()  {
    }
  FollowTargetInfo_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , id(_alloc)
    , object_class(_alloc)
    , score(_alloc)
    , pose(_alloc)
    , size(_alloc)
    , velocity(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::std_msgs::Int32_<ContainerAllocator>  _id_type;
  _id_type id;

   typedef  ::std_msgs::Int8_<ContainerAllocator>  _object_class_type;
  _object_class_type object_class;

   typedef  ::std_msgs::Float32_<ContainerAllocator>  _score_type;
  _score_type score;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _size_type;
  _size_type size;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _velocity_type;
  _velocity_type velocity;





  typedef boost::shared_ptr< ::mtt_msgs::FollowTargetInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mtt_msgs::FollowTargetInfo_<ContainerAllocator> const> ConstPtr;

}; // struct FollowTargetInfo_

typedef ::mtt_msgs::FollowTargetInfo_<std::allocator<void> > FollowTargetInfo;

typedef boost::shared_ptr< ::mtt_msgs::FollowTargetInfo > FollowTargetInfoPtr;
typedef boost::shared_ptr< ::mtt_msgs::FollowTargetInfo const> FollowTargetInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mtt_msgs::FollowTargetInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mtt_msgs::FollowTargetInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mtt_msgs::FollowTargetInfo_<ContainerAllocator1> & lhs, const ::mtt_msgs::FollowTargetInfo_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.id == rhs.id &&
    lhs.object_class == rhs.object_class &&
    lhs.score == rhs.score &&
    lhs.pose == rhs.pose &&
    lhs.size == rhs.size &&
    lhs.velocity == rhs.velocity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mtt_msgs::FollowTargetInfo_<ContainerAllocator1> & lhs, const ::mtt_msgs::FollowTargetInfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mtt_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mtt_msgs::FollowTargetInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mtt_msgs::FollowTargetInfo_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mtt_msgs::FollowTargetInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mtt_msgs::FollowTargetInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mtt_msgs::FollowTargetInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mtt_msgs::FollowTargetInfo_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mtt_msgs::FollowTargetInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f662c869093ee1a8404764360b3f4101";
  }

  static const char* value(const ::mtt_msgs::FollowTargetInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf662c869093ee1a8ULL;
  static const uint64_t static_value2 = 0x404764360b3f4101ULL;
};

template<class ContainerAllocator>
struct DataType< ::mtt_msgs::FollowTargetInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mtt_msgs/FollowTargetInfo";
  }

  static const char* value(const ::mtt_msgs::FollowTargetInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mtt_msgs::FollowTargetInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"std_msgs/Int32 id\n"
"std_msgs/Int8 object_class\n"
"std_msgs/Float32 score\n"
"geometry_msgs/Pose pose\n"
"geometry_msgs/Point size\n"
"geometry_msgs/Point velocity\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Int32\n"
"int32 data\n"
"================================================================================\n"
"MSG: std_msgs/Int8\n"
"int8 data\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Float32\n"
"float32 data\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::mtt_msgs::FollowTargetInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mtt_msgs::FollowTargetInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.id);
      stream.next(m.object_class);
      stream.next(m.score);
      stream.next(m.pose);
      stream.next(m.size);
      stream.next(m.velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FollowTargetInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mtt_msgs::FollowTargetInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mtt_msgs::FollowTargetInfo_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "id: ";
    s << std::endl;
    Printer< ::std_msgs::Int32_<ContainerAllocator> >::stream(s, indent + "  ", v.id);
    s << indent << "object_class: ";
    s << std::endl;
    Printer< ::std_msgs::Int8_<ContainerAllocator> >::stream(s, indent + "  ", v.object_class);
    s << indent << "score: ";
    s << std::endl;
    Printer< ::std_msgs::Float32_<ContainerAllocator> >::stream(s, indent + "  ", v.score);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "size: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.size);
    s << indent << "velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MTT_MSGS_MESSAGE_FOLLOWTARGETINFO_H
