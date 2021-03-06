// Generated by gencpp from file mp_mini_picker/moveToPoseMarkerRequest.msg
// DO NOT EDIT!


#ifndef MP_MINI_PICKER_MESSAGE_MOVETOPOSEMARKERREQUEST_H
#define MP_MINI_PICKER_MESSAGE_MOVETOPOSEMARKERREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mp_mini_picker
{
template <class ContainerAllocator>
struct moveToPoseMarkerRequest_
{
  typedef moveToPoseMarkerRequest_<ContainerAllocator> Type;

  moveToPoseMarkerRequest_()
    : pose()
    , tcpPmarker()
    , tcpRmarker()  {
      pose.assign(0.0);

      tcpPmarker.assign(0.0);

      tcpRmarker.assign(0.0);
  }
  moveToPoseMarkerRequest_(const ContainerAllocator& _alloc)
    : pose()
    , tcpPmarker()
    , tcpRmarker()  {
  (void)_alloc;
      pose.assign(0.0);

      tcpPmarker.assign(0.0);

      tcpRmarker.assign(0.0);
  }



   typedef boost::array<double, 6>  _pose_type;
  _pose_type pose;

   typedef boost::array<double, 3>  _tcpPmarker_type;
  _tcpPmarker_type tcpPmarker;

   typedef boost::array<double, 3>  _tcpRmarker_type;
  _tcpRmarker_type tcpRmarker;




  typedef boost::shared_ptr< ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator> const> ConstPtr;

}; // struct moveToPoseMarkerRequest_

typedef ::mp_mini_picker::moveToPoseMarkerRequest_<std::allocator<void> > moveToPoseMarkerRequest;

typedef boost::shared_ptr< ::mp_mini_picker::moveToPoseMarkerRequest > moveToPoseMarkerRequestPtr;
typedef boost::shared_ptr< ::mp_mini_picker::moveToPoseMarkerRequest const> moveToPoseMarkerRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace mp_mini_picker

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3e93f401cd80dbcc7b92bb016251f5d2";
  }

  static const char* value(const ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3e93f401cd80dbccULL;
  static const uint64_t static_value2 = 0x7b92bb016251f5d2ULL;
};

template<class ContainerAllocator>
struct DataType< ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mp_mini_picker/moveToPoseMarkerRequest";
  }

  static const char* value(const ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[6] pose\n\
float64[3] tcpPmarker\n\
float64[3] tcpRmarker\n\
";
  }

  static const char* value(const ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pose);
      stream.next(m.tcpPmarker);
      stream.next(m.tcpRmarker);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct moveToPoseMarkerRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mp_mini_picker::moveToPoseMarkerRequest_<ContainerAllocator>& v)
  {
    s << indent << "pose[]" << std::endl;
    for (size_t i = 0; i < v.pose.size(); ++i)
    {
      s << indent << "  pose[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.pose[i]);
    }
    s << indent << "tcpPmarker[]" << std::endl;
    for (size_t i = 0; i < v.tcpPmarker.size(); ++i)
    {
      s << indent << "  tcpPmarker[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.tcpPmarker[i]);
    }
    s << indent << "tcpRmarker[]" << std::endl;
    for (size_t i = 0; i < v.tcpRmarker.size(); ++i)
    {
      s << indent << "  tcpRmarker[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.tcpRmarker[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MP_MINI_PICKER_MESSAGE_MOVETOPOSEMARKERREQUEST_H
