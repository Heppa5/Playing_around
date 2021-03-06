// Generated by gencpp from file mp_mini_picker/moveToPointMarkerRequest.msg
// DO NOT EDIT!


#ifndef MP_MINI_PICKER_MESSAGE_MOVETOPOINTMARKERREQUEST_H
#define MP_MINI_PICKER_MESSAGE_MOVETOPOINTMARKERREQUEST_H


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
struct moveToPointMarkerRequest_
{
  typedef moveToPointMarkerRequest_<ContainerAllocator> Type;

  moveToPointMarkerRequest_()
    : point()
    , tcpPmarker()
    , tcpRmarker()  {
      point.assign(0.0);

      tcpPmarker.assign(0.0);

      tcpRmarker.assign(0.0);
  }
  moveToPointMarkerRequest_(const ContainerAllocator& _alloc)
    : point()
    , tcpPmarker()
    , tcpRmarker()  {
  (void)_alloc;
      point.assign(0.0);

      tcpPmarker.assign(0.0);

      tcpRmarker.assign(0.0);
  }



   typedef boost::array<double, 3>  _point_type;
  _point_type point;

   typedef boost::array<double, 3>  _tcpPmarker_type;
  _tcpPmarker_type tcpPmarker;

   typedef boost::array<double, 3>  _tcpRmarker_type;
  _tcpRmarker_type tcpRmarker;




  typedef boost::shared_ptr< ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator> const> ConstPtr;

}; // struct moveToPointMarkerRequest_

typedef ::mp_mini_picker::moveToPointMarkerRequest_<std::allocator<void> > moveToPointMarkerRequest;

typedef boost::shared_ptr< ::mp_mini_picker::moveToPointMarkerRequest > moveToPointMarkerRequestPtr;
typedef boost::shared_ptr< ::mp_mini_picker::moveToPointMarkerRequest const> moveToPointMarkerRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "13d0cac9b9ca66b317c4dff8be8ae51c";
  }

  static const char* value(const ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x13d0cac9b9ca66b3ULL;
  static const uint64_t static_value2 = 0x17c4dff8be8ae51cULL;
};

template<class ContainerAllocator>
struct DataType< ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mp_mini_picker/moveToPointMarkerRequest";
  }

  static const char* value(const ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[3] point\n\
float64[3] tcpPmarker\n\
float64[3] tcpRmarker\n\
";
  }

  static const char* value(const ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.point);
      stream.next(m.tcpPmarker);
      stream.next(m.tcpRmarker);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct moveToPointMarkerRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mp_mini_picker::moveToPointMarkerRequest_<ContainerAllocator>& v)
  {
    s << indent << "point[]" << std::endl;
    for (size_t i = 0; i < v.point.size(); ++i)
    {
      s << indent << "  point[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.point[i]);
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

#endif // MP_MINI_PICKER_MESSAGE_MOVETOPOINTMARKERREQUEST_H
