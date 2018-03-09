// Generated by gencpp from file mp_mini_picker/currentQResponse.msg
// DO NOT EDIT!


#ifndef MP_MINI_PICKER_MESSAGE_CURRENTQRESPONSE_H
#define MP_MINI_PICKER_MESSAGE_CURRENTQRESPONSE_H


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
struct currentQResponse_
{
  typedef currentQResponse_<ContainerAllocator> Type;

  currentQResponse_()
    : Q()  {
      Q.assign(0.0);
  }
  currentQResponse_(const ContainerAllocator& _alloc)
    : Q()  {
  (void)_alloc;
      Q.assign(0.0);
  }



   typedef boost::array<double, 6>  _Q_type;
  _Q_type Q;




  typedef boost::shared_ptr< ::mp_mini_picker::currentQResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mp_mini_picker::currentQResponse_<ContainerAllocator> const> ConstPtr;

}; // struct currentQResponse_

typedef ::mp_mini_picker::currentQResponse_<std::allocator<void> > currentQResponse;

typedef boost::shared_ptr< ::mp_mini_picker::currentQResponse > currentQResponsePtr;
typedef boost::shared_ptr< ::mp_mini_picker::currentQResponse const> currentQResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mp_mini_picker::currentQResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mp_mini_picker::currentQResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::mp_mini_picker::currentQResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mp_mini_picker::currentQResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mp_mini_picker::currentQResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mp_mini_picker::currentQResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mp_mini_picker::currentQResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mp_mini_picker::currentQResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mp_mini_picker::currentQResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4317a35b45e376d0ab631a6538e289aa";
  }

  static const char* value(const ::mp_mini_picker::currentQResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4317a35b45e376d0ULL;
  static const uint64_t static_value2 = 0xab631a6538e289aaULL;
};

template<class ContainerAllocator>
struct DataType< ::mp_mini_picker::currentQResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mp_mini_picker/currentQResponse";
  }

  static const char* value(const ::mp_mini_picker::currentQResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mp_mini_picker::currentQResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[6] Q\n\
\n\
";
  }

  static const char* value(const ::mp_mini_picker::currentQResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mp_mini_picker::currentQResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Q);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct currentQResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mp_mini_picker::currentQResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mp_mini_picker::currentQResponse_<ContainerAllocator>& v)
  {
    s << indent << "Q[]" << std::endl;
    for (size_t i = 0; i < v.Q.size(); ++i)
    {
      s << indent << "  Q[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.Q[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MP_MINI_PICKER_MESSAGE_CURRENTQRESPONSE_H