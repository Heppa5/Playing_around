// Generated by gencpp from file mp_mini_picker/moveToQResponse.msg
// DO NOT EDIT!


#ifndef MP_MINI_PICKER_MESSAGE_MOVETOQRESPONSE_H
#define MP_MINI_PICKER_MESSAGE_MOVETOQRESPONSE_H


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
struct moveToQResponse_
{
  typedef moveToQResponse_<ContainerAllocator> Type;

  moveToQResponse_()
    : ok(0)  {
    }
  moveToQResponse_(const ContainerAllocator& _alloc)
    : ok(0)  {
  (void)_alloc;
    }



   typedef int8_t _ok_type;
  _ok_type ok;




  typedef boost::shared_ptr< ::mp_mini_picker::moveToQResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mp_mini_picker::moveToQResponse_<ContainerAllocator> const> ConstPtr;

}; // struct moveToQResponse_

typedef ::mp_mini_picker::moveToQResponse_<std::allocator<void> > moveToQResponse;

typedef boost::shared_ptr< ::mp_mini_picker::moveToQResponse > moveToQResponsePtr;
typedef boost::shared_ptr< ::mp_mini_picker::moveToQResponse const> moveToQResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mp_mini_picker::moveToQResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mp_mini_picker::moveToQResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::mp_mini_picker::moveToQResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mp_mini_picker::moveToQResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mp_mini_picker::moveToQResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mp_mini_picker::moveToQResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mp_mini_picker::moveToQResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mp_mini_picker::moveToQResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mp_mini_picker::moveToQResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "719c501bbbeb289704ee5d42501844db";
  }

  static const char* value(const ::mp_mini_picker::moveToQResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x719c501bbbeb2897ULL;
  static const uint64_t static_value2 = 0x04ee5d42501844dbULL;
};

template<class ContainerAllocator>
struct DataType< ::mp_mini_picker::moveToQResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mp_mini_picker/moveToQResponse";
  }

  static const char* value(const ::mp_mini_picker::moveToQResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mp_mini_picker::moveToQResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 ok\n\
\n\
";
  }

  static const char* value(const ::mp_mini_picker::moveToQResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mp_mini_picker::moveToQResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ok);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct moveToQResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mp_mini_picker::moveToQResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mp_mini_picker::moveToQResponse_<ContainerAllocator>& v)
  {
    s << indent << "ok: ";
    Printer<int8_t>::stream(s, indent + "  ", v.ok);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MP_MINI_PICKER_MESSAGE_MOVETOQRESPONSE_H
