// Generated by gencpp from file image_processing/error_control.msg
// DO NOT EDIT!


#ifndef IMAGE_PROCESSING_MESSAGE_ERROR_CONTROL_H
#define IMAGE_PROCESSING_MESSAGE_ERROR_CONTROL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace image_processing
{
template <class ContainerAllocator>
struct error_control_
{
  typedef error_control_<ContainerAllocator> Type;

  error_control_()
    : error_steer(0.0)
    , control_error_steer(0.0)
    , steer_integral(0.0)
    , steer_deriv(0.0)
    , pid_error_value(0.0)
    , theta_error_value(0.0)
    , dist_tv(0.0)
    , status(0)  {
    }
  error_control_(const ContainerAllocator& _alloc)
    : error_steer(0.0)
    , control_error_steer(0.0)
    , steer_integral(0.0)
    , steer_deriv(0.0)
    , pid_error_value(0.0)
    , theta_error_value(0.0)
    , dist_tv(0.0)
    , status(0)  {
  (void)_alloc;
    }



   typedef float _error_steer_type;
  _error_steer_type error_steer;

   typedef float _control_error_steer_type;
  _control_error_steer_type control_error_steer;

   typedef float _steer_integral_type;
  _steer_integral_type steer_integral;

   typedef float _steer_deriv_type;
  _steer_deriv_type steer_deriv;

   typedef float _pid_error_value_type;
  _pid_error_value_type pid_error_value;

   typedef float _theta_error_value_type;
  _theta_error_value_type theta_error_value;

   typedef float _dist_tv_type;
  _dist_tv_type dist_tv;

   typedef int32_t _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::image_processing::error_control_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::image_processing::error_control_<ContainerAllocator> const> ConstPtr;

}; // struct error_control_

typedef ::image_processing::error_control_<std::allocator<void> > error_control;

typedef boost::shared_ptr< ::image_processing::error_control > error_controlPtr;
typedef boost::shared_ptr< ::image_processing::error_control const> error_controlConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::image_processing::error_control_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::image_processing::error_control_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::image_processing::error_control_<ContainerAllocator1> & lhs, const ::image_processing::error_control_<ContainerAllocator2> & rhs)
{
  return lhs.error_steer == rhs.error_steer &&
    lhs.control_error_steer == rhs.control_error_steer &&
    lhs.steer_integral == rhs.steer_integral &&
    lhs.steer_deriv == rhs.steer_deriv &&
    lhs.pid_error_value == rhs.pid_error_value &&
    lhs.theta_error_value == rhs.theta_error_value &&
    lhs.dist_tv == rhs.dist_tv &&
    lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::image_processing::error_control_<ContainerAllocator1> & lhs, const ::image_processing::error_control_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace image_processing

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::image_processing::error_control_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::image_processing::error_control_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::image_processing::error_control_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::image_processing::error_control_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::image_processing::error_control_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::image_processing::error_control_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::image_processing::error_control_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1d6a944f0c56f70ef6f19b82ff1f68dd";
  }

  static const char* value(const ::image_processing::error_control_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1d6a944f0c56f70eULL;
  static const uint64_t static_value2 = 0xf6f19b82ff1f68ddULL;
};

template<class ContainerAllocator>
struct DataType< ::image_processing::error_control_<ContainerAllocator> >
{
  static const char* value()
  {
    return "image_processing/error_control";
  }

  static const char* value(const ::image_processing::error_control_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::image_processing::error_control_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 error_steer\n"
"float32 control_error_steer\n"
"float32 steer_integral\n"
"float32 steer_deriv\n"
"float32 pid_error_value\n"
"float32 theta_error_value\n"
"float32 dist_tv\n"
"int32 status\n"
;
  }

  static const char* value(const ::image_processing::error_control_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::image_processing::error_control_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.error_steer);
      stream.next(m.control_error_steer);
      stream.next(m.steer_integral);
      stream.next(m.steer_deriv);
      stream.next(m.pid_error_value);
      stream.next(m.theta_error_value);
      stream.next(m.dist_tv);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct error_control_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::image_processing::error_control_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::image_processing::error_control_<ContainerAllocator>& v)
  {
    s << indent << "error_steer: ";
    Printer<float>::stream(s, indent + "  ", v.error_steer);
    s << indent << "control_error_steer: ";
    Printer<float>::stream(s, indent + "  ", v.control_error_steer);
    s << indent << "steer_integral: ";
    Printer<float>::stream(s, indent + "  ", v.steer_integral);
    s << indent << "steer_deriv: ";
    Printer<float>::stream(s, indent + "  ", v.steer_deriv);
    s << indent << "pid_error_value: ";
    Printer<float>::stream(s, indent + "  ", v.pid_error_value);
    s << indent << "theta_error_value: ";
    Printer<float>::stream(s, indent + "  ", v.theta_error_value);
    s << indent << "dist_tv: ";
    Printer<float>::stream(s, indent + "  ", v.dist_tv);
    s << indent << "status: ";
    Printer<int32_t>::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IMAGE_PROCESSING_MESSAGE_ERROR_CONTROL_H
