// Generated by gencpp from file ros_its_msgs/CLWarning.msg
// DO NOT EDIT!


#ifndef ROS_ITS_MSGS_MESSAGE_CLWARNING_H
#define ROS_ITS_MSGS_MESSAGE_CLWARNING_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ros_its_msgs
{
template <class ContainerAllocator>
struct CLWarning_
{
  typedef CLWarning_<ContainerAllocator> Type;

  CLWarning_()
    : car_name()
    , CLWType()  {
    }
  CLWarning_(const ContainerAllocator& _alloc)
    : car_name(_alloc)
    , CLWType(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _car_name_type;
  _car_name_type car_name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _CLWType_type;
  _CLWType_type CLWType;





  typedef boost::shared_ptr< ::ros_its_msgs::CLWarning_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ros_its_msgs::CLWarning_<ContainerAllocator> const> ConstPtr;

}; // struct CLWarning_

typedef ::ros_its_msgs::CLWarning_<std::allocator<void> > CLWarning;

typedef boost::shared_ptr< ::ros_its_msgs::CLWarning > CLWarningPtr;
typedef boost::shared_ptr< ::ros_its_msgs::CLWarning const> CLWarningConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ros_its_msgs::CLWarning_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ros_its_msgs::CLWarning_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ros_its_msgs::CLWarning_<ContainerAllocator1> & lhs, const ::ros_its_msgs::CLWarning_<ContainerAllocator2> & rhs)
{
  return lhs.car_name == rhs.car_name &&
    lhs.CLWType == rhs.CLWType;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ros_its_msgs::CLWarning_<ContainerAllocator1> & lhs, const ::ros_its_msgs::CLWarning_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ros_its_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ros_its_msgs::CLWarning_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_its_msgs::CLWarning_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_its_msgs::CLWarning_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_its_msgs::CLWarning_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_its_msgs::CLWarning_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_its_msgs::CLWarning_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ros_its_msgs::CLWarning_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a3e52a7449922f3c43207fb5cac4c802";
  }

  static const char* value(const ::ros_its_msgs::CLWarning_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa3e52a7449922f3cULL;
  static const uint64_t static_value2 = 0x43207fb5cac4c802ULL;
};

template<class ContainerAllocator>
struct DataType< ::ros_its_msgs::CLWarning_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ros_its_msgs/CLWarning";
  }

  static const char* value(const ::ros_its_msgs::CLWarning_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ros_its_msgs::CLWarning_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string car_name\n"
"\n"
"string CLWType\n"
;
  }

  static const char* value(const ::ros_its_msgs::CLWarning_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ros_its_msgs::CLWarning_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.car_name);
      stream.next(m.CLWType);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CLWarning_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ros_its_msgs::CLWarning_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ros_its_msgs::CLWarning_<ContainerAllocator>& v)
  {
    s << indent << "car_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.car_name);
    s << indent << "CLWType: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.CLWType);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROS_ITS_MSGS_MESSAGE_CLWARNING_H
