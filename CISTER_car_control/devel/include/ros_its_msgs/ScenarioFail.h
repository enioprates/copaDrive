// Generated by gencpp from file ros_its_msgs/ScenarioFail.msg
// DO NOT EDIT!


#ifndef ROS_ITS_MSGS_MESSAGE_SCENARIOFAIL_H
#define ROS_ITS_MSGS_MESSAGE_SCENARIOFAIL_H


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
struct ScenarioFail_
{
  typedef ScenarioFail_<ContainerAllocator> Type;

  ScenarioFail_()
    : car_name()
    , FailType()  {
    }
  ScenarioFail_(const ContainerAllocator& _alloc)
    : car_name(_alloc)
    , FailType(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _car_name_type;
  _car_name_type car_name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _FailType_type;
  _FailType_type FailType;





  typedef boost::shared_ptr< ::ros_its_msgs::ScenarioFail_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ros_its_msgs::ScenarioFail_<ContainerAllocator> const> ConstPtr;

}; // struct ScenarioFail_

typedef ::ros_its_msgs::ScenarioFail_<std::allocator<void> > ScenarioFail;

typedef boost::shared_ptr< ::ros_its_msgs::ScenarioFail > ScenarioFailPtr;
typedef boost::shared_ptr< ::ros_its_msgs::ScenarioFail const> ScenarioFailConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ros_its_msgs::ScenarioFail_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ros_its_msgs::ScenarioFail_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ros_its_msgs::ScenarioFail_<ContainerAllocator1> & lhs, const ::ros_its_msgs::ScenarioFail_<ContainerAllocator2> & rhs)
{
  return lhs.car_name == rhs.car_name &&
    lhs.FailType == rhs.FailType;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ros_its_msgs::ScenarioFail_<ContainerAllocator1> & lhs, const ::ros_its_msgs::ScenarioFail_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ros_its_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ros_its_msgs::ScenarioFail_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_its_msgs::ScenarioFail_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_its_msgs::ScenarioFail_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_its_msgs::ScenarioFail_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_its_msgs::ScenarioFail_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_its_msgs::ScenarioFail_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ros_its_msgs::ScenarioFail_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e62d5c88cdc7df31630e13cbde053d43";
  }

  static const char* value(const ::ros_its_msgs::ScenarioFail_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe62d5c88cdc7df31ULL;
  static const uint64_t static_value2 = 0x630e13cbde053d43ULL;
};

template<class ContainerAllocator>
struct DataType< ::ros_its_msgs::ScenarioFail_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ros_its_msgs/ScenarioFail";
  }

  static const char* value(const ::ros_its_msgs::ScenarioFail_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ros_its_msgs::ScenarioFail_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"string car_name\n"
"\n"
"string FailType\n"
"\n"
"\n"
;
  }

  static const char* value(const ::ros_its_msgs::ScenarioFail_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ros_its_msgs::ScenarioFail_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.car_name);
      stream.next(m.FailType);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ScenarioFail_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ros_its_msgs::ScenarioFail_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ros_its_msgs::ScenarioFail_<ContainerAllocator>& v)
  {
    s << indent << "car_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.car_name);
    s << indent << "FailType: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.FailType);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROS_ITS_MSGS_MESSAGE_SCENARIOFAIL_H
