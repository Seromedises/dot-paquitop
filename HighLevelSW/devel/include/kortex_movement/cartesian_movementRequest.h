// Generated by gencpp from file kortex_movement/cartesian_movementRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_MOVEMENT_MESSAGE_CARTESIAN_MOVEMENTREQUEST_H
#define KORTEX_MOVEMENT_MESSAGE_CARTESIAN_MOVEMENTREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kortex_movement
{
template <class ContainerAllocator>
struct cartesian_movementRequest_
{
  typedef cartesian_movementRequest_<ContainerAllocator> Type;

  cartesian_movementRequest_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , thetax(0.0)
    , thetay(0.0)
    , thetaz(0.0)  {
    }
  cartesian_movementRequest_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , thetax(0.0)
    , thetay(0.0)
    , thetaz(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;

   typedef float _thetax_type;
  _thetax_type thetax;

   typedef float _thetay_type;
  _thetay_type thetay;

   typedef float _thetaz_type;
  _thetaz_type thetaz;





  typedef boost::shared_ptr< ::kortex_movement::cartesian_movementRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_movement::cartesian_movementRequest_<ContainerAllocator> const> ConstPtr;

}; // struct cartesian_movementRequest_

typedef ::kortex_movement::cartesian_movementRequest_<std::allocator<void> > cartesian_movementRequest;

typedef boost::shared_ptr< ::kortex_movement::cartesian_movementRequest > cartesian_movementRequestPtr;
typedef boost::shared_ptr< ::kortex_movement::cartesian_movementRequest const> cartesian_movementRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_movement::cartesian_movementRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_movement::cartesian_movementRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_movement::cartesian_movementRequest_<ContainerAllocator1> & lhs, const ::kortex_movement::cartesian_movementRequest_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.thetax == rhs.thetax &&
    lhs.thetay == rhs.thetay &&
    lhs.thetaz == rhs.thetaz;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_movement::cartesian_movementRequest_<ContainerAllocator1> & lhs, const ::kortex_movement::cartesian_movementRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_movement

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::kortex_movement::cartesian_movementRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_movement::cartesian_movementRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_movement::cartesian_movementRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_movement::cartesian_movementRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_movement::cartesian_movementRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_movement::cartesian_movementRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_movement::cartesian_movementRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "343f88ee256329e1ad031142d25c7292";
  }

  static const char* value(const ::kortex_movement::cartesian_movementRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x343f88ee256329e1ULL;
  static const uint64_t static_value2 = 0xad031142d25c7292ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_movement::cartesian_movementRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_movement/cartesian_movementRequest";
  }

  static const char* value(const ::kortex_movement::cartesian_movementRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_movement::cartesian_movementRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n"
"float32 y\n"
"float32 z\n"
"float32 thetax\n"
"float32 thetay\n"
"float32 thetaz\n"
;
  }

  static const char* value(const ::kortex_movement::cartesian_movementRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_movement::cartesian_movementRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.thetax);
      stream.next(m.thetay);
      stream.next(m.thetaz);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct cartesian_movementRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_movement::cartesian_movementRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_movement::cartesian_movementRequest_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "thetax: ";
    Printer<float>::stream(s, indent + "  ", v.thetax);
    s << indent << "thetay: ";
    Printer<float>::stream(s, indent + "  ", v.thetay);
    s << indent << "thetaz: ";
    Printer<float>::stream(s, indent + "  ", v.thetaz);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_MOVEMENT_MESSAGE_CARTESIAN_MOVEMENTREQUEST_H