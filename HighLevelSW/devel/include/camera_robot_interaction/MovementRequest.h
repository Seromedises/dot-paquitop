// Generated by gencpp from file camera_robot_interaction/MovementRequest.msg
// DO NOT EDIT!


#ifndef CAMERA_ROBOT_INTERACTION_MESSAGE_MOVEMENTREQUEST_H
#define CAMERA_ROBOT_INTERACTION_MESSAGE_MOVEMENTREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace camera_robot_interaction
{
template <class ContainerAllocator>
struct MovementRequest_
{
  typedef MovementRequest_<ContainerAllocator> Type;

  MovementRequest_()
    : r11(0.0)
    , r12(0.0)
    , r13(0.0)
    , r14(0.0)
    , r21(0.0)
    , r22(0.0)
    , r23(0.0)
    , r24(0.0)
    , r31(0.0)
    , r32(0.0)
    , r33(0.0)
    , r34(0.0)
    , gripper(0.0)
    , rotate(false)
    , rest_pos(false)  {
    }
  MovementRequest_(const ContainerAllocator& _alloc)
    : r11(0.0)
    , r12(0.0)
    , r13(0.0)
    , r14(0.0)
    , r21(0.0)
    , r22(0.0)
    , r23(0.0)
    , r24(0.0)
    , r31(0.0)
    , r32(0.0)
    , r33(0.0)
    , r34(0.0)
    , gripper(0.0)
    , rotate(false)
    , rest_pos(false)  {
  (void)_alloc;
    }



   typedef float _r11_type;
  _r11_type r11;

   typedef float _r12_type;
  _r12_type r12;

   typedef float _r13_type;
  _r13_type r13;

   typedef float _r14_type;
  _r14_type r14;

   typedef float _r21_type;
  _r21_type r21;

   typedef float _r22_type;
  _r22_type r22;

   typedef float _r23_type;
  _r23_type r23;

   typedef float _r24_type;
  _r24_type r24;

   typedef float _r31_type;
  _r31_type r31;

   typedef float _r32_type;
  _r32_type r32;

   typedef float _r33_type;
  _r33_type r33;

   typedef float _r34_type;
  _r34_type r34;

   typedef float _gripper_type;
  _gripper_type gripper;

   typedef uint8_t _rotate_type;
  _rotate_type rotate;

   typedef uint8_t _rest_pos_type;
  _rest_pos_type rest_pos;





  typedef boost::shared_ptr< ::camera_robot_interaction::MovementRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::camera_robot_interaction::MovementRequest_<ContainerAllocator> const> ConstPtr;

}; // struct MovementRequest_

typedef ::camera_robot_interaction::MovementRequest_<std::allocator<void> > MovementRequest;

typedef boost::shared_ptr< ::camera_robot_interaction::MovementRequest > MovementRequestPtr;
typedef boost::shared_ptr< ::camera_robot_interaction::MovementRequest const> MovementRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::camera_robot_interaction::MovementRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::camera_robot_interaction::MovementRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::camera_robot_interaction::MovementRequest_<ContainerAllocator1> & lhs, const ::camera_robot_interaction::MovementRequest_<ContainerAllocator2> & rhs)
{
  return lhs.r11 == rhs.r11 &&
    lhs.r12 == rhs.r12 &&
    lhs.r13 == rhs.r13 &&
    lhs.r14 == rhs.r14 &&
    lhs.r21 == rhs.r21 &&
    lhs.r22 == rhs.r22 &&
    lhs.r23 == rhs.r23 &&
    lhs.r24 == rhs.r24 &&
    lhs.r31 == rhs.r31 &&
    lhs.r32 == rhs.r32 &&
    lhs.r33 == rhs.r33 &&
    lhs.r34 == rhs.r34 &&
    lhs.gripper == rhs.gripper &&
    lhs.rotate == rhs.rotate &&
    lhs.rest_pos == rhs.rest_pos;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::camera_robot_interaction::MovementRequest_<ContainerAllocator1> & lhs, const ::camera_robot_interaction::MovementRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace camera_robot_interaction

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::camera_robot_interaction::MovementRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::camera_robot_interaction::MovementRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::camera_robot_interaction::MovementRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::camera_robot_interaction::MovementRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::camera_robot_interaction::MovementRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::camera_robot_interaction::MovementRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::camera_robot_interaction::MovementRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5f15cf2b3ea3f6f2b16936c710d373fd";
  }

  static const char* value(const ::camera_robot_interaction::MovementRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5f15cf2b3ea3f6f2ULL;
  static const uint64_t static_value2 = 0xb16936c710d373fdULL;
};

template<class ContainerAllocator>
struct DataType< ::camera_robot_interaction::MovementRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "camera_robot_interaction/MovementRequest";
  }

  static const char* value(const ::camera_robot_interaction::MovementRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::camera_robot_interaction::MovementRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 r11\n"
"float32 r12\n"
"float32 r13\n"
"float32 r14\n"
"float32 r21\n"
"float32 r22\n"
"float32 r23\n"
"float32 r24\n"
"float32 r31\n"
"float32 r32\n"
"float32 r33\n"
"float32 r34\n"
"float32 gripper\n"
"bool rotate\n"
"bool rest_pos\n"
;
  }

  static const char* value(const ::camera_robot_interaction::MovementRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::camera_robot_interaction::MovementRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.r11);
      stream.next(m.r12);
      stream.next(m.r13);
      stream.next(m.r14);
      stream.next(m.r21);
      stream.next(m.r22);
      stream.next(m.r23);
      stream.next(m.r24);
      stream.next(m.r31);
      stream.next(m.r32);
      stream.next(m.r33);
      stream.next(m.r34);
      stream.next(m.gripper);
      stream.next(m.rotate);
      stream.next(m.rest_pos);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MovementRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::camera_robot_interaction::MovementRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::camera_robot_interaction::MovementRequest_<ContainerAllocator>& v)
  {
    s << indent << "r11: ";
    Printer<float>::stream(s, indent + "  ", v.r11);
    s << indent << "r12: ";
    Printer<float>::stream(s, indent + "  ", v.r12);
    s << indent << "r13: ";
    Printer<float>::stream(s, indent + "  ", v.r13);
    s << indent << "r14: ";
    Printer<float>::stream(s, indent + "  ", v.r14);
    s << indent << "r21: ";
    Printer<float>::stream(s, indent + "  ", v.r21);
    s << indent << "r22: ";
    Printer<float>::stream(s, indent + "  ", v.r22);
    s << indent << "r23: ";
    Printer<float>::stream(s, indent + "  ", v.r23);
    s << indent << "r24: ";
    Printer<float>::stream(s, indent + "  ", v.r24);
    s << indent << "r31: ";
    Printer<float>::stream(s, indent + "  ", v.r31);
    s << indent << "r32: ";
    Printer<float>::stream(s, indent + "  ", v.r32);
    s << indent << "r33: ";
    Printer<float>::stream(s, indent + "  ", v.r33);
    s << indent << "r34: ";
    Printer<float>::stream(s, indent + "  ", v.r34);
    s << indent << "gripper: ";
    Printer<float>::stream(s, indent + "  ", v.gripper);
    s << indent << "rotate: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.rotate);
    s << indent << "rest_pos: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.rest_pos);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CAMERA_ROBOT_INTERACTION_MESSAGE_MOVEMENTREQUEST_H