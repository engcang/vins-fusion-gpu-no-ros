#ifndef ROS_THINGS_H
#define ROS_THINGS_H

#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <stdint.h>
#include <sys/time.h>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <boost/math/special_functions/round.hpp>
#include <iostream>

//Time.h
namespace  ros //DataStucture
{
 /*********************************************************************
 ** Functions
 *********************************************************************/

     inline void normalizeSecNSec(uint64_t& sec, uint64_t& nsec)
     {
       uint64_t nsec_part = nsec % 1000000000UL;
       uint64_t sec_part = nsec / 1000000000UL;
     
       if (sec_part > UINT_MAX)
         throw std::runtime_error("Time is out of dual 32-bit range");
     
       sec += sec_part;
       nsec = nsec_part;
     }
     
     inline void normalizeSecNSec(uint32_t& sec, uint32_t& nsec)
     {
       uint64_t sec64 = sec;
       uint64_t nsec64 = nsec;
     
       normalizeSecNSec(sec64, nsec64);
     
       sec = (uint32_t)sec64;
       nsec = (uint32_t)nsec64;
    }
 
   inline void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec)
   {
    int64_t nsec_part = nsec;
    int64_t sec_part = sec;
 
    while (nsec_part >= 1000000000L)
    {
      nsec_part -= 1000000000L;
      ++sec_part;
    }
     while (nsec_part < 0)
    {
      nsec_part += 1000000000L;
      --sec_part;
    }
    if (sec_part < 0 || sec_part > INT_MAX)
     throw std::runtime_error("Time is out of dual 32-bit range");
 
    sec = sec_part;
    nsec = nsec_part;
   }

  template<class T>
  class TimeBase
  {
  public:
    uint32_t sec, nsec;

    TimeBase() : sec(0), nsec(0) { }
    TimeBase(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec)
    {
      normalizeSecNSec(sec, nsec);
    }
    explicit TimeBase(double t) { fromSec(t); }
    ~TimeBase() {}
  
    bool operator==(const T &rhs) const
    {return sec == rhs.sec && nsec == rhs.nsec;}
    inline bool operator!=(const T &rhs) const { return !(*static_cast<const T*>(this) == rhs); }
    bool operator>(const T &rhs) const
    {
     if(sec > rhs.sec)
       return true;
     else if(sec==rhs.sec && nsec > rhs.nsec)
       return true;
     return false;
    }
    bool operator<(const T &rhs) const
    {
     if(sec < rhs.sec)
       return true;
     else if(sec==rhs.sec && nsec < rhs.nsec)
       return true;
     return false;
    }
    bool operator>=(const T &rhs) const
    {
     if(sec > rhs.sec)
       return true;
     else if(sec==rhs.sec && nsec >= rhs.nsec)
       return true;
     return false;
    }
    bool operator<=(const T &rhs) const
     {
     if(sec < rhs.sec)
       return true;
     else if(sec==rhs.sec && nsec <= rhs.nsec)
       return true;
     return false;
    }

    double toSec()  const { return (double)sec + 1e-9*(double)nsec; };
    T& fromSec(double t) {
      sec = (uint32_t)floor(t);
      nsec = (uint32_t)boost::math::round((t-sec) * 1e9);
      // avoid rounding errors
      sec += (nsec / 1000000000ul);
      nsec %= 1000000000ul;
      return *static_cast<T*>(this);
    }

    uint64_t toNSec() const {return (uint64_t)sec*1000000000ull + (uint64_t)nsec;  }
    T& fromNSec(uint64_t t);

    inline bool isZero() const { return sec == 0 && nsec == 0; }
    inline bool is_zero() const { return isZero(); }

  };

  class  Time : public TimeBase<Time>
  {
  public:
    Time()
      : TimeBase<Time>()
    {}

    Time(uint32_t _sec, uint32_t _nsec)
      : TimeBase<Time>(_sec, _nsec)
    {}

    explicit Time(double t) { fromSec(t); }


  };
}
//Time.h

namespace std_msgs
{

//Float32.h
template <class ContainerAllocator>
struct Float32_
{
  typedef Float32_<ContainerAllocator> Type;

  Float32_()
    : data(0.0)  {
    }
  Float32_(const ContainerAllocator& _alloc)
    : data(0.0)  {
  (void)_alloc;
    }



   typedef float _data_type;
  _data_type data;




  typedef boost::shared_ptr< ::std_msgs::Float32_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::std_msgs::Float32_<ContainerAllocator> const> ConstPtr;

};
//Float32.h

//Header.h
template <class ContainerAllocator>
struct Header_
{
  typedef Header_<ContainerAllocator> Type;

  Header_()
    : seq(0)
    , stamp()
    , frame_id()  {
    }
  Header_(const ContainerAllocator& _alloc)
    : seq(0)
    , stamp()
    , frame_id(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _seq_type;
  _seq_type seq;

   typedef ros::Time _stamp_type;
  _stamp_type stamp;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _frame_id_type;
  _frame_id_type frame_id;




  typedef boost::shared_ptr< std_msgs::Header_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< std_msgs::Header_<ContainerAllocator> const> ConstPtr;

}; // struct Header_

typedef std_msgs::Header_<std::allocator<void> > Header;

typedef boost::shared_ptr< std_msgs::Header > HeaderPtr;
typedef boost::shared_ptr< std_msgs::Header const> HeaderConstPtr;

//Header.h

}


namespace geometry_msgs
{

//Vector3.h
template <class ContainerAllocator>
struct Vector3_
{
  typedef Vector3_<ContainerAllocator> Type;

  Vector3_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  Vector3_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;




  typedef boost::shared_ptr< geometry_msgs::Vector3_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< geometry_msgs::Vector3_<ContainerAllocator> const> ConstPtr;

}; // struct Vector3_

typedef geometry_msgs::Vector3_<std::allocator<void> > Vector3;

typedef boost::shared_ptr< geometry_msgs::Vector3 > Vector3Ptr;
typedef boost::shared_ptr< geometry_msgs::Vector3 const> Vector3ConstPtr;
//Vector3.h

//Quaternion.h
template <class ContainerAllocator>
struct Quaternion_
{
  typedef Quaternion_<ContainerAllocator> Type;

  Quaternion_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , w(0.0)  {
    }
  Quaternion_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , w(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef double _w_type;
  _w_type w;




  typedef boost::shared_ptr< ::geometry_msgs::Quaternion_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::Quaternion_<ContainerAllocator> const> ConstPtr;

}; // struct Quaternion_

typedef ::geometry_msgs::Quaternion_<std::allocator<void> > Quaternion;

typedef boost::shared_ptr< ::geometry_msgs::Quaternion > QuaternionPtr;
typedef boost::shared_ptr< ::geometry_msgs::Quaternion const> QuaternionConstPtr;
//Quaternion.h

//Point.h
template <class ContainerAllocator>
struct Point_
{
  typedef Point_<ContainerAllocator> Type;

  Point_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  Point_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;




  typedef boost::shared_ptr< ::geometry_msgs::Point_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::Point_<ContainerAllocator> const> ConstPtr;

}; // struct Point_

typedef ::geometry_msgs::Point_<std::allocator<void> > Point;

typedef boost::shared_ptr< ::geometry_msgs::Point > PointPtr;
typedef boost::shared_ptr< ::geometry_msgs::Point const> PointConstPtr;
//Point.h

//Pose.h
template <class ContainerAllocator>
struct Pose_
{
  typedef Pose_<ContainerAllocator> Type;

  Pose_()
    : position()
    , orientation()  {
    }
  Pose_(const ContainerAllocator& _alloc)
    : position(_alloc)
    , orientation(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _orientation_type;
  _orientation_type orientation;




  typedef boost::shared_ptr< ::geometry_msgs::Pose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::Pose_<ContainerAllocator> const> ConstPtr;

}; // struct Pose_

typedef ::geometry_msgs::Pose_<std::allocator<void> > Pose;

typedef boost::shared_ptr< ::geometry_msgs::Pose > PosePtr;
typedef boost::shared_ptr< ::geometry_msgs::Pose const> PoseConstPtr;
//Pose.h

//PoseWithCovariance.h
template <class ContainerAllocator>
struct PoseWithCovariance_
{
  typedef PoseWithCovariance_<ContainerAllocator> Type;

  PoseWithCovariance_()
    : pose()
    , covariance()  {
      covariance.assign(0.0);
  }
  PoseWithCovariance_(const ContainerAllocator& _alloc)
    : pose(_alloc)
    , covariance()  {
  (void)_alloc;
      covariance.assign(0.0);
  }



   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef boost::array<double, 36>  _covariance_type;
  _covariance_type covariance;




  typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> const> ConstPtr;

}; // struct PoseWithCovariance_

typedef ::geometry_msgs::PoseWithCovariance_<std::allocator<void> > PoseWithCovariance;

typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovariance > PoseWithCovariancePtr;
typedef boost::shared_ptr< ::geometry_msgs::PoseWithCovariance const> PoseWithCovarianceConstPtr;
//PoseWithCovariance.h

//PoseStamped.h
template <class ContainerAllocator>
struct PoseStamped_
{
  typedef PoseStamped_<ContainerAllocator> Type;

  PoseStamped_()
    : header()
    , pose()  {
    }
  PoseStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;




  typedef boost::shared_ptr< ::geometry_msgs::PoseStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::PoseStamped_<ContainerAllocator> const> ConstPtr;

}; // struct PoseStamped_

typedef ::geometry_msgs::PoseStamped_<std::allocator<void> > PoseStamped;

typedef boost::shared_ptr< ::geometry_msgs::PoseStamped > PoseStampedPtr;
typedef boost::shared_ptr< ::geometry_msgs::PoseStamped const> PoseStampedConstPtr;
//PoseStamped.h

//PointStamped.h
template <class ContainerAllocator>
struct PointStamped_
{
  typedef PointStamped_<ContainerAllocator> Type;

  PointStamped_()
    : header()
    , point()  {
    }
  PointStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , point(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _point_type;
  _point_type point;




  typedef boost::shared_ptr< ::geometry_msgs::PointStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::PointStamped_<ContainerAllocator> const> ConstPtr;

}; // struct PointStamped_

typedef ::geometry_msgs::PointStamped_<std::allocator<void> > PointStamped;

typedef boost::shared_ptr< ::geometry_msgs::PointStamped > PointStampedPtr;
typedef boost::shared_ptr< ::geometry_msgs::PointStamped const> PointStampedConstPtr;
//PointStamped.h

//Point32.h
template <class ContainerAllocator>
struct Point32_
{
  typedef Point32_<ContainerAllocator> Type;

  Point32_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  Point32_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;




  typedef boost::shared_ptr< ::geometry_msgs::Point32_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::Point32_<ContainerAllocator> const> ConstPtr;

}; // struct Point32_

typedef ::geometry_msgs::Point32_<std::allocator<void> > Point32;

typedef boost::shared_ptr< ::geometry_msgs::Point32 > Point32Ptr;
typedef boost::shared_ptr< ::geometry_msgs::Point32 const> Point32ConstPtr;
//Point32.h

//Twist.h
template <class ContainerAllocator>
struct Twist_
{
  typedef Twist_<ContainerAllocator> Type;

  Twist_()
    : linear()
    , angular()  {
    }
  Twist_(const ContainerAllocator& _alloc)
    : linear(_alloc)
    , angular(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _linear_type;
  _linear_type linear;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _angular_type;
  _angular_type angular;




  typedef boost::shared_ptr< ::geometry_msgs::Twist_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::Twist_<ContainerAllocator> const> ConstPtr;

}; // struct Twist_

typedef ::geometry_msgs::Twist_<std::allocator<void> > Twist;

typedef boost::shared_ptr< ::geometry_msgs::Twist > TwistPtr;
typedef boost::shared_ptr< ::geometry_msgs::Twist const> TwistConstPtr;
//Twist.h

//TwistWithCovariance.h
template <class ContainerAllocator>
struct TwistWithCovariance_
{
  typedef TwistWithCovariance_<ContainerAllocator> Type;

  TwistWithCovariance_()
    : twist()
    , covariance()  {
      covariance.assign(0.0);
  }
  TwistWithCovariance_(const ContainerAllocator& _alloc)
    : twist(_alloc)
    , covariance()  {
  (void)_alloc;
      covariance.assign(0.0);
  }



   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _twist_type;
  _twist_type twist;

   typedef boost::array<double, 36>  _covariance_type;
  _covariance_type covariance;




  typedef boost::shared_ptr< ::geometry_msgs::TwistWithCovariance_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::TwistWithCovariance_<ContainerAllocator> const> ConstPtr;

}; // struct TwistWithCovariance_

typedef ::geometry_msgs::TwistWithCovariance_<std::allocator<void> > TwistWithCovariance;

typedef boost::shared_ptr< ::geometry_msgs::TwistWithCovariance > TwistWithCovariancePtr;
typedef boost::shared_ptr< ::geometry_msgs::TwistWithCovariance const> TwistWithCovarianceConstPtr;
//TwistWithCovariance.h

}



namespace sensor_msgs
{

// ChannelFloat32.h
template <class ContainerAllocator>
struct ChannelFloat32_
{
  typedef ChannelFloat32_<ContainerAllocator> Type;

  ChannelFloat32_()
    : name()
    , values()  {
    }
  ChannelFloat32_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , values(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _values_type;
  _values_type values;




  typedef boost::shared_ptr< sensor_msgs::ChannelFloat32_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< sensor_msgs::ChannelFloat32_<ContainerAllocator> const> ConstPtr;

}; // struct ChannelFloat32_

typedef sensor_msgs::ChannelFloat32_<std::allocator<void> > ChannelFloat32;

typedef boost::shared_ptr< sensor_msgs::ChannelFloat32 > ChannelFloat32Ptr;
typedef boost::shared_ptr< sensor_msgs::ChannelFloat32 const> ChannelFloat32ConstPtr;
// ChannelFloat32.h

//PointCloud.h
template <class ContainerAllocator>
struct PointCloud_
{
  typedef PointCloud_<ContainerAllocator> Type;

  PointCloud_()
    : header()
    , points()
    , channels()  {
    }
  PointCloud_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , points(_alloc)
    , channels(_alloc)  {
  (void)_alloc;
    }



   typedef  std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< geometry_msgs::Point32_<ContainerAllocator> , typename ContainerAllocator::template rebind< geometry_msgs::Point32_<ContainerAllocator> >::other >  _points_type;
  _points_type points;

   typedef std::vector< sensor_msgs::ChannelFloat32_<ContainerAllocator> , typename ContainerAllocator::template rebind< sensor_msgs::ChannelFloat32_<ContainerAllocator> >::other >  _channels_type;
  _channels_type channels;




  typedef boost::shared_ptr< sensor_msgs::PointCloud_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< sensor_msgs::PointCloud_<ContainerAllocator> const> ConstPtr;

}; // struct PointCloud_

typedef sensor_msgs::PointCloud_<std::allocator<void> > PointCloud;

typedef boost::shared_ptr< sensor_msgs::PointCloud > PointCloudPtr;
typedef boost::shared_ptr< sensor_msgs::PointCloud const> PointCloudConstPtr;
//PointCloud.h

//Imu.h
template <class ContainerAllocator>
struct Imu_
{
  typedef Imu_<ContainerAllocator> Type;

  Imu_()
    : header()
    , orientation()
    , orientation_covariance()
    , angular_velocity()
    , angular_velocity_covariance()
    , linear_acceleration()
    , linear_acceleration_covariance()  {
      orientation_covariance.assign(0.0);

      angular_velocity_covariance.assign(0.0);

      linear_acceleration_covariance.assign(0.0);
  }
  Imu_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , orientation(_alloc)
    , orientation_covariance()
    , angular_velocity(_alloc)
    , angular_velocity_covariance()
    , linear_acceleration(_alloc)
    , linear_acceleration_covariance()  {
  (void)_alloc;
      orientation_covariance.assign(0.0);

      angular_velocity_covariance.assign(0.0);

      linear_acceleration_covariance.assign(0.0);
  }



   typedef  std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  geometry_msgs::Quaternion_<ContainerAllocator>  _orientation_type;
  _orientation_type orientation;

   typedef boost::array<double, 9>  _orientation_covariance_type;
  _orientation_covariance_type orientation_covariance;

   typedef  geometry_msgs::Vector3_<ContainerAllocator>  _angular_velocity_type;
  _angular_velocity_type angular_velocity;

   typedef boost::array<double, 9>  _angular_velocity_covariance_type;
  _angular_velocity_covariance_type angular_velocity_covariance;

   typedef  geometry_msgs::Vector3_<ContainerAllocator>  _linear_acceleration_type;
  _linear_acceleration_type linear_acceleration;

   typedef boost::array<double, 9>  _linear_acceleration_covariance_type;
  _linear_acceleration_covariance_type linear_acceleration_covariance;




  typedef boost::shared_ptr< sensor_msgs::Imu_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< sensor_msgs::Imu_<ContainerAllocator> const> ConstPtr;

}; // struct Imu_

typedef sensor_msgs::Imu_<std::allocator<void> > Imu;

typedef boost::shared_ptr< sensor_msgs::Imu > ImuPtr;
typedef boost::shared_ptr< sensor_msgs::Imu const> ImuConstPtr;
}
//Imu.h



namespace nav_msgs
{

//Path.h
template <class ContainerAllocator>
struct Path_
{
  typedef Path_<ContainerAllocator> Type;

  Path_()
    : header()
    , poses()  {
    }
  Path_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , poses(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::geometry_msgs::PoseStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::other >  _poses_type;
  _poses_type poses;




  typedef boost::shared_ptr< ::nav_msgs::Path_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nav_msgs::Path_<ContainerAllocator> const> ConstPtr;

}; // struct Path_

typedef ::nav_msgs::Path_<std::allocator<void> > Path;

typedef boost::shared_ptr< ::nav_msgs::Path > PathPtr;
typedef boost::shared_ptr< ::nav_msgs::Path const> PathConstPtr;
//Path.h

//Odometry.h
template <class ContainerAllocator>
struct Odometry_
{
  typedef Odometry_<ContainerAllocator> Type;

  Odometry_()
    : header()
    , child_frame_id()
    , pose()
    , twist()  {
    }
  Odometry_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , child_frame_id(_alloc)
    , pose(_alloc)
    , twist(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _child_frame_id_type;
  _child_frame_id_type child_frame_id;

   typedef  ::geometry_msgs::PoseWithCovariance_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::TwistWithCovariance_<ContainerAllocator>  _twist_type;
  _twist_type twist;




  typedef boost::shared_ptr< ::nav_msgs::Odometry_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nav_msgs::Odometry_<ContainerAllocator> const> ConstPtr;

}; // struct Odometry_

typedef ::nav_msgs::Odometry_<std::allocator<void> > Odometry;

typedef boost::shared_ptr< ::nav_msgs::Odometry > OdometryPtr;
typedef boost::shared_ptr< ::nav_msgs::Odometry const> OdometryConstPtr;

//Odometry.h
}


#endif