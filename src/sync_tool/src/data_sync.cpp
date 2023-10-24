#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/bind/bind.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include "rosbag/bag.h"
#include "ctime"
#include "time.h"
#include <string>

using namespace boost::placeholders;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;


rosbag::Bag bag_record;

string int2string(int value)
{
    stringstream ss;
    ss<<value;
    return ss.str();
}

void callback( const sensor_msgs::ImageConstPtr& img_msg,
               const sensor_msgs::ImuConstPtr& imu_msg,
               const dvs_msgs::EventArray::ConstPtr& event_msg,
               const sensor_msgs::ImageConstPtr& render_msg,    
               const sensor_msgs::PointCloud2ConstPtr& radar_msg        
)
{
    ROS_INFO("Enter Publish");

    bag_record.write("davis_left/image_raw",img_msg->header.stamp.now(),*img_msg);
    bag_record.write("davis_left/imu",imu_msg->header.stamp.now(),*imu_msg);
    bag_record.write("davis_left/events ",event_msg->header.stamp.now(),*event_msg);
    bag_record.write("dvs_rendering",render_msg->header.stamp.now(),*render_msg);
    bag_record.write("ti_mmwave/radar_scan_pcl",radar_msg->header.stamp.now(),*radar_msg);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync_node");
  ros::NodeHandle nh;
  ROS_INFO("start message filter");

  time_t t=std::time(nullptr);
  struct tm * now = std::localtime( & t );
  string file_name;
  file_name=int2string(now->tm_year)+
          '-'+int2string(now->tm_mon)+
          '-'+int2string(now->tm_mday)+
          '-'+int2string(now->tm_hour)+
          '-'+int2string(now->tm_min)+
          '-'+int2string(now->tm_sec)+
            ".bag";
  bag_record.open(file_name,rosbag::bagmode::Write);

  string img_listen_to, imu_listen_to, event_listen_to, render_listen_to, radar_listen_to;
  nh.getParam("img_listen_to", img_listen_to);
  nh.getParam("imu_listen_to", imu_listen_to);
  nh.getParam("event_listen_to", event_listen_to);
  nh.getParam("render_listen_to", render_listen_to);
  nh.getParam("radar_listen_to", radar_listen_to);

  message_filters::Subscriber<sensor_msgs::Image> img_sub_(nh,img_listen_to,2000,ros::TransportHints().tcpNoDelay()) ;  
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub_(nh,imu_listen_to,50000,ros::TransportHints().tcpNoDelay()) ;  
  message_filters::Subscriber<dvs_msgs::EventArray> event_sub_(nh,event_listen_to,1000,ros::TransportHints().tcpNoDelay()) ;  
  message_filters::Subscriber<sensor_msgs::Image> render_sub_(nh, render_listen_to,2000,ros::TransportHints().tcpNoDelay()) ;  
  message_filters::Subscriber<sensor_msgs::PointCloud2> radar_sub_(nh, radar_listen_to,1000,ros::TransportHints().tcpNoDelay()) ; 
  

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
  sensor_msgs::Imu,
  dvs_msgs::EventArray,
  sensor_msgs::Image,
  sensor_msgs::PointCloud2> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(300),
                                                   img_sub_,
                                                   imu_sub_,
                                                   event_sub_,
                                                   render_sub_,
                                                   radar_sub_
                                                  );
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));
  ROS_INFO("not working");
  ros::spin();
  bag_record.close();
  return 0;
}