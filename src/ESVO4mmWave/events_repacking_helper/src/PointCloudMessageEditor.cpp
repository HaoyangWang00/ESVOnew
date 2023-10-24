#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

struct PointCloudEditor
{
    PointCloudEditor(double frequency, std::string &messageTopic)
        : bFirstMessage_(true)
    {
        pclCloud_ = pcl::PointCloud<pcl::PointXYZ>();
        duration_threshold_ = 1.0 / frequency;
        message_topic_ = messageTopic;
    }

    void resetBuffer(const ros::Time &timestamp)
    {
        start_time_ = timestamp;
        end_time_ = ros::Time(start_time_.toSec() + duration_threshold_);
        pclCloud_.points.clear();
        pclCloud_.header.stamp = pcl_conversions::toPCL(end_time_);
    }

    void insertPoint(ros::Time &timestamp, const pcl::PointXYZ &point, rosbag::Bag *bag)
    {
        if (bFirstMessage_)
        {
            resetBuffer(ros::Time(point.data[3]));  // Assuming timestamp is in the fourth element
            bFirstMessage_ = false;
        }

        if (timestamp.toSec() >= end_time_.toSec())
        {
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(pclCloud_, cloud_msg);
            cloud_msg.header.stamp.fromSec(end_time_.toSec());
            cloud_msg.header.frame_id = "ti_mmwave";  // Change the frame_id if needed
            bag->write(message_topic_.c_str(), end_time_, cloud_msg);
            resetBuffer(end_time_);
        }
        pclCloud_.points.push_back(point);
    }

    // variables
    pcl::PointCloud<pcl::PointXYZ> pclCloud_;
    double duration_threshold_;
    ros::Time start_time_, end_time_;
    bool bFirstMessage_;
    std::string message_topic_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pointcloud_editor");

    std::string src_bag_path(argv[1]);
    std::string dst_bag_path(argv[2]);
    rosbag::Bag bag_src, bag_dst;
    bag_src.open(src_bag_path.c_str(), rosbag::bagmode::Read);
    bag_dst.open(dst_bag_path.c_str(), rosbag::bagmode::Write);

    if (!bag_src.isOpen())
    {
        ROS_INFO("No rosbag is found in the given path.");
        exit(-1);
    }
    else
    {
        ROS_INFO("***********Input Bag File Name ***********");
        ROS_INFO(argv[1]);
        ROS_INFO("******************************************");
    }

    if (!bag_dst.isOpen())
    {
        ROS_INFO("The dst bag is not opened.");
        exit(-1);
    }
    else
    {
        ROS_INFO("***********Output Bag File Name ***********");
        ROS_INFO(argv[2]);
        ROS_INFO("***************************");
    }

    const double frequency = 1000;

    // process point clouds
    std::vector<std::string> topics;
    topics.push_back(std::string("/ti_mmwave/radar_scan_pcl"));  // Change the topic name
    std::vector<std::string> topics_rename;
    topics_rename.push_back(std::string("/ti_mmwave/radar_sliced_pcl"));  // Change the new topic name

    for (size_t i = 0; i < topics.size(); i++)
    {
        rosbag::View view(bag_src, rosbag::TopicQuery(topics[i]));
        PointCloudEditor pclEditor(frequency, topics_rename[i]);

        // topic loop
        for (rosbag::MessageInstance const m : view)
        {
            sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            pcl::fromROSMsg(*msg, pcl_cloud);

            pclEditor.resetBuffer(msg->header.stamp);

            // point loop
            for (const pcl::PointXYZ &point : pcl_cloud.points)
            {
                pclEditor.insertPoint(point, &bag_dst);
            }
        }
    }

    bag_src.close();
    bag_dst.close();
    return 0;
}
