#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ti_mmwave_rospkg/RadarScan.h>  // 替换成正确的消息包名

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_scan_to_point_cloud");

    // 从命令行参数获取输入和输出 rosbag 文件的路径
    if (argc < 3)
    {
        ROS_ERROR("Usage: radar_scan_to_point_cloud <input_bag_file> <output_bag_file>");
        return 1;
    }

    std::string input_bag_file = argv[1];
    std::string output_bag_file = argv[2];

    // 打开输入 rosbag 文件
    rosbag::Bag input_bag;
    input_bag.open(input_bag_file, rosbag::bagmode::Read);

    // 打开输出 rosbag 文件
    rosbag::Bag output_bag;
    output_bag.open(output_bag_file, rosbag::bagmode::Write);

    // 初始化 PointCloud2 消息
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "ti_mmwave";  // 设置坐标系

    // 设置频率，以秒为单位
    double desired_frequency = 300;  // 100Hz
    bool first = true;
    ros::Time last_publish_time;

    // 遍历输入 rosbag 文件中的 RadarScan 消息
    rosbag::View view(input_bag, rosbag::TopicQuery("/ti_mmwave/radar_scan"));  // 替换成正确的主题名称和包名

    for (const rosbag::MessageInstance &m : view)
    {
        ti_mmwave_rospkg::RadarScan::ConstPtr radar_scan = m.instantiate<ti_mmwave_rospkg::RadarScan>();
        if (radar_scan != nullptr)
        {
            // 将 RadarScan 数据转换为 PointCloud2 数据
            pcl::PointXYZ point;
            point.x = radar_scan->x;
            point.y = radar_scan->y;
            point.z = radar_scan->z;
            point_cloud.push_back(point);

            // 计算时间间隔并检查是否需要发布 PointCloud2
            ros::Time current_time = radar_scan->header.stamp;
            if(first){
                last_publish_time = current_time;
                first = false;
            }

            double time_diff = (current_time - last_publish_time).toSec();
            // ROS_INFO("time_diff: %f",time_diff);
            if (time_diff >= 1.0 / desired_frequency)
            {
                cloud_msg.header.stamp = current_time;
                pcl::toROSMsg(point_cloud, cloud_msg);
                output_bag.write("/ti_mmwave/point_cloud_topic", current_time, cloud_msg);  // 替换成正确的主题名称和包名

                // 清空 PointCloud 以接收新的数据
                point_cloud.clear();
                last_publish_time = current_time;
            }
        }
    }

    input_bag.close();
    output_bag.close();

    return 0;
}
