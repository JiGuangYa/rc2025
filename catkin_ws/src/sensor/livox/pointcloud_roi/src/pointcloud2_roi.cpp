#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>

class PointCloudROIFilter
{
public:
    PointCloudROIFilter(ros::NodeHandle& nh)
    {
        nh.param("pointcloud_roi/debug", debug, false);
        nh.param("pointcloud_roi/min_height", min_height_, -0.5);
        nh.param("pointcloud_roi/max_height", max_height_, 1.0);
        nh.param("pointcloud_roi/obstacle_ahead_x", obs_x, 0.3);
        nh.param("pointcloud_roi/obstacle_ahead_y", obs_y, 0.2);
        nh.param("pointcloud_roi/obstacle_min_pts", min_pts, 10);
        nh.param("pointcloud_roi/obstacle_max_pts", max_pts, 100);

        nh.param<std::string>("pointcloud_roi/pc2_sub_name", pc2_sub_name, "livox/lidar");
        nh.param<std::string>("pointcloud_roi/pc2_roi_pub_name", pc2_roi_pub_name, "livox/lidar_roi");
        nh.param<std::string>("pointcloud_roi/pc2_obs_pub_name", pc2_obs_pub_name, "livox/obstacle");
        nh.param<std::string>("pointcloud_roi/bool_obs_pub_name", bool_obs_pub_name, "obstacle_ahead");

        ROS_INFO_STREAM("\33[32mPointcloud ROI CPP Node Successfully Initialized!\n"
                        "\n\33[34m#################################\033[1m\33[35m Settings \033[0m\33[34m#################################\n"
                        "\n\33[36m pc2_sub_name: \t\t\33[37m" << pc2_sub_name << "\n"
                        "\n\33[36m pc2_roi_pub_name: \t\33[37m" << pc2_roi_pub_name << "\n"
                        "\n\33[36m pc2_obs_pub_name: \t\33[37m" << pc2_obs_pub_name << "\n"
                        "\n\33[36m bool_obs_pub_name: \t\33[37m" << bool_obs_pub_name << "\n"
                        "\n\33[36m z range: \t\t\33[37m[" << min_height_ << ", " << max_height_ << "] (m)\n"
                        "\n\33[36m obstacle range: \t\33[37m" << "x:" << obs_x << "  y:" << obs_y << " (m)\n"
                        "\n\33[36m debug: \t\t\33[37m" << debug << "\n"
                        "\n\33[34m#################################\033[1m\33[35m Settings \033[0m\33[34m#################################\n");
        // 订阅点云数据
        pc2_sub = nh.subscribe(pc2_sub_name, 1, &PointCloudROIFilter::pointCloudCallback, this);
        // 发布处理后的点云数据
        pc2_roi_pub = nh.advertise<sensor_msgs::PointCloud2>(pc2_roi_pub_name, 1);
        pc2_obs_pub = nh.advertise<sensor_msgs::PointCloud2>(pc2_obs_pub_name, 1);
        obstacle_pub = nh.advertise<std_msgs::Bool>(bool_obs_pub_name, 1);
    }

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        // auto start_time = std::chrono::high_resolution_clock::now();

        // 将ROS消息转换为PCL点云格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud);

        // 创建通过滤波器并设置参数
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_height_, max_height_);

        // 过滤点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pass.filter(*cloud_filtered);

        // 将过滤后的点云转换回ROS消息格式
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header = input->header;

        // 发布过滤后的点云
        pc2_roi_pub.publish(output);

        bool obstacle_detected = false;
        int obstacle_cnt = 0;
        // 创建新的点云对象以存储符合条件的点
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 遍历点云数据，检查是否有点在x方向的阈值范围内
        for (const auto& point : cloud_filtered->points)
        {
            if (point.x > 0 && point.x <= obs_x && std::abs(point.y) <= obs_y)
            {
                obstacle_cnt += 1;
                obstacle_cloud->points.push_back(point);
            }

            if (obstacle_cnt >= min_pts) {
                obstacle_detected = true;
            }

            if (obstacle_cnt >= max_pts) {
                break;
            }
        }
        if (debug) {
            std::string str = obstacle_detected ? "Yes ":"No  ";
            std::cout << str << "obstacle_cnt: " << obstacle_cnt << "\tmin_pts: " << min_pts << "\tmax_pts: " << max_pts << std::endl;
        }

        // 发布障碍物检测结果
        std_msgs::Bool obstacle_msg;
        obstacle_msg.data = obstacle_detected;
        obstacle_pub.publish(obstacle_msg);

        // 将符合条件的点云转换回ROS消息格式并发布
        sensor_msgs::PointCloud2 obs_output;
        if (obstacle_detected) {
            pcl::toROSMsg(*obstacle_cloud, obs_output);
            obs_output.header = input->header;
            pc2_obs_pub.publish(obs_output);
        }
        else {
            pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::toROSMsg(*empty_cloud, obs_output);
            obs_output.header = input->header;
            obs_output.header = input->header;
            pc2_obs_pub.publish(obs_output);
        }

        // auto end_time = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> elapsed_time = end_time - start_time;
        // ROS_INFO(" took %.6f seconds", elapsed_time.count());
    }

    bool debug;
    double min_height_;
    double max_height_;
    double obs_x;
    double obs_y;
    int min_pts;
    int max_pts;
    std::string pc2_sub_name;
    std::string pc2_roi_pub_name;
    std::string pc2_obs_pub_name;
    std::string bool_obs_pub_name;
    ros::Subscriber pc2_sub;
    ros::Publisher pc2_roi_pub;
    ros::Publisher pc2_obs_pub;
    ros::Publisher obstacle_pub;
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_roi_filter");
    ros::NodeHandle nh;
    PointCloudROIFilter filter(nh);
    ros::spin();
    return 0;
}

