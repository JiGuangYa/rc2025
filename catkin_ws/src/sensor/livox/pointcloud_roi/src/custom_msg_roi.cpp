#include <ros/ros.h>
#include <livox_ros_driver2/CustomMsg.h>


double min_height, max_height;
std::string pc2_sub_name, pc2_roi_pub_name;

ros::Publisher pub;


void pointCloudCallback(const livox_ros_driver2::CustomMsg::ConstPtr& msg) {
    // auto start = std::chrono::system_clock::now();

    livox_ros_driver2::CustomMsg filtered_msg;

    filtered_msg.header = msg->header;
    filtered_msg.timebase = msg->timebase;
    
    for (const auto& point : msg->points) {
        if (point.z >= min_height && point.z <= max_height) {
            filtered_msg.points.push_back(point);
        }
    }
    
    filtered_msg.point_num = filtered_msg.points.size();
    
    pub.publish(filtered_msg);

    // auto end = std::chrono::system_clock::now();
    // int fps = 1000000 / std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "ROI fps: " << fps << " point_num: " << filtered_msg.point_num << std::endl;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_roi");
    ros::NodeHandle nh;

    nh.param("pointcloud_roi/min_height", min_height, -0.5);
    nh.param("pointcloud_roi/max_height", max_height, 1.0);

    nh.param<std::string>("pointcloud_roi/pc2_sub_name", pc2_sub_name, "livox/lidar");
    nh.param<std::string>("pointcloud_roi/pc2_pub_name", pc2_roi_pub_name, "livox/lidar_roi");

    ROS_INFO_STREAM("\33[32mApriltag Localization CPP Node Successfully Initialized!\n"
                    "\n\33[34m#################################\033[1m\33[35m Settings \033[0m\33[34m#################################\n"
                    "\n\33[36m pc2_sub_name: \t\33[37m" << pc2_sub_name << "\n"
                    "\n\33[36m pc2_pub_name: \t\33[37m" << pc2_roi_pub_name << "\n"
                    "\n\33[36m z range: \t\33[37m[" << min_height << ", " << max_height << "] (m)\n"
                    "\n\33[34m#################################\033[1m\33[35m Settings \033[0m\33[34m#################################\n");

    ros::Subscriber sub = nh.subscribe(pc2_sub_name, 5, pointCloudCallback);

    pub = nh.advertise<livox_ros_driver2::CustomMsg>(pc2_roi_pub_name, 5);

    ros::spin();

    return 0;
}
