#include "icp_lidar_ceres_ros_wrapper/icp_lidar_ceres_ros_wrapper.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "icp_lidar_ceres_ros");
    icp_lidar_ceres_ros_wrapper icp;
    icp.run();
    return 0;
}