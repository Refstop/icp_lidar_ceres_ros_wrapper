#ifndef ICP_LIDAR_CERES_ROS_WRAPPER
#define ICP_LIDAR_CERES_ROS_WRAPPER

#include <ros/ros.h>
#include <icp_lidar_ceres/icp_lidar_ceres.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <typeinfo>
#define RAD2DEG(rad) rad*(180/M_PI)
#define DEG2RAD(deg) deg*(M_PI/180)

using namespace std;
using namespace Eigen;

class icp_lidar_ceres_ros_wrapper : public icp_lidar_ceres {
    public:
    icp_lidar_ceres_ros_wrapper();
    ~icp_lidar_ceres_ros_wrapper() {}
    void Scan1Callback(const sensor_msgs::LaserScan::ConstPtr& msg); // reference_points
    void Scan2Callback(const sensor_msgs::LaserScan::ConstPtr& msg); // points_to_be_aligned
    void run();

    private:
    ros::NodeHandle nh_;
    ros::Subscriber scan1_sub_, scan2_sub_;
    ros::Publisher aligned_scan_pub_;
    ros::Publisher basescan_pub_;
    float angle_max_, angle_min_, angle_increment_, range_min_, range_max_;
    bool scan1, scan2;
    string scan1_str, scan2_str, frame_str;
    void push_back_(MatrixXd& m, Vector2d&& values, std::size_t row);
};

#endif