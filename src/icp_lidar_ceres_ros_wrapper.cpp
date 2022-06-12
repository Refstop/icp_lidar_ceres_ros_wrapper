#include "icp_lidar_ceres_ros_wrapper/icp_lidar_ceres_ros_wrapper.h"

icp_lidar_ceres_ros_wrapper::icp_lidar_ceres_ros_wrapper(): icp_lidar_ceres(), scan1(false), scan2(false) {
    nh_.getParam("/icp_lidar_ceres_ros_wrapper/scan1", scan1_str);
    nh_.getParam("/icp_lidar_ceres_ros_wrapper/scan2", scan2_str);
    nh_.getParam("/icp_lidar_ceres_ros_wrapper/frame", frame_str);
    scan1_sub_ = nh_.subscribe(scan1_str, 1, &icp_lidar_ceres_ros_wrapper::Scan1Callback, this);
    scan2_sub_ = nh_.subscribe(scan2_str, 1, &icp_lidar_ceres_ros_wrapper::Scan2Callback, this);
    aligned_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);
    basescan_pub_ = nh_.advertise<sensor_msgs::PointCloud>("pointcloud", 1);
}

void icp_lidar_ceres_ros_wrapper::Scan1Callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    this->reference_points.resize(1,2);
    std::vector<float> ranges = msg->ranges;
    // std::cout << "scan_sick.size(): " << ranges.size() << endl;
    angle_max_ = msg->angle_max;
    angle_min_ = msg->angle_min;
    angle_increment_ = msg->angle_increment;
    range_min_ = msg->range_min;
    range_max_ = msg->range_max;
    int row = 0;
    for(int i = 0; i < ranges.size(); i++) {
        if(0 <= i && i <= ranges.size()) {
            if(ranges[i] != 0) {
                push_back_(this->reference_points, Vector2d(ranges[i]*cos(-angle_min_ - i*angle_increment_), ranges[i]*sin(-angle_min_ - i*angle_increment_)), row);
                row++;
            }
        }
    }
    this->reference_points.transposeInPlace();
    // string x_str = "", y_str = "";
    // for(int i = 0; i < this->reference_points.cols(); i++) {
    //     x_str = x_str + to_string(this->reference_points(0, i)) + ' ';
    //     y_str = y_str + to_string(this->reference_points(1, i)) + ' ';
    // }
    // ofstream writeFile("scan_sick.txt");
    // if(writeFile.is_open()) {
    //     writeFile << x_str + "\n";
    //     cout << x_str << endl;
    //     writeFile << y_str;
    //     cout << y_str << endl;
    //     writeFile.close();
    // }
    scan1 = true;
}

void icp_lidar_ceres_ros_wrapper::Scan2Callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    this->points_to_be_aligned.resize(1,2);
    std::vector<float> ranges = msg->ranges;
    // std::cout << "scan_yd.size(): " << ranges.size() << endl;
    int row = 0;
    int range_q = ranges.size()/4;
    // cout << range_q << endl;
    for(int i = 0; i < ranges.size(); i++) {
        if(0 <= i && i <= range_q*2) {
            if(ranges[i] != 0) {
                push_back_(this->points_to_be_aligned, Vector2d(ranges[i]*cos(-angle_min_ - i*angle_increment_), ranges[i]*sin(-angle_min_ - i*angle_increment_)), row);
                row++;
            }
        }
    }
    this->points_to_be_aligned.transposeInPlace();
    // string x_str = "", y_str = "";
    // for(int i = 0; i < this->points_to_be_aligned.cols(); i++) {
    //     x_str = x_str + to_string(this->points_to_be_aligned(0, i)) + ' ';
    //     y_str = y_str + to_string(this->points_to_be_aligned(1, i)) + ' ';
    // }
    // ofstream writeFile("scan_yd.txt");
    // if(writeFile.is_open()) {
    //     writeFile << x_str + "\n";
    //     writeFile << y_str;
    //     writeFile.close();
    // }
    scan2 = true;
}

void icp_lidar_ceres_ros_wrapper::run() {
    MatrixXd aligned_points;
    sensor_msgs::LaserScan result_laserscan;
    ros::Rate rate(25);
    while(ros::ok()) {
        if(scan1 && scan2) {
            cout << "Publishing aligned scan topic (/scan)" << endl;
            int n = (angle_max_-angle_min_) / angle_increment_;
            aligned_points = icp_non_linear(this->reference_points, this->points_to_be_aligned, 100, 10, 8, false);
            sensor_msgs::PointCloud basepoint;
            vector<geometry_msgs::Point32> points;
            vector<sensor_msgs::ChannelFloat32> Channels;
            basepoint.header.stamp = ros::Time::now();
            basepoint.header.frame_id = frame_str;

            for (int i = 0; i < aligned_points.cols(); i++) {
                geometry_msgs::Point32 point;
                sensor_msgs::ChannelFloat32 Channel;
                point.x = aligned_points(0,i);
                point.y = -aligned_points(1,i);
                point.z = 0;
                points.push_back(point);
                Channels.push_back(Channel);
            }
            basepoint.points = points;
            basepoint.channels = Channels;
            basescan_pub_.publish(basepoint);

            // std::vector<float> ranges(n);
            // for(int i = 0; i < aligned_points.cols(); i++) {
            //     double angle = atan2(aligned_points(1,i), aligned_points(0,i));
            //     int index = (-angle_min_ - angle) / angle_increment_;
            //     ranges[index] = sqrt(pow(aligned_points(0,i), 2) + pow(aligned_points(1,i), 2));
            // }
            // // for(int i=0; i<ranges.size(); i++) cout << ranges[i] << ' ';
            // result_laserscan.header.frame_id = frame_str;
            // result_laserscan.angle_min = angle_min_;
            // result_laserscan.angle_max = angle_max_;
            // result_laserscan.angle_increment = angle_increment_;
            // result_laserscan.range_min = range_min_;
            // result_laserscan.range_max = range_max_;
            // result_laserscan.set_ranges_size(n);
            // for(int i = 0; i < ranges.size(); i++) {
            //     result_laserscan.ranges[i] = ranges[i];
            // }
            // // result_laserscan.ranges = ranges; // malloc(): memory corruption??
            // aligned_scan_pub_.publish(result_laserscan); // scan publish
            scan1 = false; scan2 = false;
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void icp_lidar_ceres_ros_wrapper::push_back_(MatrixXd& m, Vector2d&& values, std::size_t row) {
    if(row >= m.rows()) {
        m.conservativeResize(row + 1, Eigen::NoChange);
    }
    m.row(row) = values;
}