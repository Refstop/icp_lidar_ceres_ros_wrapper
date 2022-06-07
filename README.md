# icp_lidar_ceres_ros_wrapper
icp_lidar_ceres 패키지의 class를 상속받아 ROS 환경에서 통신할 수 있도록 wrapping한 패키지입니다. icp_lidar_ceres_ros 패키지는 icp 알고리즘이 모두 한 패키지에 포함되어 있는 데 비해, 해당 패키지는 ROS 통신 코드만을 작성하여 icp_lidar_ceres 패키지와 연결한 것이 차이점입니다.

## Dependencies
- [Eigen Library](https://eigen.tuxfamily.org/index.php?title=Main_Page)  
- [knncpp kd tree Library](https://github.com/Rookfighter/knn-cpp)  
- [Ceres Library](https://github.com/ceres-solver/ceres-solver)  
- [icp_lidar_ceres](https://github.com/Refstop/icp_lidar_ceres)  
Eigen Library, knncpp kd tree Library, Ceres Library는 `make install` 한 후 사용이 가능하며, icp_lidar_ceres는 해당 패키지와 같은 `catkin_ws/src` 폴더 내에 clone 한 후 `catkin build` 명령어를 통해 컴파일 및 빌드한 후 실행이 가능합니다.

## 예제 실행 방법
```
cd ~/catkin_ws/src/
git clone https://github.com/Refstop/icp_lidar_ceres.git
git clone https://github.com/Refstop/icp_lidar_ceres_ros_wrapper.git
cd ..
catkin build
```
catkin build 이후의 예제 실행 방법은 다음과 같습니다.
```
roslaunch icp_lidar_ceres_ros_wrapper icp_lidar_ceres_wrapper.launch
```
해당 launch 파일 실행 시 패키지에 기본적으로 저장된 `2d_lidars_scan.bag` 파일에 대한 icp 알고리즘 실행 결과가 rviz에 표시됩니다.

## 예제 실행 결과
- `2d_lidars_scan.bag` 파일의 정렬 결과
<p align="center"><img src="/figs/icp_lidar_ceres_ros_wrapper.gif"></p>