//
// Created by sakura on 2021/4/21.
//
#include "utility.h"
#include "tools/geometryEstimate.h"
#include "tools/geometryICP.h"

#include <opencv/cv.h>
#include <glog/logging.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace std;

class odometry {
private:
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud;
    ros::Publisher pubOdometry;

    tf::StampedTransform odo_pose;
    tf::TransformBroadcaster tfBroadcaster;

    nav_msgs::Odometry Odometry;

    geometryEstimate geoEs;
    geometryICP geoICP;
    pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;

    float downsample_resolution = 0.25;
    double time_last;
    string results_path;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> Poses;

    odometry() :
            nh("~") {
        string setting_file;
        if (ros::param::get("setting_file", setting_file))
            cout << "param get! " << endl << "setting_file:" << setting_file << endl;
        else
            ROS_ERROR("Could not load setting_file!\n");
        cv::FileStorage fs(setting_file, cv::FileStorage::READ);
        string pointCloudTopic = "/velodyne_points", odometry_Topic = "/laser_odometry";
        fs["pointCloudTopic"] >> pointCloudTopic;
        fs["downsample_resolution"] >> downsample_resolution;
        fs["results_path"] >> results_path;
        fs["odometry_Topic"] >> odometry_Topic;

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 10, &odometry::cloudHandler, this);
        pubOdometry = nh.advertise<nav_msgs::Odometry>(odometry_Topic, 10);

        Odometry.header.frame_id = "/map";

        time_last = -1;

        geoEs.setDebugPrint(1);
        geoICP.setMaxCorrespondenceDistance(1.0);
        geoICP.setDebugPrint(1);

        voxelgrid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);

        odo_pose.frame_id_ = "/odo";
        odo_pose.child_frame_id_ = "/velodyne";

    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);

        voxelgrid.setInputCloud(laserCloudIn);
        voxelgrid.filter(*laserCloudIn);
        Eigen::Isometry3d Pose;

        if (time_last < 0) {
            geoEs.extract(laserCloudIn);
            geoICP.setTargetCloud(geoEs.mPlaneCloud, geoEs.mLineCloud);
            Pose = Eigen::Isometry3d::Identity();
        } else {
            geoEs.extract(laserCloudIn);
            geoICP.setSourceCloud(geoEs.mPlaneCloud, geoEs.mLineCloud);
            Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
            geoICP.computeTransformation(guess);
            Pose = Poses.back() * geoICP.getFinalPose();
            geoICP.swapSourceAndTarget();
        }

        time_last++;
        Odometry.header.stamp = laserCloudMsg->header.stamp;
        Poses.push_back(Pose);
        Odometry.pose.pose = tf2::toMsg(Pose);
        pubOdometry.publish(Odometry);

        tf::Transform trans;
        tf::transformEigenToTF(Pose, trans);
        odo_pose.setData(trans);
        odo_pose.stamp_ = laserCloudMsg->header.stamp;
        tfBroadcaster.sendTransform(odo_pose);

    }

    void save_results() {
        std::ofstream ofs(results_path + "traj_GICP_campus.txt");
        cout << "traj size: " << Poses.size() << endl;
        for (auto &pose : Poses) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 4; j++) {
                    if (i || j) {
                        ofs << " ";
                    }

                    ofs << pose(i, j);
                }
            }
            ofs << std::endl;
        }
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "lidar_slam");

    odometry odo;

    ROS_INFO("\033[1;32m---->\033[0m Lidar Odometry Started.");

    ros::spin();

    odo.save_results();

    return 0;
}
