//
// Created by sakura on 2021/4/23.
//
#include <iostream>
#include <thread>
#include <fstream>

#include "WzSerialportPlus.h"
#include "curl.h"

#include <opencv/cv.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;

class global_localization {
private:
    ros::NodeHandle nh;
    ros::Subscriber odometrySub;

    tf::StampedTransform slam_pose;
    tf::TransformBroadcaster tfBroadcaster;
    tf::TransformListener tfListener;

    vector<vector<double>> gps_poses, compass_poses;
    vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>> odometryPoses;
    vector<double> timeOdometry;

    WzSerialportPlus wzSerialportPlus_GPS, wzSerialportPlus_Compass;
    baiduMap coordinateChange;

    Eigen::Matrix3d externalReference;

    string resultPath;

    int num = 0;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    global_localization() :
            nh("~") {
        string setting_file;
        if (ros::param::get("setting_file", setting_file))
            cout << "param get! " << endl << "setting_file:" << setting_file << endl;
        else
            ROS_ERROR("Could not load setting_file!\n");
        cv::FileStorage fs(setting_file, cv::FileStorage::READ);
        double sleepTimeCompass, sleepTimeGPS;
        string deviceNameGPS, deviceNameCompass,odometry_Topic;
        fs["sleepTimeGPS"] >> sleepTimeGPS;
        fs["sleepTimeCompass"] >> sleepTimeCompass;
        fs["deviceNameGPS"] >> deviceNameGPS;
        fs["deviceNameCompass"] >> deviceNameCompass;
        fs["results_path"] >> resultPath;
        fs["odometry_Topic"] >> odometry_Topic;

        odometrySub=nh.subscribe<nav_msgs::Odometry>(odometry_Topic,10,&global_localization::odometryCallBack, this);

        wzSerialportPlus_GPS.setReceiveCalback([&](char *data, int length) {
            double longitude = 0, latitude = 0;
            if (length == 29)
                printf("invalid GNSS data: %s", data);
            else if (length == 52) {
                for (int i = 0; i < 7; i++)
                    data++;
                int lon[10], lat[11];
                for (int i = 7; i < 17; i++) {
                    lat[i - 7] = (int) (*data - '0');
                    data++;
                }
                for (int i = 17; i < 20; i++)
                    data++;
                for (int i = 20; i < 31; i++) {
                    lon[i - 20] = (int) (*data - '0');
                    data++;
                }
                latitude = lat[0] * 10 + lat[1] + (lat[2] * 10 + lat[3]) / 60.0 +
                           (lat[5] * 1000 + lat[6] * 100 + lat[7] * 10 + lat[8]) / 600000.0;
                longitude = lon[0] * 100 + lon[1] * 10 + lon[2] + (lon[3] * 10 + lon[4]) / 60.0 +
                            (lon[6] * 1000 + lon[7] * 100 + lon[8] * 10 + lon[9]) / 600000.0;
                coordinate Wgs84(latitude,longitude),Mercator;
                coordinateChange.Wgs84ToMercator(Wgs84,Mercator);
                ros::Time time(ros::Time::now());
                std::vector<double> pose{time.toSec(), Mercator.longitude, Mercator.latitude};
                gps_poses.push_back(pose);
                usleep(sleepTimeGPS);
            }
        });

        wzSerialportPlus_Compass.setReceiveCalback([&](char *data, int length) {

            data++;
            data++;
            data++;
            data++;

            int sign = 1; //符号位
            double roll, pitch, yaw;
            int a, b, c, d, e, f;

            a = (*data) / 16;
            b = (*data++) % 16;
            c = (*data) / 16;
            d = (*data++) % 16;
            e = (*data) / 16;
            f = (*data++) % 16;
            if (a)
                a = -1;
            else
                a = 1;
            roll = a * (b * 100 + c * 10 + d + e * 0.1 + f * 0.01);

            a = (*data) / 16;
            b = (*data++) % 16;
            c = (*data) / 16;
            d = (*data++) % 16;
            e = (*data) / 16;
            f = (*data++) % 16;

            if (a)
                a = -1;
            else
                a = 1;
            pitch = a * (b * 100 + c * 10 + d + e * 0.1 + f * 0.01);

            a = (*data) / 16;
            b = (*data++) % 16;
            c = (*data) / 16;
            d = (*data++) % 16;
            e = (*data) / 16;
            f = (*data++) % 16;

            if (a)
                a = -1;
            else
                a = 1;
            yaw = a * (b * 100 + c * 10 + d + e * 0.1 + f * 0.01);

            ros::Time time(ros::Time::now());
            std::vector<double> pose{time.toSec(), roll, pitch, yaw};
            compass_poses.push_back(pose);
            usleep(sleepTimeCompass);
            wzSerialportPlus_Compass.send_hex();

        });


        slam_pose.frame_id_ = "/map";
        slam_pose.child_frame_id_ = "/odo";

    }

    void publish_tf() {
        tf::StampedTransform transform;
        try {
            tfListener.lookupTransform("/odo", "/velodyne", ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            return;
        }

        double timeOdo,timeCompass,timeGPS;
        timeOdo=transform.stamp_.toSec();
        timeCompass=compass_poses.back()[0];
        timeGPS=gps_poses.back()[0];
        if (abs(timeOdo-timeGPS)>0.2 || abs(timeOdo-timeCompass)>0.2 ) {
            ROS_INFO("time stamp Inconsistent!");
            return;
        }
        Eigen::Isometry3d velToOdo;
        tf::transformTFToEigen(transform, velToOdo);
        Eigen::Matrix3d globalPose;
        double heading=compass_poses.back()[3];
        double x=gps_poses.back()[1];
        double  y=gps_poses.back()[2];
        globalPose.setIdentity();
        globalPose(0,0)=cos(heading);
        globalPose(0,1)=-sin(heading);
        globalPose(1,0)=sin(heading);
        globalPose(1,1)=cos(heading);
        globalPose(0,2)=x;
        globalPose(1,2)=y;
        globalPose=externalReference*globalPose.eval();

        Eigen::Isometry3d globalPose3d;
        globalPose3d.prerotate(Eigen::AngleAxisd(atan2(globalPose(0,0),globalPose(1,0)), Eigen::Vector3d::UnitZ()));
        globalPose3d.pretranslate(Eigen::Vector3d(globalPose(0,2),globalPose(1,2),0));

        Eigen::Isometry3d odoToMap = globalPose3d * velToOdo.inverse();

        tf::Transform trans;
        tf::transformEigenToTF(odoToMap, trans);
        slam_pose.setData(trans);
        slam_pose.stamp_ = ros::Time::now();
        tfBroadcaster.sendTransform(slam_pose);
    }

    void saveResult() {
        std::ofstream gps(resultPath + "gps.txt"), compass(resultPath + "compass.txt");
        cout << "gps size: " << gps_poses.size() << endl;
        cout << "compass size: " << compass_poses.size() << endl;
        for (auto pose : gps_poses)
            gps << fixed << pose[0] << " " << pose[1] << " " << pose[2] << std::endl;
        for (auto pose : compass_poses)
            compass << fixed << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3] << std::endl;
    }

    void calibration() {
        bool wellConstrained=false;
        while (!wellConstrained){
            double xMin=0,xMax=0,yMin=0,yMax=0;
            for (auto iter:odometryPoses){
                auto x=iter.translation().x(),y=iter.translation().y();
                if (x>xMax)
                    xMax=x;
                if (x<xMin)
                    xMin=x;
                if (y>yMax)
                    yMax=y;
                if (y<yMin)
                    yMin=y;
            }
            if (gps_poses.size()>100 && compass_poses.size()>100&& xMax-xMin>10 && yMax-yMin>10)
                wellConstrained= true;
            else
                usleep(1e6);
        }

        ROS_INFO("Calibration begin!");
        vector<vector<int>> associate;
        int numLidar=0,numGPS=0,numCompass=0;

        while (numLidar<timeOdometry.size()){
            if (numGPS<gps_poses.size() && abs(timeOdometry[numLidar]-gps_poses[numGPS][0])>abs(timeOdometry[numLidar]-gps_poses[numGPS+1][0]))
                numGPS++;
            if (numCompass<compass_poses.size() && abs(timeOdometry[numLidar]-compass_poses[numCompass][0])>abs(timeOdometry[numLidar]-compass_poses[numCompass+1][0]))
                numCompass++;

            if (abs(timeOdometry[numLidar]-gps_poses[numGPS][0])<0.1 && abs(timeOdometry[numLidar]-compass_poses[numCompass][0])<0.1)
                associate.push_back(vector<int>{numLidar,numGPS,numCompass});
            numLidar++;
        }

        Eigen::Vector2d averageLidar, averageGPS, transLidarGPS;
        for (int i = 0; i < associate.size(); i++) {
            averageLidar += odometryPoses[associate[i][0]].translation().head<2>();
            averageGPS += Eigen::Vector2d (gps_poses[associate[i][1]][1],gps_poses[associate[i][1]][2]);
        }
        averageGPS /= associate.size();
        averageLidar /= associate.size();
        Eigen::Matrix2d H, RLidarGPS;
        H.setZero();
        for (int i = 0; i < associate.size(); i++) {
            H +=  Eigen::Vector2d (gps_poses[associate[i][1]][1],gps_poses[associate[i][1]][2])*
                    (odometryPoses[associate[i][0]].translation().head<2>() - averageGPS).transpose();
        }
        Eigen::JacobiSVD<Eigen::Matrix2d> svdLidarGPS(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
        RLidarGPS = svdLidarGPS.matrixV() * svdLidarGPS.matrixU().transpose();
        transLidarGPS = averageGPS - RLidarGPS * averageLidar;

        externalReference.setIdentity();
        externalReference.block<2,2>(0,0)=RLidarGPS;
        externalReference.block<2,1>(0,2)=transLidarGPS;

        ROS_INFO("Calibration Done");
        ROS_INFO_STREAM("externalReference: "<< externalReference << endl );

    }

    void odometryCallBack(const nav_msgs::OdometryConstPtr &odometryMsg){
        Eigen::Isometry3d odometryPose;
        tf2::fromMsg(odometryMsg->pose.pose,odometryPose);
        odometryPoses.push_back(odometryPose);
        timeOdometry.push_back(odometryMsg->header.stamp.toSec());
    }

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "lidar_slam");
    ROS_INFO("\033[1;32m---->\033[0m Global Localization Started.");

    global_localization global;

    global.calibration();

    ros::Rate rate(10);
    while (ros::ok()) {
        global.publish_tf();
        rate.sleep();
    }

    global.saveResult();

    return 0;
}

