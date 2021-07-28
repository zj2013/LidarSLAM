//
// Created by sakura on 2021/6/23.
//

#ifndef LIDAR_SLAM_GEOMETRYESTIMATE_H
#define LIDAR_SLAM_GEOMETRYESTIMATE_H

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/auto_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

class geometryEstimate {

public:
    geometryEstimate();

    //feature extracting
    void extract(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    //visual extracted features
    void visualization();

    //extract line feature or not
    bool usingLine;

    pcl::PointCloud<pcl::PointXYZI>::Ptr mCloud;
    std::vector<int> labels;
    std::vector<int> labelValid;
    int labelNum,labelNumPlane,labelNumLine;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr mPlaneCloud,mLineCloud,mDownSampledCloud;
    void setDebugPrint(int debugLevel){
        debugPrintLevel=debugLevel;
    };

private:

    //Number of threads openmp used
    int num_threads_;

    //minimum plane number
    int planeThreshod;

        //vector angle threshold for vertical plane and ground
    int angleThreshodVerticalPlane,angleThreadGround;

    //intensity , normal and towards threshod when region grow
    double intensityThreshod,normalThreshod,towardsThreshod;

    //line eigen value threshod when line point select
    double lineThreshod;

    //the number of k-nearest neighbors
    int nearK;

    //Minimum number for valid label
    int labelThreshodPlane,labelThreshodLine;

    //normal consist ratio for valid label
    double labelConsistThreshod;

    //debug information level
    int debugPrintLevel;
};


#endif //LIDAR_SLAM_GEOMETRYESTIMATE_H
