//
// Created by sakura on 2021/5/16.
//

#ifndef LIDAR_SLAM_UTILITY_H
#define LIDAR_SLAM_UTILITY_H

#include <dirent.h>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>

using namespace std;

void load_clouds(const string& path, vector<string>& lidar_paths) {
    struct dirent* ptr;
    DIR* dir;
    dir = opendir(path.c_str());
    while (ptr = readdir(dir)) {
        if (ptr->d_name[0] == '.') continue;
        lidar_paths.push_back(path + ptr->d_name);
    }
    sort(lidar_paths.begin(), lidar_paths.end());
    closedir(dir);
}

void load_poses_kitti(const string& path, vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& Poses) {
    ifstream pose(path);
    std::string line;
    while (getline(pose, line)) {
        if (!line.empty()) {
            std::stringstream ss(line);
            Eigen::Isometry3d Pose_(Eigen::Isometry3d::Identity());
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 4; ++j) {
                    ss >> Pose_(i, j);
                }
            Poses.push_back(Pose_);
        }
    }
    pose.close();
}

template<typename matrix>
float rotationError(matrix &pose_error) {
    auto a = pose_error(0,0);
    auto b = pose_error(1,1);
    auto c = pose_error(2,2);
    auto d = 0.5*(a+b+c-1.0);
    return acos(max(min(d,1.0),-1.0));
}

template<typename matrix>
float translationError(matrix &pose_error) {
    auto dx = pose_error(0,3);
    auto dy = pose_error(1,3);
    auto dz = pose_error(2,3);
    return sqrt(dx*dx+dy*dy+dz*dz);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr loadKitticloud(const string &filename) {
    FILE *file = fopen(filename.c_str(), "rb");
    if (!file) {
        std::cerr << "error: failed to load " << filename << std::endl;
        return nullptr;
    }

    std::vector<float> buffer(1000000);
    size_t num_points = fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
    fclose(file);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    cloud->resize(num_points);

    for (int i = 0; i < num_points; i++) {
        auto &pt = cloud->at(i);
        pt.x = buffer[i * 4];
        pt.y = buffer[i * 4 + 1];
        pt.z = buffer[i * 4 + 2];
        pt.intensity = buffer[i * 4 + 3];
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    return cloud;
}

template<typename matrix>
void load_kitti_calib(const string &calib_path,matrix &calib){

    calib=matrix::Identity();
    ifstream in(calib_path);
    string line;
    for (int i=0;i<4;i++)
        getline(in, line);
    in>>line;
    for (int i=0;i<3;i++)
        for (int j=0;j<4;j++)
            in>>calib(i,j);
    in.close();

}
#endif //LIDAR_SLAM_UTILITY_H
