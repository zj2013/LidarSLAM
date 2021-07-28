//
// Created by sakura on 2021/6/27.
//

#ifndef LIDAR_SLAM_GEOMETRYICP_H
#define LIDAR_SLAM_GEOMETRYICP_H

#include <chrono>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

enum class RegularizationMethod { NONE, PLANE, FROBENIUS };

class geometryICP {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    geometryICP();

    void setSourceCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr PlaneSource);
    void setSourceCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr PlaneSource, pcl::PointCloud<pcl::PointXYZINormal>::Ptr LineSource);
    void setTargetCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr PlaneTarget);
    void setTargetCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr PlaneTarget, pcl::PointCloud<pcl::PointXYZINormal>::Ptr LineTarget);
    void swapSourceAndTarget();
    void computeTransformation(const Eigen::Isometry3d &guess=Eigen::Isometry3d::Identity());
    Eigen::Isometry3d getFinalPose();
    double getFitScore();
    void setDebugPrint(int debugLevel);
    void setMaxCorrespondenceDistance(double distance_threshold){
        corrDistThreshold=distance_threshold;
    };
    void transCompare(const Eigen::Isometry3d &trans);

private:
    //information print level,num of openmp threads,max iteration for LM and single delta
    int debugPrintLevel,numThread,maxIteration,lmMaxIteration;

    Eigen::Isometry3d finalTrans;

    double fitScore,lmLambda,corrDistThreshold;
    RegularizationMethod regularizationMethod;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr mPlaneSource,mPlaneTarget,mLineSource,mLineTarget;
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr kdTreePlaneSource,kdTreePlaneTarget,kdTreeLineSource,kdTreeLineTarget;

    Eigen::Matrix<double, 6, 6> finalHessian;

    std::vector<int> correspondences;

    bool isConverged(const Eigen::Isometry3d& delta) ;

    double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H = nullptr, Eigen::Matrix<double, 6, 1>* b = nullptr);

    double computeError(const Eigen::Isometry3d& trans);

    bool stepOptimize(Eigen::Isometry3d& x0, Eigen::Isometry3d& delta);

    void updateCorrespondences(const Eigen::Isometry3d& trans);

    inline Eigen::Matrix3f skew(const Eigen::Vector3f& x) {
        Eigen::Matrix3f skew = Eigen::Matrix3f::Zero();
        skew(0, 1) = -x[2];
        skew(0, 2) = x[1];
        skew(1, 0) = x[2];
        skew(1, 2) = -x[0];
        skew(2, 0) = -x[1];
        skew(2, 1) = x[0];

        return skew;
    }

    inline Eigen::Matrix3d skewd(const Eigen::Vector3d& x) {
        Eigen::Matrix3d skew = Eigen::Matrix3d::Zero();
        skew(0, 1) = -x[2];
        skew(0, 2) = x[1];
        skew(1, 0) = x[2];
        skew(1, 2) = -x[0];
        skew(2, 0) = -x[1];
        skew(2, 1) = x[0];

        return skew;
    }

    inline Eigen::Quaterniond so3_exp(const Eigen::Vector3d& omega) {
        double theta_sq = omega.dot(omega);

        double theta;
        double imag_factor;
        double real_factor;
        if(theta_sq < 1e-10) {
            double theta_quad = theta_sq * theta_sq;
            imag_factor = 0.5 - 1.0 / 48.0 * theta_sq + 1.0 / 3840.0 * theta_quad;
            real_factor = 1.0 - 1.0 / 8.0 * theta_sq + 1.0 / 384.0 * theta_quad;
        } else {
            theta = std::sqrt(theta_sq);
            double half_theta = 0.5 * theta;
            imag_factor = std::sin(half_theta) / theta;
            real_factor = std::cos(half_theta);
        }

        return Eigen::Quaterniond(real_factor, imag_factor * omega.x(), imag_factor * omega.y(), imag_factor * omega.z());
    }

};


#endif //LIDAR_SLAM_GEOMETRYICP_H
