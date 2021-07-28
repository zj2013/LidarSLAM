//
// Created by sakura on 2021/6/27.
//

#include <pcl/common/transforms.h>
#include "tools/geometryICP.h"

geometryICP::geometryICP() {

    lmLambda = 1e-9;
    debugPrintLevel = 1;
    finalTrans.setIdentity();
    fitScore = std::numeric_limits<double>::max();
    numThread = omp_get_max_threads();
    maxIteration = 64;
    lmMaxIteration = 10;
    regularizationMethod = RegularizationMethod::PLANE;
    corrDistThreshold = 1;
    kdTreePlaneTarget.reset(new pcl::search::KdTree<pcl::PointXYZINormal>);
    kdTreePlaneSource.reset(new pcl::search::KdTree<pcl::PointXYZINormal>);
    kdTreeLineSource.reset(new pcl::search::KdTree<pcl::PointXYZINormal>);
    kdTreeLineTarget.reset(new pcl::search::KdTree<pcl::PointXYZINormal>);
    mPlaneSource.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
    mPlaneTarget.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
    mLineSource.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
    mLineTarget.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
}

void geometryICP::swapSourceAndTarget() {
    mPlaneSource.swap(mPlaneTarget);
    mPlaneTarget->header = mPlaneSource->header;
    mLineSource.swap(mLineTarget);
    mLineTarget->header = mLineSource->header;
    kdTreePlaneSource.swap(kdTreePlaneTarget);
    kdTreeLineSource.swap(kdTreeLineTarget);
    correspondences.clear();
}

void geometryICP::setTargetCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr PlaneTarget) {
    mPlaneTarget.swap(PlaneTarget);
    kdTreePlaneTarget->setInputCloud(mPlaneTarget);
    mLineTarget.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
}

void geometryICP::setSourceCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr PlaneSource) {
    mPlaneSource.swap(PlaneSource);
    kdTreePlaneSource->setInputCloud(mPlaneSource);
    mLineSource.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
}

void geometryICP::computeTransformation(const Eigen::Isometry3d &guess) {
    Eigen::Isometry3d x0 = guess;

    lmLambda = -1.0;
    bool converged = false;

//    auto t1 = std::chrono::steady_clock::now();

    int i = 0;
    for (; i < maxIteration && !converged; i++) {

        Eigen::Isometry3d delta;
        if (!stepOptimize(x0, delta)) {
            std::cerr << "lm not converged!!" << std::endl;
            break;
        }

        converged = isConverged(delta);
    }

    finalTrans = x0;
    auto t2 = std::chrono::steady_clock::now();

    if (debugPrintLevel) {
//        LOG(INFO) << "geometry ICP cost: "
//                  << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000 << "ms with "
//                  << i << " iterations";
//        if (debugPrintLevel > 1)
//            LOG(INFO) << "Fit score: " << getFitScore();
    }

}

Eigen::Isometry3d geometryICP::getFinalPose() {
    return finalTrans;
}

double geometryICP::getFitScore() {
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudAlign(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::transformPointCloud(*mPlaneSource, *cloudAlign, finalTrans.cast<float>().matrix());

    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);

    double fitness_score = 0.0, max_range = 10;
    int nr = 0;
    //For each aligned point
    for (size_t i = 0; i < cloudAlign->size(); ++i) {
        // Find its nearest neighbor in the target
        kdTreePlaneTarget->nearestKSearch(cloudAlign->points[i], 1, nn_indices, nn_dists);
        if (nn_dists[0] <= max_range) {
            fitness_score += nn_dists[0];
            nr++;
        }
    }

    if (mLineSource->size() > 0 && mLineTarget->size() > 0) {
        pcl::transformPointCloud(*mLineSource, *cloudAlign, finalTrans.cast<float>().matrix());
        for (size_t i = 0; i < cloudAlign->size(); ++i) {
            kdTreeLineTarget->nearestKSearch(cloudAlign->points[i], 1, nn_indices, nn_dists);
            if (nn_dists[0] <= max_range) {
                fitness_score += nn_dists[0];
                nr++;
            }
        }
    }

    if (nr > 0)
        fitScore = fitness_score / nr;
    else
        fitScore = std::numeric_limits<double>::max();
    return fitScore;
}

void geometryICP::setDebugPrint(int debugLevel) {
    debugPrintLevel = debugLevel;
}

void geometryICP::updateCorrespondences(const Eigen::Isometry3d &trans) {

    Eigen::Isometry3f trans_f = trans.cast<float>();

    if (mLineSource->size() > 0 && mLineTarget->size() > 0)
        correspondences.resize(mPlaneSource->size() + mLineSource->size());
    else
        correspondences.resize(mPlaneSource->size());

    std::vector<int> k_indices(1);
    std::vector<float> k_sq_dists(1);

#pragma omp parallel for num_threads(numThread) firstprivate(k_indices, k_sq_dists) schedule(guided, 8)
    for (int i = 0; i < mPlaneSource->size(); i++) {
        pcl::PointXYZINormal pt;
        pt.getVector4fMap() = trans_f * mPlaneSource->points[i].getVector4fMap();

        kdTreePlaneTarget->nearestKSearch(pt, 1, k_indices, k_sq_dists);

        correspondences[i] = k_sq_dists[0] < corrDistThreshold * corrDistThreshold ? k_indices[0] : -1;
    }

    if (mLineSource->size() > 0 && mLineTarget->size() > 0) {
#pragma omp parallel for num_threads(numThread) firstprivate(k_indices, k_sq_dists) schedule(guided, 8)
        for (int i = 0; i < mLineSource->size(); i++) {
            pcl::PointXYZINormal pt;
            pt.getVector4fMap() = trans_f * mLineSource->points[i].getVector4fMap();

            kdTreeLineTarget->nearestKSearch(pt, 1, k_indices, k_sq_dists);

            correspondences[i + mPlaneSource->size()] =
                    k_sq_dists[0] < corrDistThreshold * corrDistThreshold ? k_indices[0] : -1;
        }
    }
}

bool geometryICP::isConverged(const Eigen::Isometry3d &delta) {

    double rotation_epsilon_ = 2e-3, transformation_epsilon_ = 5e-4;

    Eigen::Matrix3d R = delta.linear() - Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = delta.translation();

    Eigen::Matrix3d r_delta = 1.0 / rotation_epsilon_ * R.array().abs();
    Eigen::Vector3d t_delta = 1.0 / transformation_epsilon_ * t.array().abs();

    return std::max(r_delta.maxCoeff(), t_delta.maxCoeff()) < 1;
}

double geometryICP::computeError(const Eigen::Isometry3d &trans) {
    double sum_errors = 0.0;

#pragma omp parallel for num_threads(numThread) reduction(+ : sum_errors) schedule(guided, 8)
    for (int i = 0; i < mPlaneSource->size(); i++) {
        int target_index = correspondences[i];
        if (target_index < 0) {
            continue;
        }

        Eigen::Vector4d poseSource, poseTarget, PlaneSource, PlaneTarget, transPlaneSource;
        poseSource = mPlaneSource->points[i].getVector4fMap().cast<double>();
        poseTarget = mPlaneTarget->points[target_index].getVector4fMap().cast<double>();
        poseSource(3) = poseTarget(3) = 1;
        PlaneSource = mPlaneSource->points[i].getNormalVector4fMap().cast<double>();
        PlaneTarget = mPlaneTarget->points[target_index].getNormalVector4fMap().cast<double>();
        transPlaneSource = trans.matrix().transpose().inverse() * PlaneSource;

        double error_1, error_2;
        error_1 = poseTarget.transpose() * transPlaneSource;
        error_2 = PlaneTarget.transpose() * trans.matrix() * poseSource;
        sum_errors += error_1 * error_1 + error_2 * error_2;
    }

    if (mLineTarget->size() > 0 && mLineSource->size() > 0) {
#pragma omp parallel for num_threads(numThread) reduction(+ : sum_errors) schedule(guided, 8)
        for (int i = 0; i < mLineSource->size(); i++) {
            int target_index = correspondences[i + mPlaneSource->size()];
            if (target_index < 0) {
                continue;
            }

            Eigen::Vector4d poseSource, poseTarget;
            Eigen::Vector3d LineSource, LineTarget;
            poseSource = mLineSource->points[i].getVector4fMap().cast<double>();
            poseTarget = mLineTarget->points[target_index].getVector4fMap().cast<double>();
            poseSource(3) = poseTarget(3) = 1;
            LineSource = mLineSource->points[i].getNormalVector3fMap().cast<double>();
            LineTarget = mLineTarget->points[target_index].getNormalVector3fMap().cast<double>();

            Eigen::Vector3d error_1, error_2;
            error_1 = (trans.inverse() * poseTarget - poseSource).head<3>().cross(LineSource);
            error_2 = (poseTarget - trans * poseSource).head<3>().cross(LineTarget);
            sum_errors += (error_1.transpose() * error_1 + error_2.transpose() * error_2).norm();
        }
    }

    return sum_errors;
}

double
geometryICP::linearize(const Eigen::Isometry3d &trans, Eigen::Matrix<double, 6, 6> *H, Eigen::Matrix<double, 6, 1> *b) {
    updateCorrespondences(trans);

    double sum_errors = 0.0;
    std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs(numThread);
    std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bs(numThread);
    for (int i = 0; i < numThread; i++) {
        Hs[i].setZero();
        bs[i].setZero();
    }

#pragma omp parallel for num_threads(numThread) reduction(+ : sum_errors) schedule(guided, 8)
    for (int i = 0; i < mPlaneSource->size(); i++) {
        int target_index = correspondences[i];
        if (target_index < 0) {
            continue;
        }

        Eigen::Vector4d poseSource, poseTarget, PlaneSource, PlaneTarget, transPlaneSource;
        poseSource = mPlaneSource->points[i].getVector4fMap().cast<double>();
        poseTarget = mPlaneTarget->points[target_index].getVector4fMap().cast<double>();
        poseSource(3) = poseTarget(3) = 1;
        PlaneSource = mPlaneSource->points[i].getNormalVector4fMap().cast<double>();
        PlaneTarget = mPlaneTarget->points[target_index].getNormalVector4fMap().cast<double>();
        transPlaneSource = trans.matrix().transpose().inverse() * PlaneSource;

        double error_1, error_2;
        error_1 = poseTarget.transpose() * transPlaneSource;
        error_2 = PlaneTarget.transpose() * trans.matrix() * poseSource;
        sum_errors += error_1 * error_1 + error_2 * error_2;
//        sum_errors += error_2 * error_2;
        Eigen::Matrix<double, 1, 6> jlossexp1, jlossexp2;
        jlossexp1.head<3>() = PlaneSource.head<3>().transpose() * skewd((trans.inverse() * poseTarget).head<3>());
        jlossexp1.tail<3>() = -PlaneSource.head<3>().transpose();

        jlossexp2.head<3>() = -PlaneTarget.head<3>().transpose() * skewd((trans * poseSource).head<3>());
        jlossexp2.tail<3>() = PlaneTarget.head<3>().transpose();

        Eigen::Matrix<double, 6, 6> Hi = jlossexp1.transpose() * jlossexp1 + jlossexp2.transpose() * jlossexp2;
        Eigen::Matrix<double, 6, 1> bi = error_1 * jlossexp1.transpose() + error_2 * jlossexp2.transpose();

        Hs[omp_get_thread_num()] += Hi;
        bs[omp_get_thread_num()] += bi;
    }

    if (mLineTarget->size() > 0 && mLineSource->size() > 0) {
#pragma omp parallel for num_threads(numThread) reduction(+ : sum_errors) schedule(guided, 8)
        for (int i = 0; i < mLineSource->size(); i++) {
            int target_index = correspondences[i + mPlaneSource->size()];
            if (target_index < 0) {
                continue;
            }

            Eigen::Vector4d poseSource, poseTarget;
            Eigen::Vector3d LineSource, LineTarget;
            poseSource = mLineSource->points[i].getVector4fMap().cast<double>();
            poseTarget = mLineTarget->points[target_index].getVector4fMap().cast<double>();
            poseSource(3) = poseTarget(3) = 1;
            LineSource = mLineSource->points[i].getNormalVector3fMap().cast<double>();
            LineTarget = mLineTarget->points[target_index].getNormalVector3fMap().cast<double>();

            Eigen::Vector3d error_1, error_2;
            error_1 = (trans.inverse() * poseTarget - poseSource).head<3>().cross(LineSource);
            error_2 = (poseTarget - trans * poseSource).head<3>().cross(LineTarget);
            sum_errors += (error_1.transpose() * error_1 + error_2.transpose() * error_2).norm();

            Eigen::Matrix<double, 3, 6> jlossexp1, jlossexp2;
            jlossexp1.block<3, 3>(0, 0) = -skewd(LineSource) * skewd((trans.inverse() * poseTarget).head<3>());
            jlossexp1.block<3, 3>(0, 3) = skewd(LineSource);

            jlossexp2.block<3, 3>(0, 0) = -skewd(LineTarget) * skewd((trans * poseSource).head<3>());
            jlossexp2.block<3, 3>(0, 3) = skewd(LineTarget);

            Eigen::Matrix<double, 6, 6> Hi = jlossexp1.transpose() * jlossexp1 + jlossexp2.transpose() * jlossexp2;
            Eigen::Matrix<double, 6, 1> bi = jlossexp1.transpose() * error_1 + jlossexp2.transpose() * error_2;

            Hs[omp_get_thread_num()] += Hi;
            bs[omp_get_thread_num()] += bi;
        }

    }

    if (H && b) {
        H->setZero();
        b->setZero();
        for (int i = 0; i < numThread; i++) {
            (*H) += Hs[i];
            (*b) += bs[i];
        }
    }

    return sum_errors;
}

bool geometryICP::stepOptimize(Eigen::Isometry3d &x0, Eigen::Isometry3d &delta) {
    Eigen::Matrix<double, 6, 6> H;
    Eigen::Matrix<double, 6, 1> b;
    double y0 = linearize(x0, &H, &b);

    if (lmLambda < 0.0) {
        double lm_init_lambda_factor_ = 1e-9;
        lmLambda = lm_init_lambda_factor_ * H.diagonal().array().abs().maxCoeff();
    }

    double nu = 2.0;
    for (int i = 0; i < lmMaxIteration; i++) {
        Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(H + lmLambda * Eigen::Matrix<double, 6, 6>::Identity());
        Eigen::Matrix<double, 6, 1> d = solver.solve(-b);

        delta.setIdentity();
        delta.linear() = so3_exp(d.head<3>()).toRotationMatrix();
        delta.translation() = d.tail<3>();

        Eigen::Isometry3d xi = delta * x0;
        double yi = computeError(xi);
        double rho = (y0 - yi) / (d.dot(lmLambda * d - b));

//        std::cout<<"H:\n"<<H<<"\nb: "<<b.transpose()<<"\nd:"<<d.transpose()<<"\n rho: "<<rho<<std::endl;

        if (rho < 0) {
            if (isConverged(delta)) {
                return true;
            }

            lmLambda = nu * lmLambda;
            nu = 2 * nu;
            continue;
        }

        x0 = xi;
        lmLambda = lmLambda * std::max(1.0 / 3.0, 1 - std::pow(2 * rho - 1, 3));
        finalHessian = H;
        return true;
    }

    return false;
}

void geometryICP::transCompare(const Eigen::Isometry3d &trans) {
    double error1, error2;
    updateCorrespondences(finalTrans);

    for (int i = 0; i < mPlaneSource->size(); i++) {
        int target_index = correspondences[i];
        if (target_index < 0) {
            continue;
        }
        Eigen::Vector4d poseSource, poseTarget, PlaneSource, PlaneTarget, transPlaneSource;
        poseSource = mPlaneSource->points[i].getVector4fMap().cast<double>();
        poseTarget = mPlaneTarget->points[target_index].getVector4fMap().cast<double>();
        poseSource(3) = poseTarget(3) = 1;
        PlaneSource = mPlaneSource->points[i].getNormalVector4fMap().cast<double>();
        PlaneTarget = mPlaneTarget->points[target_index].getNormalVector4fMap().cast<double>();
        transPlaneSource = finalTrans.matrix().transpose().inverse() * PlaneSource;

        double error_1, error_2;
        error_1 = poseTarget.transpose() * transPlaneSource;
        error_2 = PlaneTarget.transpose() * finalTrans.matrix() * poseSource;
        error1 += error_1 * error_1 + error_2 * error_2;
    }

    if (mLineTarget->size() > 0 && mLineSource->size() > 0) {
        for (int i = 0; i < mLineSource->size(); i++) {
            int target_index = correspondences[i + mPlaneSource->size()];
            if (target_index < 0) {
                continue;
            }

            Eigen::Vector4d poseSource, poseTarget;
            Eigen::Vector3d LineSource, LineTarget;
            poseSource = mLineSource->points[i].getVector4fMap().cast<double>();
            poseTarget = mLineTarget->points[target_index].getVector4fMap().cast<double>();
            poseSource(3) = poseTarget(3) = 1;
            LineSource = mLineSource->points[i].getNormalVector3fMap().cast<double>();
            LineTarget = mLineTarget->points[target_index].getNormalVector3fMap().cast<double>();

            Eigen::Vector3d error_1, error_2;
            error_1 = (finalTrans.inverse() * poseTarget - poseSource).head<3>().cross(LineSource);
            error_2 = (poseTarget - finalTrans * poseSource).head<3>().cross(LineTarget);
            error1 += (error_1.transpose() * error_1 + error_2.transpose() * error_2).norm();
        }
    }

    updateCorrespondences(trans);
    for (int i = 0; i < mPlaneSource->size(); i++) {
        int target_index = correspondences[i];
        if (target_index < 0) {
            continue;
        }
        Eigen::Vector4d poseSource, poseTarget, PlaneSource, PlaneTarget, transPlaneSource;
        poseSource = mPlaneSource->points[i].getVector4fMap().cast<double>();
        poseTarget = mPlaneTarget->points[target_index].getVector4fMap().cast<double>();
        poseSource(3) = poseTarget(3) = 1;
        PlaneSource = mPlaneSource->points[i].getNormalVector4fMap().cast<double>();
        PlaneTarget = mPlaneTarget->points[target_index].getNormalVector4fMap().cast<double>();
        transPlaneSource = trans.matrix().transpose().inverse() * PlaneSource;

        double error_1, error_2;
        error_1 = poseTarget.transpose() * transPlaneSource;
        error_2 = PlaneTarget.transpose() * trans.matrix() * poseSource;
        error2 += error_1 * error_1 + error_2 * error_2;

        if (error_1 * error_1 + error_2 * error_2 > 1000) {
            std::cout << "mPlaneSource: " << PlaneSource.transpose() << "\ntrans*mPlaneSource: "
                      << transPlaneSource.transpose() << " \nposeTarget: "
                      << poseTarget.transpose()
                      << "\ntran*poseSource: " << (trans.matrix() * poseSource).transpose()
                      << "\ntran*poseSource*trans*mPlaneSource: "
                      << (trans.matrix() * PlaneSource).transpose() * transPlaneSource
                      << "\nposeSource*mPlaneSource: " << poseSource.transpose() * PlaneSource
                      << "\nmPlaneTarget*tran*poseSource: " << error_2
                      << "\ntran*poseSource*mPlaneTarget " << (trans.matrix() * poseSource).transpose() * PlaneTarget
                      << "\nerror_1: " << error_1 << " error_2: " << error_2 << "\n\n";
        }
    }

    if (mLineTarget->size() > 0 && mLineSource->size() > 0) {
        std::cout << "line number: " << mLineSource->size() << std::endl;
        for (int i = 0; i < mLineSource->size(); i++) {
            int target_index = correspondences[i + mPlaneSource->size()];
            if (target_index < 0) {
                continue;
            }

            Eigen::Vector4d poseSource, poseTarget;
            Eigen::Vector3d LineSource, LineTarget;
            poseSource = mLineSource->points[i].getVector4fMap().cast<double>();
            poseTarget = mLineTarget->points[target_index].getVector4fMap().cast<double>();
            poseSource(3) = poseTarget(3) = 1;
            LineSource = mLineSource->points[i].getNormalVector3fMap().cast<double>();
            LineTarget = mLineTarget->points[target_index].getNormalVector3fMap().cast<double>();

            Eigen::Vector3d error_1, error_2;
            error_1 = (trans.inverse() * poseTarget - poseSource).head<3>().cross(LineSource);
            error_2 = (poseTarget - trans * poseSource).head<3>().cross(LineTarget);
            error2 += (error_1.transpose() * error_1 + error_2.transpose() * error_2).norm();

            if ((error_1.transpose() * error_1 + error_2.transpose() * error_2).norm() > 100) {
                std::cout << "trans.inverse()*poseTarget-poseSource: "
                          << (trans.inverse() * poseTarget - poseSource).transpose()
                          << "\nLineSource: " << LineSource.transpose()
                          << "\nposeTarget-trans*poseSource: " << (poseTarget - trans * poseSource).transpose()
                          << "\nerror_1: " << error_1.transpose() * error_1 << "\nerror2: "
                          << error_2.transpose() * error_2 << "\n";
            }
        }
    }

    if (debugPrintLevel) {
//        std::cout << "finalPose:\n" << finalTrans.matrix() << "\ntrans:\n" << trans.matrix() << std::endl;
//        LOG(INFO) << "finalTrans error: " << error1 << "  , trans error: " << error2;
//        double fitScore1, fitScore2;
//        updateCorrespondences(finalTrans);
//        fitScore1 = getFitScore();
//        finalTrans = trans;
//        updateCorrespondences(finalTrans);
//        fitScore2 = getFitScore();
//        LOG(INFO) << "finalTrans fit score: " << fitScore1 << "  , trans fit score: " << fitScore2;
    }

}

void geometryICP::setSourceCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr PlaneSource,
                                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr LineSource) {
    mPlaneSource.swap(PlaneSource);
    kdTreePlaneSource->setInputCloud(mPlaneSource);
    mLineSource.swap(LineSource);
    if (mLineSource->size() > 0)
        kdTreeLineSource->setInputCloud(mLineSource);
}

void geometryICP::setTargetCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr PlaneTarget,
                                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr LineTarget) {
    mPlaneTarget.swap(PlaneTarget);
    kdTreePlaneTarget->setInputCloud(mPlaneTarget);
    mLineTarget.swap(LineTarget);
    if (mLineTarget->size() > 0)
        kdTreeLineTarget->setInputCloud(mLineTarget);
}
