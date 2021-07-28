//
// Created by sakura on 2021/6/23.
//
#include "tools/geometryEstimate.h"

struct callback_args {
    // structure used to pass arguments to the callback function
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void pp_callback(const pcl::visualization::PointPickingEvent &event, void *args) {
    struct callback_args *data = (struct callback_args *) args;
    if (event.getPointIndex() == -1)
        return;
    pcl::PointXYZRGB current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.push_back(current_point);
    // Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,
                                                      "clicked_points");
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

geometryEstimate::geometryEstimate() {

    num_threads_ = omp_get_max_threads();
    angleThreshodVerticalPlane = 15;
    angleThreadGround = 75;
    intensityThreshod = 50;
    normalThreshod = sin(9 * M_PI / 180);
    towardsThreshod = sin(25 * M_PI / 180);
    labelThreshodPlane = 50;
    labelThreshodLine = 10;
    labelConsistThreshod = 0.9;
    planeThreshod = 10;
    nearK = 20;
    debugPrintLevel = 1;
    labelNum = 0;
    lineThreshod = 0.6;
    usingLine = false;
}

void handleEigenValue(const Eigen::Vector3f &EigenValue, std::vector<int> &index) {
    if (EigenValue(0) >= EigenValue[1]) {
        if (EigenValue(1) >= EigenValue[2]) {
            std::vector<int>{0, 1, 2}.swap(index);
            return;
        }
        if (EigenValue[0] > EigenValue[2]) {
            std::vector<int>{0, 2, 1}.swap(index);
            return;
        } else {
            std::vector<int>{2, 0, 1}.swap(index);
            return;
        }
    } else {
        if (EigenValue(0) >= EigenValue[2]) {
            std::vector<int>{1, 0, 2}.swap(index);
            return;
        }
        if (EigenValue[1] > EigenValue[2]) {
            std::vector<int>{1, 2, 0}.swap(index);
            return;
        } else {
            std::vector<int>{2, 1, 0}.swap(index);
            return;
        }
    }
}

void geometryEstimate::extract(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    mCloud.swap(cloud);
    mPlaneCloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
    mLineCloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>);

    //Neighbour find
//    auto t1 = std::chrono::steady_clock::now();
    int point_number = static_cast<int> (mCloud->size());
    pcl::search::KdTree<pcl::PointXYZI>::Ptr search_(new pcl::search::KdTree<pcl::PointXYZI>);
    search_->setInputCloud(mCloud);
    std::vector<std::vector<int>> point_neighbours_(point_number, std::vector<int>());
    std::vector<float> distances(nearK);
    std::vector<int> neighbours(nearK);
#pragma omp parallel for shared (point_neighbours_) firstprivate(distances, neighbours) num_threads(num_threads_) schedule(guided, 8)
    for (int i_point = 0; i_point < point_number; i_point++) {
        search_->nearestKSearch(i_point, nearK, neighbours, distances);
        point_neighbours_[i_point].swap(neighbours);
    }

//    auto t2 = std::chrono::steady_clock::now();
//    if (debugPrintLevel)
//        LOG(INFO) << "Find Neighbour cost: "
//                  << std::chrono::duration_cast<std::chrono::duration<double >>(t2 - t1).count() * 1000 << "ms";

    //normal estimate and parameters solve
    pcl::PointCloud<pcl::Normal> mCLoudNormal, mCloudTowards;
    mCLoudNormal.resize(mCloud->size());
    mCloudTowards.resize(mCloud->size());
    vector<int>(point_number, 0).swap(labels);
    vector<int> lineIndex;
    vector<vector<int>> lineIndexOmp(num_threads_);

#pragma omp parallel for shared (mCLoudNormal, mCloudTowards) num_threads(num_threads_) schedule(guided, 8)
    for (int i_point = 0; i_point < point_number; i_point++) {
        Eigen::Vector4f norm;
        // Placeholder for the 3x3 covariance matrix at each surface patch
        EIGEN_ALIGN16
        Eigen::Matrix3f covariance_matrix;
        // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
        Eigen::Vector4f xyz_centroid;
        pcl::computeMeanAndCovarianceMatrix(*mCloud, point_neighbours_[i_point], covariance_matrix, xyz_centroid);
        Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);

        Eigen::Vector3f eigenValue = es.eigenvalues().real().array().abs();
        Eigen::Matrix3f eigenVector = es.eigenvectors().real();

        std::vector<int> index;
        handleEigenValue(eigenValue, index);

        norm.head<3>() = eigenVector.col(index[2]);

        norm[3] = 0;
        norm[3] = -1 * norm.head<3>().dot(mCloud->points[i_point].getVector3fMap());

        double line = (eigenValue.maxCoeff() - eigenValue(index[1])) / eigenValue.maxCoeff();

        if (line > lineThreshod)
            lineIndexOmp[omp_get_thread_num()].push_back(i_point);

        Eigen::Vector3f towards = eigenVector.col(index[0]);
//        pcl::flipNormalTowardsViewpoint(mCloud->points[i_point], 0, 0, 0, norm);
//        pcl::flipNormalTowardsViewpoint(mCloud->points[i_point],0,0,0,towards);
        mCLoudNormal[i_point].getNormalVector4fMap() = norm;
        mCloudTowards[i_point].getNormalVector3fMap() = towards;
    }

    for (auto iter:lineIndexOmp)
        lineIndex.insert(lineIndex.end(), iter.begin(), iter.end());
    sort(lineIndex.begin(),lineIndex.end());

//    auto t3 = std::chrono::steady_clock::now();
//    if (debugPrintLevel)
//        LOG(INFO) << "Normal Estimate cost: "
//                  << std::chrono::duration_cast<std::chrono::duration<double >>(t3 - t2).count() * 1000 << "ms";

    //interest region select

    double angleThreshodPlane_ = cos(angleThreshodVerticalPlane * M_PI / 180), angleThreadGround_ = cos(
            angleThreadGround * M_PI / 180);
#pragma omp parallel for shared (mCloud, labels) num_threads(num_threads_) schedule(guided, 8)
    for (int i = 0; i < point_number; i++) {
        auto p = mCLoudNormal[i];
        double pro = sqrt(p.normal_x * p.normal_x + p.normal_y * p.normal_y);
        if (pro > angleThreshodPlane_ || pro < angleThreadGround_)
            labels[i] = 1;
    }

//    auto t4 = std::chrono::steady_clock::now();
//    if (debugPrintLevel)
//        LOG(INFO) << "Interest region select cost: "
//                  << std::chrono::duration_cast<std::chrono::duration<double >>(t4 - t3).count() * 1000 << "ms";

    //normal region grow
    labelNum = 1;
    int seed = 0;
    std::vector<std::vector<int>> labelPoints;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normalMeans;
    while (seed < point_number) {
        std::queue<int> seeds;
        seeds.push(seed);
        labels[seed] = labelNum + 1;
        std::vector<int> labelPoint{seed};
        Eigen::Vector3f meanNormal = Eigen::Vector3f::Zero();
        while (!seeds.empty()) {
            auto point = mCLoudNormal[seeds.front()].getNormalVector3fMap();
            meanNormal += point;
            for (auto iter:point_neighbours_[seeds.front()])
                if (labels[iter] == 1) {
                    auto nei = mCLoudNormal[iter].getNormalVector3fMap();
                    double errorNormal;
                    errorNormal = point.cross(nei).norm();
                    if (errorNormal < normalThreshod) {
                        labels[iter] = labelNum + 1;
                        seeds.push(iter);
                        labelPoint.push_back(iter);
                    }
                }
            seeds.pop();
        }

        meanNormal.normalize();
        normalMeans.push_back(meanNormal);
        labelPoints.push_back(labelPoint);

        while (seed < point_number && labels[seed] > 1)
            seed++;
        labelNum++;
    }

//    auto t5 = std::chrono::steady_clock::now();
//
//    if (debugPrintLevel)
//        LOG(INFO) << "Intensity region grow cost: "
//                  << std::chrono::duration_cast<std::chrono::duration<double >>(t5 - t4).count() * 1000 << "ms";

    int numPlanes = 0, groundPlane = 0;
    vector<int>(labelNum + 1, 0).swap(labelValid);

    for (int i = 0; i < labelPoints.size(); i++)
        if (labelPoints[i].size() > labelThreshodPlane) {
            labelValid[i + 2] = 1;
            auto norm = normalMeans[i];
            double pro = sqrt(norm[0] * norm[0] + norm[1] * norm[1]);
            if (pro < angleThreadGround_) {
                groundPlane = 1;
                for (auto iter:labelPoints[i]) {
                    pcl::PointXYZINormal point;
                    point.getVector4fMap() = mCloud->points[iter].getVector4fMap();
                    point.getNormalVector4fMap() = mCLoudNormal[iter].getNormalVector4fMap();
                    mPlaneCloud->push_back(point);
                }
            } else {
                numPlanes++;
                for (auto iter:labelPoints[i]) {
                    pcl::PointXYZINormal point;
                    point.getVector4fMap() = mCloud->points[iter].getVector4fMap();
                    point.getNormalVector4fMap() = mCLoudNormal[iter].getNormalVector4fMap();
                    mPlaneCloud->push_back(point);
                }
            }
        }

//    auto t6 = std::chrono::steady_clock::now();
//    if (debugPrintLevel)
//        LOG(INFO) << "Plane point size: " << mPlaneCloud->size()
//                  << " with " << numPlanes + groundPlane << " planes. Valid point extract cost: " <<
//                  std::chrono::duration_cast<std::chrono::duration<double >>(t6 - t5).count() * 1000 << "ms";

    if (numPlanes + groundPlane > planeThreshod) {
        usingLine = false;
        return;
    }

    usingLine = true;

    //towards region grow
    seed = 0;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> towardsMeans;
    std::vector<std::vector<int>> lineLabelPoints;
    std::vector<int> lineIndexValid;
    for (int i = 0; i < lineIndex.size(); i++)
        if (!labelValid[labels[lineIndex[i]]]) {
            lineIndexValid.push_back(lineIndex[i]);
            labels[lineIndex[i]]=labelNum;
        }
    lineIndexValid.swap(lineIndex);
    labelNumPlane = labelNum;
    labelValid.push_back(0);
    while (seed < lineIndex.size()) {
        std::queue<int> seeds;
        seeds.push(lineIndex[seed]);
        labels[lineIndex[seed]] = labelNum + 1;
        std::vector<int> labelPoint{lineIndex[seed]};
        Eigen::Vector3f towardsMean = Eigen::Vector3f::Zero();
        while (!seeds.empty()) {
            auto point = mCloudTowards[seeds.front()].getNormalVector3fMap();
            towardsMean += point;
            for (auto iter:point_neighbours_[seeds.front()]) {
                if (labels[iter] == labelNumPlane) {
                    auto nei = mCloudTowards[iter].getNormalVector3fMap();
                    double errorTowards;
                    errorTowards = point.cross(nei).norm();
                    if (errorTowards < towardsThreshod) {
                        labels[iter] = labelNum + 1;
                        seeds.push(iter);
                        labelPoint.push_back(iter);
                    }
                }
            }
            seeds.pop();
        }

        towardsMean.normalize();
        towardsMeans.push_back(towardsMean);
        lineLabelPoints.push_back(labelPoint);

        while (seed < lineIndex.size() && labels[lineIndex[seed]] > labelNumPlane)
            seed++;
        labelNum++;
    }
//    static int ii=0;
//    cout<<ii<<endl;
//    ofstream linelabel("linelabel"+ to_string(ii++)+".txt");
//    for (auto iter:lineIndex)
//        linelabel<<labels[iter]<<endl;
//    linelabel.close();


//    auto t7 = std::chrono::steady_clock::now();
//
//    if (debugPrintLevel)
//        LOG(INFO) << "towards region grow cost: "
//                  << std::chrono::duration_cast<std::chrono::duration<double >>(t7 - t6).count() * 1000 << "ms";

    labelValid.resize(labelNum + 2);

    int numLines = 0;
    for (int i = 0; i < lineLabelPoints.size(); i++) {
        if (lineLabelPoints[i].size() > labelThreshodLine) {
            labelValid[i + labelNumPlane + 1] = 1;
            numLines++;
            for (auto iter:lineLabelPoints[i]) {
                pcl::PointXYZINormal point;
                point.getVector4fMap() = mCloud->points[iter].getVector4fMap();
                point.getNormalVector3fMap() = mCloudTowards[iter].getNormalVector3fMap();
                mLineCloud->push_back(point);
            }
        } else
            labelValid[i + labelNumPlane + 1] = 0;
    }

//    auto t8 = std::chrono::steady_clock::now();
//    if (debugPrintLevel)
//        LOG(INFO) << "Line point size: " << mLineCloud->size()
//                  << " with " << numLines << " Lines. Valid point extract cost: " <<
//                  std::chrono::duration_cast<std::chrono::duration<double >>(t8 - t7).count() * 1000 << "ms";
//
    if (debugPrintLevel == 2)
        visualization();

}

void geometryEstimate::visualization() {

    std::vector<std::vector<int>> colors(labelNum, std::vector<int>(3, 0));

    std::random_device random;
    std::default_random_engine randomEngine(random());
    std::uniform_int_distribution<int> color(0, 255);

    for (int i = 0; i < labelNum; i++) {
        colors[i][0] = color(randomEngine);
        colors[i][1] = color(randomEngine);
        colors[i][2] = color(randomEngine);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudViewInitial(new pcl::PointCloud<pcl::PointXYZRGB>), cloudViewFinal(
            new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int i = 0; i < mCloud->size(); i++) {
        pcl::PointXYZRGB p;
        p.getVector3fMap() = mCloud->points[i].getVector3fMap();
        p.r = p.g = p.b = 255;
        cloudViewInitial->push_back(p);
        if (labelValid[labels[i]]) {
            if (labels[i] < labelNumPlane) {
                int label = labels[0];
                p.r = colors[label][0];
                p.g = colors[label][1];
                p.b = colors[label][2];
                cloudViewFinal->push_back(p);
            } else {
                int label = labels[i];
                p.r = colors[label][0];
                p.g = colors[label][1];
                p.b = colors[label][2];
                cloudViewFinal->push_back(p);
            }
        }
    }

    pcl::visualization::PCLVisualizer viewer("viewer");
    int v1 = 1, v2 = 2;
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1, 1.0, v2);
    viewer.addPointCloud(cloudViewInitial, "All", v1);
    viewer.addPointCloud(cloudViewFinal, "Valid", v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "All");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Valid");
    viewer.setCameraPosition(0, 0, 150, 0, 1, 0);

//    struct callback_args cb_args;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZRGB>);
//    cb_args.clicked_points_3d = clicked_points_3d;
//    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
//    viewer.registerPointPickingCallback(pp_callback, (void *) &cb_args);

    while (!viewer.wasStopped())
        viewer.spinOnce();

}