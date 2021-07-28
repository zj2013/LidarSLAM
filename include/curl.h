//
// Created by sakura on 2021/7/27.
//

#ifndef LIDAR_SLAM_CURL_H
#define LIDAR_SLAM_CURL_H

#include <curl/curl.h>
#include <string>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <chrono>
#include <cmath>

using namespace std;

class coordinate {
public:
    explicit coordinate(double La = 0, double Lo = 0) :
            latitude(La), longitude(Lo) {

    }

    double latitude, longitude;
};

class baiduMap{
public:

    baiduMap(){};
    void getJsonFromUrl(const string &url, Json::Value &back);
    void BaiduPathWgs84(coordinate origin, coordinate destination);
    void Wgs84ToMercator(const coordinate &Wgs84, coordinate &xy);

private:

    void PointTest();
    void PoseDraw();

};

#endif //LIDAR_SLAM_CURL_H
