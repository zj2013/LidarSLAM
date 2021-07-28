//
// Created by sakura on 2021/6/30.
//

#include <curl.h>

size_t write_data(void *ptr, size_t size, size_t nmemb, void *stream) {
    std::string data((const char *) ptr, (size_t) size * nmemb);
    *((std::stringstream *) stream) << data << std::endl;
    return size * nmemb;
}

double dis(const coordinate &poseA, const coordinate &poseB) {
    auto diffA = poseA.longitude - poseB.longitude;
    auto diffB = poseA.latitude - poseB.latitude;
    return sqrt(diffA * diffA + diffB * diffB);
}

double tran(double input){
    int abc,de,fghi;
    abc=floor(input/100);
    de= (int )floor(input)%100;
    fghi=(input- floor(input))*10000;

    return abc+de/60.0+fghi/600000.0;
}

void baiduMap::getJsonFromUrl(const string &url, Json::Value &back) {
    CURL *curl;             //定义CURL类型的指针
    CURLcode res;          //定义CURLcode类型的变量，保存返回状态码
    stringstream callBack, modified;
    string json;

    curl = curl_easy_init();        //初始化一个CURL类型的指针
    if (curl != nullptr) {
        //设置URL
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &callBack);
        //调用curl_easy_perform 执行我们的设置.并进行相关的操作. 在这 里只在屏幕上显示出来.
        res = curl_easy_perform(curl);
        if (res != CURLE_OK) {
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
        }
        //清除curl操作.
        curl_easy_cleanup(curl);
    }

    json = callBack.str();
    json.erase(std::remove(json.begin(), json.end(), '\n'), json.end());
    modified << json;

    Json::CharReaderBuilder builder;
    builder["collectComments"] = true;
    JSONCPP_STRING errs;
    if (!parseFromStream(builder, modified, &back, &errs)) {
        std::cout << errs << std::endl;
    }
}

void baiduMap::BaiduPathWgs84(coordinate origin, coordinate destination) {
    std::string web, geoConv;
    Json::Value root, steps;

    //31.8397115
//117.2507621
//31.8433966	117.2511572
// 31.839921  117.251108
    origin.latitude = 31.839921;
    origin.longitude = 117.251108;
    destination.latitude = 31.8433966;
    destination.longitude = 117.2511572;

    geoConv = "https://api.map.baidu.com/geoconv/v1/?coords=" + to_string(origin.longitude) + "," +
              to_string(origin.latitude) + "&from=1&to=5&ak=pcOrLHGdG9YfvxdswG6gjkn3VWD7G6m7";
    getJsonFromUrl(geoConv, root);
    origin.longitude = stod(root["result"][0]["x"].asString());
    origin.latitude = stod(root["result"][0]["y"].asString());

    geoConv = "https://api.map.baidu.com/geoconv/v1/?coords=" + to_string(destination.longitude) + "," +
              to_string(destination.latitude) + "&from=1&to=5&ak=pcOrLHGdG9YfvxdswG6gjkn3VWD7G6m7";
    getJsonFromUrl(geoConv, root);
    destination.longitude = stod(root["result"][0]["x"].asString());
    destination.latitude = stod(root["result"][0]["y"].asString());


    web = "https://api.map.baidu.com/directionlite/v1/walking?origin=" + to_string(origin.latitude) + "," +
          to_string(origin.longitude) +
          "&destination=" + to_string(destination.latitude) + "," + to_string(destination.longitude) +
          "&ak=pcOrLHGdG9YfvxdswG6gjkn3VWD7G6m7";


    getJsonFromUrl(web, root);
    cout << root << endl;
    steps = root["result"]["routes"][0]["steps"];

    std::vector<std::pair<double, double>> paths;
    for (int i = 0; i < steps.size(); i++) {
        std::string path = steps[i]["path"].asString();
        for (auto &iter:path) {
            if (iter == ',' || iter == ';')
                iter = ' ';
        }
        stringstream pathS;
        pathS << path;
        if (i != 0)
            paths.pop_back();
        while (!pathS.eof()) {
            double x, y;
            pathS >> x >> y;
            paths.emplace_back(x, y);
        }
    }


    for (auto iter:paths)
        cout << fixed << setprecision(12) << iter.first << " " << iter.second << endl;
}

void baiduMap::Wgs84ToMercator(const coordinate &Wgs84, coordinate &xy) {
    std::string geoConv;
    Json::Value root;
    geoConv = "https://api.map.baidu.com/geoconv/v1/?coords=" + to_string(Wgs84.longitude) + "," +
              to_string(Wgs84.latitude) + "&from=1&to=4&ak=pcOrLHGdG9YfvxdswG6gjkn3VWD7G6m7";
    getJsonFromUrl(geoConv, root);
    xy.longitude = stod(root["result"][0]["x"].asString());
    xy.latitude = stod(root["result"][0]["y"].asString());

}

void baiduMap::PointTest() {
    vector<coordinate> poses{coordinate(31.840188, 117.249835), coordinate(31.840184, 117.249873),
                             coordinate(31.83971, 117.250802), coordinate(31.839743, 117.250822),
                             coordinate(31.841053, 117.25177), coordinate(31.841066, 117.251748),
                             coordinate(31.84301, 117.251305), coordinate(31.843025, 117.251295),
                             coordinate(31.842195, 117.251151), coordinate(31.842184, 117.251166)};

    ofstream p("poses.txt");
    for (int i = 0; i < 5; i++) {
        coordinate xy1, xy2;
        Wgs84ToMercator(poses[2 * i], xy1);
        Wgs84ToMercator(poses[2 * i + 1], xy2);
        cout << dis(xy1, xy2) << endl;
        p << fixed << xy1.latitude << " " << xy1.longitude << endl;
        p << fixed << xy2.latitude << " " << xy2.longitude << endl;
    }
    p.close();

    for (int i = 0; i < 4; i++)
        for (int j = i + 1; j < 5; j++) {
            coordinate start1, start2, end1, end2;
            Wgs84ToMercator(poses[2 * i], start1);
            Wgs84ToMercator(poses[2 * i + 1], start2);
            Wgs84ToMercator(poses[2 * j], end1);
            Wgs84ToMercator(poses[2 * j + 1], end2);
            cout << dis(start1, end1) << " " << dis(start2, end2) << endl;
        }
};

void baiduMap::PoseDraw() {
    vector<double> latitudes, longitudes;
    ifstream gps("/home/sakura/Campus/calibration/4/gps.txt");
    int nums = 0;

    auto Start = chrono::high_resolution_clock::now();
    while (!gps.eof()) {
        string line;
        getline(gps, line);
        if (line.size()>0){
            stringstream ss(line);
            double time;
            coordinate wgs,Mercator;
            ss>>time>>wgs.latitude>>wgs.longitude;
            Wgs84ToMercator(wgs, Mercator);

            latitudes.push_back(Mercator.latitude);
            longitudes.push_back(Mercator.longitude);
            nums++;
            cout<<nums<<endl;
        }

//        int la, lo;
//        la = line.find("LL,");
//        lo = line.find(",N,");
//        if (line.size() > 15)
//            nums++;
//
//        if (la > 0 && lo > 0 && line.size() > 50) {
//            coordinate wgs,Mercator;
//            wgs.latitude = tran(stod(line.substr(la + 3, 10)));
//            wgs.longitude = tran(stod(line.substr(lo + 3, 10)));
//
//            Wgs84ToMercator(wgs,Mercator);
//
//            latitudes.push_back(Mercator.latitude);
//            longitudes.push_back(Mercator.longitude);
//        }

    }
    auto End = chrono::high_resolution_clock::now();

    gps.close();

    cout << nums << " " << latitudes.size() << endl;
    cout << "Trans cost: "
              << std::chrono::duration_cast<std::chrono::duration<double>>(End - Start).count() << "s";
    cout<< "fps: " << nums /
                            (std::chrono::duration_cast<std::chrono::duration<double>>(End - Start).count());

    ofstream out("/home/sakura/Campus/calibration/4/coordinate.txt");
    for (int i = 0; i < latitudes.size(); i++) {
        out << fixed << setprecision(6) << latitudes[i] << " " << longitudes[i] << endl;
    }
    out.close();

}