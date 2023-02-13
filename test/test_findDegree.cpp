#include <fstream>
#include <iostream>
#include <string>

#include "realsense.h"
#include "yaml-cpp/yaml.h"

using namespace std;
using namespace cv;
using namespace realsense;

void drawLines(cv::Mat input, const std::vector<cv::Vec2f>& lines);
double calcDeg(const std::vector<cv::Vec2f>& lines);

static void canny_on_trackbar(int canny_ratio, void*) {}

static void Hough_on_trackbar(int hough_threshold, void*) {}

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "parameter:<path of yaml>" << endl;
        return -1;
    }

    YAML::Node node = YAML::LoadFile(argv[1]);
    int canny_ratio = node["find_degree"]["canny"].as<int>();
    int hough_threshold = node["find_degree"]["hough"].as<int>();
    double degree = node["find_degree"]["degreeLimit"].as<double>();

    Mat gray;
    Mat canny;
    Mat realsenseRgb;
    // Mat drawLine = imread("test.jpg");

    Mat realsenseRgb_copy;

    vector<cv::Vec2f> lines;
    lines.clear();

    RealSense rs;
    Config config;

    // realsenseInit
    config.device_type = DeviceType::CAMERA_ID;
    config.camera_id = 0;

    StreamConfig color_config;
    color_config.stream = RS2_STREAM_COLOR;
    color_config.width = 1280;
    color_config.height = 720;

    StreamConfig depth_config;
    depth_config.stream = RS2_STREAM_DEPTH;
    depth_config.width = 640;
    depth_config.height = 480;

    config.stream_configs.push_back(color_config);
    config.stream_configs.push_back(depth_config);

    cout << "connect RealSense" << endl;
    rs.connect(config);

    while (1) {
        rs.update();
        rs.retrieve_color_image(realsenseRgb);

        realsenseRgb.copyTo(realsenseRgb_copy);

        cvtColor(realsenseRgb, gray, COLOR_BGR2GRAY);
        namedWindow("Canny", WINDOW_AUTOSIZE);  // Create Window
        createTrackbar("ratio", "Canny", &canny_ratio, 20, canny_on_trackbar);
        Canny(gray, canny, 70,
              canny_ratio * 70);  // origin:70/210,good:70/700,70/770

        namedWindow("Hough", WINDOW_AUTOSIZE);  // Create Window
        createTrackbar("threshold", "Hough", &hough_threshold, 200,
                       Hough_on_trackbar);
        HoughLines(canny, lines, 1, CV_PI / 180,
                   hough_threshold);  // origin:100,good:160-175
        drawLines(realsenseRgb, lines);
        degree = 90 - calcDeg(lines);

        if(degree >= 90)
            degree = degree - 180;

        imshow("Canny", canny);
        imshow("Hough", realsenseRgb);
        imshow("Origin", realsenseRgb_copy);

        cout << "degree:" << degree << endl;

        if (waitKey(1) == 27) break;
    }

    cout << "canny_threshold:" << canny_ratio * 70 << endl;
    cout << "hough_threshold:" << hough_threshold << endl;
    cout << "degree:" << degree << endl;

    node["find_degree"]["canny"] = canny_ratio * 70;
    node["find_degree"]["hough"] = hough_threshold;
    if(degree >= 0)
        node["find_degree"]["degreeLimit"] = degree + 1;
    else
        node["find_degree"]["degreeLimit"] = degree - 1;

    ofstream fout(argv[1]);
    fout << node;
    fout.close();

    return 0;
}

void drawLines(cv::Mat input, const std::vector<cv::Vec2f>& lines) {
    for (size_t i = 0; i < lines.size(); i++) {
        float r = lines[i][0];
        float theta = lines[i][1];
        if (theta < CV_PI / 4.0 || theta > 3 * CV_PI / 4.0) {
            cv::Point pt1(r / cos(theta), 0);
            cv::Point pt2((r - input.rows * sin(theta)) / cos(theta),
                          input.rows);
            line(input, pt1, pt2, cv::Scalar(255, 0, 0), 2);

        } else {
            cv::Point pt1(0, r / sin(theta));
            cv::Point pt2(input.cols,
                          (r - input.cols * cos(theta)) / sin(theta));
            cv::line(input, pt1, pt2, cv::Scalar(255, 0, 0), 2);
        }
    }
}

double calcDeg(const std::vector<cv::Vec2f>& lines) {
    double ret = -999;
    double s = 0, c = 0;
    std::vector<double> degs;
    degs.clear();

    if (lines.size() > 0) {
        ret = 0;
        for (const cv::Vec2f& line : lines) {
            degs.push_back(line[1] * 2.0);
        }
        for (double& d : degs) {
            s += sin(d);
            c += cos(d);
        }
        s /= degs.size();
        c /= degs.size();

        ret = atan2(s, c);
        ret /= 2;
        ret = ret * 180.0 / CV_PI;
    }
    return ret;
}
