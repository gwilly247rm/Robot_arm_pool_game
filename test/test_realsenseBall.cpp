#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#define OPENCV

#include "realsense.h"
#include "yaml-cpp/yaml.h"
#include "yolo_v2_class.hpp"

using namespace std;
using namespace cv;
using namespace realsense;

void DrawBox(Mat img, String name, bbox_t obj);
std::vector<cv::Mat> cut(std::vector<bbox_t> ballList, cv::Mat realsenseRgb);
int white_threshold(std::vector<cv::Mat> cropImages, int threshold_min,
                    int kernel_size);
cv::Mat white_threshold2(cv::Mat cropImages, int threshold_min);

static void Hough_on_trackbar(int hough_threshold, void*) {}
static void threshold_trackbar(int threshold_min, void*) {}
static void kernel_size_trackbar(int kernel_size, void*) {}

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "parameter:<path of yaml>" << endl;
        return -1;
    }

    YAML::Node node = YAML::LoadFile(argv[1]);

    int max;

    // realsense
    RealSense rs;
    Config config;
    Mat realsenseRgb;
    Mat realsenseRgb_copy;
    Mat realsenseRgb_yolo;
    Mat realsenseRgb_cut;
    Mat white_ball;

    // HoughCircle
    vector<cv::Vec3f> circles;
    vector<cv::Vec3f> Wcircle;
    circles.clear();
    Wcircle.clear();
    Mat Allgray;
    Mat gray;

    int min_dist = node["find_white_center"]["min_dist"].as<int>();
    int canny_edge_detection =
        node["find_white_center"]["canny_edge_detection"].as<int>();
    int center_detection =
        node["find_white_center"]["center_detection"].as<int>();

    // yolo
    // Detector detect("./yolov4-realsense.cfg",
    // "./yolov4_final-realsense.weights");
    Detector detect("./yolov4-realsense.cfg", "./yolov4_final-realsense.weights");

    std::vector<bbox_t> objList;

    // threshold
    std::vector<cv::Mat> cropImages;
    std::vector<bbox_t> ballList;
    int threshold_min =
        node["white_threshold"]["realsense"]["threshold_min"].as<int>();
    int kernel_size =
        node["white_threshold"]["realsense"]["kernel_size"].as<int>();

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

    // run
    while (1) {
        rs.update();
        rs.retrieve_color_image(realsenseRgb);

        realsenseRgb_copy = realsenseRgb.clone();
        realsenseRgb_yolo = realsenseRgb.clone();
        realsenseRgb_cut = realsenseRgb.clone();

        // Draw center line
       /* line(realsenseRgb, Point(0, realsenseRgb.rows / 2),
             Point(realsenseRgb.cols, realsenseRgb.rows / 2),
             cv::Scalar(255, 100, 255), 2);
        line(realsenseRgb, Point(realsenseRgb.cols / 2, 0),
             Point(realsenseRgb.cols / 2, realsenseRgb.rows),
             cv::Scalar(255, 100, 255), 2);*/

        cvtColor(realsenseRgb_copy, Allgray, COLOR_BGR2GRAY);
        GaussianBlur(Allgray, Allgray, Size(9, 9), 2, 2);

        namedWindow("AllHoughCircles", WINDOW_AUTOSIZE);  // Create Window
        createTrackbar("min_dist", "AllHoughCircles", &min_dist, 250,
                       Hough_on_trackbar);
        createTrackbar("Canny_edge_detection", "AllHoughCircles",
                       &canny_edge_detection, 200, Hough_on_trackbar);
        createTrackbar("Center_detection", "AllHoughCircles", &center_detection,
                       100, Hough_on_trackbar);
        HoughCircles(
            Allgray, circles, HOUGH_GRADIENT, 1, min_dist, canny_edge_detection,
            center_detection, 0,
            0);  // min_dist:150,canny_edge_detection:100,center_detection:30

        /// Draw the circles detected
        for (size_t i = 0; i < circles.size(); i++) {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle(realsenseRgb_copy, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            // circle outline
            circle(realsenseRgb_copy, center, radius, Scalar(255, 0, 255), 3, 8,
                   0);
        }
        imshow("AllHoughCircles", realsenseRgb_copy);

        objList.clear();
        ballList.clear();

        objList = detect.detect(realsenseRgb, 0.8);

        for (size_t i = 0; i < objList.size(); i++) {
            if (objList[i].obj_id == 0) {
                DrawBox(realsenseRgb_yolo, "ball", objList[i]);
                ballList.push_back(objList[i]);
            }
        }

        if (ballList.size() == 0) {
            cout << "don't find any ball" << endl;
        }

        else {
            // find white
            cropImages.clear();
            cropImages = cut(ballList, realsenseRgb_cut);
            namedWindow("white", WINDOW_AUTOSIZE);
            createTrackbar("threshold", "white", &threshold_min, 255,
                           threshold_trackbar);
            createTrackbar("kernel_size", "white", &kernel_size, 50,
                           kernel_size_trackbar);

            max = white_threshold(cropImages, threshold_min, kernel_size);
            /*white_ball = white_threshold2(cropImages[max], threshold_min);
            imshow("white_ball", white_ball);*/

            // medianBlur(realsenseRgb, realsenseRgb, 5);
            cvtColor(cropImages[max], gray, COLOR_BGR2GRAY);
            GaussianBlur(gray, gray, Size(9, 9), 2, 2);

            HoughCircles(gray, Wcircle, HOUGH_GRADIENT, 1, min_dist,
                         canny_edge_detection, center_detection, 0, 0);

            /// Draw the circles detected
            for (size_t i = 0; i < Wcircle.size(); i++) {
                Point center(cvRound(Wcircle[i][0]), cvRound(Wcircle[i][1]));
                int radius = cvRound(Wcircle[i][2]);
                // circle center
                circle(cropImages[max], center, 3, Scalar(0, 255, 0), -1, 8, 0);
                // circle outline
                circle(cropImages[max], center, radius, Scalar(255, 0, 255), 3,
                       8, 0);
            }

            imshow("white_threshold", cropImages[max]);
        }

        imshow("realsenseRgb", realsenseRgb);

        if (waitKey(1) == 27) break;
    }

    node["white_threshold"]["realsense"]["threshold_min"] = threshold_min;
    node["white_threshold"]["realsense"]["kernel_size"] = kernel_size;

    node["find_white_center"]["min_dist"] = min_dist;
    node["find_white_center"]["canny_edge_detection"] = canny_edge_detection;
    node["find_white_center"]["center_detection"] = center_detection;

    ofstream fout(argv[1]);
    fout << node;
    fout.close();

    return 0;
}

void DrawBox(Mat img, String name, bbox_t obj) {
    putText(img, name + " : " + to_string((int)(obj.prob * 100)) + "%",
            Point(obj.x, obj.y), 0, 0.5, Scalar(255, 0, 0), 2);
    rectangle(img, Point(obj.x, obj.y), Point(obj.x + obj.w, obj.y + obj.h),
              Scalar(255, 0, 0), 1);
}

std::vector<cv::Mat> cut(vector<bbox_t> ballList, cv::Mat realsenseRgb) {
    cout << "--cut--" << endl;
    std::vector<cv::Mat> cropImages;

    for (size_t i = 0; i < ballList.size(); i++) {
        Rect rect(ballList[i].x, ballList[i].y, ballList[i].w, ballList[i].h);
        Mat cropImage = Mat(realsenseRgb, rect);
        cropImages.push_back(cropImage);
    }

    return cropImages;
}

int white_threshold(std::vector<cv::Mat> cropImages, int threshold_min,
                    int kernel_size) {
    cout << "--white_threshold--" << endl;
    Mat cropImages_gray;
    Mat cropImages_threshold;
    vector<int> white_count;
    unsigned int max = 0;
    int j, k;

    white_count.resize(cropImages.size());

    // turn gray and do threshole
    for (size_t i = 0; i < cropImages.size(); i++) {
        cvtColor(cropImages[i], cropImages_gray, COLOR_BGR2GRAY);

        threshold(cropImages_gray, cropImages_threshold, threshold_min, 255,
                  THRESH_BINARY);

        Mat element =
            getStructuringElement(MORPH_RECT, Size(kernel_size, kernel_size));
        //侵蝕
        erode(cropImages_threshold, cropImages_threshold, element);
        //膨脹
        dilate(cropImages_threshold, cropImages_threshold, element);

        // count white
        for (j = 0; j < cropImages_threshold.rows; j++) {
            for (k = 0; k < cropImages_threshold.cols; k++) {
                if (cropImages_threshold.at<uchar>(j, k) == 255) {
                    white_count[i]++;
                }
            }
        }
        std::string Name = "threshold" + std::to_string(i);
        imshow(Name,cropImages_threshold);
    }

    if (white_count.size() > 1) {
        for (size_t i = 1; i < white_count.size(); i++) {
            if (white_count[i] > white_count[max]) max = i;
        }
    }

    cout << "white_count_max[" << max << "]=" << white_count[max] << endl;
    imshow("white", cropImages[max]);

    return max;
}
