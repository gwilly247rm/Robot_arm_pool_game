#include <iostream>
#include <opencv2/opencv.hpp>
#define OPENCV
#include <sl/Camera.hpp>
#include "yolo_v2_class.hpp"
#include "yaml-cpp/yaml.h"

using namespace std;
using namespace cv;

void DrawBox(Mat img, String name, bbox_t obj);
std::vector<cv::Mat> cut(std::vector<bbox_t> ballList, cv::Mat realsenseRgb);
static void threshold_trackbar(int threshold_min, void*);
static void kernel_size_trackbar(int kernel_size, void*);
void white_threshold(std::vector<cv::Mat> cropImages, int threshold_min, int kernel_size);
cv::Mat slMat2cvMat(sl::Mat& input);

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "parameter:<path of yaml>" << endl;
        return -1;
    }

    YAML::Node yamlConfig = YAML::LoadFile(argv[1]);

    // zed
    sl::Mat zedLeftRgb;
    Mat zedRgb;
    Mat zedRgb_copy;

    sl::Camera zed;
    sl::InitParameters init_parameters;
    sl::RuntimeParameters runtime_parameters;

    // yolo
    Detector detect("./yolov4.cfg", "./yolov4_final.weights");

    std::vector<bbox_t> objList;

    // threshold
    std::vector<cv::Mat> cropImages;
    std::vector<bbox_t> ballList;
    int threshold_min = yamlConfig["white_threshold"]["zed"]["threshold_min"].as<int>();
    int kernel_size = yamlConfig["white_threshold"]["zed"]["kernel_size"].as<int>();

    namedWindow("threshold", WINDOW_AUTOSIZE);  // Create Window
    namedWindow("kernel_size", WINDOW_AUTOSIZE);  // Create Window

    // zedInit
    init_parameters.camera_resolution =
        sl::RESOLUTION::HD1080;       // Use HD1080 video mode
    init_parameters.camera_fps = 30;  // Set fps at 30
    init_parameters.sdk_gpu_id = 0;
    init_parameters.sdk_cuda_ctx = (CUcontext)detect.get_cuda_context();

    // Open Zed
    cout << "Open Zed" << endl;
    sl::ERROR_CODE state = zed.open(init_parameters);
    if (state != sl::ERROR_CODE::SUCCESS) {
        cout << "Error " << state << ", exit program." << endl;
        return -1;
    }

    // run
    while (1) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS)
        {
            zed.retrieveImage(zedLeftRgb, sl::VIEW::LEFT);
        }

        cvtColor(slMat2cvMat(zedLeftRgb), zedRgb, COLOR_RGBA2RGB);

        zedRgb_copy = zedRgb.clone();

        objList.clear();
        ballList.clear();

        objList = detect.detect(zedRgb, 0.6);

        for (size_t i = 0; i < objList.size(); i++) {
            if (objList[i].obj_id == 0) {
                DrawBox(zedRgb_copy, "ball", objList[i]);
                ballList.push_back(objList[i]);
            }
            if (objList[i].obj_id == 1) {
                DrawBox(zedRgb_copy, "hole", objList[i]);
            }
        }

        if (ballList.size() == 0) {
            cout << "don't find any ball" << endl;
        }
        else {
            cropImages.clear();
            cropImages = cut(ballList, zedRgb);
            createTrackbar("threshold", "threshold", &threshold_min, 255,
                           threshold_trackbar);
            createTrackbar("kernel_size", "kernel_size", &kernel_size, 50,
                           kernel_size_trackbar);
            white_threshold(cropImages, threshold_min, kernel_size);
        }

        imshow("zedRgb", zedRgb_copy);

        if (waitKey(1) == 27) break;
    }

    yamlConfig["white_threshold"]["zed"]["threshold_min"] = threshold_min;
    yamlConfig["white_threshold"]["zed"]["kernel_size"] = kernel_size;

    ofstream fout(argv[1]);
    fout << yamlConfig;
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

static void threshold_trackbar(int threshold_min, void*) {}

static void kernel_size_trackbar(int kernel_size, void*) {}

void white_threshold(std::vector<cv::Mat> cropImages, int threshold_min, int kernel_size) {
    cout << "--white_threshold--" << endl;
    Mat cropImages_gray;
    Mat cropImages_threshold;
    vector<int> white_count;
    unsigned int max = 0;
    int j, k;

    white_count.resize(cropImages.size());

    // turn gray and do threshole
    for (size_t i = 0; i < cropImages.size(); i++) {

        std::string rbgName = "rgb" + std::to_string(i);
        imshow(rbgName,cropImages[i]);

        cvtColor(cropImages[i], cropImages_gray, COLOR_BGR2GRAY);

        std::string grayName = "gray" + std::to_string(i);
        imshow(grayName,cropImages_gray);

        threshold(cropImages_gray, cropImages_threshold, threshold_min, 255,
                  THRESH_BINARY);

        if(kernel_size == 0)
            kernel_size = 1;

        Mat element = getStructuringElement(MORPH_RECT, Size(kernel_size, kernel_size));
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

    /*for (size_t i = 0; i < white_count.size(); i++)
     *        cout << "white_count[" << i << "]:" << white_count[i] << endl;*/

    if (white_count.size() > 1) {
        for (size_t i = 1; i < white_count.size(); i++) {
            if (white_count[i] > white_count[max]) max = i;
        }
    }

    cout << "white_count_max[" << max << "]=" << white_count[max] << endl;
    imshow("white", cropImages[max]);
}

Mat slMat2cvMat(sl::Mat& input) {
    int cv_type = -1;  // Mapping between MAT_TYPE and CV_TYPE
    if (input.getDataType() == sl::MAT_TYPE::F32_C4) {
        cv_type = CV_32FC4;

    } else
        cv_type = CV_8UC4;  // sl::Mat used are either RGBA images or XYZ (4C)
    // point clouds
    return Mat(input.getHeight(), input.getWidth(), cv_type,
               input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

