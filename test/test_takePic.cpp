#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <string>

using namespace std;
using namespace cv;

Mat slMat2cvMat(sl::Mat& input);
string getFileName();

int main(int argc, char** argv) {
    sl::Camera zed;
    sl::InitParameters init_parameters;
    sl::RuntimeParameters runtime_parameters;

    sl::Mat zedLeftImage;
    Mat zedImage;

    sl::Resolution image_size = zed.getCameraInformation().camera_resolution;
    int new_width = image_size.width / 2;
    int new_height = image_size.height / 2;
    sl::Resolution new_image_size(new_width, new_height);

    sl::Mat depth_image_zed(new_width, new_height, sl::MAT_TYPE::U8_C4);
    cv::Mat depth_image_ocv;

    string fileName = "";
    char kbin;
    bool run = true;
 
    // Zed init
    cout << "ZedInit" << endl;
    init_parameters.camera_resolution =
        sl::RESOLUTION::HD1080;       // Use HD1080 video mode
    init_parameters.camera_fps = 30;  // Set fps at 30
   /* init_parameters.depth_mode = sl::DEPTH_MODE::QUALITY;
    init_parameters.coordinate_units = sl::UNIT::METER;
    init_parameters.depth_minimum_distance =
        0.3;  // Set the minimum depth perception distance to 30cm(zed2 minimum
              // limit)
    init_parameters.depth_maximum_distance =
        40;  // Set the maximum depth perception distance to 40m
    runtime_parameters.sensing_mode = sl::SENSING_MODE::FILL;*/

    // Open Zed
    cout << "Open Zed" << endl;
    sl::ERROR_CODE state = zed.open(init_parameters);
    if (state != sl::ERROR_CODE::SUCCESS) {
        cout << "Error " << state << ", exit program." << endl;
        return -1;
    }

    while (run) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            zed.retrieveImage(zedLeftImage, sl::VIEW::LEFT);
           // zed.retrieveImage(depth_image_zed, sl::VIEW::DEPTH, sl::MEM::CPU, new_image_size);

            cv::cvtColor(slMat2cvMat(zedLeftImage), zedImage,
                         cv::COLOR_RGBA2RGB);
           // depth_image_ocv = slMat2cvMat(depth_image_zed);

            imshow("picture", zedImage);
           // imshow("depth", depth_image_ocv);

            kbin = cv::waitKey(1);
            switch (kbin) {
                case 27:
                    run = false;
                    break;
                case 'c':
                    fileName = getFileName();
                    cv::imwrite("./pic/" + fileName, zedImage);
                    //cv::imwrite("./pic/depth_" + fileName, depth_image_ocv);
                    cout << "Write: " << fileName << endl;
                    //cout << "Write: depth_" << fileName << endl;
                    break;
            }
            //        imshow("picture", zedImage);
        }
    }
    zed.close();
    return 0;
}

Mat slMat2cvMat(sl::Mat& input) {
    int cv_type = -1;  // Mapping between MAT_TYPE and CV_TYPE
    if (input.getDataType() == sl::MAT_TYPE::F32_C4) {
        cv_type = CV_32FC4;

    } else
        cv_type = CV_8UC4;  // sl::Mat used are either RGBA images or XYZ (4C)
    // point clouds
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type,
                   input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

string getFileName() {
    auto now = chrono::system_clock::now();
    time_t time = chrono::system_clock::to_time_t(now);
    stringstream ss;
    ss << put_time(localtime(&time), "%y%m%d_%H%M%S");
    uint64_t microSec =
        chrono::duration_cast<chrono::microseconds>(now.time_since_epoch())
            .count();
    int micro = microSec % 1000000;
    ss << "_" << setfill('0') << std::setw(6) << micro;

    return "Pic_" + ss.str() + ".jpg";
}
