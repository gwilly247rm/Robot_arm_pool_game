#include "SenseModel.h"

using namespace std;
using namespace cv;
using namespace realsense;

SenseModel::SenseModel() {
    rgb.resize(2);
    depth.resize(2);
}

SenseModel::~SenseModel() { zed.close(); }

int SenseModel::SenseInit(Detector& detect) {
    cout << "-----Sense init-----" << endl;

    // Open Realsense
    thread realsenseThread(&SenseModel::realsenseInit, this);

    // Zed init
    cout << "ZedInit" << endl;
    /*sl::InputType input;
    input.setFromSerialNumber(27596636);
    init_parameters.input = input;*/
    init_parameters.camera_resolution =
        sl::RESOLUTION::HD1080;       // Use HD1080 video mode
    init_parameters.camera_fps = 15;  // Set fps at 15
    init_parameters.depth_mode = sl::DEPTH_MODE::QUALITY;
    init_parameters.coordinate_units = sl::UNIT::METER;
    init_parameters.depth_minimum_distance =
        0.3;  // Set the minimum depth perception distance to 30cm(zed2
              // minimum// limit)
    init_parameters.depth_maximum_distance =
        3;  // Set the maximum depth perception distance to 3m
    init_parameters.sdk_gpu_id = 0;
    init_parameters.sdk_cuda_ctx = (CUcontext)detect.get_cuda_context();

    // Open Zed
    cout << "Open Zed" << endl;
    sl::ERROR_CODE state = zed.open(init_parameters);
    if (state != sl::ERROR_CODE::SUCCESS) {
        cout << "Error " << state << ", exit program." << endl;
        return -1;
    }

    // Set sensing mode in FILL
    runtime_parameters.sensing_mode = sl::SENSING_MODE::FILL;

    realsenseThread.join();

    return 0;
}

void SenseModel::realsenseInit() {
    cout << "Open RealSense" << endl;

    Config config;
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

    StreamConfig infrared_config;
    infrared_config.stream = RS2_STREAM_INFRARED;
    infrared_config.width = 640;
    infrared_config.height = 480;

    config.stream_configs.push_back(color_config);
    config.stream_configs.push_back(depth_config);
    config.stream_configs.push_back(infrared_config);

    rs.connect(config);

    rs.set_color_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);

    if (rs.depth_supports(RS2_OPTION_MIN_DISTANCE))
        rs.set_depth_option(RS2_OPTION_MIN_DISTANCE, 0);

    if (rs.depth_supports(RS2_OPTION_MAX_DISTANCE))
        rs.set_depth_option(RS2_OPTION_MAX_DISTANCE, 400);

    rs.set_align_stream(RS2_STREAM_COLOR);

    rs.enable_hole_filling_filter(true);
    rs.set_hole_filling_filter_option(RS2_OPTION_HOLES_FILL, 1);
}

void SenseModel::run() {
    cout << "-----Sense run-----" << endl;

    // clean
    rgb.clear();
    depth.clear();

    thread realsense(&SenseModel::realsenseProcess, this);
    thread zed(&SenseModel::zedProcess, this);

    realsense.join();
    zed.join();
}

void SenseModel::realsenseProcess() {
    Mat realsenseRgb;
    Mat realsenseDepth;

    rs.update();

    rs.retrieve_color_image(realsenseRgb);
    rs.retrieve_depth_measure(realsenseDepth);
    rs.retrieve_xyz_measure(realsenseXyz);

    rgb[0] = realsenseRgb;
    depth[0] = realsenseDepth;
}

void SenseModel::zedProcess() {
    sl::Mat zedLeftRgb;
    sl::Mat zedLeftDepth;

    Mat zedRgb;
    Mat zedDepth;

    if (zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS) {
        zed.retrieveImage(zedLeftRgb, sl::VIEW::LEFT);
        zed.retrieveMeasure(zedLeftDepth, sl::MEASURE::DEPTH);
        zed.retrieveMeasure(zedLeftXyz, sl::MEASURE::XYZ);
    }

    cvtColor(slMat2cvMat(zedLeftRgb), zedRgb, COLOR_RGBA2RGB);
    cvtColor(slMat2cvMat(zedLeftDepth), zedDepth, COLOR_RGBA2RGB);
    zedXyz = slMat2cvMat(zedLeftXyz);

    rgb[1] = zedRgb;
    depth[1] = zedDepth;
}

void SenseModel::showImgs() {
    cout << "-----Sense show-----" << endl;
    for (int i = 0; i < 2; i++) {
        string rgb_name = "rgbImg" + to_string(i);
        namedWindow(rgb_name, WINDOW_AUTOSIZE);
        imshow(rgb_name, rgb[i]);

         /*string depth_name = "depthImg" + to_string(i);
         namedWindow(depth_name, WINDOW_AUTOSIZE);
         imshow(depth_name, depth[i]);*/
    }
}

void SenseModel::getImgs(SenseData* senseData) {
    senseData->rgb.clear();
    senseData->rgb.resize(2);

    senseData->depth.clear();
    senseData->depth.resize(2);

    senseData->xyz = zedXyz.clone();
    senseData->realMat = realsenseXyz.clone();

    for (size_t i = 0; i < 2; i++) {
        senseData->rgb[i] = rgb[i].clone();
        senseData->depth[i] = depth[i].clone();
    }
}

Mat SenseModel::slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
      }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer
    // from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
  }

