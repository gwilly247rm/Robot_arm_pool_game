#include "track.h"

using namespace std;
using namespace cv;

Track::Track() {
    pidLimit = 0;
}

Track::~Track() {}

void Track::getSenseData(SenseData* senseData) {
    realSenseRgb = senseData->rgb[0];
    realSenseDepth = senseData->depth[0];
    realSenseRealMat = senseData->realMat;
}

int Track::find_white(Detector& detect, const YAML::Node& findNode) {
    cout << "-----find white-----" << endl;
    int iResult;
    objList.clear();
    ballList.clear();
    ballRect.clear();
    cropImages.clear();
    circles.clear();

    iResult = yolo_detect(detect);

    if (iResult == 0)//yolo don't find ball
    {
        iResult = hough_circle_detect(findNode);

        if (iResult == 0)//hough_circle don't find ball
            return iResult;
    }

    cutImage();
    //histogram();
    white_threshold(findNode);
    iResult = find_white_center(findNode);

    return iResult;
}

// yolo detect ball
int Track::yolo_detect(Detector& detect) {
    cout << "-----yolo_detect-----" << endl;
    yoloImage = realSenseRgb.clone();

    objList = detect.detect(realSenseRgb, 0.9);

    for (size_t i = 0; i < objList.size(); i++) {
        if (objList[i].obj_id == 0) {
            DrawBox(yoloImage, "ball", objList[i]);
            ballList.push_back(objList[i]);
        }
    }

    if(ballList.size() == 0)
    {
        cout << "yolo don't find any ball" << endl;

        string fileName = "";
        fileName = getFileName();
        imwrite("./pic/" + fileName, realSenseRgb);
        cout << "Write: " << fileName << endl;

        return 0;
    }

    else
    {
        for (size_t i = 0; i < ballList.size(); i++) {
            if(ballList[i].x+ballList[i].w > realSenseRgb.cols)
                ballList[i].w = ballList[i].w - (ballList[i].x + ballList[i].w - realSenseRgb.cols);

            if(ballList[i].y+ballList[i].h > realSenseRgb.rows)
                ballList[i].h = ballList[i].h - (ballList[i].y + ballList[i].h - realSenseRgb.rows);

            Rect rect(ballList[i].x, ballList[i].y, ballList[i].w, ballList[i].h);

            ballRect.push_back(rect);
        }

        imshow("yolo", yoloImage);

        return 1;
    }
}

// HoughCircles detect ball
int Track::hough_circle_detect(const YAML::Node& findNode) {
    cout << "-----hough_circle_detect-----" << endl;
    Mat gray;
    int x, y, w;

    cvtColor(realSenseRgb, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(9, 9), 2, 2);

    HoughCircles(gray, circles, HOUGH_GRADIENT, 1, findNode["find_white_center"]["min_dist"].as<int>(), findNode["find_white_center"]["canny_edge_detection"].as<int>(), findNode["find_white_center"]["center_detection"].as<int>(), 0, 0);

    if(circles.size() == 0)
    {
        cout << "hough don't find any ball" << endl;
        return 0;
    }

    else
    {
        /// Draw the circles detected
        for (size_t i = 0; i < circles.size(); i++) {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle(realSenseRgb, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            // circle outline
            circle(realSenseRgb, center, radius, Scalar(255, 0, 255), 3, 8, 0);

            x = cvRound(circles[i][0]) - cvRound(circles[i][2]);
            y = cvRound(circles[i][1]) - cvRound(circles[i][2]);
            w = 2 * cvRound(circles[i][2]);

            if(x + w > realSenseRgb.cols)
                w = w - (x + w - realSenseRgb.cols);

            if(y + w > realSenseRgb.rows)
                w = w - (y + w - realSenseRgb.rows);

            Rect rect(x, y, w, w);

            ballRect.push_back(rect);
        }

        imshow("Hough_Circle", realSenseRgb);

        return 1;
    }
}

void Track::cutImage() {
    cout << "-----cut-----" << endl;

    for (size_t i = 0; i < ballRect.size(); i++) {
        Rect rect(ballRect[i].x, ballRect[i].y, ballRect[i].width, ballRect[i].height);
        Mat cropImage = Mat(realSenseRgb, rect);
        cropImages.push_back(cropImage);
        /*std::string imageName = std::to_string(i) + ".jpg ";
        imshow(imageName, cropImage);*/
    }
}

void Track::white_threshold(const YAML::Node& findNode) {
    cout << "-----white_threshold-----" << endl;
    Mat cropImages_gray;
    Mat cropImages_threshold;
    vector<int> white_count;
    int max = 0;
    int j,k;

    white_count.resize(cropImages.size());

    // turn gray and do threshole
    for (size_t i = 0; i < cropImages.size(); i++) {
        cvtColor(cropImages[i], cropImages_gray, COLOR_BGR2GRAY);
        threshold(cropImages_gray, cropImages_threshold, findNode["white_threshold"]["realsense"]["threshold_min"].as<int>(), 255,
                  THRESH_BINARY);

        Mat element = getStructuringElement(MORPH_RECT, Size(findNode["white_threshold"]["realsense"]["kernel_size"].as<int>(), findNode["white_threshold"]["realsense"]["kernel_size"].as<int>()));

        //侵蝕
        erode(cropImages_threshold, cropImages_threshold, element);
        //膨脹
        dilate(cropImages_threshold, cropImages_threshold, element);

        // count white
        for (j = 0; j < cropImages_threshold.rows; j++)
            for (k = 0; k < cropImages_threshold.cols; k++)
                if (cropImages_threshold.at<uchar>(j, k) == 255)
                    white_count[i]++;

        /*std::string Name = "threshold" + std::to_string(i);
        imshow(Name,cropImages_threshold);

        cout << i << "white_count:" << white_count[i] << endl;*/
    }

    if (white_count.size() > 1) {
        for (size_t i = 1; i < white_count.size(); i++)
            if (white_count[i] > white_count[max]) max = i;
    }

    imshow("white ball", cropImages[max]);

    whiteNum = max;
    white = cropImages[max];
}

void Track::histogram() {
    cv::Mat sample;
    cv::Mat sample_c;
    std::vector<cv::Mat> cropImages_hist;
    std::vector<double> hist;

    unsigned int min = 0;
    const int channels[3] = {0, 1, 2};
    const int histSize[3] = {32, 32, 32};
    float hranges[2] = {0, 256};  // range of value in every dim
    const float* histRanges[3] = {hranges, hranges, hranges};

    Point temp;

    sample = imread("sample2.jpg");

    // get sample picture's hist
    calcHist(&sample, 1, channels, Mat(), sample_c, 1, histSize, histRanges,
             true, false);

    // get cut image's hist
    cropImages_hist.resize(cropImages.size());
    for (size_t i = 0; i < cropImages.size(); i++) {
        calcHist(&cropImages[i], 1, channels, Mat(), cropImages_hist[i], 1,
                 histSize, histRanges, true, false);
    }

    // compare Hist
    for (size_t i = 0; i < cropImages_hist.size(); i++) {
        hist.push_back(
            compareHist(cropImages_hist[i], sample_c, CV_COMP_BHATTACHARYYA));
    }

    // get the number of mini hist and mini hist value
    for (size_t i = 0; i < hist.size(); i++) {
        cout << "hhhisttt[" << i << "]:" << hist[i] << endl;
        if (hist[i] < hist[min]) {
            min = i;
        }
    }
    // cout << "don't find white ball" << endl;
    cout << "hist[" << min << "]=" << hist[min] << endl;
    imshow("white ball", cropImages[min]);

    whiteNum = min;

    cropImages_hist.clear();
    hist.clear();
}

int Track::find_white_center(const YAML::Node& findNode)
{
    cout << "-----find_white_center-----" << endl;
    Mat gray;

    cvtColor(white, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(9, 9), 2, 2);

    HoughCircles(gray, circles, HOUGH_GRADIENT, 1, findNode["find_white_center"]["min_dist"].as<int>(), findNode["find_white_center"]["canny_edge_detection"].as<int>(), findNode["find_white_center"]["center_detection"].as<int>(), 0, 0);

    if(circles.size() > 0)
    {
        /// Draw the circles detected
        for (size_t i = 0; i < circles.size(); i++) {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle(white, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            // circle outline
            circle(white, center, radius, Scalar(255, 0, 255), 3, 8, 0);
          }

        imshow("HoughCircle", white);

        whiteCenter.x = ballRect[whiteNum].x + cvRound(circles[0][0]);
        whiteCenter.y = ballRect[whiteNum].y + cvRound(circles[0][1]);

        return 1;
    }

    else
    {
        cout << "Hough don't find any ball" << endl;

        if(ballList.size() > 0)
        {
            whiteCenter.x =  ballRect[whiteNum].x + ballRect[whiteNum].width/2;
            whiteCenter.y =  ballRect[whiteNum].y + ballRect[whiteNum].height/2;

            return 1;
        }
        else
        {
            cout << "no one can find ball" << endl;
            return 0;
        }
    }
}

int Track::pidmove(PlanData* planData, PID* pid, YAML::Node& moveNode) {
    double err_x;
    double err_y;
    double error;
    double* position;
    vector<double> Pos;

    Pos.resize(6);

    cout << "-----pid move-----" << endl;
    pid->init();

    pidLimit++;

    /*err_x = realSenseRealMat.at<Vec3f>(
        ballList[whiteNum].y + (ballList[whiteNum].h / 2),
        ballList[whiteNum].x + (ballList[whiteNum].w / 2))[0] -
    realSenseRealMat.at<Vec3f>((realSenseRgb.rows/2), (realSenseRgb.cols/2))[0];
    err_y = realSenseRealMat.at<Vec3f>(
        ballList[whiteNum].y + (ballList[whiteNum].h / 2),
        ballList[whiteNum].x + (ballList[whiteNum].w / 2))[1] -
    realSenseRealMat.at<Vec3f>((realSenseRgb.rows/2), (realSenseRgb.cols/2))[1];*/

    err_x = realSenseRealMat.at<Vec3f>(whiteCenter.y, whiteCenter.x)[0] - realSenseRealMat.at<Vec3f>((realSenseRgb.rows/2), (realSenseRgb.cols/2))[0];
    err_y = realSenseRealMat.at<Vec3f>(whiteCenter.y, whiteCenter.x)[1] - realSenseRealMat.at<Vec3f>((realSenseRgb.rows/2), (realSenseRgb.cols/2))[1];

    error = sqrt((double)(err_x * err_x) + (double)(err_y * err_y));

    cout << "error" << error << endl;

    if (error < 0.001 || pidLimit > 3) {
        pidLimit = 0;
        return 0;
    }

    position = pid->pidXY(err_x, err_y, 0);

    moveNode["zed"]["move2realsenseX"] = moveNode["zed"]["move2realsenseX"].as<double>() + (position[0] * 1000);
    moveNode["zed"]["move2realsenseY"] = moveNode["zed"]["move2realsenseY"].as<double>() + (-1 * position[1] * 1000);

    Pos = {position[0] * 1000, -1 * position[1] * 1000, 0, 0, 0, 0};

    move(planData, Pos);

    return 1;
}

void Track::final_move(PlanData* planData, const YAML::Node& moveNode, ActData* actData) {
    cout << "-----final run-----" << endl;
    vector<double> Pos;

    Pos.resize(6);

    double depth = realSenseRealMat.at<Vec3f>(whiteCenter.y, whiteCenter.x)[2];

    cout <<  "depth:" << depth << endl;

    if(depth ==  0)
    {
        cout << "depth const" << endl;
        depth = 0.26;
    }

    if(depth > 0.27)
    {
        cout << "depth over" << endl;
        Pos[2] = 0.26;
    }

    Pos[2] = (depth*(-1000)+moveNode["realsense"]["move2cueZ"].as<double>())+30;

    Pos[0] = moveNode["realsense"]["move2cueX"].as<double>();
    Pos[1] = moveNode["realsense"]["move2cueY"].as<double>();
    Pos[5] = planData->finalDeg;

    move(planData, Pos);

    armPos = actData->currentPos;
}

void Track::final_move_no_rotate(PlanData* planData, const YAML::Node& moveNode, ActData* actData) {
    cout << "-----final run-----" << endl;
    vector<double> Pos;

    Pos.resize(6);

    double depth = realSenseRealMat.at<Vec3f>(whiteCenter.y, whiteCenter.x)[2];

    cout <<  "depth:" << depth << endl;

    if(depth ==  0)
    {
        cout << "depth const" << endl;
        depth = 0.26;
    }

    if(depth > 0.27)
    {
        cout << "depth over" << endl;
        Pos[2] = 0.26;
    }

    Pos[2] = (depth*(-1000)+moveNode["realsense"]["move2cueZ"].as<double>())+30;

    Pos[0] = moveNode["realsense"]["move2cueX"].as<double>();
    Pos[1] = moveNode["realsense"]["move2cueY"].as<double>();

    move(planData, Pos);

    armPos = actData->currentPos;
 }

int Track::exception_move(PlanData* planData, const YAML::Node& moveNode, ActData* actData) {
    vector<double> Pos;
    int check = 1;

    Pos.resize(6);

    for(size_t i = 0; i < armPos.size(); i++)
        if (armPos[i] !=  actData->currentPos[i])
            check = 0;

    if(check == 1){
        cout << "-----exception run-----" << endl;

        double depth = realSenseRealMat.at<Vec3f>(whiteCenter.y, whiteCenter.x)[2];

        cout <<  "depth:" << depth << endl;

        if(depth ==  0)
        {
            cout << "depth const" << endl;
            depth = 0.26;
        }

        if(depth > 0.27)
        {
            cout << "depth over" << endl;
            Pos[2] = 0.26;
        }

        Pos[2] = (depth*(-1000)+moveNode["realsense"]["move2cueZ"].as<double>())+30;

        //if(Pos[2]<-46) //change by table high
        //Pos[2] = -46;

        Pos[0] = moveNode["realsense"]["move2cueX"].as<double>();
        Pos[1] = moveNode["realsense"]["move2cueY"].as<double>();

        if(planData->Pos[0].x < planData->Pos[1].x)
           Pos[5] = 40;
        else
           Pos[5] = 140;

        move(planData, Pos);
    }

    return check;;
}

void Track::final_move2(PlanData* planData) {
    cout << "-----final run down-----" << endl;
    vector<double> Pos;
    Pos.resize(6);

    Pos = {0, 0, -30, 0, 0, 0};

    move(planData, Pos);

    planData->action.cuer.ON = 1;
}

void Track::move(PlanData* planData, std::vector<double> pos) {
    planData->action.act.coordType = HiwinSDK::CoordType::Coord;
    planData->action.act.moveType = HiwinSDK::MoveType::Relative;
    planData->action.act.ctrlType = HiwinSDK::CtrlType::PTP;
    planData->action.act.value = {pos[0], pos[1], pos[2],
                                  pos[3], pos[4], pos[5]};
}

void Track::DrawBox(Mat img, String name, bbox_t obj) {
    putText(img, name + " : " + to_string((int)(obj.prob * 100)) + "%",
            Point(obj.x, obj.y), 0, 0.5, Scalar(255, 0, 0), 2);
    rectangle(img, Point(obj.x, obj.y), Point(obj.x + obj.w, obj.y + obj.h),
              Scalar(255, 0, 0), 1);
}

string Track::getFileName() {
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
