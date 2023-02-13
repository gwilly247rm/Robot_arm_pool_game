#include "find.h"

#include <chrono>
#include <thread>

using namespace std;
using namespace cv;

Find::Find() {
    pos.resize(7);
    iResult1 = 0;
    iResult2 = 0;
}

Find::~Find() {}

void Find::getRgbImgs(SenseData* senseData) {
    realSenseRgb = senseData->rgb[0].clone();
    zedRgb = senseData->rgb[1].clone();
    zedSmallRgb = resizeImage(zedRgb);
}

void Find::getDepthImgs(SenseData* senseData) {
    realSenseDepth = senseData->depth[0].clone();
    zedDepth = senseData->depth[1].clone();
}

int Find::run(Detector& detect, PlanData* planData, const YAML::Node& findNode)
{
    cout << "-----find run-----" << endl;
    if (pos.size() > 7)
        pos.erase(pos.begin() + 7, pos.begin() + pos.size());

    yolo_detect(&detect);

    iResult1 = find_hole();
    iResult2 = find_ball(planData, findNode) + iResult2;

    imshow("yolo", zedSmallRgb);

    if (iResult1 == 1 && iResult2 >= 5)
    {
        save_ball_pos();
        rotate();

        planData->Pos.assign(pos.begin(), pos.end());

        iResult2 = 0;
        whiteList.clear();

        return 1;
    }
    else
        return 0;
}

// yolo detect everything
void Find::yolo_detect(Detector* detect) {
    cout << "-----yolo_detect-----" << endl;
    objList.clear();
    ballList.clear();
    holeList.clear();

    objList = detect->detect(zedRgb, 0.8);

    std::vector<bbox_t> temp_objList;
    temp_objList.assign(objList.begin(), objList.end());

    for (size_t i = 0; i < objList.size(); i++) {
        if (objList[i].obj_id == 0) {
            temp_objList[i].x = temp_objList[i].x * 0.5;
            temp_objList[i].y = temp_objList[i].y * 0.5;
            temp_objList[i].w = temp_objList[i].w * 0.5;
            temp_objList[i].h = temp_objList[i].h * 0.5;

            if(objList[i].w < 75 || objList[i].h < 75)
            {
                DrawBox(zedSmallRgb, "ball", temp_objList[i]);
                ballList.push_back(objList[i]);
            }
        }
        else if (objList[i].obj_id == 1) {
            temp_objList[i].x = temp_objList[i].x * 0.5;
            temp_objList[i].y = temp_objList[i].y * 0.5;
            temp_objList[i].w = temp_objList[i].w * 0.5;
            temp_objList[i].h = temp_objList[i].h * 0.5;

            DrawBox(zedSmallRgb, "hole", temp_objList[i]);
            holeList.push_back(objList[i]);
        }
    }
}

int Find::find_degree(PlanData* planData, const YAML::Node& findNode) {
    cout << "-----find_degree-----" << endl;

    vector<cv::Vec2f> lines;
    lines.clear();

    drawLine = realSenseRgb.clone();
    cvtColor(drawLine, gray, COLOR_BGR2GRAY);
    Canny(gray, canny, 70, findNode["find_degree"]["canny"].as<int>());
    HoughLines(canny, lines, 1, CV_PI / 180, findNode["find_degree"]["hough"].as<int>());
    drawLines(drawLine, lines);
    degree = calcDeg(lines);
    degree = 90 - degree;

    if(degree >= 90)
        degree = degree - 180;

    cout << "degree:" << degree << endl;
    imshow("degree", drawLine);

    waitKey(1);

    if (degree > 90 || degree < -90 || abs(degree) > abs(findNode["find_degree"]["degreeLimit"].as<double>())) {
        cout << "degree error" << endl;
        return 0;
    }

    planData->degree = degree;

    return 1;
}

int Find::find_hole() {
    cout << "-----find_hole-----" << endl;

    if (holeList.size() == 0) {
        cout << "don't find any hole" << endl;
        return 0;
    }

    cout << "yolo find " << holeList.size() << " hole" << endl;

    vector<Point> temp_pos;
    Point temp;
    float rad;
    float w;
    float h;
    vector<int> x1;
    vector<int> x2;
    int min = 0;

    for (size_t i = 0; i < holeList.size(); i++) {
        temp.x = holeList[i].x + (holeList[i].w / 2);
        temp.y = holeList[i].y + (holeList[i].h / 2);
        temp_pos.push_back(temp);
    }
    //degree = 0;
    if (degree > 90) {
        rad = ((180 - degree) * CV_PI) / 180;
        w = abs(600 * cos(rad));//origin:590
        h = 600 * sin(rad);
    } else {
        rad = (degree * CV_PI) / 180;
        w = abs(600 * cos(rad));
        h = -(600 * sin(rad));
    }

    if (temp_pos.size() > 1) {
        // find same x
        for (size_t i = 0; i < temp_pos.size() - 1; i++) {
            for (size_t j = i + 1; j < temp_pos.size(); j++) {
                if (abs(temp_pos[i].x - temp_pos[j].x) < 100) {
                    x1.push_back(i);
                    x2.push_back(j);
                }
            }
        }

        if (x1.size() > 0) {
            if (x1.size() > 1) {
                for (size_t i = 1; i < x1.size(); i++) {
                    if (abs(x1[i] - 540) < abs(x1[min] - 540)) {
                        min = i;
                    }
                }
            }
            if (temp_pos[x1[min]].y < temp_pos[x2[min]].y) {
                pos[1].x = temp_pos[x1[min]].x;
                pos[1].y = temp_pos[x1[min]].y;
                pos[2].x = temp_pos[x2[min]].x;
                pos[2].y = temp_pos[x2[min]].y;
            }
            else {
                pos[1].x = temp_pos[x2[min]].x;
                pos[1].y = temp_pos[x2[min]].y;
                pos[2].x = temp_pos[x1[min]].x;
                pos[2].y = temp_pos[x1[min]].y;
            }

            pos[3].x = pos[1].x - w;
            pos[3].y = pos[1].y - h;
            pos[4].x = pos[2].x - w;
            pos[4].y = pos[2].y - h;

            pos[5].x = pos[1].x + w;
            pos[5].y = pos[1].y + h;
            pos[6].x = pos[2].x + w;
            pos[6].y = pos[2].y + h;
        }
    }

    else
        cout << "don't find two hole" << endl;

    return 1;
}

int Find::find_ball(PlanData* planData, const YAML::Node& findNode) {
    cout << "-----find ball-----" << endl;

    int iResult;
    ballR = 0;

    if (ballList.size() == 0) {
        cout << "don't find any ball" << endl;
        return 0;
    }
    else
        cout << "yolo find " << ballList.size() << " balls" << endl;

    /*for (size_t i = 0; i < ballList.size(); i++)
        ballR = ballList[i].w + ballList[i].h + ballR;*/

    for (size_t i = 0; i < ballList.size(); i++)
        ballR = ballList[i].h + ballR;// min

    ballR = ballR / ballList.size();
    cout << "ballR:" << ballR << endl;
    planData->ballR = ballR;

    cutImage();
    //iResult = histogram();
    iResult = white_threshold(findNode);

    return iResult;
}

void Find::drawLines(cv::Mat input, const std::vector<cv::Vec2f>& lines) {
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

double Find::calcDeg(const std::vector<cv::Vec2f>& lines) {
    double ret = -999;
    double s = 0, c = 0;
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

void Find::cutImage() {
    cout << "-----cut-----" << endl;
    cropImages.clear();

    for (size_t i = 0; i < ballList.size(); i++) {
        Rect rect(ballList[i].x, ballList[i].y, ballList[i].w,
                  ballList[i].h);
        Mat cropImage = Mat(zedRgb, rect);
        cropImages.push_back(cropImage);
        /*std::string imageName = std::to_string(i) + ".jpg ";
         *        imwrite(imageName, cropImage);
         *        imshow(imageName, cropImage);*/
    }
}

int Find::histogram() {
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

    int check;
    unsigned int shift = 10;

    sample = imread("sample1.jpg");

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
        //cout << "hhhisttt[" << i << "]:" << hist[i] << endl;
        if (hist[i] < hist[min]) {
            min = i;
        }
    }

    if (hist[min] < 0.6) {
        // cout << "don't find white ball" << endl;
        cout << "hist[" << min << "]=" << hist[min] << endl;
        imshow("white ball", cropImages[min]);

        pos[0].x = ballList[min].x + (ballList[min].w / 2);
        pos[0].y = ballList[min].y + (ballList[min].h / 2);

        for (size_t i = 0; i < ballList.size(); i++) {
            check = 1;
            if (i != min) {
                if (degree <= 90 && degree >= 0) {
                    if ((ballList[i].x + ballList[i].w / 2) <
                            pos[3].x - shift ||
                        (ballList[i].y + ballList[i].h / 2) >
                            pos[4].y + shift ||
                        (ballList[i].x + ballList[i].w / 2) >
                            pos[6].x + shift ||
                        (ballList[i].y + ballList[i].h / 2) < pos[5].y - shift)
                        check = 0;
                } else if (degree > 90 && degree <= 180) {
                    if ((ballList[i].x + ballList[i].w / 2) <
                            pos[4].x - shift ||
                        (ballList[i].y + ballList[i].h / 2) >
                            pos[6].y + shift ||
                        (ballList[i].x + ballList[i].w / 2) >
                            pos[5].x + shift ||
                        (ballList[i].y + ballList[i].h / 2) < pos[3].y - shift)
                        check = 0;
                }

                if (check == 1) {
                    temp.x = (ballList[i].x + (ballList[i].w / 2));
                    temp.y = (ballList[i].y + (ballList[i].h / 2));
                    pos.push_back(temp);
                }
            }
        }
    }

    else {
        cout << "don't find any white" << endl;
        pos[0] = Point(0, 0);
        return 0;
    }

    cropImages.clear();
    cropImages_hist.clear();
    hist.clear();

    return 1;
}

int Find::white_threshold(const YAML::Node& findNode) {
    cout << "-----threshold-----" << endl;
    Mat cropImages_gray;
    Mat cropImages_threshold;
    vector<int> white_count;
    unsigned int max = 0;
    int j,k;
    Point white;

    white_count.resize(cropImages.size());

    // turn gray and do threshole
    for (size_t i = 0; i < cropImages.size(); i++) {
        cvtColor(cropImages[i], cropImages_gray, COLOR_BGR2GRAY);
        threshold(cropImages_gray, cropImages_threshold, findNode["white_threshold"]["zed"]["threshold_min"].as<int>(), 255,
                  THRESH_BINARY);

        Mat element = getStructuringElement(MORPH_RECT, Size(findNode["white_threshold"]["zed"]["kernel_size"].as<int>(), findNode["white_threshold"]["zed"]["kernel_size"].as<int>()));
        //侵蝕
        erode(cropImages_threshold, cropImages_threshold, element);
        //膨脹
        dilate(cropImages_threshold, cropImages_threshold, element);

        // count white
        for (j = 0; j < cropImages_threshold.rows; j++)
        {
            for (k = 0; k < cropImages_threshold.cols; k++)
            {
                if (cropImages_threshold.at<uchar>(j, k) == 255)
                {
                    white_count[i]++;
                }
            }
        }

        /*std::string Name = "threshold" + std::to_string(i);
        imshow(Name,cropImages_threshold);*/
    }

    /*for (size_t i = 0; i < white_count.size(); i++)
        cout << "white_count[" << i << "]:" << white_count[i] << endl;*/

    if (white_count.size() > 1)
    {
        for (size_t i = 1; i < white_count.size(); i++)
        {
            if (white_count[i] > white_count[max]) max = i;
        }
    }

    //if (white_count[max] > 1200 && white_count[max] < 3000) {
        cout << "white_count_max[" << max << "]=" << white_count[max] << endl;
        imshow("white", cropImages[max]);

        white.x = ballList[max].x + (ballList[max].w / 2);
        white.y = ballList[max].y + (ballList[max].h / 2);

        whiteList.push_back(white);
//}

    /*else {
        cout << "don't find any white" << endl;
        pos[0] = Point(0, 0);
        return 0;
    }*/

    return 1;
}

void Find::save_ball_pos()
{
    unsigned int shift = 10;
    Point temp;
    Point whiteCenter = Point (0, 0);
    vector<int> whiteNum;
    vector<int> count;
    double dist;
    int max = 0;
    int check;

    whiteNum.resize(whiteList.size());
    count.resize(whiteNum.size());

    for (size_t i = 0; i < whiteNum.size(); i++)
        whiteNum[i] = i;

    for (size_t i = 0; i < (whiteList.size() - 1); i++) {
        for(size_t j = i+1; j < whiteList.size(); j++) {
            dist = sqrt(pow((whiteList[i].x - whiteList[j].x), 2) +
            pow((whiteList[i].y - whiteList[j].y), 2));

            if(dist < ballR/2)
                whiteNum[j] = whiteNum[i];
        }
    }

    for (size_t i = 0; i < whiteNum.size(); i++)
    {
        count[whiteNum[i]]++;
    }

    for (size_t i = 0; i < count.size(); i++)
    {
        if(count[i] > count[max])
            max = i;
    }

    for (size_t i = 0; i < whiteNum.size(); i++)
    {
        if(whiteNum[i] == max)
            whiteCenter = whiteCenter + whiteList[i];
    }

    whiteCenter.x = cvRound(whiteCenter.x / count[max]);
    whiteCenter.y = cvRound(whiteCenter.y / count[max]);

    pos[0].x = whiteCenter.x;
    pos[0].y = whiteCenter.y;

    /*pos[0].x = whiteList[whiteNum[max]].x;
    pos[0].y = whiteList[whiteNum[max]].y;*/

    for (size_t i = 0; i < ballList.size(); i++) {
        check = 1;

        temp.x = (ballList[i].x + (ballList[i].w / 2));
        temp.y = (ballList[i].y + (ballList[i].h / 2));

        dist = sqrt(pow((temp.x - pos[0].x), 2) + pow((temp.y - pos[0].y), 2));

        if (dist > ballR/2) {
            if (degree <= 90 && degree >= 0) {
                if ((ballList[i].x + ballList[i].w / 2) <
                    pos[3].x - shift ||
                    (ballList[i].y + ballList[i].h / 2) >
                    pos[4].y + shift ||
                    (ballList[i].x + ballList[i].w / 2) >
                    pos[6].x + shift ||
                    (ballList[i].y + ballList[i].h / 2) < pos[5].y - shift)
                    check = 0;
            } else if (degree > 90 && degree <= 180) {
                if ((ballList[i].x + ballList[i].w / 2) <
                    pos[4].x - shift ||
                    (ballList[i].y + ballList[i].h / 2) >
                    pos[6].y + shift ||
                    (ballList[i].x + ballList[i].w / 2) >
                    pos[5].x + shift ||
                    (ballList[i].y + ballList[i].h / 2) < pos[3].y - shift)
                    check = 0;
            }

            if (check == 1) {
                pos.push_back(temp);
            }
        }
    }

    whiteNum.clear();
    count.clear();
}

void Find::rotate() {
    cout << "-----rotate-----" << endl;

    Point2f center;
    Mat temp =  Mat(3, 1, CV_64F);
    Mat temp2 =  Mat(2, 1, CV_64F);

    center.x = (pos[1].x + pos[2].x)/2;
    center.y = (pos[1].y + pos[2].y)/2;

    Mat rot_mat = cv::getRotationMatrix2D(center, -degree, 1.0);

    for(size_t i = 0; i < pos.size(); i++)
    {
        temp.at<double>(0) = (double)pos[i].x;
        temp.at<double>(1) = (double)pos[i].y;
        temp.at<double>(2) = 1;

        temp2 = rot_mat * temp;

        pos[i].x = (int)temp2.at<double>(0);
        pos[i].y = (int)temp2.at<double>(1);
    }
}

void Find::DrawBox(Mat img, String name, bbox_t obj) {
    putText(img, name + " : " + to_string((int)(obj.prob * 100)) + "%",
            Point(obj.x, obj.y), 0, 0.5, Scalar(255, 0, 0), 2);
    rectangle(img, Point(obj.x, obj.y), Point(obj.x + obj.w, obj.y + obj.h),
              Scalar(255, 0, 0), 1);
}

void Find::onMouse(int event, int x, int y, int flags, void* ptr) {
    Point* p = (Point*)ptr;

    if (event == EVENT_LBUTTONDOWN) *p = Point(x, y);
}

Mat Find::resizeImage(Mat img) {
    cv::Size displaySize(960, 540);
    cv::Mat resizeImage(displaySize, CV_8UC4);
    cv::resize(img, resizeImage, displaySize);

    return resizeImage;
}
