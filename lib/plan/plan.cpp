#include "plan.h"
#include <climits>

using namespace std;
using namespace cv;

Plan::Plan() {}

Plan::~Plan() {}

void Plan::getPos(PlanData* planData) {
    // get ballR
    ballR = planData->ballR;

    // get pos from find
    plan_pos.assign(planData->Pos.begin(), planData->Pos.end());

    for (size_t i = 0; i < plan_pos.size(); i++)
        cout << "plan_pos[" << i << "]" << plan_pos[i] << endl;
    // waitKey(0);
}

int Plan::run(SenseData* senseData, PlanData* planData,
              const YAML::Node& moveNode) {
    predict_point = Point(0, 0);
    if (noBall() == 1) {
        cout << "---noBall---" << endl;
        return 0;
    } else if (manyBall(planData) == 1) {
        cout << "---manyBall---" << endl;
    } else if (easyBall(planData) == 1) {
        cout << "---easyBall---" << endl;
    } else if (kissBall(planData) == 1) {
        cout << "---kissBall---" << endl;
    } else {
        randomBall(planData);
        cout << "---randomBall---" << endl;
    }
    draw();
    calcDeg(senseData, planData);
    move(planData, moveNode);

    return 0;
}

void Plan::easyBall_run(SenseData* senseData, PlanData* planData,
                        const YAML::Node& moveNode) {
    /*if (easyBall(planData) == 1) {
        cout << "---easyBall---" << endl;
        draw();
    } */

    /*if (kissBall(planData) == 1) {
        cout << "---kissBall---" << endl;
        draw();
    }*/

    /*else {
        cout << "can't hit!" << endl;
        predict_point = Point(0, 0);
        draw();
    }*/

    cout << "---randomBall---" << endl;
    randomBall(planData);

    draw();
    calcDeg(senseData, planData);
    move(planData, moveNode);
}

// calc cuePos && rotate
void Plan::calcDeg(SenseData* senseData, PlanData* planData) {
    whiteXYZPos = pixel2xyzPos(senseData, plan_pos[0]);
    predictXYZPos = pixel2xyzPos(senseData, predict_point);

    cout << "whiteXYZPos:" << whiteXYZPos << endl;

    double distWhite2Final = sqrt(pow(whiteXYZPos.y - predictXYZPos.y, 2) +
                                  pow(whiteXYZPos.x - predictXYZPos.x, 2));

    Point2d temp1;
    Point2d temp2 = Point(1, 0);
    double temp;
    double white_finalDeg;

    temp1.x = predictXYZPos.x - whiteXYZPos.x;
    temp1.y = predictXYZPos.y - whiteXYZPos.y;

    temp = temp1.x * temp2.x + temp1.y * temp2.y;
    // temp = dot(temp1, temp2);

    white_finalDeg = acos(temp / distWhite2Final) * 180 / CV_PI;

    cout << "plan calculate degree:" << white_finalDeg << endl;

    // planData->degree;
    if (whiteXYZPos.y > predictXYZPos.y)
        planData->finalDeg = white_finalDeg;
    else
        planData->finalDeg = -(white_finalDeg);
}

Point2f Plan::pixel2xyzPos(SenseData* senseData, Point pixel) {
    cv::Point2f xyzPos;

    if (pixel.x >= 0 && pixel.x < senseData->xyz.cols && pixel.y >= 0 &&
        pixel.y < senseData->xyz.rows) {
        xyzPos.x = senseData->xyz.at<cv::Vec4f>(pixel.y, pixel.x)[0];
        xyzPos.y = senseData->xyz.at<cv::Vec4f>(pixel.y, pixel.x)[1];
    }

    else
        cout << "xyzPos value out of image_size" << endl;

    // cout << "xyzPos" << xyzPos << endl;

    return xyzPos;
}

void Plan::move(PlanData* planData, const YAML::Node& moveNode) {
    double x = whiteXYZPos.x * 1000 +
               moveNode["zed"]["move2realsenseX"]
                   .as<double>();  // zed & realsense RelPosition
    double y = whiteXYZPos.y * (-1000) +
               moveNode["zed"]["move2realsenseY"]
                   .as<double>();  // zed & realsense RelPosition

    planData->action.act.coordType = HiwinSDK::CoordType::Coord;
    planData->action.act.moveType = HiwinSDK::MoveType::Relative;
    planData->action.act.ctrlType = HiwinSDK::CtrlType::PTP;
    planData->action.act.value = {x, y, -270, 0, 0, 0};
}

int Plan::noBall() {
    // cout << "plan_pos_size:" << plan_pos.size() << endl;

    if (plan_pos.size() < 8)
        return 1;
    else
        return 0;
}

int Plan::manyBall(PlanData* planData) {
    double dist;
    int max = 0;
    int countNum = 0;
    int save;
    vector<Point> temp_plan_pos;
    vector<int> mark;
    vector<int> count;

    predict_point = Point(0, 0);

    // vector clean
    mark.clear();
    count.clear();

    for (size_t i = 7; i < plan_pos.size(); i++)
        temp_plan_pos.push_back(plan_pos[i]);

    mark.resize(temp_plan_pos.size());
    count.resize(temp_plan_pos.size());

    for (size_t i = 0; i < mark.size(); i++) mark[i] = -1;

    for (size_t i = 0; i < count.size(); i++) count[i] = 0;

    // find ball group
    for (size_t i = 0; i < temp_plan_pos.size(); i++) {
        for (size_t j = i + 1; j < temp_plan_pos.size(); j++) {
            dist = sqrt(pow((temp_plan_pos[i].x - temp_plan_pos[j].x), 2) +
                        pow((temp_plan_pos[i].y - temp_plan_pos[j].y), 2));

            if (dist <= (ballR * 1.2)) {
                if (mark.at(j) == -1) {
                    if (mark.at(i) == -1) {
                        mark[i] = i;
                    }
                    mark[j] = mark[i];
                }

                else if (mark.at(j) != -1 && mark.at(i) == -1)
                    mark[i] = mark[j];

                else if (mark.at(j) != -1 && mark.at(i) != -1 &&
                         mark.at(j) != mark.at(i)) {
                    if (mark.at(j) < mark.at(i)) {
                        save = mark[i];
                        for (size_t k = 0; k < mark.size(); k++) {
                            if (mark.at(k) == save) {
                                mark[k] = mark[j];
                            }
                        }
                    } else {
                        save = mark[j];
                        for (size_t k = 0; k < mark.size(); k++) {
                            if (mark.at(k) == save) {
                                mark[k] = mark[i];
                            }
                        }
                    }
                }
            }
        }
    }

    for (size_t k = 0; k < mark.size(); k++)
        if (mark.at(k) == -1) mark[k] = k;

    for (size_t i = 0; i < mark.size(); i++) count[mark[i]]++;

    for (size_t i = 1; i < count.size(); i++)
        if (count[i] > count[max]) max = i;

    cout << "there has " << count[max] << " number of ball is a group" << endl;

    // if more than 6->it's a ball group
    if (count[max] > 6) {
        for (size_t i = 0; i < mark.size(); i++) {
            if (mark[i] == max) {
                predict_point.x = predict_point.x + temp_plan_pos[i].x;
                predict_point.y = predict_point.y + temp_plan_pos[i].y;
                countNum++;
            }
        }

        predict_point.x = predict_point.x / countNum;
        predict_point.y = predict_point.y / countNum;

        cout << "predict_point:" << predict_point.x << "," << predict_point.y
             << endl;

        planData->action.cuer.air = 5;

        return 1;
    }

    else
        return 0;
}

int Plan::easyBall(PlanData* planData) {
    int min = 0;
    int max = 0;
    int flag = 0;
    int count1;
    int count2;
    int check;
    int shift = 15;  // table edge
    int iResult;
    int ball, hole;
    float a1, b1, a2, b2;
    float vertical_x;
    float dist;
    float dist1, dist2;
    float o2p;
    vector<int> holeNum1;
    vector<int> holeNum2;
    vector<int> filter1;
    vector<int> filter2;
    vector<int> predict_hole;
    vector<double> temp1;
    vector<double> temp2;
    vector<double> temp_dist;
    vector<double> toCompare1;
    vector<double> toCompare2;
    vector<Point2f> predict;
    vector<Point2f> temp_predict1;
    vector<Point2f> temp_predict2;
    Point2f every_predict_point;
    Point2f wt;
    Point2f th;
    double wtLength;
    double thLength;
    double shift_x;
    double shift_y;

    // clear vector
    holeNum1.clear();
    holeNum2.clear();
    filter1.clear();
    filter2.clear();
    temp_dist.clear();
    temp1.clear();
    temp2.clear();
    toCompare1.clear();
    toCompare2.clear();
    predict.clear();
    predict_hole.clear();
    temp_predict1.clear();
    temp_predict2.clear();

    // targetball2every hole obstacle
    for (size_t i = 7; i < plan_pos.size(); i++) {
        for (size_t h = 1; h < 7; h++) {
            count1 = 0;
            // target ball and every hole linear function
            a1 = (plan_pos[i].y - plan_pos[h].y) /
                 (plan_pos[i].x - plan_pos[h].x);
            b1 = plan_pos[i].y - (a1 * plan_pos[i].x);

            // calc dot-line distance && dot-dot distance
            for (size_t j = 7; j < plan_pos.size(); j++) {
                if (j != i) {
                    if (plan_pos[i].x == plan_pos[h].x) {
                        dist1 = abs(plan_pos[j].x - plan_pos[i].x);
                    } else {
                        dist1 = abs(a1 * plan_pos[j].x - plan_pos[j].y + b1) /
                                sqrt(1 + a1 * a1);
                    }

                    if (dist1 < ballR) {  // Exception1
                        if (plan_pos[i].x == plan_pos[h].x) {
                            if (plan_pos[h].y > plan_pos[i].y) {
                                if (plan_pos[j].y < plan_pos[h].y &&
                                    plan_pos[j].y > plan_pos[i].y)
                                    count1++;
                            } else {
                                if (plan_pos[j].y > plan_pos[h].y &&
                                    plan_pos[j].y < plan_pos[i].y)
                                    count1++;
                            }
                        } else if (plan_pos[i].y == plan_pos[h].y) {
                            if (plan_pos[h].x > plan_pos[i].x) {
                                if (plan_pos[j].x < plan_pos[h].x &&
                                    plan_pos[j].x > plan_pos[i].x)
                                    count1++;
                            } else {
                                if (plan_pos[j].x > plan_pos[h].x &&
                                    plan_pos[j].x < plan_pos[i].x)
                                    count1++;
                            }
                        } else {
                            vertical_x = (b1 - (plan_pos[j].y +
                                                (1 / a1) * plan_pos[j].x)) /
                                         ((-1 / a1) - a1);
                            if (plan_pos[i].x < plan_pos[h].x) {
                                if (vertical_x < plan_pos[h].x &&
                                    vertical_x > plan_pos[i].x)
                                    count1++;
                            } else {
                                if (vertical_x > plan_pos[h].x &&
                                    vertical_x < plan_pos[i].x)
                                    count1++;
                            }
                        }
                    }
                }
            }
            if (count1 == 0) {
                filter1.push_back(h);  // filter1: save can hit hole
            }
        }  // second for

        /*for (size_t j = 0; j < filter1.size(); j++) {
            cout<<"filter1:"<<filter1[j]<<endl;
        }*/

        if (filter1.size() != 0) {
            // find targetball_canHitBall predict point
            // tagetball && filter hole vector to find predict point
            for (size_t k = 0; k < filter1.size(); k++) {
                check = 1;
                double distBall2Hole =
                    sqrt(pow(plan_pos[i].x - plan_pos[filter1[k]].x, 2) +
                         pow(plan_pos[i].y - plan_pos[filter1[k]].y, 2));

                shift_x = ballR * abs(plan_pos[i].x - plan_pos[filter1[k]].x) /
                          distBall2Hole;
                shift_y = ballR * abs(plan_pos[i].y - plan_pos[filter1[k]].y) /
                          distBall2Hole;

                if (plan_pos[filter1[k]].x < plan_pos[i].x &&
                    plan_pos[filter1[k]].y < plan_pos[i].y) {
                    every_predict_point.x = plan_pos[i].x + shift_x;
                    every_predict_point.y = plan_pos[i].y + shift_y;
                } else if (plan_pos[filter1[k]].x < plan_pos[i].x &&
                           plan_pos[filter1[k]].y > plan_pos[i].y) {
                    every_predict_point.x = plan_pos[i].x + shift_x;
                    every_predict_point.y = plan_pos[i].y - shift_y;
                } else if (plan_pos[filter1[k]].x > plan_pos[i].x &&
                           plan_pos[filter1[k]].y < plan_pos[i].y) {
                    every_predict_point.x = plan_pos[i].x - shift_x;
                    every_predict_point.y = plan_pos[i].y + shift_y;
                } else {
                    every_predict_point.x = plan_pos[i].x - shift_x;
                    every_predict_point.y = plan_pos[i].y - shift_y;
                }

                if (planData->degree <= 90 && planData->degree >= 0) {
                    if (every_predict_point.x < plan_pos[3].x + shift ||
                        every_predict_point.y > plan_pos[4].y - shift ||
                        every_predict_point.x > plan_pos[6].x - shift ||
                        every_predict_point.y < plan_pos[5].y + shift)
                        check = 0;
                } else if (planData->degree > 90 && planData->degree <= 180) {
                    if (every_predict_point.x < plan_pos[4].x + shift ||
                        every_predict_point.y > plan_pos[6].y - shift ||
                        every_predict_point.x > plan_pos[5].x - shift ||
                        every_predict_point.y < plan_pos[3].y + shift)
                        check = 0;
                }
                if (check == 1) {
                    predict.push_back(every_predict_point);
                    predict_hole.push_back(filter1[k]);
                }
            }

            // filter predict: whiteball2predict obstacle
            // whiteBall and its predict_point linear function
            for (size_t k = 0; k < predict.size(); k++) {
                count2 = 0;
                a2 = (plan_pos[0].y - predict[k].y) /
                     (plan_pos[0].x - predict[k].x);
                b2 = plan_pos[0].y - (a2 * plan_pos[0].x);
                // calc dot-line distance && dot-dot distance
                for (size_t j = 7; j < plan_pos.size(); j++) {
                    if (plan_pos[0].x == predict[k].x) {
                        dist2 = abs(plan_pos[j].x - plan_pos[0].x);
                    } else {
                        dist2 = abs(a2 * plan_pos[j].x - plan_pos[j].y + b2) /
                                sqrt(1 + a2 * a2);
                    }

                    o2p = sqrt(pow(plan_pos[j].x - predict[k].x, 2) +
                               pow(plan_pos[j].y - predict[k].y, 2));
                    if (dist2 < ballR) {
                        if (plan_pos[0].x == predict[k].x) {
                            if (predict[k].y > plan_pos[0].y) {
                                if (plan_pos[j].y < predict[k].y &&
                                    plan_pos[j].y > plan_pos[0].y)
                                    count2++;
                            } else {
                                if (plan_pos[j].y > predict[k].y &&
                                    plan_pos[j].y < plan_pos[0].y)
                                    count2++;
                            }
                        } else if (plan_pos[0].y == predict[k].y) {
                            if (predict[k].x > plan_pos[i].x) {
                                if (plan_pos[j].x < predict[k].x &&
                                    plan_pos[j].x > plan_pos[0].x)
                                    count2++;
                            } else {
                                if (plan_pos[j].x > predict[k].x &&
                                    plan_pos[j].x < plan_pos[0].x)
                                    count2++;
                            }
                        } else {
                            vertical_x = (b2 - (plan_pos[j].y +
                                                (1 / a2) * plan_pos[j].x)) /
                                         ((-1 / a2) - a2);
                            if (plan_pos[0].x < predict[k].x) {
                                if (vertical_x < predict[k].x &&
                                    vertical_x > plan_pos[0].x)
                                    count2++;
                            } else {
                                if (vertical_x > predict[k].x &&
                                    vertical_x < plan_pos[0].x)
                                    count2++;
                            }
                        }
                    }
                    if (o2p <= ballR - 1) {
                        count2++;
                    }
                }
                if (count2 == 0) {
                    filter2.push_back(
                        predict_hole[k]);  // filter2:save can hit hole
                }
            }

            if (filter2.size() != 0) {
                // whether dist of ball2hole < ballR or not
                temp_dist.resize(filter2.size());
                for (size_t k = 0; k < filter2.size(); k++) {
                    temp_dist[k] =
                        sqrt(pow(plan_pos[i].x - plan_pos[filter2[k]].x, 2) +
                             pow(plan_pos[i].y - plan_pos[filter2[k]].y, 2));
                }

                for (size_t k = 0; k < temp_dist.size(); k++) {
                    if (temp_dist[k] < 1.5 * ballR) flag++;
                }

                // version1 : <ballR
                // find targetBall to its bestHole least dist
                temp1.resize(filter2.size());
                for (size_t k = 0; k < filter2.size(); k++) {
                    temp1[k] =
                        sqrt(pow(plan_pos[i].x - plan_pos[filter2[k]].x, 2) +
                             pow(plan_pos[i].y - plan_pos[filter2[k]].y, 2));
                    if (temp1[k] < temp1[min]) min = k;
                }
                //cout << "min:" << min << endl;
                toCompare1.push_back(
                    temp1[min]);  // save every can hit ball the least dist
                holeNum1.push_back(filter2[min]);  // save every can hit hole

                for (size_t k = 0; k < predict_hole.size(); k++) {
                    if (filter2[min] == predict_hole[k])
                        temp_predict1.push_back(predict[k]);
                }
               /* cout << i << " ball can hit " << filter2[min] << " hole"
                     << endl;*/

                min = 0;

                // version2
                // find targetBall to its bestHole least dot
                // (whiteball targetball)   dot   (targetball  filter_holes)
                temp2.resize(filter2.size());
                wt.x = plan_pos[i].x - plan_pos[0].x;
                wt.y = plan_pos[i].y - plan_pos[0].y;

                wtLength = sqrt(pow(wt.x, 2) + pow(wt.y, 2));

                for (size_t k = 0; k < filter2.size(); k++) {
                    th.x = plan_pos[filter2[k]].x - plan_pos[i].x;
                    th.y = plan_pos[filter2[k]].y - plan_pos[i].y;
                    thLength = sqrt(pow(th.x, 2) + pow(th.y, 2));
                    temp2[k] = dot(wt, th) / (wtLength * thLength);
                    if (temp2[k] > temp2[max]) max = k;
                }

                toCompare2.push_back(
                    temp2[max]);  // save every can hit ball the least dot
                holeNum2.push_back(filter2[max]);  // save every can hit hole

                for (size_t k = 0; k < predict_hole.size(); k++) {
                    if (filter2[max] == predict_hole[k])
                        temp_predict2.push_back(predict[k]);
                }
                // cout << i << " ball can hit " << filter2[min] << " hole" <<
                // endl;
                max = 0;
            }
        }

        temp_dist.clear();
        temp1.clear();
        temp2.clear();
        filter1.clear();
        filter2.clear();
        predict.clear();
        predict_hole.clear();
    }

    if (flag > 0) {
        if (holeNum1.size() != 0) {
            cout << "easy ball:" << plan_pos[min + 7] << endl;
            cout << "target hole:" << plan_pos[holeNum1[min]] << endl;
            predict_point = temp_predict1[min];
            cout << "predict_point:" << predict_point.x << ","
            << predict_point.y << endl;

            iResult = 1;
        }
        else
            iResult = 0;
    }
    else {
        if (holeNum2.size() != 0) {
            cout << "easy ball:" << plan_pos[max + 7] << endl;
            cout << "target hole:" << plan_pos[holeNum2[max]] << endl;
            predict_point = temp_predict2[max];
            cout << "predict_point:" << predict_point.x << ","
            << predict_point.y << endl;

            iResult = 1;
        }
        else
            iResult = 0;
    }

    if(iResult == 1)
    {
        /*if(flag > 0)
        {
            ball = min;
            hole = holeNum1[ball];
        }
        else
        {
            ball = max;
            hole = holeNum2[ball];
        }

        dist1 = sqrt(pow(plan_pos[0].x - plan_pos[ball + 7].x, 2) + pow(plan_pos[0].y - plan_pos[ball + 7].y, 2));
        dist2 = sqrt(pow(plan_pos[ball + 7].x - plan_pos[hole].x, 2) + pow(plan_pos[ball + 7].y - plan_pos[hole].y, 2));

        dist = dist1 + dist2;

        cout << "-------------------------------dist" << dist << endl;

        if (dist >= 0 && dist < 600)
            planData->action.cuer.air = 2;
        else if (dist >= 600 && dist < 800)
            planData->action.cuer.air = 3;
        else if (dist >= 800 && dist < 1000)
            planData->action.cuer.air = 4;
        else if (dist >= 1000)
            planData->action.cuer.air = 5;*/

        planData->action.cuer.air = 3;

        return iResult;
    }
    else
        return iResult;
}

int Plan::kissBall(PlanData* planData) {
    int min = 0;
    int flag = 0;
    int count1 = 0;
    int count2 = 0;
    int count3 = 0;
    int check;
    int shift = 15;  // table edge
    float a, b;
    float vertical_x;
    float dist;
    float dist1, dist2, dist3;
    float o2p;
    vector<int> filter1;
    vector<int> filter2;
    vector<int> filter3;
    vector<unsigned int> firstTarget;
    vector<int> finalFirstTarget;
    vector<int> finalSecondTarget;
    vector<int> finalHole;
    vector<double> temp_dist;
    vector<double> temp;
    vector<Point2f> finalPredict;
    vector<Point2f> finalPredict1;
    vector<Point2f> predict;
    vector<Point2f> temp_predict;
    vector<Point2f> predict1;
    Point2f every_predict_point;
    Point2f wt1;
    Point2f t1t2;
    Point2f t2hole;
    double wt1Length;
    double t1t2Length;
    double t2hLength;
    double shift_x;
    double shift_y;
    // version1 parameter
    vector<double> sumDist;

    filter1.clear();
    filter2.clear();
    filter3.clear();
    firstTarget.clear();
    finalFirstTarget.clear();
    finalSecondTarget.clear();
    finalPredict.clear();
    finalPredict1.clear();
    temp_dist.clear();
    sumDist.clear();
    temp.clear();
    predict.clear();
    temp_predict.clear();
    predict1.clear();
    finalHole.clear();

    // secondTargetball2every hole
    for (size_t i = 7; i < plan_pos.size(); i++) {
        for (size_t h = 1; h < 7; h++) {
            count1 = 0;
            // second target ball and every hole linear function
            a = (plan_pos[i].y - plan_pos[h].y) /
                (plan_pos[i].x - plan_pos[h].x);
            b = plan_pos[i].y - (a * plan_pos[i].x);

            // calc dot-line distance && dot-dot distance
            for (size_t j = 7; j < plan_pos.size(); j++) {
                if (j != i) {
                    if (plan_pos[i].x == plan_pos[h].x) {
                        dist = abs(plan_pos[j].x - plan_pos[i].x);
                    } else {
                        dist = abs(a * plan_pos[j].x - plan_pos[j].y + b) /
                               sqrt(1 + a * a);
                    }

                    if (dist < ballR) {  // Exception1
                        if (plan_pos[i].x == plan_pos[h].x) {
                            if (plan_pos[h].y > plan_pos[i].y) {
                                if (plan_pos[j].y < plan_pos[h].y &&
                                    plan_pos[j].y > plan_pos[i].y)
                                    count1++;
                            } else {
                                if (plan_pos[j].y > plan_pos[h].y &&
                                    plan_pos[j].y < plan_pos[i].y)
                                    count1++;
                            }
                        } else if (plan_pos[i].y == plan_pos[h].y) {
                            if (plan_pos[h].x > plan_pos[i].x) {
                                if (plan_pos[j].x < plan_pos[h].x &&
                                    plan_pos[j].x > plan_pos[i].x)
                                    count1++;
                            } else {
                                if (plan_pos[j].x > plan_pos[h].x &&
                                    plan_pos[j].x < plan_pos[i].x)
                                    count1++;
                            }
                        } else {
                            vertical_x = (b - (plan_pos[j].y +
                                               (1 / a) * plan_pos[j].x)) /
                                         ((-1 / a) - a);
                            if (plan_pos[i].x < plan_pos[h].x) {
                                if (vertical_x < plan_pos[h].x &&
                                    vertical_x > plan_pos[i].x)
                                    count1++;
                            } else {
                                if (vertical_x > plan_pos[h].x &&
                                    vertical_x < plan_pos[i].x)
                                    count1++;
                            }
                        }
                    }//if
                }//for j
            }//for h
            if (count1 == 0)
                filter1.push_back(h);  // filter1: save can hit hole
        }//for i

        /*for (size_t k = 0; k < filter1.size(); k++) {
            cout << i << "filter1[" << k << "]:" << filter1[k] << endl;
        }*/

        if (filter1.size() != 0) {
            // find second targetball_canHitBall predict point
            // second tagetball && filter hole vector to find predict point
            for (size_t k = 0; k < filter1.size(); k++) {
                check = 1;
                double distBall2Hole =
                    sqrt(pow(plan_pos[i].x - plan_pos[filter1[k]].x, 2) +
                         pow(plan_pos[i].y - plan_pos[filter1[k]].y, 2));

                shift_x = ballR * abs(plan_pos[i].x - plan_pos[filter1[k]].x) /
                          distBall2Hole;
                shift_y = ballR * abs(plan_pos[i].y - plan_pos[filter1[k]].y) /
                          distBall2Hole;

                if (plan_pos[filter1[k]].x < plan_pos[i].x &&
                    plan_pos[filter1[k]].y < plan_pos[i].y) {
                    every_predict_point.x = plan_pos[i].x + shift_x;
                    every_predict_point.y = plan_pos[i].y + shift_y;
                } else if (plan_pos[filter1[k]].x < plan_pos[i].x &&
                           plan_pos[filter1[k]].y > plan_pos[i].y) {
                    every_predict_point.x = plan_pos[i].x + shift_x;
                    every_predict_point.y = plan_pos[i].y - shift_y;
                } else if (plan_pos[filter1[k]].x > plan_pos[i].x &&
                           plan_pos[filter1[k]].y < plan_pos[i].y) {
                    every_predict_point.x = plan_pos[i].x - shift_x;
                    every_predict_point.y = plan_pos[i].y + shift_y;
                } else {
                    every_predict_point.x = plan_pos[i].x - shift_x;
                    every_predict_point.y = plan_pos[i].y - shift_y;
                }
                if (planData->degree <= 90 && planData->degree >= 0) {
                    if (every_predict_point.x < plan_pos[3].x + shift ||
                        every_predict_point.y > plan_pos[4].y - shift ||
                        every_predict_point.x > plan_pos[6].x - shift ||
                        every_predict_point.y < plan_pos[5].y + shift)
                        check = 0;
                } else if (planData->degree > 90 && planData->degree <= 180) {
                    if (every_predict_point.x < plan_pos[4].x + shift ||
                        every_predict_point.y > plan_pos[6].y - shift ||
                        every_predict_point.x > plan_pos[5].x - shift ||
                        every_predict_point.y < plan_pos[3].y + shift)
                        check = 0;
                }
                if (check == 1) {
                    temp_predict.push_back(every_predict_point);
                    filter2.push_back(filter1[k]);  // filter2:save can hit hole
                }
            }//for k

            // filter predict: firstTarget2predict obstacle
            // firstTarget and predict_point linear function
            for (size_t k = 0; k < temp_predict.size(); k++) {
                for (size_t j = 7; j < plan_pos.size(); j++) {
                    count2 = 0;
                    if (i != j) {
                        a = (plan_pos[j].y - temp_predict[k].y) /
                            (plan_pos[j].x - temp_predict[k].x);
                        b = plan_pos[j].y - (a * plan_pos[j].x);
                        // calc dot-line distance && dot-dot distance
                        for (size_t s = 7; s < plan_pos.size(); s++) {
                            if (s != j) {
                                if (plan_pos[j].x == temp_predict[k].x) {
                                    dist = abs(plan_pos[s].x - plan_pos[j].x);
                                } else {
                                    dist = abs(a * plan_pos[s].x -
                                               plan_pos[s].y + b) /
                                           sqrt(1 + a * a);
                                }

                                o2p = sqrt(
                                    pow(plan_pos[s].x - temp_predict[k].x, 2) +
                                    pow(plan_pos[s].y - temp_predict[k].y, 2));

                                /*cout << "j(first):" << j << ", i(second):" <<
                                i
                                     << ", s(obstacle):" << s << endl;
                                cout << "dist:" << dist << endl;*/

                                if (dist < ballR) {
                                    if (plan_pos[j].x == temp_predict[k].x) {
                                        if (temp_predict[k].y > plan_pos[j].y) {
                                            if (plan_pos[s].y <
                                                    temp_predict[k].y &&
                                                plan_pos[s].y > plan_pos[j].y)
                                                count2++;
                                        } else {
                                            if (plan_pos[s].y >
                                                    temp_predict[k].y &&
                                                plan_pos[s].y < plan_pos[j].y)
                                                count2++;
                                        }
                                    } else if (plan_pos[j].y ==
                                               temp_predict[k].y) {
                                        if (temp_predict[k].x > plan_pos[j].x) {
                                            if (plan_pos[s].x <
                                                    temp_predict[k].x &&
                                                plan_pos[s].x > plan_pos[j].x)
                                                count2++;
                                        } else {
                                            if (plan_pos[s].x >
                                                    temp_predict[k].x &&
                                                plan_pos[s].x < plan_pos[j].x)
                                                count2++;
                                        }
                                    } else {
                                        vertical_x =
                                            (b - (plan_pos[s].y +
                                                  (1 / a) * plan_pos[s].x)) /
                                            ((-1 / a) - a);
                                        if (plan_pos[j].x < temp_predict[k].x) {
                                            if (vertical_x <
                                                    temp_predict[k].x &&
                                                vertical_x > plan_pos[j].x)
                                                count2++;
                                        } else {
                                            if (vertical_x >
                                                    temp_predict[k].x &&
                                                vertical_x < plan_pos[j].x)
                                                count2++;
                                        }
                                    }
                                }
                                if (o2p <= ballR - 1) {
                                    count2++;
                                }
                            }
                        }
                    } else
                        count2++;
                    if (count2 == 0) {
                        filter3.push_back(
                            filter2[k]);  // filter3:save can hit hole
                        predict.push_back(temp_predict[k]);
                        firstTarget.push_back(j);  // save first target ball
                    }
                }
            }

            /*for (size_t k = 0; k < filter2.size(); k++) {
                cout << i << "filter2[" << k << "]:" << filter2[k] << endl;
            }*/

            if (firstTarget.size() != 0) {
                // find first targetball predict point
                // first tagetball && predict point vector to find first
                // targetball predict point
                for (size_t k = 0; k < firstTarget.size(); k++) {
                    check = 1;
                    double distBall2Hole =
                        sqrt(pow(plan_pos[firstTarget[k]].x - predict[k].x, 2) +
                             pow(plan_pos[firstTarget[k]].y - predict[k].y, 2));

                    shift_x = ballR *
                              abs(plan_pos[firstTarget[k]].x - predict[k].x) /
                              distBall2Hole;
                    shift_y = ballR *
                              abs(plan_pos[firstTarget[k]].y - predict[k].y) /
                              distBall2Hole;

                    if (predict[k].x < plan_pos[firstTarget[k]].x &&
                        predict[k].y < plan_pos[firstTarget[k]].y) {
                        every_predict_point.x =
                            plan_pos[firstTarget[k]].x + shift_x;
                        every_predict_point.y =
                            plan_pos[firstTarget[k]].y + shift_y;
                    } else if (predict[k].x < plan_pos[firstTarget[k]].x &&
                               predict[k].y > plan_pos[firstTarget[k]].y) {
                        every_predict_point.x =
                            plan_pos[firstTarget[k]].x + shift_x;
                        every_predict_point.y =
                            plan_pos[firstTarget[k]].y - shift_y;
                    } else if (predict[k].x > plan_pos[firstTarget[k]].x &&
                               predict[k].y < plan_pos[firstTarget[k]].y) {
                        every_predict_point.x =
                            plan_pos[firstTarget[k]].x - shift_x;
                        every_predict_point.y =
                            plan_pos[firstTarget[k]].y + shift_y;
                    } else {
                        every_predict_point.x =
                            plan_pos[firstTarget[k]].x - shift_x;
                        every_predict_point.y =
                            plan_pos[firstTarget[k]].y - shift_y;
                    }
                    if (planData->degree <= 90 && planData->degree >= 0) {
                        if (every_predict_point.x < plan_pos[3].x + shift ||
                            every_predict_point.y > plan_pos[4].y - shift ||
                            every_predict_point.x > plan_pos[6].x - shift ||
                            every_predict_point.y < plan_pos[5].y + shift)
                            check = 0;
                    } else if (planData->degree > 90 &&
                               planData->degree <= 180) {
                        if (every_predict_point.x < plan_pos[4].x + shift ||
                            every_predict_point.y > plan_pos[6].y - shift ||
                            every_predict_point.x > plan_pos[5].x - shift ||
                            every_predict_point.y < plan_pos[3].y + shift)
                            check = 0;
                    }
                    if (check == 1) predict1.push_back(every_predict_point);
                }
                // filter predict: white2predict obstacle
                // white and predict_point linear function
                for (size_t k = 0; k < predict1.size(); k++) {
                    count3 = 0;
                    a = (plan_pos[0].y - predict1[k].y) /
                        (plan_pos[0].x - predict1[k].x);
                    b = plan_pos[0].y - (a * plan_pos[0].x);
                    // calc dot-line distance && dot-dot distance
                    for (size_t j = 7; j < plan_pos.size(); j++) {
                        if (j != i) {
                            if (plan_pos[0].x == predict1[k].x) {
                                dist = abs(plan_pos[j].x - plan_pos[0].x);
                            } else {
                                dist =
                                    abs(a * plan_pos[j].x - plan_pos[j].y + b) /
                                    sqrt(1 + a * a);
                            }

                            o2p = sqrt(pow(plan_pos[j].x - predict1[k].x, 2) +
                                       pow(plan_pos[j].y - predict1[k].y, 2));

                            if (dist < ballR) {
                                if (plan_pos[0].x == predict1[k].x) {
                                    if (predict1[k].y > plan_pos[0].y) {
                                        if (plan_pos[j].y < predict1[k].y &&
                                            plan_pos[j].y > plan_pos[0].y)
                                            count3++;
                                    } else {
                                        if (plan_pos[j].y > predict1[k].y &&
                                            plan_pos[j].y < plan_pos[0].y)
                                            count3++;
                                    }
                                } else if (plan_pos[0].y == predict1[k].y) {
                                    if (predict1[k].x > plan_pos[0].x) {
                                        if (plan_pos[j].x < predict1[k].x &&
                                            plan_pos[j].x > plan_pos[0].x)
                                            count3++;
                                    } else {
                                        if (plan_pos[j].x > predict1[k].x &&
                                            plan_pos[j].x < plan_pos[0].x)
                                            count3++;
                                    }
                                } else {
                                    vertical_x =
                                        (b - (plan_pos[j].y +
                                              (1 / a) * plan_pos[j].x)) /
                                        ((-1 / a) - a);
                                    if (plan_pos[0].x < predict1[k].x) {
                                        if (vertical_x < predict1[k].x &&
                                            vertical_x > plan_pos[0].x)
                                            count3++;
                                    } else {
                                        if (vertical_x > predict1[k].x &&
                                            vertical_x < plan_pos[0].x)
                                            count3++;
                                    }
                                }
                            }
                            if (o2p <= ballR - 1) {
                                count3++;
                            }
                        }
                    }
                    if (count3 == 0) {
                        finalFirstTarget.push_back(
                            firstTarget[k]);  // save final firstTargetball
                        finalPredict1.push_back(
                            predict1[k]);  // save final firstTargetball predict
                                           // point
                        finalPredict.push_back(
                            predict[k]);  // save final secondTargetball predict
                                          // point
                        finalHole.push_back(filter3[k]);  // save final hole
                        finalSecondTarget.push_back(
                            i);  // save final secondTargetball
                    }
                }
            }  // firstTarget!=0
        }      // filter1!=0

        temp_predict.clear();
        predict.clear();
        predict1.clear();
        filter1.clear();
        filter2.clear();
        filter3.clear();
        firstTarget.clear();
    }  // for

    if (finalFirstTarget.size() != 0) {
        // whether dist of ball2hole < ballR or not
        temp_dist.resize(finalFirstTarget.size());
        for (size_t k = 0; k < finalFirstTarget.size(); k++) {
            temp_dist[k] = sqrt(
                pow(plan_pos[finalSecondTarget[k]].x - plan_pos[finalHole[k]].x,
                    2) +
                pow(plan_pos[finalSecondTarget[k]].y - plan_pos[finalHole[k]].y,
                    2));
        }

        for (size_t k = 0; k < temp_dist.size(); k++) {
            if (temp_dist[k] < 1.5 * ballR) flag++;
        }
        // version1
        if (flag > 0) {
            // choose the least sumDist
            sumDist.resize(finalFirstTarget.size());
            for (size_t k = 0; k < finalFirstTarget.size(); k++) {
                sumDist[k] = sqrt(pow(plan_pos[finalSecondTarget[k]].x -
                                    plan_pos[finalHole[k]].x,
                                2) +
                            pow(plan_pos[finalSecondTarget[k]].y -
                                    plan_pos[finalHole[k]].y,
                                2));
                if (sumDist[k] < sumDist[min]) min = k;
            }
        }  // version1

        // version2
        else {
            // choose the least dot
            // (whiteball FirstTarget)   dot   (FirstTarget  SecondTarget) +
            // (FirstTarget SecondTarget)   dot   (SecondTarget  filter_holes)
            temp.resize(finalFirstTarget.size());
            for (size_t k = 0; k < finalFirstTarget.size(); k++) {
                wt1.x = plan_pos[finalFirstTarget[k]].x - plan_pos[0].x;
                wt1.y = plan_pos[finalFirstTarget[k]].y - plan_pos[0].y;
                wt1Length = sqrt(pow(wt1.x, 2) + pow(wt1.y, 2));

                t1t2.x = plan_pos[finalSecondTarget[k]].x -
                         plan_pos[finalFirstTarget[k]].x;
                t1t2.y = plan_pos[finalSecondTarget[k]].y -
                         plan_pos[finalFirstTarget[k]].y;
                t1t2Length = sqrt(pow(t1t2.x, 2) + pow(t1t2.y, 2));

                t2hole.x =
                    plan_pos[finalHole[k]].x - plan_pos[finalSecondTarget[k]].x;
                t2hole.y =
                    plan_pos[finalHole[k]].y - plan_pos[finalSecondTarget[k]].y;
                t2hLength = sqrt(pow(t2hole.x, 2) + pow(t2hole.y, 2));

                temp[k] = (dot(wt1, t1t2) / (wt1Length * t1t2Length)) +
                          (dot(t1t2, t2hole) / (t1t2Length * t2hLength));
                if (temp[k] > temp[min]) min = k;  // min->max
            }                                      // version2
        }
        cout << "finalFirstTarget:" << finalFirstTarget[min] << " , "
             << plan_pos[finalFirstTarget[min]] << endl
             << "finalSecondTarget:" << finalSecondTarget[min] << " , "
             << plan_pos[finalSecondTarget[min]] << endl
             << "hole:" << finalHole[min] << endl;

        predict_point = finalPredict1[min];
        // predict_point = finalPredict[min];

        dist1 = sqrt(pow(plan_pos[0].x - plan_pos[finalFirstTarget[min]].x, 2) + pow(plan_pos[0].y - plan_pos[finalFirstTarget[min]].y, 2));
        dist2 = sqrt(pow(plan_pos[finalFirstTarget[min]].x - plan_pos[finalSecondTarget[min]].x, 2) + pow(plan_pos[finalFirstTarget[min]].y - plan_pos[finalSecondTarget[min]].y, 2));
        dist3 = sqrt(pow(plan_pos[finalSecondTarget[min]].x - plan_pos[finalHole[min]].x, 2) + pow(plan_pos[finalSecondTarget[min]].y - plan_pos[finalHole[min]].y, 2));

        dist = dist1 + dist2 + dist3;

        cout << "-------------------------------dist" << dist << endl;

        if (dist >= 0 && dist < 600)
        planData->action.cuer.air = 2;
        else if (dist >= 600 && dist < 800)
        planData->action.cuer.air = 3;
        else if (dist >= 800 && dist < 1000)
        planData->action.cuer.air = 4;
        else if (dist >= 1000)
        planData->action.cuer.air = 5;

        return 1;
    }
    return 0;
}

void Plan::randomBall(PlanData* planData) {
    int num = 0;
    srand(time(NULL));
    num = (rand() % (plan_pos.size() - 7)) + 7;

    predict_point.x = plan_pos[num].x;
    predict_point.y = plan_pos[num].y;

    planData->action.cuer.air = 5;
}

int Plan::dot(Point v1, Point v2) {
    // calc dot
    return v1.x * v2.x + v1.y * v2.y;
}

void Plan::draw() {
    // draw
    input = Mat::zeros(540, 960, CV_8UC3);
    namedWindow("input",WINDOW_AUTOSIZE);

    line(input, plan_pos[1] / 2, plan_pos[5] / 2, cv::Scalar(0, 255, 0), 2);
    line(input, plan_pos[5] / 2, plan_pos[6] / 2, cv::Scalar(0, 0, 255), 2);
    line(input, plan_pos[6] / 2, plan_pos[2] / 2, cv::Scalar(255, 0, 0), 2);
    line(input, plan_pos[2] / 2, plan_pos[4] / 2, cv::Scalar(255, 255, 0), 2);
    line(input, plan_pos[4] / 2, plan_pos[3] / 2, cv::Scalar(0, 255, 255), 2);
    line(input, plan_pos[3] / 2, plan_pos[1] / 2, cv::Scalar(255, 0, 255), 2);

    if (plan_pos[0] != Point2f(0, 0))
        circle(input, Point(plan_pos[0].x / 2, plan_pos[0].y / 2), 0,
               Scalar(255, 255, 255), ballR / 2);

    if (predict_point != Point(0, 0))
        circle(input, Point(predict_point.x / 2, predict_point.y / 2),
               ballR / 4, Scalar(0, 255, 255), 2);

    for (size_t i = 7; i < plan_pos.size(); i++) {
        circle(input, Point(plan_pos[i].x / 2, plan_pos[i].y / 2), 0,
               Scalar((i * 80) % 155 + 81, (i * 160) % 155 + 81,
                      (i * 240) % 155 + 81),
               ballR / 2);
    }

    imshow("input", input);
    waitKey(1);
}
