#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {

    //cout << "num = " << (2/(2-2)) << endl;
    //Mat temp =  Mat(1, 2, CV_8UC1);

    //cout << sin(45*CV_PI/180) <<  endl;

    /*double a = 0.5;
    cout << (asin(a)*180/CV_PI) <<  endl;
    cout << (asin(-a)*180/CV_PI) <<  endl;*/

   /* double input;

    cout << "set input" << endl;
    cin >> input;
    cout << "input:" << input << endl;
    cout << "test" <<endl;*/

   /*char key;
   Mat img = imread("black_circle.jpg");

   while(1)
   {
       imshow("img",img);

       key = waitKey(0);

       cout << "key:" << key << "/ascii:" << (int)key <<endl;

       if(key==27)
           break;
   }*/

   /*int count;
   int font_face = FONT_HERSHEY_SCRIPT_SIMPLEX;

   for(count = 0; count < 10; count++)
   {
       Mat image = Mat::zeros(Size(300, 500), CV_8UC3);

       image.setTo(Scalar(0, 0, 0));

       string text = to_string(count);

       Point origin;
       origin.x = image.cols / 2 - 90;
       origin.y = image.rows / 2 + 80;

       putText(image, text, origin, font_face, 10, Scalar(0, 255, 255), 8, 8, 0);

       imshow("count", image);
       if(waitKey(0) == 27)
           break;
   }*/

   if(1)
       cout << "1" << endl;


   return 0;
}
