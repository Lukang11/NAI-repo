#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
// g++ `pkg-config --cflags opencv4` cv2.cpp `pkg-config --libs opencv4`
using namespace std;
using namespace cv;
int main(int argc, char **argv)
{
    VideoCapture vc(0);
    if (!vc.isOpened())
        return 1;
    cv::Mat frame;

    cv::namedWindow("Window", WINDOW_AUTOSIZE);
    int i = 0;
    while (waitKey(10) != 27)
    {
        vc >> frame;
        std::cout << "frame info:" << frame.rows << "" << frame.cols << std::endl;
        flip(frame, frame, 1);
        cv::imshow("Frame", frame);
    }
    return 0;
}