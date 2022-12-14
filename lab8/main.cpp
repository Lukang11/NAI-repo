#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
    int hmin = 40, smin = 120, vmin = 140;
    int hmax = 90, smax = 240, vmax = 255;

    VideoCapture camera(1);
    if (!camera.isOpened())
    {
        cerr << "Nie udało się otworzyć kamery!" << endl;
        return 1;
    }

     namedWindow("Trackbars", (640, 200));
     createTrackbar("Hue Min", "Trackbars", &hmin, 255);
     createTrackbar("Hue Max", "Trackbars", &hmax, 255);
     createTrackbar("Sat Min", "Trackbars", &smin, 255);
     createTrackbar("Sat Max", "Trackbars", &smax, 255);
     createTrackbar("Val Min", "Trackbars", &vmin, 255);
     createTrackbar("Val Max", "Trackbars", &vmax, 255);

    Mat frame;
    while (true)
    {
        Scalar lower(hmin, smin, vmin);
        Scalar upper(hmax, smax, vmax);

        camera >> frame;
        if (frame.empty())
            break;
        Mat hsv;
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        Size size_k(5, 5);
        Mat gblured;
        GaussianBlur(hsv, gblured, size_k, 0);

        Mat mask;
        inRange(gblured, lower, upper, mask);

        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (contours.size() >= 2)
        {
            sort(contours.begin(), contours.end(), [](const vector<Point> &c1, const vector<Point> &c2)
                 { return contourArea(c1) > contourArea(c2); });

            vector<vector<Point>> largestContours;
            for (int i = 0; i < min(2, (int)contours.size()); i++)
                largestContours.push_back(contours[i]);

            bool inLine = true;

            int margin = 10;

            Rect rect1 = boundingRect(largestContours[0]);
            Rect rect2 = boundingRect(largestContours[1]);

            if (abs(rect1.y - rect2.y) > margin)
            {
                inLine = false;
            }

            if (inLine)
            {
                drawContours(frame, largestContours, -1, (0, 0, 255), 2);

                int centerX = (rect1.x + rect2.x + rect2.width) / 2;
                int centerY = (rect1.y + rect1.height / 2);

                double distance = sqrt(pow(rect1.x - rect2.x, 2) + pow(rect1.y - rect2.y, 2));
                double lineSize = distance / 100;
                if (lineSize > 4)
                {
                    lineSize = 4;
                }
                if (lineSize < 1)
                {
                    lineSize = 1;
                }

                line(frame, Point(centerX - (int)lineSize * 10, centerY - (int)lineSize * 5), Point(centerX + (int)lineSize * 10, centerY - (int)lineSize * 5), Scalar(0, 0, 255), (int)lineSize);
                line(frame, Point(centerX - (int)lineSize * 10, centerY), Point(centerX + (int)lineSize * 10, centerY), Scalar(0, 0, 255), (int)lineSize);
            }
        }

        imshow("Camera", frame);

        int key = waitKey(1);
        if (key == 27) 
            break;
    }
    camera.release();
    destroyAllWindows();
    return 0;
}