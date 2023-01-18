#include <chrono>
#include <list>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <random>
#include <tuple>

std::random_device rd;
std::mt19937 generator(rd());

std::pair<cv::Mat, cv::Mat> get_camera_properties_ps3eye()
{
    // coefficients for approximated PS3 Eye camera
    //! Camera matrix
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 600, 0, 320,
            0, 600, 240,
            0, 0, 1);
    //! Distortion coefficients
    cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);

    return {camera_matrix, dist_coeffs};
}

cv::Mat get_object_points()
{
    const double MARKER_MM = 20;
    cv::Mat obj_points(4, 1, CV_32FC3);
    obj_points.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-MARKER_MM / 2.f, MARKER_MM / 2.f, 0);
    obj_points.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(MARKER_MM / 2.f, MARKER_MM / 2.f, 0);
    obj_points.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(MARKER_MM / 2.f, -MARKER_MM / 2.f, 0);
    obj_points.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-MARKER_MM / 2.f, -MARKER_MM / 2.f, 0);
    return obj_points;
}

int main()
{

    using namespace cv;
    using namespace std;

    list<pair<std::chrono::steady_clock::time_point, vector<Point2f>>> detectedPositions;

    VideoCapture cam(0);
    Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    namedWindow("markers", WINDOW_NORMAL);

    auto [camera_matrix, dist_coeffs] = get_camera_properties_ps3eye();
    auto obj_points = get_object_points();

    Mat car_image = imread("car0.png");
    Mat car_image2 = imread("green_car.png");
    Mat car_image3 = imread("blue_car.png");
    Mat car_image4 = imread("C:\\Users\\48883\\CLionProjects\\ArucoGame\\purple.png");

    Mat goal_image = imread("C:\\Users\\48883\\CLionProjects\\ArucoGame\\obstacle.png");

    int carId=0;

    vector<Mat> images;
    images.push_back(car_image);
    images.push_back(car_image2);
    images.push_back(car_image3);
    images.push_back(car_image4);

    Vec2f car_position(200, 200);
    Vec2f car2_position(300,300);
    Vec2f car3_position(400,400);
    Vec2f car4_position(500,500);

    vector<Vec2f> carPos;
    carPos.push_back(car_position);
    carPos.push_back(car2_position);
    carPos.push_back(car3_position);
    carPos.push_back(car4_position);

    vector<Mat> insetImages;

    vector<int> points;
    points.push_back(0);
    points.push_back(0);
    points.push_back(0);
    points.push_back(0);



    vector<Vec2f> goals;
    int counter = 0;
    int points2 = 0;
    while (waitKey(10) != 27) {
        Mat inputImage;
        Mat detected;
        cam >> inputImage;
        resize(inputImage, inputImage, Size(600, 600), 0, 0, INTER_CUBIC);
        detected = inputImage.clone();

        if ((counter++ % 20 == 0) && (goals.size() < 10)) {
            std::uniform_int_distribution<int> distr_x(0 + goal_image.cols, detected.cols - goal_image.cols);
            std::uniform_int_distribution<int> distr_y(0 + goal_image.rows, detected.rows - goal_image.rows);
            Vec2f p;
            p.val[0] = distr_x(generator);
            p.val[1] = distr_y(generator);
            goals.push_back(p);
        }

        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners, rejectedCandidates;
        aruco::detectMarkers(inputImage, dictionary, markerCorners,
                             markerIds, parameters, rejectedCandidates);
        Mat outputImage = inputImage.clone();
        aruco::drawDetectedMarkers(detected, markerCorners, markerIds);


        if (markerIds.size() != 0) {
            for (int i = 0; i < markerIds.size(); i++) {
                if (markerIds[i] == 20){
                    carId = 0;
                }
                else if (markerIds[i] == 21){
                    carId = 1;
                }
                else if (markerIds[i]==22){
                    carId = 2;
                }
                else if (markerIds[i]==23){
                    carId = 3;
                }
                auto found = markerCorners.at(i);

                cv::Vec3d rvecs, tvecs;
                cv::solvePnP(obj_points, found, camera_matrix, dist_coeffs, rvecs, tvecs);
                cv::drawFrameAxes(detected, camera_matrix, dist_coeffs, rvecs, tvecs, 10.1);

                Mat rot_mat;
                Rodrigues(rvecs, rot_mat);
                double angle_x = atan2(rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
                angle_x = -angle_x + ((angle_x < 0) ? (-M_PI) : M_PI);
                double angle_y = -asin(rot_mat.at<double>(2, 0));

                auto new_pos = carPos[carId];
                new_pos.val[0] += 10.0 * angle_y;
                new_pos.val[1] -= 10.0 * angle_x;
                auto new_pos_scr = new_pos;
                new_pos_scr.val[0] -= images[carId].cols / 2;
                new_pos_scr.val[1] -= images[carId].rows / 2;
                if ((new_pos_scr.val[0] > 0) && (new_pos_scr.val[0] < (detected.cols - images[carId].cols - 1)) && (new_pos_scr.val[1] > 0) && (new_pos_scr.val[1] < (detected.rows - images[carId].rows - 1))) {
                    carPos[carId] = new_pos;
                }

                Mat insetImage(detected,
                               Rect(
                                       carPos[carId].val[0] - (images[carId].cols / 2),
                                       carPos[carId].val[1] - (images[carId].rows / 2),
                                       images[carId].rows, images[carId].cols));
                images[carId].copyTo(insetImage);

                for (int i = 0; i < goals.size(); i++) {
                    for (int j = 0; j < markerIds.size(); j++) {
                        if (cv::norm(goals[i] - carPos[carId]) < (goal_image.cols / 2)) {
                            points[carId]++;
                            std::cout <<"car " << carId << " points " << points[carId] << std::endl;
                            goals.erase(goals.begin() + i);
                            i--;
                        }
                    }

                }
            }
        }

        for (auto goal : goals) {
            Mat insetImage(detected,
                           Rect(
                                   goal.val[0] - (goal_image.cols / 2),
                                   goal.val[1] - (goal_image.rows / 2),
                                   goal_image.rows, goal_image.cols));
            goal_image.copyTo(insetImage);

        }

        imshow("markers", detected);

    }

    return 0;
}