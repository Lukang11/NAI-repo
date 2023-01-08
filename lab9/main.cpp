#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

int main()
{
    // Ładujemy bibliotekę OpenCV do wykrywania i odczytywania kodów ArUco
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();

    // Otwieramy kamerę laptopa
    VideoCapture capture(0);
    if (!capture.isOpened())
    {
        cerr << "Nie udało się otworzyć kamery" << endl;
        return -1;
    }

    while (true)
    {
        // Pobieramy klatkę z kamery
        Mat frame;
        capture >> frame;
        if (frame.empty())
        {
            cerr << "Nie udało się pobrać klatki z kamery" << endl;
            break;
        }

        // Wykrywamy marker ArUco na obrazie i odczytujemy ich kody
        vector<vector<Point2f>> corners;
        vector<int> ids;
        aruco::detectMarkers(frame, dictionary, corners, ids, parameters);

        // Jeśli znaleziono co najmniej dwa markery ArUco, obliczamy odległość pomiędzy nimi
        if (corners.size() >= 2)
        {
            // Współrzędne pierwszego markeru ArUco
            Point2f point1 = corners[0][0];
            // Współrzędne drugiego markeru ArUco
            Point2f point2 = corners[1][0];

            // Obliczamy odległość euklidesową pomiędzy markerami
            float distance = sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2));

            cout << "Odległość pomiędzy markerami wynosi: " << distance << endl;
        }
        else{
            cout << "Nie znaleziono dwóch markerów aruco" << endl;
        }
        aruco::drawDetectedMarkers(frame,corners,ids);
        imshow("Detektor aruco",frame);
        char key =(char) waitKey(1);
        if(key == 'q'){
            break;
        }
    }

     return 0;
    }