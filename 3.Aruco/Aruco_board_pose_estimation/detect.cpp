#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
using namespace std;
using namespace cv;

int main()
{
Mat cameraMatrix =(Mat_<double>(3,3)<<525.4228, 0.0, 305.959,0.0, 525.8927, 237.5957,0.0, 0.0, 1.0);

Mat distCoeffs=(Mat_<double>(1,5)<<-0.135429313396, 0.36515500, -0.0006460,-0.0014183711660084132, -0.268449577);

cv::VideoCapture inputVideo(1);

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5, 7, 0.04, 0.01, dictionary);


while(inputVideo.grab()) {
    cv::Mat image, imageCopy;
    inputVideo.retrieve(image);
    image.copyTo(imageCopy);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(image, dictionary, corners, ids);
    // if at least one marker detected
    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        cv::Vec3d rvec, tvec;
        int valid = estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs, rvec, tvec);
        // if at least one board marker detected
        if(valid > 0)
            cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
	
    }
    cv::imshow("out", imageCopy);
    char key = (char) cv::waitKey(10);
    if (key == 27)
        break;
	
}

	inputVideo.release();
	return(0);
}
