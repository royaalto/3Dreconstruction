//Copyright: Wang Fei, Zhang Yi
//All Rights Reserved
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

int main() {

    VideoCapture wf0(1);
    if  (!wf0.isOpened()) cout << "wf0 doesn't work" << endl;
  
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    detectorParams->doCornerRefinement = true; // do corner refinement in markers
    Mat camMatrix = (Mat_<double>(3,3) << 5.2154551281844294e+02, 0., 4.1585350167273896e+02, 0., 5.2097355148436213e+02, 2.4350949737767709e+02, 0., 0., 1. );
    Mat distCoeffs = (Mat_<double>(1,8) << 7.9718015364515754e-02, -5.2960436121112597e-01, 0., 0., 0., 0., 0., -1.2970793941672798e+00);

    while(wf0.grab()) {
	
	Mat frame0, frameCopy0;
	wf0.retrieve(frame0);
        vector< int > ids0;
        vector< vector< Point2f > > corners0;
	float markerLength = 3.591858;//166.5/75.22
	vector< Vec3d > rvecs, tvecs;
        aruco::detectMarkers(frame0, dictionary, corners0, ids0);
        frame0.copyTo(frameCopy0);
        Mat rotation;
        if(ids0.size() > 0) {
	    	aruco::estimatePoseSingleMarkers(corners0, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
           	aruco::drawDetectedMarkers(frameCopy0, corners0, ids0); 
		for(int i=0; i<ids0.size(); i++){
			aruco::drawAxis(frameCopy0, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);}
		for(int j=0; j<ids0.size(); j++){
			Rodrigues(rvecs[j], rotation, noArray());}
        }
	cout << "rotation" << rotation << endl;
	for(int j = 0; j < rvecs.size(); j++){
	cout << "rvecs" << rvecs[j] << endl;}
	for(int j = 0; j < tvecs.size(); j++){
	cout << "tvecs" << tvecs[j] << endl;}
        imshow("wf0", frameCopy0);
	char key = (char)waitKey(1);
	if(key == 27) break;
/*

rotation[0.9852906654797197, -0.1123573525400376, 0.1287560866472986;
 -0.08473600967702624, -0.9755353298520266, -0.2028561778071166;
 0.1483984945427273, 0.1889620214241245, -0.970706568060998]
rvecs[2.93271, -0.147021, 0.206742]
tvecs[-10.9006, -2.30396, 34.1518]





*/

        }//while

    wf0.release();

    return 0;
}//main
