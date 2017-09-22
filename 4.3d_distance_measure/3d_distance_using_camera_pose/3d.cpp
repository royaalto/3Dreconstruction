
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
    //int k=0;  
    VideoCapture wf1(0);
    VideoCapture wf2(1);
    if  (!wf1.isOpened()) cout << "wf1 doesn't work" << endl;
    if  (!wf2.isOpened()) cout << "wf2 doesn't work" << endl;
	/*wf1.set(CV_CAP_PROP_FRAME_WIDTH, 480);
	wf1.set(CV_CAP_PROP_FRAME_HEIGHT, 270);
	wf2.set(CV_CAP_PROP_FRAME_WIDTH, 480);
	wf2.set(CV_CAP_PROP_FRAME_HEIGHT, 270);*/
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    //detectorParams->doCornerRefinement = true; // do corner refinement in markers

    while(wf1.grab() && wf2.grab()) {
	
	
	Mat frame0, frameCopy0;
        Mat frame1, frameCopy1;
	wf1.retrieve(frame0);
	wf2.retrieve(frame1);	
	
        vector< int > ids1;
        vector< vector< Point2f > > corners0old;
	vector< int > ids2;
        vector< vector< Point2f > > corners1old;

        aruco::detectMarkers(frame0, dictionary, corners0old, ids1);
	aruco::detectMarkers(frame1, dictionary, corners1old, ids2);
	
        frame0.copyTo(frameCopy0);
        frame1.copyTo(frameCopy1);
        if(ids1.size() > 0) {
	    //cout << "right camera has detedted " << ids1.size() << " markers" << endl;
            aruco::drawDetectedMarkers(frameCopy0, corners0old, ids1); 
        }
	if(ids2.size() > 0) {
	    //cout << "left camera has detedted " << ids1.size() << " markers" << endl;
            aruco::drawDetectedMarkers(frameCopy1, corners1old, ids2); 
        }
        imshow("wf2", frameCopy1);
        imshow("wf1", frameCopy0);
	vector<Point2f> corners0;
	vector<Point2f> corners1;
//data transfer


	for (int i = 0; i< corners0old.size(); i++)
	{
		for(int j=0; j < corners0old[i].size(); j++)
			{
				corners0.push_back(corners0old[i][j]);
			}
	}	

	for (int i = 0; i< corners1old.size(); i++)
	{
		for(int j=0; j < corners1old[i].size(); j++)
			{
				corners1.push_back(corners1old[i][j]);
			}
	}

	/*if(corners0.size() > 0)	{
        cout << "corners0  has " << corners0.size() << " corners" << endl;
	cout << "corners0" << corners0 << endl << endl;
	}
	if(corners1.size() > 0)	{
        cout << "corners1  has " << corners1.size() << " corners" << endl;
	cout << "corners1" << corners1 << endl << endl;
	}*/

	
//undistort
	if((ids1.size() == 2) && (ids2.size() == 2) &&(ids1.size() == ids2.size()) &&(ids1.size() == corners0old.size())){
     
//image output

       /* char filename0[80];
	char filename1[80];
	sprintf(filename0,"/home/irob/wf/image/0/test_%d.jpg",k);
	sprintf(filename1,"/home/irob/wf/image/1/test_%d.jpg",k);
        imwrite(filename0, frameCopy0);
	imwrite(filename1, frameCopy1);*/
	
	//focal length, 
	//the values have been calculated using camera calibration (where is the code)
        //Important Note: if you change the location of the camera, you need to do the calibration again and provide the numbers below
   	Mat cameraMatrix0 = (Mat_<double>(3,3) << 533.81069575904655, 0., 319.40538920267261, 0., 534.40168496974275, 240.22107451961429, 0., 0., 1.);
	Mat cameraMatrix1 = (Mat_<double>(3,3) << 533.81069575904655, 0., 317.90761254312912, 0., 534.40168496974275, 237.01117359875337, 0., 0., 1.);
	Mat disCoeffs0 = (Mat_<double>(1,8) << 4.3896687130449204e-02, -3.0862476046571435e-01, 0., 0., 0.,0., 0., -1.4085605754447097e+00);
	Mat disCoeffs1 = (Mat_<double>(1,8) << 8.3060503864075069e-02, -9.6114711277749051e-02, 0., 0., 0.,0., 0., 8.0954403370078909e-01);
	
	Mat undistortedcorners0;
	Mat undistortedcorners1;
        undistortPoints(corners0, undistortedcorners0, cameraMatrix0, disCoeffs0, noArray(), cameraMatrix0);
        undistortPoints(corners1, undistortedcorners1, cameraMatrix1, disCoeffs1, noArray(), cameraMatrix1);
	//cout << "corners0 = " << corners0 << endl << endl;
	//cout << "undistortedcorners0 = " << undistortedcorners0 << endl << endl;

	
// the second method
	//the rotation of the camera with respect each other using stereo calibration	
        Mat rotation = (Mat_<double>(3,3) << 9.9960511402650210e-01, 2.8006790352574761e-02, 2.2881665609795236e-03, -2.6081572560851422e-02, 9.5502381144141324e-01, -2.9537987601166910e-01, -1.0457895812107796e-02, 2.9520355565958706e-01, 9.5537714706869570e-01);
	Mat translation = (Mat_<double>(3,1) << 7.2849409036319734e-01, 7.2497179194296608e+00, -2.6589484320994005e-03);
	Mat Identity = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	Mat zeros = (Mat_<double>(3,1) << 0, 0, 0);
        Mat E0;
	Mat E1;
	hconcat(rotation,translation,E0);
	hconcat(Identity,zeros,E1);
	Mat points4D;
	Mat projectionMatrix0;
        Mat projectionMatrix1;
	projectionMatrix0 = cameraMatrix0 * E0;
	projectionMatrix1 = cameraMatrix1 * E1;
	triangulatePoints(projectionMatrix0,projectionMatrix1,undistortedcorners1,undistortedcorners0,points4D);
        /*if(sizeof points4D > 0){
	cout << "points4D = " << points4D << endl << endl;}*/
	Mat points3D0;
	Mat points3D1;
 	
	convertPointsFromHomogeneous(points4D.col(0).t(),points3D0);
	convertPointsFromHomogeneous(points4D.col(4).t(),points3D1);
	//cout << "points3D0 = " << points3D0 << endl << endl;
	//cout << "points3D1 = " << points3D1 << endl << endl;
	double distance;
	distance = norm(points3D0-points3D1) * 75.22;
	if(distance < 2000){
	cout << distance << endl;}
	
        //k++;
	
	}//if
	char key = (char)waitKey(1);
	if(key == 27) break;

 	corners0old.clear();
	corners1old.clear();
	ids1.clear();
	ids2.clear();
	corners0.clear();
	corners1.clear();
 	
        }//while

    wf1.release();
    wf2.release();
    return 0;
}//main
