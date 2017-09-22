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
#include <fstream>
#include <cstdio>
stereo_calib.xml

using namespace std;
using namespace cv;



  
    

int main() {
	
    //int k=0;  
    VideoCapture wf0(0);
    VideoCapture wf1(1);
    system("./focal.sh");
    if  (!wf0.isOpened()) cout << "wf0 doesn't work" << endl;
    if  (!wf1.isOpened()) cout << "wf1 doesn't work" << endl;

    Mat cameraMatrix0 = (Mat_<double>(3,3) << 5.4485118814026600e+02, 0., 4.1443044380802519e+02, 0., 5.4576749173675739e+02, 2.2667947513730675e+02, 0., 0., 1. );
    Mat cameraMatrix1 = (Mat_<double>(3,3) << 5.4485118814026600e+02, 0., 4.1966381551959955e+02, 0., 5.4576749173675739e+02, 2.4467613116487783e+02, 0., 0., 1. );
    Mat disCoeffs0 = (Mat_<double>(1,8) << 2.5717177475599478e-03, 6.6078395753668318e-01, 0., 0., 0., 0., 0., 3.2437476241783281e+00);
    Mat disCoeffs1 = (Mat_<double>(1,8) << -1.0390001446052489e-01, 1.6405992220690535e+00, 0., 0., 0., 0., 0., 5.5402913994601883e+00);

    /*Mat rotation = (Mat_<double>(3,3) << 9.9835204581563630e-01, -5.1070530052704087e-02, 2.6172381929612028e-02, 5.4438711013037870e-02, 9.8712805101922418e-01, -1.5038163995058282e-01, -1.8155422302237496e-02, 1.5155860863416379e-01, 9.8828152304386618e-01);
    Mat translation = (Mat_<double>(3,1) << 2.1942794429470736e-01, 8.1751653423354647e+00, -8.0721822467195758e-01);
    Mat Identity = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    Mat zeros = (Mat_<double>(3,1) << 0, 0, 0);*/
    Mat rotation = (Mat_<double>(3,3) << 0.9999889675454452, 0.00359667719343658, 0.00302137395250094,-0.003694471620141019, 0.9994481614584068, 0.03301095934471421, -0.00290097687729678, -0.03302175753312678, 0.9994504229138035);
    Mat translation = (Mat_<double>(3,1) << 1.386282033588004, -0.1921256231990436, 16.33664897163355);
    Mat Identity = (Mat_<double>(3,3) << 0.9956445906387504, 0.01312586774021187, 0.09230146655314599, -0.02750153141011065, 0.9873345541450319, 0.1562502606760376, -0.08908150707017569, -0.1581081585096021, 0.9833952894491659);
    Mat zeros = (Mat_<double>(3,1) << 0.8800856600781387,-6.523823834938135,18.94003583966276);
    Mat E0;
    Mat E1;
    hconcat(rotation,translation,E0);
    hconcat(Identity,zeros,E1);
	
    Mat projectionMatrix0;
    Mat projectionMatrix1;
    projectionMatrix0 = cameraMatrix0 * E0;
    projectionMatrix1 = cameraMatrix1 * E1;

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    detectorParams->doCornerRefinement = true; // do corner refinement in markers

    Mat rotationaruco = (Mat_<float>(3,3) << 0.9852906654797197, -0.1123573525400376, 0.1287560866472986, -0.08473600967702624, -0.9755353298520266, -0.2028561778071166, 0.1483984945427273, 0.1889620214241245, -0.970706568060998);
    Mat translationaruco = (Mat_<float>(3,1) << -10.9006, -2.30396, 34.1518);
   
    while(wf0.grab() && wf1.grab()) {
	
	
	Mat frame0, frameCopy0;
        Mat frame1, frameCopy1;
	wf0.retrieve(frame0);
	wf1.retrieve(frame1);	
	
        vector< int > ids0;
        vector< vector< Point2f > > corners0old;
	vector< int > ids1;
        vector< vector< Point2f > > corners1old;

        aruco::detectMarkers(frame0, dictionary, corners0old, ids0);
	aruco::detectMarkers(frame1, dictionary, corners1old, ids1);
	
        frame0.copyTo(frameCopy0);
        frame1.copyTo(frameCopy1);
        if(ids0.size() > 0) {
	    //cout << "right camera has detedted " << ids0.size() << " markers" << endl;
            aruco::drawDetectedMarkers(frameCopy0, corners0old, ids0); 
        }
	if(ids1.size() > 0) {
	    //cout << "left camera has detedted " << ids1.size() << " markers" << endl;
            aruco::drawDetectedMarkers(frameCopy1, corners1old, ids1); 
        }
        imshow("wf1", frameCopy1);
        imshow("wf0", frameCopy0);
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
	if((ids0.size() > 0) && (ids1.size() > 0) &&(ids0.size() == ids1.size()) ){
     
//image output

      	//char filename0[80];
	//char filename1[80];
	//sprintf(filename0,"/home/irob/wf/image/0/test_%d.jpg",k);
	//sprintf(filename1,"/home/irob/wfguvcview -r 2 -d /dev/video1/image/1/test_%d.jpg",k);
        //imwrite(filename0, frameCopy0);
	//imwrite(filename1, frameCopy1);
	
	Mat undistortedcorners0;
	Mat undistortedcorners1;
        undistortPoints(corners0, undistortedcorners0, cameraMatrix0, disCoeffs0, noArray(), cameraMatrix0);
        undistortPoints(corners1, undistortedcorners1, cameraMatrix1, disCoeffs1, noArray(), cameraMatrix1);
	
// the second method
	//the rotation of the camera with respect each other using stereo calibration	
        
	int geshu1 = -1;
	int geshu2 = -1;
	int geshu3 = -1;
	int geshu4 = -1;
	int geshu5 = -1;
	int geshu6 = -1;
//the first marker
	for(int i = 0; i < ids0.size(); i++)
	{
      		if(ids0[i] == 1)
		{
			geshu1 = i;
		}
	}

	for(int i = 0; i < ids1.size(); i++)
	{
      		if(ids1[i] == 1)
		{
			geshu2 = i;
		}
	}
	
	Mat firstpoint0(1,1,CV_32FC2,Scalar(undistortedcorners0.at<float>(geshu1*8),undistortedcorners0.at<float>(geshu1*8+1)));
	Mat firstpoint1(1,1,CV_32FC2,Scalar(undistortedcorners1.at<float>(geshu2*8),undistortedcorners1.at<float>(geshu2*8+1)));
	//cout<<"firstpoint0="<<firstpoint0<<endl;
	//cout<<"firstpoint1="<<firstpoint1<<endl;
	
	
//the second marker
	for(int i = 0; i < ids0.size(); i++)
	{
      		if(ids0[i] == 49)
		{
			geshu3 = i;
		}
	}

	for(int i = 0; i < ids1.size(); i++)
	{
      		if(ids1[i] == 49)
		{
			geshu4 = i;
		}
	}
	//cout << geshu3 << endl;
	//cout << geshu4 << endl;
		
	Mat secondpoint0(1,1,CV_32FC2,Scalar(undistortedcorners0.at<float>(geshu3*8),undistortedcorners0.at<float>(geshu3*8+1)));
	Mat secondpoint1(1,1,CV_32FC2,Scalar(undistortedcorners1.at<float>(geshu4*8),undistortedcorners1.at<float>(geshu4*8+1)));
//the third marker	
	for(int i = 0; i < ids0.size(); i++)
	{
      		if(ids0[i] == 9)
		{
			geshu5 = i;
		}
	}

	for(int i = 0; i < ids1.size(); i++)
	{
      		if(ids1[i] == 9)
		{
			geshu6 = i;
		}
	}
	//cout << geshu3 << endl;
	//cout << geshu4 << endl;
		
	Mat thirdpoint0(1,1,CV_32FC2,Scalar(undistortedcorners0.at<float>(geshu5*8),undistortedcorners0.at<float>(geshu5*8+1)));
	Mat thirdpoint1(1,1,CV_32FC2,Scalar(undistortedcorners1.at<float>(geshu6*8),undistortedcorners1.at<float>(geshu6*8+1)));
	//cout<<"secondpoint1="<<secondpoint1<<endl;
	//cout<<"thirdpoint0="<<thirdpoint0<<endl;
	//cout<<"thirdpoint1="<<thirdpoint1<<endl;
	/*for(int size = 0; size<ids0.size(); size++)
	{
		cout<<"ids0="<<ids0[size] << endl << endl;
		
	}
	
	for(int size = 0; size<ids1.size(); size++)
	{
		cout<<"ids1="<<ids1[size] << endl << endl;
		
	}*/
	
	
	Mat firstpoint4D;// the first point on marker 1 
	Mat secondpoint4D;// the first point on marker 49
	Mat thirdpoint4D;
        if((geshu1 >=0) && (geshu2 >=0)){
	triangulatePoints(projectionMatrix0,projectionMatrix1,firstpoint0,firstpoint1,firstpoint4D);}
	if((geshu3 >=0) && (geshu4 >=0)){
	triangulatePoints(projectionMatrix0,projectionMatrix1,secondpoint0,secondpoint1,secondpoint4D);}
	if((geshu5 >=0) && (geshu6 >=0)){
	triangulatePoints(projectionMatrix0,projectionMatrix1,thirdpoint0,thirdpoint1,thirdpoint4D);}
       // if(sizeof points4D > 0){
	//cout << "points4D = " << points4D << endl << endl;}

	Mat firstpoint3D;
	Mat secondpoint3D;
	Mat thirdpoint3D;
	if((geshu1 >=0) && (geshu2 >=0) && (geshu3 >=0) && (geshu4 >=0) && (geshu5 >=0) && (geshu6 >=0)){
	convertPointsFromHomogeneous(firstpoint4D.t(),firstpoint3D);
	convertPointsFromHomogeneous(secondpoint4D.t(),secondpoint3D);
	convertPointsFromHomogeneous(thirdpoint4D.t(),thirdpoint3D);
	//cout << "firstpoint3D" << firstpoint3D << endl;
	//cout << "secondpoint3D" << secondpoint3D << endl;
	//cout << "thirdpoint3D" << thirdpoint3D << endl;
	double distance;
	distance = norm(firstpoint3D-secondpoint3D) * 76.17;
	cout << "distance" <<distance << endl;
	Mat firstpoint3Dwf = (Mat_<float>(3,1) << firstpoint3D.at<float>(0), firstpoint3D.at<float>(1), firstpoint3D.at<float>(2));
	Mat secondpoint3Dwf = (Mat_<float>(3,1) << secondpoint3D.at<float>(0), secondpoint3D.at<float>(1), secondpoint3D.at<float>(2));
	Mat thirdpoint3Dwf = (Mat_<float>(3,1) << thirdpoint3D.at<float>(0), thirdpoint3D.at<float>(1), thirdpoint3D.at<float>(2));
	//cout << "firstpoint3D" << firstpoint3D << endl;
	//cout << "firstpoint3Dwf" << firstpoint3Dwf << endl;
	Mat firstpoint3Daruco = rotationaruco.t() * (firstpoint3Dwf - translationaruco);
	Mat secondpoint3Daruco = rotationaruco.t() * (secondpoint3Dwf - translationaruco);
	Mat thirdpoint3Daruco = rotationaruco.t() * (thirdpoint3Dwf - translationaruco);
	//cout << "firstpoint3Daruco" << firstpoint3Daruco << endl;
   	//cout << "secondpoint3Daruco" << secondpoint3Daruco << endl;
        //cout << "thirdpoint3Daruco" << thirdpoint3Daruco << endl;
	double distancearuco;
	distancearuco = norm(firstpoint3Daruco-secondpoint3Daruco) * 76.17;
	//cout << "distancearuco" <<distancearuco << endl;
	}
	
	
	
	//cout << "points3D0 = " << points3D0 << endl << endl;
	



//cout << "points3D1 = " << points3D1 << endl << endl;
        
        //k++;
	
	}//if
	char key = (char)waitKey(1);
	if(key == 27) break;

 	
        }//while

    wf0.release();
    wf1.release();
    return 0;
}//main
