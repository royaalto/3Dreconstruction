
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
	//push the camera matrix and distortion parameter in
	//Mat cameraMatrix0 = (Mat_<double>(3,3) << 4.9017730310572176e+02, 0., 4.1597606784764656e+02, 0., 4.9008746731207685e+02, 2.2449491360583880e+02, 0., 0., 1.  );
        //Mat cameraMatrix1 = (Mat_<double>(3,3) << 4.9017730310572176e+02, 0., 4.1601674869052391e+02, 0., 4.9008746731207685e+02, 2.4421694221350097e+02, 0., 0., 1. );
        //Mat disCoeffs0 = (Mat_<double>(1,8) << 4.0602979601797561e-02, 6.1108861985519437e-02, 0., 0., 0., 0., 0., 5.8496451008821182e-01);
        //Mat disCoeffs1 = (Mat_<double>(1,8) << 6.9343482090908071e-02, -1.4221041739514378e-01, 0., 0., 0., 0., 0., 3.1985776436377272e-01);
       



	Mat cameraMatrix0 = (Mat_<double>(3,3) << 1.1385804651219248e+03, 0., 9.4447915302888919e+02, 0.,1.1415293830231926e+03, 5.0953544327216872e+02, 0., 0., 1.);
    //Mat cameraMatrix1 = (Mat_<double>(3,3) << 5.4485118814026600e+02, 0., 4.1966381551959955e+02, 0., 5.4576749173675739e+02, 2.4467613116487783e+02, 0., 0., 1. );
    Mat disCoeffs0 = (Mat_<double>(1,8) << 1.0189954033135300e-01, -3.0561461278657870e-01, 0., 0., 0.,0., 0., -2.0392038647084787e-01, 0., 0., 0., 0., 0., 0.);
    //Mat disCoeffs1 = (Mat_<double>(1,8) << -1.0390001446052489e-01, 1.6405992220690535e+00, 0., 0., 0., 0., 0., 5.5402913994601883e+00);	
	
	//push the world points in	
	vector<cv::Point3f> w_point;
	w_point.push_back(Point3f(0,0,0));//P1 mm
	w_point.push_back(Point3f(4.702673939071438,0,0));//P2
	w_point.push_back(Point3f(4.702673939071438,3.2592789676732736,0));//P3
	w_point.push_back(Point3f(0,3.2592789676732736,0));//P4
	
	for(int i=0;i<w_point.size();i++)
	{
		cout<<w_point[i]<<endl;	
	}
	
	//push the corresponding points in
	vector<Point2f> pix_point;
	pix_point.push_back(Point2f(444,264)); //P1 pixel
	pix_point.push_back(Point2f(596,271)); //P2 pixel
	pix_point.push_back(Point2f(597,378)); //P3 pixel
	pix_point.push_back(Point2f(442,370)); //P4 pixel
	
	//create the r and t matrix
	Mat rvec = Mat::zeros(3,1,CV_64FC1);
	Mat tvec = Mat::zeros(3,1,CV_64FC1);
	
	
	//get the r v
	solvePnP(w_point,pix_point,cameraMatrix0,disCoeffs0,rvec,tvec,false,CV_ITERATIVE);
	
	//transfer the rvec to r matrix
	double rm[9];
	Mat rotM(3,3,CV_64FC1,rm);
	Rodrigues(rvec, rotM);
	cout<<"Mat"<<rotM<<endl;
	cout<<"t"<<tvec<<endl;
	
	return 0;
	
}
