
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
       // Mat cameraMatrix1 = (Mat_<double>(3,3) << 4.9017730310572176e+02, 0., 4.1601674869052391e+02, 0., 4.9008746731207685e+02, 2.4421694221350097e+02, 0., 0., 1. );
        //Mat disCoeffs0 = (Mat_<double>(1,8) << 4.0602979601797561e-02, 6.1108861985519437e-02, 0., 0., 0., 0., 0., 5.8496451008821182e-01);
        //Mat disCoeffs1 = (Mat_<double>(1,8) << 6.9343482090908071e-02, -1.4221041739514378e-01, 0., 0., 0., 0., 0., 3.1985776436377272e-01);



	//Mat cameraMatrix0 = (Mat_<double>(3,3) << 5.4485118814026600e+02, 0., 4.1443044380802519e+02, 0., 5.4576749173675739e+02, 2.2667947513730675e+02, 0., 0., 1. );
    Mat cameraMatrix1 = (Mat_<double>(3,3) << 1.1385804651219248e+03, 0., 9.3151178884618059e+02, 0.,1.1415293830231926e+03, 5.4845040623462990e+02, 0., 0., 1.);
   // Mat disCoeffs0 = (Mat_<double>(1,8) << 2.5717177475599478e-03, 6.6078395753668318e-01, 0., 0., 0., 0., 0., 3.2437476241783281e+00);
    Mat disCoeffs1 = (Mat_<double>(1,8) << 8.4786644144057857e-02, -3.6547579878081382e-01, 0., 0., 0.,0., 0., -8.7829043259283324e-01, 0., 0., 0., 0., 0., 0.);
	
	
	
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
	pix_point.push_back(Point2f(423,80)); //P1 pixel
	pix_point.push_back(Point2f(562,71)); //P2 pixel
	pix_point.push_back(Point2f(571,162)); //P3 pixel
	pix_point.push_back(Point2f(427,170)); //P4 pixel
	
	//create the r and t matrix
	Mat rvec = Mat::zeros(3,1,CV_64FC1);
	Mat tvec = Mat::zeros(3,1,CV_64FC1);
	
	
	//get the r v
	solvePnP(w_point,pix_point,cameraMatrix1,disCoeffs1,rvec,tvec,false,CV_ITERATIVE);
	
	//transfer the rvec to r matrix
	double rm[9];
	Mat rotM(3,3,CV_64FC1,rm);
	Rodrigues(rvec, rotM);
	cout<<"Mat"<<rotM<<endl;
	cout<<"t"<<tvec<<endl;
	
	return 0;
	
}
