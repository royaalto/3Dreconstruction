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
    int k=0;  
    int size;
    bool tingzhi = false;
    VideoCapture wf0(0);
    VideoCapture wf1(1);
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
    Mat rotation = (Mat_<double>(3,3) << -0.06535483705023792, 0.9975475718201428, 0.02505169115795639, 0.9957051923471367, 0.06684279657596925, -0.06405630709660479, -0.06557373869993879, 0.02075770945071544, -0.9976317969527995);
    Mat translation = (Mat_<double>(3,1) << 0.2133204784749352, 0.4635042067074261, 17.66904505860917);
    Mat Identity = (Mat_<double>(3,3) << -0.002426926620803571, 0.9992270351901371, 0.03923572571402227, 0.9804465502600073, 0.01009812966067936, -0.1965263083167429, -0.1967706078417928, 0.03799157699391884, -0.9797132069983984);
    Mat zeros = (Mat_<double>(3,1) << -0.9907274715117037, -5.453805782935899, 18.76053877854365);
    Mat E0;
    Mat E1;
    hconcat(rotation,translation,E0);
    hconcat(Identity,zeros,E1);
	
    Mat projectionMatrix0;
    Mat projectionMatrix1;
    projectionMatrix0 = cameraMatrix0 * E0;
    projectionMatrix1 = cameraMatrix1 * E1;

    Mat element = getStructuringElement( MORPH_CROSS, Size(3,3) );
    Mat point3dall;
    //vector<Point3f> point3dall; //3d points detected of ball
    //ax+by+cz+d=0
    Point3f A(1.4107102, -3.4164886, -12.85817);
    Point3f B(-4.563271, -0.1623369, -12.894791);
    Point3f C(1.2372987, 2.5631371, -12.551242);
    double a;
    double b;
    double c;
    double d;
    a = (B.y-A.y)*(C.z-A.z)-(B.z-A.z)*(C.y-A.y);
    b = (B.z-A.z)*(C.x-A.x)-(B.x-A.x)*(C.z-A.z);
    c = (B.x-A.x)*(C.y-A.y)-(B.y-A.y)*(C.x-A.x);
    d = -(a*A.x+b*A.y+c*A.z);
//the center of the trash bin
    Point3f trash(-1.9257398, -1.0841837, -12.717084);
    //Point3f trash(-1.8257398, -1.0, -12.717084);
    Mat trashcenter(1,1,CV_32FC3,Scalar(trash.x,trash.y,trash.z));
    Mat imagePoints0;
    Mat imagePoints1;
    Mat rvec0;
    Mat rvec1;
    Rodrigues(rotation, rvec0, noArray());
    Rodrigues(Identity, rvec1, noArray());
    projectPoints(trashcenter, rvec0, translation, cameraMatrix0, disCoeffs0, imagePoints0, noArray() );
    projectPoints(trashcenter, rvec1, zeros, cameraMatrix1, disCoeffs1, imagePoints1, noArray() );
    //cout << "imagePoints0" << imagePoints0 << endl;
    Point2f imagePoint0(imagePoints0.at<float>(0),imagePoints0.at<float>(1));
    Point2f imagePoint1(imagePoints1.at<float>(0),imagePoints1.at<float>(1));

    double x0;
    double y0;
    double z0;
    double m;
    double n;
    double p;
    double intersectionZ;
    double intersectionX;
    double intersectionY;
    double distance;
    while(wf0.grab() && wf1.grab()) {
	char key1 = (char)waitKey(1);
	if(key1 == 32) {tingzhi=true;cout << "you have pressed space" << endl;}
	Mat frame0;
        Mat frame1;
	Mat frame0HSV;
	Mat frame1HSV;
	Mat HSVframe0;
	Mat HSVframe1;
	Mat HSVframe0blue;
	Mat HSVframe1blue;
	Mat HSVframe0green;
	Mat HSVframe1green;
	wf0.retrieve(frame0);
	wf1.retrieve(frame1);	
	
       	cvtColor(frame0,frame0HSV,COLOR_BGR2HSV);
	cvtColor(frame1,frame1HSV,COLOR_BGR2HSV);


	inRange(frame0HSV,Scalar(100,100,90),Scalar(140, 255, 255),HSVframe0);	
	inRange(frame1HSV,Scalar(100,100,90),Scalar(140, 255, 255),HSVframe1);
//blue
	/*inRange(frame0HSV,Scalar(100,100,120),Scalar(140, 255, 255),HSVframe0blue);	
	inRange(frame1HSV,Scalar(100,100,120),Scalar(140, 255, 255),HSVframe1blue);
//green
	inRange(frame0HSV,Scalar(35,55,120),Scalar(57, 165, 180),HSVframe0green);	
	inRange(frame1HSV,Scalar(35,55,120),Scalar(57, 165, 180),HSVframe1green);

	add(HSVframe0blue, HSVframe0green, HSVframe0, noArray());
	add(HSVframe1blue, HSVframe1green, HSVframe1, noArray());*/

        

	//imshow("HSVframe0before",HSVframe0);
	
	/*erode( HSVframe0, HSVframe0, NULL, Point(-1,-1), 2 );
	dilate( HSVframe0, HSVframe0, NULL, Point(-1,-1), 15 );

	erode( HSVframe1, HSVframe1, NULL, Point(-1,-1), 2 );
	dilate( HSVframe1, HSVframe1, NULL, Point(-1,-1), 15 );*/

	erode( HSVframe0, HSVframe0, element);
	erode( HSVframe0, HSVframe0, element);
	dilate( HSVframe0, HSVframe0, element);
	dilate( HSVframe0, HSVframe0, element);
	dilate( HSVframe0, HSVframe0, element);
	erode( HSVframe1, HSVframe1, element);
	erode( HSVframe1, HSVframe1, element);
	dilate( HSVframe1, HSVframe1, element);
	dilate( HSVframe1, HSVframe1, element);
	dilate( HSVframe1, HSVframe1, element);

	//imshow("HSVframe0",HSVframe0);

	vector< vector<Point> > contours0;
	vector< vector<Point> > contours1;
	vector< Vec4i> hierarchy0;
	vector< Vec4i> hierarchy1;
        Mat temp0;
	Mat temp1;
	HSVframe0.copyTo(temp0);
	HSVframe1.copyTo(temp1);
	findContours(temp0,contours0,hierarchy0,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
	findContours(temp1,contours1,hierarchy1,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
	
	Point2f center0;
	Point2f center1;
	float radius0;
	float radius1;
	vector< vector< Point> >  largestcontour0;
	vector< vector< Point> >  largestcontour1;
	if(contours0.size() > 0){
	
	//largestcontour.push_back(contours.at(contours.size()-1));

	for(int w = 0; w < contours0.size(); w++){
		if( (contourArea(contours0[w]) < 100) && (contourArea(contours0[w]) > 7000) ){
		contours0.erase(contours0.begin() + w);
		}
	}

 	for(int w = 0; w < contours0.size(); w++){
		RotatedRect box = minAreaRect(contours0[w]);
		if( ((box.size.width/box.size.height) < 0.3) && ((box.size.width/box.size.height) > 2.5) ){
		contours0.erase(contours0.begin() + w);
		}
	}

	largestcontour0.push_back(contours0.at(0)); // if contours found are more than one, could use difference
	for(int w = 0; w < contours0.size(); w++){
	if (contourArea(contours0[w]) > contourArea(largestcontour0[0])){
	largestcontour0[0] = contours0[w];
	}		
	}
	}//if

	if(contours1.size() > 0){
	
	//largestcontour.push_back(contours.at(contours.size()-1));

	for(int w = 0; w < contours1.size(); w++){
		if( (contourArea(contours1[w]) < 800) && (contourArea(contours1[w]) > 10000) ){
		contours1.erase(contours1.begin() + w);
		}
	}

 	for(int w = 0; w < contours1.size(); w++){
		RotatedRect box = minAreaRect(contours1[w]);
		if( ((box.size.width/box.size.height) < 0.3) && ((box.size.width/box.size.height) > 2.5) ){
		contours1.erase(contours1.begin() + w);
		}
	}

	largestcontour1.push_back(contours1.at(0));
	for(int w = 0; w < contours1.size(); w++){
	if (contourArea(contours1[w]) > contourArea(largestcontour1[0])){
	largestcontour1[0] = contours1[w];
	}		
	}
	}//if

	if(!largestcontour0.empty()){
	minEnclosingCircle(largestcontour0[0], center0, radius0);}
	if(!largestcontour1.empty()){
	minEnclosingCircle(largestcontour1[0], center1, radius1);}
        //cout << "center0" << center0 << endl;
//draw 
  	circle(frame0,center0,10,(255,0,0),1,8);
	circle(frame0,imagePoint0,10,(255,0,0),1,8);
        imshow("frame0",frame0);
	circle(frame1,center1,10,(255,0,0),1,8);
	circle(frame1,imagePoint1,10,(255,0,0),1,8);
        imshow("frame1",frame1);

//undistort
	if( (!largestcontour0.empty()) && (!largestcontour1.empty()) ){
     
//image output

      	
	
	Mat undistortedcenter0;
	Mat undistortedcenter1;
	Mat center0new(1,1,CV_32FC2,Scalar(center0.x,center0.y));
	Mat center1new(1,1,CV_32FC2,Scalar(center1.x,center1.y));
	//cout << "center0new" << center0new << endl;
        undistortPoints(center0new, undistortedcenter0, cameraMatrix0, disCoeffs0, noArray(), cameraMatrix0);
        undistortPoints(center1new, undistortedcenter1, cameraMatrix1, disCoeffs1, noArray(), cameraMatrix1);
	//cout << "center0 = " << center0 << endl << endl;
	//cout << "center1 = " << center1 << endl << endl;
       
	//cout << "undistortedcenter0 = " << undistortedcenter0 << endl << endl;
	//cout << "undistortedcenter1 = " << undistortedcenter1 << endl << endl;
	
// the second method
	//the rotation of the camera with respect each other using stereo calibration	
       
	
	//Mat firstpoint0(1,1,CV_32FC2,Scalar(undistortedcorners0.at<float>(geshu1*8),undistortedcorners0.at<float>(geshu1*8+1)));
	//Mat firstpoint1(1,1,CV_32FC2,Scalar(undistortedcorners1.at<float>(geshu2*8),undistortedcorners1.at<float>(geshu2*8+1)));	
	//Mat secondpoint1(1,1,CV_32FC2,Scalar(undistortedcorners1.at<float>(geshu4*8),undistortedcorners1.at<float>(geshu4*8+1)));
	//Mat secondpoint0(1,1,CV_32FC2,Scalar(undistortedcorners0.at<float>(geshu3*8),undistortedcorners0.at<float>(geshu3*8+1)));

	Mat point4D;
	triangulatePoints(projectionMatrix0,projectionMatrix1,undistortedcenter0,undistortedcenter1,point4D);
	Mat point3D;
	
	convertPointsFromHomogeneous(point4D.t(),point3D);
	//cout << "point3D" << point3D << endl;
	//cout << "point3D" << point3D << endl;

//plane equation and distance abs(ax+by+cz+d)/sqrt(aa+bb+cc)
	double distancetoplane;
	distancetoplane =  (a*point3D.at<float>(0) + b*point3D.at<float>(1) + c*point3D.at<float>(2) + d)/sqrt(a*a + b*b + c*c);
	//distancetoplane = abs( a*point3D.x + b*point3D.y + c*point3D.z + d )/sqrt(a*a + b*b + c*c);
	distancetoplane = distancetoplane * 75.17;
	//cout << "distancetoplane" << distancetoplane << endl;
//line equation (x-x0)/m=(y-y0)/n=(z-z0)/p
	//cout << "point3D.at<float>(0)" << point3D.at<float>(0) << endl;
	
	/*cout << "point3dall" << point3dall << endl;
	cout << "point3dall.rows" << point3dall.rows << endl;
	cout << "point3dall.cols" << point3dall.cols << endl;
	cout << "point3dall.at<float>(0,0)" << point3dall.at<float>(0,0) << endl;
	cout << "point3dall.at<float>(0,1)" << point3dall.at<float>(0,1) << endl;
	cout << "point3dall.at<float>(0,2)" << point3dall.at<float>(0,2) << endl;
	cout << "point3dall.at<float>(point3dall.rows-1,1)" << point3dall.at<float>(point3dall.rows-1,1) << endl;*/
	
	if(distancetoplane < -150)
	{
	point3dall.push_back(point3D);
	//cout << "point3dall" << point3dall << endl;
	}

        //size = point3dall.rows;

	if(distancetoplane > -150){
		point3dall.push_back(point3D);
		
		if(tingzhi == true){
 		x0 = point3dall.at<float>(point3dall.rows-1,0);
		y0 = point3dall.at<float>(point3dall.rows-1,1);
		z0 = point3dall.at<float>(point3dall.rows-1,2);
		m = point3dall.at<float>(point3dall.rows-1,0) - point3dall.at<float>(point3dall.rows-2,0);
		n = point3dall.at<float>(point3dall.rows-1,1) - point3dall.at<float>(point3dall.rows-2,1);
		p = point3dall.at<float>(point3dall.rows-1,2) - point3dall.at<float>(point3dall.rows-2,2);
		intersectionZ = ((a*m*z0)/p - a*x0 + (b*n*z0)/p - b*y0 - d)/((a*m)/p + (b*n)/p + c);
		intersectionX = m*(intersectionZ - z0)/p + x0;
		intersectionY = n*(intersectionZ - z0)/p + y0;
		distance = sqrt((trash.x - intersectionX)*(trash.x - intersectionX) + (trash.y - intersectionY)*(trash.y - intersectionY) + (trash.z - intersectionZ)*(trash.z - intersectionZ)) * 75.17;
		cout << "point3dall.at<float>(point3dall.rows-1)" << point3dall.at<float>(point3dall.rows-1) << endl;
		cout << "point3dall.at<float>(point3dall.rows-2)" << point3dall.at<float>(point3dall.rows-2) << endl;

		if(distance < 1200){

		Mat imagePointslast0;
		Mat imagePointslast1;
		Mat imagePointslastbutone0;
		Mat imagePointslastbutone1;
		projectPoints(point3dall.row(point3dall.rows-1), rvec0, translation, cameraMatrix0, disCoeffs0, imagePointslast0, noArray() );
    		projectPoints(point3dall.row(point3dall.rows-1), rvec1, zeros, cameraMatrix1, disCoeffs1, imagePointslast1, noArray() );
		projectPoints(point3dall.row(point3dall.rows-2), rvec0, translation, cameraMatrix0, disCoeffs0, imagePointslastbutone0, noArray() );
    		projectPoints(point3dall.row(point3dall.rows-2), rvec1, zeros, cameraMatrix1, disCoeffs1, imagePointslastbutone1, noArray() );
    		//cout << "imagePoints0" << imagePoints0 << endl;
    		Point2f imagePointlast0(imagePointslast0.at<float>(0),imagePointslast0.at<float>(1));
    		Point2f imagePointlast1(imagePointslast1.at<float>(0),imagePointslast1.at<float>(1));
		Point2f imagePointlastbutone0(imagePointslastbutone0.at<float>(0),imagePointslastbutone0.at<float>(1));
		Point2f imagePointlastbutone1(imagePointslastbutone1.at<float>(0),imagePointslastbutone1.at<float>(1));
		line(frame0, imagePointlast0, imagePointlastbutone0, (135,222,189), 1, 8);
		line(frame1, imagePointlast1, imagePointlastbutone1, (135,222,189), 1, 8);

		//cout << "frame number:   " << k << endl;
                cout << "distance    " << distance << endl;
		//cout << "trash" << trash << endl;
		//cout << "point3dall.row(point3dall.rows-1)" << point3dall.row(point3dall.rows-1) << endl;
		double distance2;
		Point3f without(point3dall.at<float>(point3dall.rows-1,0), point3dall.at<float>(point3dall.rows-1,1), point3dall.at<float>(point3dall.rows-1,2) );
		distance2 = norm(trash - without) * 75.17;
		//cout << "distance without estimation" << distance2 << endl;
		}
                if(distance > 1200){
		cout << "errors occured, but has been corrected" << endl;
		x0 = point3dall.at<float>(point3dall.rows-2,0);
		y0 = point3dall.at<float>(point3dall.rows-2,1);
		z0 = point3dall.at<float>(point3dall.rows-2,2);
		m = point3dall.at<float>(point3dall.rows-2,0) - point3dall.at<float>(point3dall.rows-3,0);
		n = point3dall.at<float>(point3dall.rows-2,1) - point3dall.at<float>(point3dall.rows-3,1);
		p = point3dall.at<float>(point3dall.rows-2,2) - point3dall.at<float>(point3dall.rows-3,2);
		intersectionZ = ((a*m*z0)/p - a*x0 + (b*n*z0)/p - b*y0 - d)/((a*m)/p + (b*n)/p + c);
		intersectionX = m*(intersectionZ - z0)/p + x0;
		intersectionY = n*(intersectionZ - z0)/p + y0;
		distance = sqrt((trash.x - intersectionX)*(trash.x - intersectionX) + (trash.y - intersectionY)*(trash.y - intersectionY) + (trash.z - intersectionZ)*(trash.z - intersectionZ)) * 75.17;

		Mat imagePointssecond0;
		Mat imagePointssecond1;
		Mat imagePointsthird0;
		Mat imagePointsthird1;
		projectPoints(point3dall.row(point3dall.rows-2), rvec0, translation, cameraMatrix0, disCoeffs0, imagePointssecond0, noArray() );
    		projectPoints(point3dall.row(point3dall.rows-2), rvec1, zeros, cameraMatrix1, disCoeffs1, imagePointssecond1, noArray() );
		projectPoints(point3dall.row(point3dall.rows-3), rvec0, translation, cameraMatrix0, disCoeffs0, imagePointsthird0, noArray() );
    		projectPoints(point3dall.row(point3dall.rows-3), rvec1, zeros, cameraMatrix1, disCoeffs1, imagePointsthird1, noArray() );
    		//cout << "imagePoints0" << imagePoints0 << endl;
    		Point2f imagePointsecond0(imagePointssecond0.at<float>(0),imagePointssecond0.at<float>(1));
    		Point2f imagePointsecond1(imagePointssecond1.at<float>(0),imagePointssecond1.at<float>(1));
		Point2f imagePointthird0(imagePointsthird0.at<float>(0),imagePointsthird0.at<float>(1));
		Point2f imagePointthird1(imagePointsthird1.at<float>(0),imagePointsthird1.at<float>(1));
		line(frame0, imagePointsecond0, imagePointthird0, (135,222,189), 1, 8);
		line(frame1, imagePointsecond1, imagePointthird1, (135,222,189), 1, 8);
	
		//cout << "frame number:  " << k << endl;
		cout << "distance    " << distance << endl;
   		//cout << "trash" << trash << endl;
		//cout << "point3dall.row(point3dall.rows-2)" << point3dall.row(point3dall.rows-2) << endl;
		double distance2;
		Point3f without(point3dall.at<float>(point3dall.rows-2,0), point3dall.at<float>(point3dall.rows-2,1), point3dall.at<float>(point3dall.rows-2,2) );
		distance2 = norm(trash - without) * 75.17;
		//cout << "distance without estimation" << distance2 << endl;
		}
		
		tingzhi = false;
		
              		}
	}
	
      	 //if(key1 != 32) cout << "you should press space" << endl;
	
	//vconcat(point3dall, point3D, point3dall);
	//cout << "point3dall" << point3dall << endl;
	/*char filename0[80];
	char filename1[80];
	sprintf(filename0,"/home/irob/wf/image/experiment/0/test_%d.ppm",k);
	sprintf(filename1,"/home/irob/wf/image/experiment/1/test_%d.ppm",k);
        imwrite(filename0, frame0);
	imwrite(filename1, HSVframe0);*/
       
	k++;
	
	}//if
	char key = (char)waitKey(1);
	if(key == 27) break;

 	
 	
        }//while

    wf0.release();
    wf1.release();
    return 0;
}//main
