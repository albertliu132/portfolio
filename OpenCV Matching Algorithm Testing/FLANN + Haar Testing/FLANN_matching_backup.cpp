//#define DIAGONAL_CHECK

#include <stdio.h>
#include <iostream>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

int main( int argc, char** argv )
{
	if( argc != 2 ){ 
		std::cout<<"Unexpected number of arguments, expected one image file" << std::endl; 
		return -1; 
	}

  	Mat RaisenBran = imread("TestImages/RaisenBran.jpg", IMREAD_GRAYSCALE );
	Mat RiceKrispies = imread("TestImages/RiceKrispies.jpg", IMREAD_GRAYSCALE);
	Mat CinnamonToastCrunch = imread("TestImages/CinnamonToastCrunch.jpg", IMREAD_GRAYSCALE);
  	Mat img_2 = imread(argv[1], IMREAD_GRAYSCALE);

  	if( !RaisenBran.data || !RiceKrispies.data || !CinnamonToastCrunch.data || !img_2.data ){
	       	std::cout<< " --(!) Error reading images " << std::endl;
	       	return -1; 
	}

//-- Detecting keypoints and descriptors of test image and template images
	int minHessian = 400;
	Ptr<SURF> detector = SURF::create(minHessian);
	std::vector<KeyPoint> keypointsTestImage, keypointsRaisen, keypointsRice, keypointsCinnamon;
	detector->detect(img_2, keypointsTestImage);
	detector->detect(RaisenBran, keypointsRaisen);
	detector->detect(RiceKrispies, keypointsRice);
	detector->detect(CinnamonToastCrunch, keypointsCinnamon);
	
	Ptr<SURF> extractor = SURF::create();
	Mat descriptorTestImage; 
	Mat descriptorRaisen; 
	Mat descriptorRice; 
	Mat descriptorCinnamon;
	extractor->compute(img_2, keypointsTestImage, descriptorTestImage);
	extractor->compute(RaisenBran, keypointsRaisen, descriptorRaisen);
	extractor->compute(RiceKrispies, keypointsRice, descriptorRice);
	extractor->compute(CinnamonToastCrunch, keypointsCinnamon, descriptorCinnamon);

//-- Identifying which image it best matches
	int notEnoughRaisenMatches = 0;
	int notEnoughRiceMatches = 0;
	int notEnoughCinnamonMatches = 0;
//-- Testing Raisen Bran First	
//-- Step 1: Matching descriptor vectors using FLANN matcher
  	FlannBasedMatcher matcher;
  	std::vector<DMatch> matches;
  	matcher.match(descriptorRaisen, descriptorTestImage, matches );

  	double max_dist = 0; double min_dist = 100;
//-- Quick calculation of max and min distances between keypoints
  	for( int i = 0; i < descriptorRaisen.rows; i++ ){ 
		double dist = matches[i].distance;
    		if( dist < min_dist ) min_dist = dist;
    		if( dist > max_dist ) max_dist = dist;
  	}

  	printf("-- Max dist : %f \n", max_dist );
  	printf("-- Min dist : %f \n", min_dist );

//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
//-- small
//-- PS.- radiusMatch can also be used here.
  	std::vector< DMatch > good_matches;

  	for( int i = 0; i < descriptorRaisen.rows; i++ ){ 
		if( matches[i].distance <= max(2*min_dist, 0.02) ){ 
			good_matches.push_back( matches[i]); 
		}
  	}

  //-- Draw only "good" matches
  	Mat img_matches;
  	drawMatches(RaisenBran, keypointsRaisen, img_2, keypointsTestImage,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	if(good_matches.size() < 10) notEnoughRaisenMatches = 1;

//-- Localise the object
	std::vector<Point2f>obj;
	std::vector<Point2f>scene;

	for(int i = 0; i<good_matches.size(); i++){
		//-- Get the keypoints from the good matches
    		obj.push_back(keypointsRaisen[good_matches[i].queryIdx].pt);
    		scene.push_back(keypointsTestImage[good_matches[i].trainIdx].pt);
	}

//-- Get homography matrix
	Mat homographyMatrix = findHomography(obj,scene,RANSAC);

//-- Get the corners from the image_1 ( the object to be "detected" )
	std::vector<Point2f> obj_corners(4);
  	obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint(RaisenBran.cols, 0);
  	obj_corners[2] = cvPoint(RaisenBran.cols, RaisenBran.rows); obj_corners[3] = cvPoint(0, RaisenBran.rows);
  	std::vector<Point2f> scene_corners(4);

  	perspectiveTransform( obj_corners, scene_corners, homographyMatrix);

//-- Draw lines between the corners (the mapped object in the scene - image_2 )
  	line( img_matches, scene_corners[0] + Point2f( RaisenBran.cols, 0), scene_corners[1] + Point2f( RaisenBran.cols, 0), Scalar(0, 255, 0), 4 );
  	line( img_matches, scene_corners[1] + Point2f( RaisenBran.cols, 0), scene_corners[2] + Point2f( RaisenBran.cols, 0), Scalar( 0, 255, 0), 4 );
  	line( img_matches, scene_corners[2] + Point2f( RaisenBran.cols, 0), scene_corners[3] + Point2f(RaisenBran.cols, 0), Scalar( 0, 255, 0), 4 );
  	line( img_matches, scene_corners[3] + Point2f( RaisenBran.cols, 0), scene_corners[0] + Point2f( RaisenBran.cols, 0), Scalar( 0, 255, 0), 4 );

//-- Show detected matches
  	imshow( "Good Matches & Homography Calculation", img_matches );

  	for( int i = 0; i < (int)good_matches.size(); i++ ){ 
		printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
	}

	std::cout << "Homography Matrix = " << endl << " " << homographyMatrix << endl << endl;

//-- Try and determine the validity of a match through homography matrix analysis
	int DCRFlag = 1;
	//-- Check determinant of the matrix to see if its too close to zero
	double HMRDeterminant = determinant(homographyMatrix);
	int HMRDFlag;
	if (HMRDeterminant>0.1) HMRDFlag = 1;
	else HMRDFlag = 0;
	std::cout << "Determinant of Raisen Bran matrix is: " << HMRDeterminant << endl;

	//-- DO SVD on homography matrix and check its values
	Mat singularValues = Mat();
	Mat U, Vt = Mat();
	int CNRFlag = 0;
	SVDecomp(homographyMatrix, singularValues, U, Vt, 2);

	std::cout << "Printing the singular values of the homography matrix: " << endl;
	Size s = singularValues.size();
	int rows = s.height;
	int cols = s.width;
	for (int i = 0; i < rows; i++)
		for(int j = 0; j < cols; j++){
			std::cout << "Element at " << i << " and " << j << " is " << singularValues.at<double>(i,j) << endl;
		}
	double conditionNumber = singularValues.at<double>(0,0)/singularValues.at<double>(2,0);
	std::cout << "Condition number is: " << conditionNumber << endl;
	if(conditionNumber <= 10000000){
		std::cout << "Condition number check passed" << endl;
		CNRFlag = 1;
	}
	
	//-- Compare absolute values of rotation portion of homography matrix
	#ifdef DIAGONAL_CHECK
	double mainDiagComp;
	double offDiagComp;
	DCRFlag = 0;		
	mainDiagComp = abs((abs(homographyMatrix.at<double>(0,0))-abs(homographyMatrix.at<double>(1,1))));
	offDiagComp = abs((abs(homographyMatrix.at<double>(0,1))-abs(homographyMatrix.at<double>(1,0))));
	if(mainDiagComp <= 0.1){
		if(offDiagComp <= 0.1){
			std::cout << "Homography Matrix is bad" << endl;
		}
	}
	else{ 
		std::cout<< "Homography matrix passes diagonal check" << endl;
		DCRFlag = 1;
	}
	std::cout << "Main diagonal comparison is: " << mainDiagComp << " Off diagonal comparison is : " << offDiagComp << endl;
	#endif
	
//-- Testing Rice Krispies 	
//-- Step 1: Matching descriptor vectors using FLANN matcher
  	//FlannBasedMatcher matcher;
  	//std::vector<DMatch> matches;
  	matcher.match(descriptorRice, descriptorTestImage, matches );

  	max_dist = 0; 
	min_dist = 100;
//-- Quick calculation of max and min distances between keypoints
  	for( int i = 0; i < descriptorRice.rows; i++ ){ 
		double dist = matches[i].distance;
    		if( dist < min_dist ) min_dist = dist;
    		if( dist > max_dist ) max_dist = dist;
  	}

  	printf("-- Max dist : %f \n", max_dist );
  	printf("-- Min dist : %f \n", min_dist );

//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
//-- small
//-- PS.- radiusMatch can also be used here.
  	//std::vector< DMatch > good_matches;

  	for( int i = 0; i < descriptorRice.rows; i++ ){ 
		if( matches[i].distance <= max(2*min_dist, 0.02) ){ 
			good_matches.push_back( matches[i]); 
		}
  	}

  //-- Draw only "good" matches
  	//Mat img_matches;
  	drawMatches(RiceKrispies, keypointsRice, img_2, keypointsTestImage,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	if(good_matches.size() < 10) notEnoughRiceMatches = 1;

//-- Localise the object
	//std::vector<Point2f>obj;
	//std::vector<Point2f>scene;

	for(int i = 0; i<good_matches.size(); i++){
		//-- Get the keypoints from the good matches
    		obj.push_back(keypointsRice[good_matches[i].queryIdx].pt);
    		scene.push_back(keypointsTestImage[good_matches[i].trainIdx].pt);
	}

//-- Get homography matrix
	homographyMatrix = findHomography(obj,scene,RANSAC);

//-- Get the corners from the image_1 ( the object to be "detected" )
	//std::vector<Point2f> obj_corners(4);
  	obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint(RiceKrispies.cols, 0);
  	obj_corners[2] = cvPoint(RiceKrispies.cols, RiceKrispies.rows); obj_corners[3] = cvPoint(0, RiceKrispies.rows);
  	//std::vector<Point2f> scene_corners(4);

  	perspectiveTransform( obj_corners, scene_corners, homographyMatrix);

//-- Draw lines between the corners (the mapped object in the scene - image_2 )
  	line( img_matches, scene_corners[0] + Point2f( RiceKrispies.cols, 0), scene_corners[1] + Point2f( RiceKrispies.cols, 0), Scalar(0, 255, 0), 4 );
  	line( img_matches, scene_corners[1] + Point2f( RiceKrispies.cols, 0), scene_corners[2] + Point2f( RiceKrispies.cols, 0), Scalar( 0, 255, 0), 4 );
  	line( img_matches, scene_corners[2] + Point2f( RiceKrispies.cols, 0), scene_corners[3] + Point2f(RiceKrispies.cols, 0), Scalar( 0, 255, 0), 4 );
  	line( img_matches, scene_corners[3] + Point2f( RiceKrispies.cols, 0), scene_corners[0] + Point2f( RiceKrispies.cols, 0), Scalar( 0, 255, 0), 4 );

//-- Show detected matches
  	imshow( "Good Matches & Homography Calculation", img_matches );

  	for( int i = 0; i < (int)good_matches.size(); i++ ){ 
		printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
	}

	std::cout << "Homography Matrix = " << endl << " " << homographyMatrix << endl << endl;

//-- Try and determine the validity of a match through homography matrix analysis
	int DCRKFlag = 1;
	//-- Check determinant of the matrix to see if its too close to zero
	double HMRKDeterminant = determinant(homographyMatrix);
	int HMRKDFlag;
	if (HMRKDeterminant>0.1) HMRKDFlag = 1;
	else HMRKDFlag = 0;
	std::cout << "Determinant of Rice Krispies matrix is: " << HMRKDeterminant << endl;

	//-- DO SVD on homography matrix and check its values
	singularValues = Mat();
	U, Vt = Mat();
	int CNRKFlag = 0;
	SVDecomp(homographyMatrix, singularValues, U, Vt, 2);

	std::cout << "Printing the singular values of the homography matrix: " << endl;
	s = singularValues.size();
	rows = s.height;
	cols = s.width;
	for (int i = 0; i < rows; i++)
		for(int j = 0; j < cols; j++){
			std::cout << "Element at " << i << " and " << j << " is " << singularValues.at<double>(i,j) << endl;
		}
	conditionNumber = singularValues.at<double>(0,0)/singularValues.at<double>(2,0);
	std::cout << "Condition number is: " << conditionNumber << endl;
	if(conditionNumber <= 10000000){
		std::cout << "Condition number check passed" << endl;
		CNRKFlag = 1;
	}
	
	//-- Compare absolute values of rotation portion of homography matrix
	#ifdef DIAGONAL_CHECK
	double mainDiagComp;
	double offDiagComp;
	DCRKFlag = 0;		
	mainDiagComp = abs((abs(homographyMatrix.at<double>(0,0))-abs(homographyMatrix.at<double>(1,1))));
	offDiagComp = abs((abs(homographyMatrix.at<double>(0,1))-abs(homographyMatrix.at<double>(1,0))));
	if(mainDiagComp <= 0.1){
		if(offDiagComp <= 0.1){
			std::cout << "Homography Matrix is bad" << endl;
		}
	}
	else{ 
		std::cout<< "Homography matrix passes diagonal check" << endl;
		DCRKFlag = 1;
	}
	std::cout << "Main diagonal comparison is: " << mainDiagComp << " Off diagonal comparison is : " << offDiagComp << endl;
	#endif
	
//-- Testing Cinnamon Toast Crunch 	
//-- Step 1: Matching descriptor vectors using FLANN matcher
  	//FlannBasedMatcher matcher;
  	//std::vector<DMatch> matches;
  	matcher.match(descriptorCinnamon, descriptorTestImage, matches );

  	max_dist = 0; 
	min_dist = 100;
//-- Quick calculation of max and min distances between keypoints
  	for( int i = 0; i < descriptorCinnamon.rows; i++ ){ 
		double dist = matches[i].distance;
    		if( dist < min_dist ) min_dist = dist;
    		if( dist > max_dist ) max_dist = dist;
  	}

  	printf("-- Max dist : %f \n", max_dist );
  	printf("-- Min dist : %f \n", min_dist );

//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
//-- small
//-- PS.- radiusMatch can also be used here.
  	//std::vector< DMatch > good_matches;
	good_matches.clear();
  	for( int i = 0; i < descriptorCinnamon.rows; i++ ){ 
		if( matches[i].distance <= max(2*min_dist, 0.02) ){ 
			good_matches.push_back( matches[i]); 
		}
  	}

  //-- Draw only "good" matches
  	//Mat img_matches;
  	drawMatches(CinnamonToastCrunch, keypointsCinnamon, img_2, keypointsTestImage,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	
	if(good_matches.size() < 10) notEnoughCinnamonMatches = 1;

//-- Localise the object
	//std::vector<Point2f>obj;
	//std::vector<Point2f>scene;

	for(int i = 0; i<good_matches.size(); i++){
		//-- Get the keypoints from the good matches
    		obj.push_back(keypointsCinnamon[good_matches[i].queryIdx].pt);
    		scene.push_back(keypointsTestImage[good_matches[i].trainIdx].pt);
	}

//-- Get homography matrix
	homographyMatrix = findHomography(obj,scene,RANSAC);

//-- Get the corners from the image_1 ( the object to be "detected" )
	//std::vector<Point2f> obj_corners(4);
  	obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint(CinnamonToastCrunch.cols, 0);
  	obj_corners[2] = cvPoint(CinnamonToastCrunch.cols, CinnamonToastCrunch.rows); obj_corners[3] = cvPoint(0, CinnamonToastCrunch.rows);
  	//std::vector<Point2f> scene_corners(4);

  	perspectiveTransform( obj_corners, scene_corners, homographyMatrix);

//-- Draw lines between the corners (the mapped object in the scene - image_2 )
  	line( img_matches, scene_corners[0] + Point2f( CinnamonToastCrunch.cols, 0), scene_corners[1] + Point2f( CinnamonToastCrunch.cols, 0), Scalar(0, 255, 0), 4 );
  	line( img_matches, scene_corners[1] + Point2f( CinnamonToastCrunch.cols, 0), scene_corners[2] + Point2f( CinnamonToastCrunch.cols, 0), Scalar( 0, 255, 0), 4 );
  	line( img_matches, scene_corners[2] + Point2f( CinnamonToastCrunch.cols, 0), scene_corners[3] + Point2f(CinnamonToastCrunch.cols, 0), Scalar( 0, 255, 0), 4 );
  	line( img_matches, scene_corners[3] + Point2f( CinnamonToastCrunch.cols, 0), scene_corners[0] + Point2f( CinnamonToastCrunch.cols, 0), Scalar( 0, 255, 0), 4 );

//-- Show detected matches
  	imshow( "Good Matches & Homography Calculation", img_matches );

  	for( int i = 0; i < (int)good_matches.size(); i++ ){ 
		printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
	}

	std::cout << "Homography Matrix = " << endl << " " << homographyMatrix << endl << endl;

//-- Try and determine the validity of a match through homography matrix analysis
	int DCCFlag = 1;
	//-- Check determinant of the matrix to see if its too close to zero
	double HMCDeterminant = determinant(homographyMatrix);
	int HMCFlag;
	if (HMCDeterminant>0.1) HMCFlag = 1;
	else HMCFlag = 0;
	std::cout << "Determinant of Cinnamon Toast Crunch matrix is: " << HMCDeterminant << endl;

	//-- DO SVD on homography matrix and check its values
	singularValues = Mat();
	U, Vt = Mat();
	int CNCFlag = 0;
	SVDecomp(homographyMatrix, singularValues, U, Vt, 2);

	std::cout << "Printing the singular values of the homography matrix: " << endl;
	s = singularValues.size();
	rows = s.height;
	cols = s.width;
	for (int i = 0; i < rows; i++)
		for(int j = 0; j < cols; j++){
			std::cout << "Element at " << i << " and " << j << " is " << singularValues.at<double>(i,j) << endl;
		}
	conditionNumber = singularValues.at<double>(0,0)/singularValues.at<double>(2,0);
	std::cout << "Condition number is: " << conditionNumber << endl;
	if(conditionNumber <= 10000000){
		std::cout << "Condition number check passed" << endl;
		CNCFlag = 1;
	}
	
	//-- Compare absolute values of rotation portion of homography matrix
	#ifdef DIAGONAL_CHECK
	double mainDiagComp;
	double offDiagComp;
	DCCFlag = 0;		
	mainDiagComp = abs((abs(homographyMatrix.at<double>(0,0))-abs(homographyMatrix.at<double>(1,1))));
	offDiagComp = abs((abs(homographyMatrix.at<double>(0,1))-abs(homographyMatrix.at<double>(1,0))));
	if(mainDiagComp <= 0.1){
		if(offDiagComp <= 0.1){
			std::cout << "Homography Matrix is bad" << endl;
		}
	}
	else{ 
		std::cout<< "Homography matrix passes diagonal check" << endl;
		DCCFlag = 1;
	}
	std::cout << "Main diagonal comparison is: " << mainDiagComp << " Off diagonal comparison is : " << offDiagComp << endl;
	#endif
	
	//-- Print homography check results
	if(HMRDFlag == 1 && CNRFlag == 1 && DCRFlag == 1 && notEnoughRaisenMatches == 0){
		std::cout << "Homography matrix is good, probable Raisen Bran match" << endl;
	}
	else std::cout << "Homography matrix is bad, no Raisen Bran match." << endl;

	if(HMRKDFlag == 1 && CNRKFlag == 1 && DCRKFlag == 1 && notEnoughRiceMatches == 0){
		std::cout << "Homography matrix is good, probable Rice Krispies match" << endl;
	}
	else std::cout << "Homography matrix is bad, no Rice Krispies match." << endl;
	
	if(HMCFlag == 1 && CNCFlag == 1 && DCCFlag == 1 && notEnoughCinnamonMatches == 0){
		std::cout << "Homography matrix is good, probable Cinnamon Toast Crunch match" << endl;
	}
	else std::cout << "Homography matrix is bad, no Cinnamon Toast Crunch match." << endl;
  	return 0;
}

