/*
 * MiyazakiMethod.cpp
 *
 *  Created on: Oct 15, 2013
 *      Author: mustafa
 */

#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <iostream>
#include <cmath>
#include <time.h>

using namespace cv;
using namespace std;


int main( int argc, const char** argv )
{

	clock_t tStart = clock();

	Mat image = imread("/home/mustafa/workspace/BeginnerProject/Images/toys.ppm", CV_LOAD_IMAGE_UNCHANGED); //opencv loads the image in BGR by default
	Mat SFImage = Mat::zeros( image.size(), image.type() );
	double adTransformMatrix [3][3] = { {1.0, -1.0/2.0, -1.0/2.0,}, {0.0, sqrt(3.0)/2.0, -sqrt(3.0)/2.0,}, {1.0/3.0, 1.0/3.0, 1.0/3.0} }; //the transformation matrix from BGR to M color space
	double adInverseTMatrix [3][3] = { {2.0/3.0, 0.0, 1.0,}, {-1.0/3.0, 1.0/sqrt(3.0), 1.0,}, {-1.0/3.0, -1.0/sqrt(3.0), 1.0} }; //the inverse transformation matrix from M to RGB color space
	double adMColorSpace [3] = {0.0, 0.0, 0.0}; //matrix for M color space values
	double adSFMColorSpace [3] = {0.0, 0.0, 0.0}; //matrix for specular free M color space
	double adBGRPixelSet [3] = {0.0, 0.0, 0.0}; //BGR values stored here
	double adSFRGBPixelSet [3] = {0.0, 0.0, 0.0}; //specular free RGB values
	double adSFBGRPixelSet [3] = {0.0, 0.0, 0.0}; //specular free BGR values
	double dFactor = 1.0; //factor used to control saturation, >1 image will saturate, <1 image will appear darker

	cout << "Image size : " << image.cols << " x " << image.rows << endl;


  if (image.empty()) //check whether the image is loaded or not
	  {	cout << "Error : Image cannot be loaded..!!" << endl;
	  return -1;
	  }


  for( int y = 0; y < image.rows; y++ )
  	  {
	  	  for( int x = 0; x < image.cols; x++ )
          	  {

	  		  	  for ( int iii = 0; iii < 3; iii++) //store each value of BGR in each pixel in anPixelSet array
	  			  	  { adBGRPixelSet[iii] = double(image.at<Vec3b>(y,x)[iii]);}

	  		  	  adMColorSpace[0] = adTransformMatrix[0][0]*adBGRPixelSet[2] + adTransformMatrix[0][1]*adBGRPixelSet[1] +adTransformMatrix[0][2]*adBGRPixelSet[0]; //convert from BGR to M color space
	  		  	  adMColorSpace[1] = adTransformMatrix[1][0]*adBGRPixelSet[2] + adTransformMatrix[1][1]*adBGRPixelSet[1] +adTransformMatrix[1][2]*adBGRPixelSet[0];
	  		  	  adMColorSpace[2] = adTransformMatrix[2][0]*adBGRPixelSet[2] + adTransformMatrix[2][1]*adBGRPixelSet[1] +adTransformMatrix[2][2]*adBGRPixelSet[0];

	  		  	  adSFMColorSpace[0] = adMColorSpace[0]; //calculate the specular free M color space
	  		      adSFMColorSpace[1] = adMColorSpace[1];
	  		      adSFMColorSpace[2] = dFactor * sqrt(adMColorSpace[0]*adMColorSpace[0] + adMColorSpace[1]*adMColorSpace[1]);

	  		      adSFRGBPixelSet[0] = adInverseTMatrix[0][0]*adSFMColorSpace[0] + adInverseTMatrix[0][1]*adSFMColorSpace[1] + adInverseTMatrix[0][2]*adSFMColorSpace[2]; //convert back from specular free M space to RGB
	  		      adSFRGBPixelSet[1] = adInverseTMatrix[1][0]*adSFMColorSpace[0] + adInverseTMatrix[1][1]*adSFMColorSpace[1] + adInverseTMatrix[1][2]*adSFMColorSpace[2];
	  		      adSFRGBPixelSet[2] = adInverseTMatrix[2][0]*adSFMColorSpace[0] + adInverseTMatrix[2][1]*adSFMColorSpace[1] + adInverseTMatrix[2][2]*adSFMColorSpace[2];

	  		      adSFBGRPixelSet[0] = adSFRGBPixelSet[2]; //convert from RGB to BGR
	  		      adSFBGRPixelSet[1] = adSFRGBPixelSet[1];
	  		      adSFBGRPixelSet[2] = adSFRGBPixelSet[0];


	  		  	  for( int c = 0; c < 3; c++ )
	  		  	  	  { SFImage.at<Vec3b>(y,x)[c] = saturate_cast<uchar>(adSFBGRPixelSet[c]);}

          	  }
  	  }


  namedWindow("Original Image", CV_WINDOW_AUTOSIZE);
  imshow("Original Image", image);

  namedWindow("Specular Free Image", CV_WINDOW_AUTOSIZE);
  imshow ("Specular Free Image", SFImage);

  printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

  waitKey(0); //wait infinite time for a keypress
  destroyWindow("Original Image");
  destroyWindow("Specular Free Image");


  return 0;
}
