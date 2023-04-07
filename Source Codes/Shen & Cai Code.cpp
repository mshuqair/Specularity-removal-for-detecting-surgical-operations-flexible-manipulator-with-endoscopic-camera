/*
 * DisplayImage.cpp
 *
 *  Created on: Sep 20, 2013
 *      Author: mustafa
 *

#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <iostream>
#include <cmath>
#include <time.h>

using namespace cv;
using namespace std;

int minValue (int anRGBvalues[3])
{
	int nMinimum = anRGBvalues[0];
	for (int iii = 1; iii < 3; iii++)
		{
			if (nMinimum > anRGBvalues[iii]) { nMinimum = anRGBvalues[iii]; }
	    }
	    return nMinimum;
}

int main( int argc, const char** argv )
{

    clock_t tStart = clock();

	int nbeta;
	int anRGBPixelSet [3];

	Mat image = imread("/home/mustafa/workspace/BeginnerProject/Images/toys.ppm", CV_LOAD_IMAGE_UNCHANGED);
	Mat SFImage = Mat::zeros( image.size(), image.type() ); //specular free image
	Mat MSFImage = Mat::zeros( image.size(), image.type() ); // modified specular free image

	if (image.empty()) //check whether the image is loaded or not
	  {	cout << "Error : Image cannot be loaded..!!" << endl;
	  return -1;
	  }

	cout << "Image size : " << image.cols << " x " << image.rows << endl;

	const int nImagePixelsCount = image.rows * image.cols; //calculate how many pixels in the image

	int anMinPixelsValues [nImagePixelsCount]; //create an array with the size of the pixels in the image to contain the min values of RGB for each pixel
	int nCounterZ = 0;

	for( int y = 0; y < image.rows; y++ )
		{
	  	  for( int x = 0; x < image.cols; x++ )
          	  {
	  		  	  for ( int ii = 0; ii < 3; ii++) //store each value of RGB in each pixel in anPixelSet array
	  		  	  	  { anRGBPixelSet[ii] = image.at<Vec3b>(y,x)[ii];}

	  		  	  nbeta = minValue(anRGBPixelSet); //get the min of these values

	  		  	  for( int c = 0; c < 3; c++ )
	  		  	  	  { SFImage.at<Vec3b>(y,x)[c] = ( image.at<Vec3b>(y,x)[c] - nbeta );} //access each value of RGB in each pixel in the image and add "beta" to it

	  		  	  anMinPixelsValues[nCounterZ] = nbeta;
	  		  	  nCounterZ++;
          	  }
		}

	int nMeanValue = 0;  //the mean value of min values of RGB for all pixels
	int nTempSum = 0;
	for (int jjj = 0; jjj < nImagePixelsCount; jjj++)  //calculating the mean value of all min RGB values of the pixels
  	  {	nTempSum = nTempSum + anMinPixelsValues [jjj]; }
	nMeanValue = nTempSum / nImagePixelsCount;


	nTempSum = 0;
	double dMeanDeviation = 0.0;
	for (int jj = 0; jj < nImagePixelsCount; jj++)  //calculating the mean deviation for the min RGb values for all pixels
  	  {	nTempSum = nTempSum + abs(anMinPixelsValues[jj] - nMeanValue); }
	dMeanDeviation = nTempSum / nImagePixelsCount;

	nTempSum = 0;
	double dStandardDeviation = 0.0;
	for (int j = 0; j < nImagePixelsCount; j++) //calculating the standard deviation for the min RGB values for alll pixels
  	  {	nTempSum = nTempSum + ((anMinPixelsValues[j] - nMeanValue) * (anMinPixelsValues[j] - nMeanValue)); }
	dStandardDeviation = sqrt (nTempSum / nImagePixelsCount);

	double dZeta = 0.50;
	double dPixelThreshold = 0.0;
	dPixelThreshold = dMeanDeviation + dZeta * dStandardDeviation;  //calculating the threshold

	double dTau = 0.0;
	int nCounterX = 0;
	for( int y = 0; y < image.rows; y++ ) //access each pixel to compare the min RGB values to determine the Tau needs to be added
     	 { for( int x = 0; x < image.cols; x++ )
    	 {
     		 if ( anMinPixelsValues[nCounterX] > dPixelThreshold)

    	 	 	 { dTau = dPixelThreshold;	}
     		 else
     		 	 { dTau = anMinPixelsValues[nCounterX];	}

     		 for( int c = 0; c < 3; c++ )
		 	 	 { MSFImage.at<Vec3b>(y,x)[c] = saturate_cast<uchar>( SFImage.at<Vec3b>(y,x)[c] + dTau );}
     		 nCounterX++;

    	 }
     	 }


	namedWindow("Original Image", CV_WINDOW_AUTOSIZE);
	imshow("Original Image", image);

	namedWindow("Modified Specular Free Image", CV_WINDOW_AUTOSIZE);
	imshow ("Modified Specular Free Image", MSFImage);

	printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

	waitKey(0); //wait infinite time for a keypress
	destroyWindow("Original Image");
	destroyWindow("modified Specular Free Image");

	return 0;
}
*/
