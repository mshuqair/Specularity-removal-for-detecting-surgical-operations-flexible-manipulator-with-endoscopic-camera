/*
 * image_processing.cpp
 *
 *  Created on: Nov 15, 2012
 *      Author: mende
 */


# include "imageprocessing.h"
# include <opencv2/legacy/legacy.hpp>
#include "std_msgs/Float32.h"

//#include "precomp.hpp"

#define _CV_SNAKE_BIG 2.e+38f
#define _CV_SNAKE_IMAGE 1
#define _CV_SNAKE_GRAD  2

extern int downH,upH, downS, upS, downV, upV;
bool SRSelection;
double dFactor; //factor used to control saturation for the specularity removal algorithm, >1 image will saturate, <1 image will appear darker
double dAlpha; //factor used to control the depth of the specularity removal 

using namespace cv;
using namespace std;

int pos0x=0;
int pos0y=0;

extern ros::Publisher tubedetect_pub_dummy;




void ImageProcessing::printDetectedPolygon()
{
	cout << "Punktliste: ";

	for (int i=0 ; i < points.size() ; i++)
	{
		cout << points[i].x << "/" << points[i].y << " | ";
	}

	cout << std::endl;
}




bool ImageProcessing::polygonSortEvalFunction (cv::Point pointi,cv::Point pointj)
{
	//Calculate distance to point 0
	int distix=pointi.x-pos0x;
	int distiy=pointi.y-pos0y;

	int distjx=pointj.x-pos0x;
	int distjy=pointj.y-pos0y;

	double disti=(double)sqrt((double)((distix*distix)+(distiy*distiy)));
	double distj=(double)sqrt((double)((distjx*distjx)+(distjy*distjy)));

	return (disti<distj);
}




ImageProcessing::ImageProcessing()
{
}


ImageProcessing::~ImageProcessing()
{
}


/* ******************************************************************************
 * Callback function for dynamic_reconfigure
 *******************************************************************************/
void ImageProcessing::configCallback(tubedetect::tubedetect_paramsConfig &config, uint32_t level)
{
	ROS_INFO("Reconfigure request : Lower H %i Upper H %i Lower S %i Upper S %i Lower V %i Upper V %i Enable SR %i SR Saturation %i SR Depth %i ", config.lower_bound_H,config.upper_bound_H, config.lower_bound_S,config.upper_bound_S, config.lower_bound_V,config.upper_bound_V,config.sr_enable,config.sr_saturation,config.sr_depth);
	downH=config.lower_bound_H;
	upH=config.upper_bound_H;
	downS=config.lower_bound_S;
	upS=config.upper_bound_S;
	downV=config.lower_bound_V;
	upV=config.upper_bound_V;
	SRSelection=config.sr_enable;
	dFactor=config.sr_saturation;
	dAlpha=config.sr_depth;
}


/* ******************************************************************************
 * This function colors a given pixel only
 *******************************************************************************/
void ImageProcessing::ColorPixel(Mat image, int row, int col, int r, int g, int b )
{
       image.at<Vec3b>(row, col)[0]=b;
       image.at<Vec3b>(row, col)[1]=g;
       image.at<Vec3b>(row, col)[2]=r;
}


/* ******************************************************************************
 * This function colors a given pixel only
 *******************************************************************************/
void ImageProcessing::ColorPixel(Mat image, Point p, int r, int g, int b )
{
       image.at<Vec3b>(p)[0]=b;
       image.at<Vec3b>(p)[1]=g;
       image.at<Vec3b>(p)[2]=r;

}

/* ******************************************************************************
 * This function colors a given pixel and it's neighbours.
 *******************************************************************************/
void ImageProcessing::ColorPixelBig(Mat image, Point p, int r, int g, int b )
{
       image.at<Vec3b>(p.y,p.x)[0]=b;
       image.at<Vec3b>(p.y,p.x)[1]=g;
       image.at<Vec3b>(p.y,p.x)[2]=r;

       if (p.y>1)
       {
    	   image.at<Vec3b>(p.y-1,p.x)[0]=b;
    	   image.at<Vec3b>(p.y-1,p.x)[1]=g;
    	   image.at<Vec3b>(p.y-1,p.x)[2]=r;
       }

       if (p.y<image.rows)
       {
    	   image.at<Vec3b>(p.y+1,p.x)[0]=b;
    	   image.at<Vec3b>(p.y+1,p.x)[1]=g;
    	   image.at<Vec3b>(p.y+1,p.x)[2]=r;
       }

       if (p.x>1)
       {
    	   image.at<Vec3b>(p.y,p.x-1)[0]=b;
    	   image.at<Vec3b>(p.y,p.x-1)[1]=g;
    	   image.at<Vec3b>(p.y,p.x-1)[2]=r;
       }

       if (p.y<image.cols)
       {
    	   image.at<Vec3b>(p.y,p.x+1)[0]=b;
    	   image.at<Vec3b>(p.y,p.x+1)[1]=g;
    	   image.at<Vec3b>(p.y,p.x+1)[2]=r;
       }
}


void ImageProcessing::drawDetectedPolygon(cv::Mat image,int r, int g, int b )
{
	//points.begin();

	cv::Point point1;
	cv::Point point2;

	point1=points[0];

	int thickness = 2;
	int lineType = 8;

	int iCounter;

	int deltaX;
	int deltaY;

	slopes.clear();
	std_msgs::Float32 slopes_msg;

	for (iCounter=1 ; iCounter < points.size()-1 ; iCounter++)
	{
		cv::line(image, point1, points[iCounter],Scalar( 0, 255, 0 ),thickness,lineType);

		deltaX=(points[iCounter].x-point1.x);
		deltaY=(points[iCounter].y-point1.y);

//		if (deltaX<0)
//			deltaX=-deltaX;

//		if (deltaY<0)
//			deltaY=-deltaY;

//		cout << deltaX << " / " << deltaY << endl;

		slopes.push_back((float)atan2((float)deltaY,(float)deltaX));

//		cout << slopes.size() << endl;
		cout << slopes.at(iCounter-1) << endl;




		slopes_msg.data=slopes.at(iCounter-1);

		tubedetect_pub_dummy.publish(slopes_msg);






//		slopes[iCounter]=atan2((float)deltaY,(float)deltaX);


		point1=points[iCounter];
	}


	slopes_msg.data=0.0;
		tubedetect_pub_dummy.publish(slopes_msg);
		slopes_msg.data=0.0;
		tubedetect_pub_dummy.publish(slopes_msg);
		slopes_msg.data=0.0;
		tubedetect_pub_dummy.publish(slopes_msg);
		slopes_msg.data=0.0;
		tubedetect_pub_dummy.publish(slopes_msg);
		slopes_msg.data=0.0;
		tubedetect_pub_dummy.publish(slopes_msg);
		slopes_msg.data=0.0;
		tubedetect_pub_dummy.publish(slopes_msg);
		slopes_msg.data=0.0;
		tubedetect_pub_dummy.publish(slopes_msg);
		slopes_msg.data=0.0;
		tubedetect_pub_dummy.publish(slopes_msg);
		slopes_msg.data=0.0;
		tubedetect_pub_dummy.publish(slopes_msg);
		slopes_msg.data=0.0;
}





cv::Point ImageProcessing::WalkTheLine(cv::Mat image, cv::Mat bw, cv::Point p1, cv::Point offset)
{
	cv::Point p2=p1+offset*20;

	LineIterator it(bw, p1, p2, 8);

	int b;
	int diameter=0;

	cv::Point pos;

	cv::Vec3b pixel;

//	cout << "total line length" << it.count << std::endl;

	pixel=(const Vec3b)*it;
	b=pixel.val[0];
	while (b==0)
	{
		++it;
		pixel=(const Vec3b)*it;
		b=pixel.val[0];
		++diameter;
	}

//	cout << "pixelvalue/diameter 1 : " << b << " / " << diameter << std::endl;

	for(int i = 0; i < it.count; i++, ++it)
	{
		pixel=(const Vec3b)*it;

		b=pixel.val[0];

		if ((image.rows==it.pos().y) || (0==it.pos().y) || (image.cols==it.pos().x) || (image.cols==it.pos().x))
		{
			pos.x=0;
			pos.y=0;
			return pos;
		}


		if (b==0)
		{
			diameter=diameter+i-1;
			break;
		}

//		cout << b << std::endl;

	}

//	cout << "pixelvalue/diameter 2 : " << b << " / "  << diameter << std::endl;

	if (diameter!=0)
	{
		LineIterator it2(bw, p1, p2, 8);

		for(int i = 0; i < diameter/2; i++, ++it2)
		{
		}

		pos=it2.pos();
	}
	else
	{
		pos.x=0;
		pos.y=0;
	}

//	cout << "diameter : " << diameter << std::endl;
//	cout << "pos" << pos.x << "/" << pos.y << std::endl;

	return pos;
}



/* ******************************************************************************
 * This function thresholds the HSV image and creates a binary image
 *******************************************************************************/
Mat ImageProcessing::GetThresholdedImage(Mat imgHSV)
{
	cv::Mat imgThresh=Mat(imgHSV.size(),CV_8UC1);
	inRange(imgHSV,cv::Scalar(downH, downS, downV), cv::Scalar(upH, upS, upV),imgThresh);

	return imgThresh;
}



/* ******************************************************************************
 * 'erodes' the given picture
 *******************************************************************************/
void ImageProcessing::Erode(cv::Mat image,int erosion_type=MORPH_RECT,int erosion_size=3)
{
	Mat erosion_element = getStructuringElement( erosion_type, Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point( erosion_size, erosion_size ) );
	cv::erode( image, image, erosion_element );
}

/* ******************************************************************************
 * 'dilates' the given picture
 *******************************************************************************/
void ImageProcessing::Dilate(cv::Mat image,int dilation_type=MORPH_RECT,int dilation_size=3)
{
	Mat dilation_element = getStructuringElement( dilation_type, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
	cv::dilate( image, image, dilation_element );
}

/* ******************************************************************************
 * This function removes specularity from image
 *******************************************************************************/
void ImageProcessing::RemoveSpecularity(Mat image)
{
	
	double adTransformMatrix [3][3] = { {1.0, -1.0/2.0, -1.0/2.0,}, {0.0, sqrt(3.0)/2.0, -sqrt(3.0)/2.0,}, {1.0/3.0, 1.0/3.0, 1.0/3.0} }; //the transformation matrix from BGR to M color space
	double adInverseTMatrix [3][3] = { {2.0/3.0, 0.0, 1.0,}, {-1.0/3.0, 1.0/sqrt(3.0), 1.0,}, {-1.0/3.0, -1.0/sqrt(3.0), 1.0} }; //the inverse transformation matrix from M to RGB color space
	double adMColorSpace [3] = {0.0, 0.0, 0.0}; //matrix for M color space values
	double adSFMColorSpace [3] = {0.0, 0.0, 0.0}; //matrix for specular free M color space
	double adBGRPixelSet [3] = {0.0, 0.0, 0.0}; //BGR values stored here
	double adSFRGBPixelSet [3] = {0.0, 0.0, 0.0}; //specular free RGB values
	double adSFBGRPixelSet [3] = {0.0, 0.0, 0.0}; //specular free BGR values
	
	Mat SFimage = Mat::zeros( image.size(), image.type() );


 	for( int y = 0; y < image.rows; y++ )
  	  {
		for( int x = 0; x < image.cols; x++ )
          	  {
			for ( int iii = 0; iii < 3; iii++) //store each value of BGR in each pixel in adBGRPixelSet array
	  			  	  { adBGRPixelSet[iii] = double(image.at<Vec3b>(y,x)[iii]);}
			
			//convert from BGR to M color space
			adMColorSpace[0] = adTransformMatrix[0][0]*adBGRPixelSet[2] + adTransformMatrix[0][1]*adBGRPixelSet[1] + adTransformMatrix[0][2]*adBGRPixelSet[0]; 
	  		adMColorSpace[1] = adTransformMatrix[1][0]*adBGRPixelSet[2] + adTransformMatrix[1][1]*adBGRPixelSet[1] + adTransformMatrix[1][2]*adBGRPixelSet[0];
	  		adMColorSpace[2] = adTransformMatrix[2][0]*adBGRPixelSet[2] + adTransformMatrix[2][1]*adBGRPixelSet[1] + adTransformMatrix[2][2]*adBGRPixelSet[0];
			
			//calculate the specular free M color space
	  		adSFMColorSpace[0] = adMColorSpace[0];
	  		adSFMColorSpace[1] = adMColorSpace[1];
	  		adSFMColorSpace[2] = dFactor * sqrt(adMColorSpace[0]*adMColorSpace[0] + adMColorSpace[1]*adMColorSpace[1]);

			//convert back from specular free M space to RGB
	  		adSFRGBPixelSet[0] = adInverseTMatrix[0][0]*adSFMColorSpace[0] + adInverseTMatrix[0][1]*adSFMColorSpace[1] + adInverseTMatrix[0][2]*adSFMColorSpace[2]; 
	  		adSFRGBPixelSet[1] = adInverseTMatrix[1][0]*adSFMColorSpace[0] + adInverseTMatrix[1][1]*adSFMColorSpace[1] + adInverseTMatrix[1][2]*adSFMColorSpace[2];
	  		adSFRGBPixelSet[2] = adInverseTMatrix[2][0]*adSFMColorSpace[0] + adInverseTMatrix[2][1]*adSFMColorSpace[1] + adInverseTMatrix[2][2]*adSFMColorSpace[2];

			//convert from RGB to BGR
	  		adSFBGRPixelSet[0] = adSFRGBPixelSet[2]; 
	  		adSFBGRPixelSet[1] = adSFRGBPixelSet[1];
	  		adSFBGRPixelSet[2] = adSFRGBPixelSet[0];


	  		for( int c = 0; c < 3; c++ )
	  			{ SFimage.at<Vec3b>(y,x)[c] = saturate_cast<uchar>(adSFBGRPixelSet[c]);}

          	  }
  	  }

	addWeighted( image, (1-dAlpha), SFimage, 1.0, 0.0, image);
}
	



cv::Mat ImageProcessing::ProcessImage(cv::Mat image)
{
		
	Mat imageHSV;
	Mat imageBW;
	Mat imageGrey;

	//Highlights Removal
	if (SRSelection)
	{ RemoveSpecularity(image); }


	//Generate HSV and Grey image
	cvtColor(image,imageHSV,CV_BGR2HSV);
	cvtColor(image,imageGrey,CV_BGR2GRAY);

	//Generate Thresholded bw image
	imageBW=GetThresholdedImage(imageHSV);

	//Closing operation
	Dilate (imageBW, MORPH_ELLIPSE, 3);
	Erode (imageBW, MORPH_ELLIPSE, 3);

	//Fill contours
	vector<vector<Point> > contours;
	//vector<Vec4i> hierarchy;
	findContours(imageBW.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);


	//Snake!!!
/*	IplImage* image2=cvCloneImage(&(IplImage)imageBW);

 	 IplImage copy = imageGrey;
	 IplImage* copy2 = &copy;

	float alpha=0.3;
	float beta=0.3;
	float gamma=0.9;
	CvSize win;
	win.width=45;
	win.height=45;
	CvTermCriteria criteria;
	criteria.max_iter=10000;
	criteria.type=CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;
	criteria.epsilon=0.2;

	cvSnakeImageLMM(imageGrey, contours, copy2, (CvPoint*)contours.data(), contours.size(), &alpha , &beta, &gamma, CV_VALUE, win, criteria,0);

//	drawContours(image, contours, -1, Scalar::all(200), 5,8);
*/

	Mat imageMasked = Mat::zeros(image.size(), image.type());
	drawContours(imageBW, contours, -1, Scalar::all(255), CV_FILLED);

	//use bw mask on visual image
	imageMasked &= image;

	//Detect tube centre
	image=DetectTubeCentre(image, imageBW, contours/*, hierarchy*/);

	if (points.size()>4)
	{
	//	cout << "No. of detected points: " << points.size() << std::endl;

		// sort list of points
		//bad hack!!
		pos0x=points[0].x;
		pos0y=points[0].y;

	//	cout << "------------------- Vor Sortierung -------------------" << std::endl;
	//	printDetectedPolygon();
	//	cout << "Sortierstartpunkt: " << pos0x << " / " << pos0y << std::endl;
//		std::sort (points.begin()+1, points.end()-1, ImageProcessing::polygonSortEvalFunction);


		pos0x=points.at(points.size()-2).x;
		pos0y=points.at(points.size()-2).y;

	//	cout << "------------------- Nach erster Sortierung -------------------" << std::endl;
	//	printDetectedPolygon();
	//	cout << "Sortierendpunkt: " << pos0x << " / " << pos0y << std::endl;

//		std::sort (points.rbegin()+1, points.rend()-1, ImageProcessing::polygonSortEvalFunction);

	//	cout << "------------------- Nach zweiter Sortierung -------------------" << std::endl;
	//	printDetectedPolygon();


		cout << std::endl << std::endl;

		drawDetectedPolygon(image,255,0,0);
	}

	cv::imshow("step2", imageBW);
//	cv::imshow("step2", image);
	cv::waitKey(1);

	return image;
}


cv::Point ImageProcessing::DetectCentrePoint(cv::Mat image, cv::Mat bw, cv::Point p1, cv::Point p2)
{
       cv::Point result(10,10);

       cv::Point Cmid;
       cv::Point Offset;

       double length;

       double actualSlope=0.0;
       double slopeChange=0.0;

       Offset.x=(p2.x-p1.x)/2;
       Offset.y=(p2.y-p1.y)/2;

//       cout << "Offset Mittelpunkt " << Offset << std::endl ;

       Cmid=p2-Offset;		// Cmid=Starting point of line

       Offset.x=(p2.y-p1.y);
       Offset.y=-(p2.x-p1.x);

       length=sqrt((double)((Offset.x*Offset.x)+(Offset.y*Offset.y)));

       Offset.x=((Offset.x*DETECTION_LINE_LENGTH)/length);
       Offset.y=((Offset.y*DETECTION_LINE_LENGTH)/length);

       actualSlope=(double)Offset.y/Offset.x;

       slopeChange=fabs(lastSlope-actualSlope);

//       cout << "Change in Slope: " << setprecision(20) << slopeChange << endl;

       if (slopeChange < 1.4)
       {
    	   line (image,Cmid+Offset, Cmid, Scalar(255,0,0),1);

    	   result=WalkTheLine(image, bw, Cmid, Offset);
       }
       else
       {
    	   result.x=0;
    	   result.y=0;
       }

       lastSlope=actualSlope;

//       cout << "result" << result.x << "/" << result.y << std::endl;


       return result;
}



double ImageProcessing::CalculateAverageAngles(vector<Point> contour)
{
	// This might be complete nonsense... have to think about it again. MM

	double AverageAngle=0.0;

	int iPointCounter=0;

    int OffsetX,OffsetY;

    double slope=0.0;

	for (iPointCounter=0 ; iPointCounter<(contour.size()-DETECTION_STEP_SIZE); iPointCounter+=DETECTION_STEP_SIZE)
	{

		OffsetY=(contour[iPointCounter].y-contour[iPointCounter+DETECTION_STEP_SIZE].y);
		OffsetX=(contour[iPointCounter].x-contour[iPointCounter+DETECTION_STEP_SIZE].x);

		if (OffsetX!=0)
		{
			slope=(double)OffsetY/(double)OffsetX;
		}


//		cout <<  "Processing Point no. : " << iPointCounter << " at Position " << contour[iPointCounter] << " Slope=" << slope << std::endl ;

		AverageAngle=AverageAngle+fabs(slope);
	}
	AverageAngle=AverageAngle/(double)((contour.size()/DETECTION_STEP_SIZE)-1);

//	cout <<  "AverageAngle : " << AverageAngle << std::endl ;

	return AverageAngle;
}





cv::Mat ImageProcessing::DetectTubeCentre (cv::Mat image, cv::Mat bw, vector<vector<Point> > contours /*, vector<Vec4i> hierarchy*/)
{
//       cout <<  "Total number of contours detected: " << contours.size() << std::endl ;

       int iContourCounter, iPointCounter;
       cv::Point centrePoint(1,1);

       int lastActivePixel=0;

       int length=0;
       int area=0;

       double AverageAngle=0.0;

       points.clear();

       for (iContourCounter=0 ; iContourCounter<contours.size(); iContourCounter++)
       {

           	 if (contours[iContourCounter].size()>MIN_CONTOUR_SIZE)
           	 {

				 length= arcLength(contours[iContourCounter],true);
				 area=contourArea(contours[iContourCounter],false);

				 //cout <<  "Area= " << area << std::endl ;
				 //cout <<  "Length= " << length << std::endl ;

				 if ((length>MIN_CONTOUR_LENGTH) && (area>MIN_CONTOUR_AREA))
				 {
					 	//cout <<  "test" << endl;

					 	AverageAngle=CalculateAverageAngles(contours[iContourCounter]);

//						cout <<  "AverageAngle before if= " << AverageAngle << " in Contour  " << iContourCounter << std::endl ;

			 			if (AverageAngle  <	MAX_AVERAGE_ANGLE_DEVIATION)
					 	{

//							cout <<  "AverageAngle = " << AverageAngle << " in Contour  " << iContourCounter << std::endl ;


							for (iPointCounter=0 ; iPointCounter<(contours[iContourCounter].size()-DETECTION_STEP_SIZE); iPointCounter+=DETECTION_STEP_SIZE)
							{
//			                            cout <<  "Processing Contour no. " << iContourCounter << " Point no. : " << iPointCounter << "Position " << contours[iContourCounter][iPointCounter] << std::endl ;

										  ColorPixelBig (image,contours[iContourCounter][iPointCounter],0,255,0);
										  ColorPixelBig (image,contours[iContourCounter][iPointCounter+DETECTION_STEP_SIZE],0,255,0);

										  centrePoint=DetectCentrePoint(image, bw, contours[iContourCounter][iPointCounter],contours[iContourCounter][iPointCounter+DETECTION_STEP_SIZE]);

										  //cout << "x/y" << centrePoint.x << " / " << centrePoint.y << std::endl;

										  if (centrePoint.x!=0)
										  {
											  ColorPixelBig (image,centrePoint,255,0,0);
											  points.push_back(centrePoint);
										  }

//										  cout << "Made it!" << std::endl;

										  lastActivePixel=iPointCounter+DETECTION_STEP_SIZE;
							}

							//Close Loop!
							centrePoint=DetectCentrePoint(image, bw, contours[iContourCounter][0],contours[iContourCounter][lastActivePixel]);

							ColorPixelBig (image,centrePoint,50,50,255);
							points.push_back(centrePoint);

					 	}
				 }
           	 }
       }

	 	//cout <<  "test2" << endl;
    return image;
}












/*F///////////////////////////////////////////////////////////////////////////////////////
//    Name:      icvSnake8uC1R
//    Purpose:
//    Context:
//    Parameters:
//               src - source image,
//               srcStep - its step in bytes,
//               roi - size of ROI,
//               pt - pointer to snake points array
//               n - size of points array,
//               alpha - pointer to coefficient of continuity energy,
//               beta - pointer to coefficient of curvature energy,
//               gamma - pointer to coefficient of image energy,
//               coeffUsage - if CV_VALUE - alpha, beta, gamma point to single value
//                            if CV_MATAY - point to arrays
//               criteria - termination criteria.
//               scheme - image energy scheme
//                         if _CV_SNAKE_IMAGE - image intensity is energy
//                         if _CV_SNAKE_GRAD  - magnitude of gradient is energy
//    Returns:
//F*/




static CvStatus icvSnake8uC1RLMM(cv::Mat image, vector<vector<Point> > contours, unsigned char *src, int srcStep, CvSize roi, CvPoint * pt, int n, float *alpha, float *beta, float *gamma, int coeffUsage, CvSize win, CvTermCriteria criteria, int scheme )
{
    int i, j, k;
    int neighbors = win.height * win.width;

    int centerx = win.width >> 1;
    int centery = win.height >> 1;

    float invn;
    int iteration = 0;
    int converged = 0;


    float *Econt;
    float *Ecurv;
    float *Eimg;
    float *E;

    float _alpha, _beta, _gamma;

    // #ifdef GRAD_SNAKE
    float *gradient = NULL;
    uchar *map = NULL;
    int map_width = ((roi.width - 1) >> 3) + 1;
    int map_height = ((roi.height - 1) >> 3) + 1;
    #define WTILE_SIZE 8
    #define TILE_SIZE (WTILE_SIZE + 2)
    short dx[TILE_SIZE*TILE_SIZE], dy[TILE_SIZE*TILE_SIZE];
    CvMat _dx = cvMat( TILE_SIZE, TILE_SIZE, CV_16SC1, dx );
    CvMat _dy = cvMat( TILE_SIZE, TILE_SIZE, CV_16SC1, dy );
    CvMat _src = cvMat( roi.height, roi.width, CV_8UC1, src );
    cv::Ptr<cv::FilterEngine> pX, pY;

    // inner buffer of convolution process
    //char ConvBuffer[400];

    /*#endif */


    // check bad arguments
    if( src == NULL )
        return CV_NULLPTR_ERR;
    if( (roi.height <= 0) || (roi.width <= 0) )
        return CV_BADSIZE_ERR;
    if( srcStep < roi.width )
        return CV_BADSIZE_ERR;
    if( pt == NULL )
        return CV_NULLPTR_ERR;
    if( n < 3 )
        return CV_BADSIZE_ERR;
    if( alpha == NULL )
        return CV_NULLPTR_ERR;
    if( beta == NULL )
        return CV_NULLPTR_ERR;
    if( gamma == NULL )
        return CV_NULLPTR_ERR;
    if( coeffUsage != CV_VALUE && coeffUsage != CV_ARRAY )
        return CV_BADFLAG_ERR;
    if( (win.height <= 0) || (!(win.height & 1)))
        return CV_BADSIZE_ERR;
    if( (win.width <= 0) || (!(win.width & 1)))
        return CV_BADSIZE_ERR;

    invn = 1 / ((float) n);

    if( scheme == _CV_SNAKE_GRAD )
    {
        pX = cv::createDerivFilter( CV_8U, CV_16S, 1, 0, 3, cv::BORDER_REPLICATE );
        pY = cv::createDerivFilter( CV_8U, CV_16S, 0, 1, 3, cv::BORDER_REPLICATE );
        gradient = (float *) cvAlloc( roi.height * roi.width * sizeof( float ));

        map = (uchar *) cvAlloc( map_width * map_height );
        // clear map - no gradient computed
        memset( (void *) map, 0, map_width * map_height );
    }
    Econt = (float *) cvAlloc( neighbors * sizeof( float ));
    Ecurv = (float *) cvAlloc( neighbors * sizeof( float ));
    Eimg = (float *) cvAlloc( neighbors * sizeof( float ));
    E = (float *) cvAlloc( neighbors * sizeof( float ));

    while( !converged )
    {
        float ave_d = 0;
        int moved = 0;

        converged = 0;
        iteration++;
        // compute average distance
        for( i = 1; i < n; i++ )
        {
            int diffx = pt[i - 1].x - pt[i].x;
            int diffy = pt[i - 1].y - pt[i].y;

            ave_d += cvSqrt( (float) (diffx * diffx + diffy * diffy) );
        }
        ave_d += cvSqrt( (float) ((pt[0].x - pt[n - 1].x) *
                                  (pt[0].x - pt[n - 1].x) +
                                  (pt[0].y - pt[n - 1].y) * (pt[0].y - pt[n - 1].y)));

        ave_d *= invn;
        //average distance computed
        for( i = 0; i < n; i++ )
        {
            // Calculate Econt
            float maxEcont = 0;
            float maxEcurv = 0;
            float maxEimg = 0;
            float minEcont = _CV_SNAKE_BIG;
            float minEcurv = _CV_SNAKE_BIG;
            float minEimg = _CV_SNAKE_BIG;
            float Emin = _CV_SNAKE_BIG;

            int offsetx = 0;
            int offsety = 0;
            float tmp;

            // compute bounds
            int left = MIN( pt[i].x, win.width >> 1 );
            int right = MIN( roi.width - 1 - pt[i].x, win.width >> 1 );
            int upper = MIN( pt[i].y, win.height >> 1 );
            int bottom = MIN( roi.height - 1 - pt[i].y, win.height >> 1 );

            maxEcont = 0;
            minEcont = _CV_SNAKE_BIG;
            for( j = -upper; j <= bottom; j++ )
            {
                for( k = -left; k <= right; k++ )
                {
                    int diffx, diffy;
                    float energy;

                    if( i == 0 )
                    {
                        diffx = pt[n - 1].x - (pt[i].x + k);
                        diffy = pt[n - 1].y - (pt[i].y + j);
                    }
                    else
                    {
                        diffx = pt[i - 1].x - (pt[i].x + k);
                        diffy = pt[i - 1].y - (pt[i].y + j);
                    }
                    Econt[(j + centery) * win.width + k + centerx] = energy =
                        (float) fabs( ave_d -
                                      cvSqrt( (float) (diffx * diffx + diffy * diffy) ));

                    maxEcont = MAX( maxEcont, energy );
                    minEcont = MIN( minEcont, energy );
                }
            }
            tmp = maxEcont - minEcont;
            tmp = (tmp == 0) ? 0 : (1 / tmp);
             for( k = 0; k < neighbors; k++ )
            {
                Econt[k] = (Econt[k] - minEcont) * tmp;
            }

            //  Calculate Ecurv
            maxEcurv = 0;
            minEcurv = _CV_SNAKE_BIG;
            for( j = -upper; j <= bottom; j++ )
            {
                for( k = -left; k <= right; k++ )
                {
                    int tx, ty;
                    float energy;

                    if( i == 0 )
                    {
                        tx = pt[n - 1].x - 2 * (pt[i].x + k) + pt[i + 1].x;
                        ty = pt[n - 1].y - 2 * (pt[i].y + j) + pt[i + 1].y;
                    }
                    else if( i == n - 1 )
                    {
                        tx = pt[i - 1].x - 2 * (pt[i].x + k) + pt[0].x;
                        ty = pt[i - 1].y - 2 * (pt[i].y + j) + pt[0].y;
                    }
                    else
                    {
                        tx = pt[i - 1].x - 2 * (pt[i].x + k) + pt[i + 1].x;
                        ty = pt[i - 1].y - 2 * (pt[i].y + j) + pt[i + 1].y;
                    }
                    Ecurv[(j + centery) * win.width + k + centerx] = energy =
                        (float) (tx * tx + ty * ty);
                    maxEcurv = MAX( maxEcurv, energy );
                    minEcurv = MIN( minEcurv, energy );
                }
            }
            tmp = maxEcurv - minEcurv;
            tmp = (tmp == 0) ? 0 : (1 / tmp);
            for( k = 0; k < neighbors; k++ )
            {
                Ecurv[k] = (Ecurv[k] - minEcurv) * tmp;
            }

            // Calculate Eimg
            for( j = -upper; j <= bottom; j++ )
            {
                for( k = -left; k <= right; k++ )
                {
                    float energy;

                    if( scheme == _CV_SNAKE_GRAD )
                    {
                        // look at map and check status
                        int x = (pt[i].x + k)/WTILE_SIZE;
                        int y = (pt[i].y + j)/WTILE_SIZE;

                        if( map[y * map_width + x] == 0 )
                        {
                            int l, m;

                            // evaluate block location
                            int upshift = y ? 1 : 0;
                            int leftshift = x ? 1 : 0;
                            int bottomshift = MIN( 1, roi.height - (y + 1)*WTILE_SIZE );
                            int rightshift = MIN( 1, roi.width - (x + 1)*WTILE_SIZE );


//                            CvRect g_roi(x*WTILE_SIZE - leftshift, y*WTILE_SIZE - upshift, leftshift + WTILE_SIZE + rightshift, upshift + WTILE_SIZE + bottomshift);
                            cv::Rect g_roi(x*WTILE_SIZE - leftshift, y*WTILE_SIZE - upshift, leftshift + WTILE_SIZE + rightshift, upshift + WTILE_SIZE + bottomshift);

//                            cv::Mat _src1;
                            CvMat _src1;


                            cvGetSubArr( &_src, &_src1, g_roi );

                            cv::Mat _src_ = cv::cvarrToMat(&_src1);
                            cv::Mat _dx_ = cv::cvarrToMat(&_dx);
                            cv::Mat _dy_ = cv::cvarrToMat(&_dy);

                            pX->apply( _src_, _dx_, cv::Rect(0,0,-1,-1), cv::Point(), true );
                            pY->apply( _src_, _dy_, cv::Rect(0,0,-1,-1), cv::Point(), true );

                            for( l = 0; l < WTILE_SIZE + bottomshift; l++ )
                            {
                                for( m = 0; m < WTILE_SIZE + rightshift; m++ )
                                {
                                    gradient[(y*WTILE_SIZE + l) * roi.width + x*WTILE_SIZE + m] =
                                        (float) (dx[(l + upshift) * TILE_SIZE + m + leftshift] *
                                                 dx[(l + upshift) * TILE_SIZE + m + leftshift] +
                                                 dy[(l + upshift) * TILE_SIZE + m + leftshift] *
                                                 dy[(l + upshift) * TILE_SIZE + m + leftshift]);
                                }
                            }
                            map[y * map_width + x] = 1;
                        }
                        Eimg[(j + centery) * win.width + k + centerx] = energy =
                            gradient[(pt[i].y + j) * roi.width + pt[i].x + k];
                    }
                    else
                    {
                        Eimg[(j + centery) * win.width + k + centerx] = energy =
                            src[(pt[i].y + j) * srcStep + pt[i].x + k];
                    }

                    maxEimg = MAX( maxEimg, energy );
                    minEimg = MIN( minEimg, energy );
                }
            }

            tmp = (maxEimg - minEimg);
            tmp = (tmp == 0) ? 0 : (1 / tmp);

            for( k = 0; k < neighbors; k++ )
            {
                Eimg[k] = (minEimg - Eimg[k]) * tmp;
            }

            // locate coefficients
            if( coeffUsage == CV_VALUE)
            {
                _alpha = *alpha;
                _beta = *beta;
                _gamma = *gamma;
            }
            else
            {
                _alpha = alpha[i];
                _beta = beta[i];
                _gamma = gamma[i];
            }

            // Find Minimize point in the neighbors
            for( k = 0; k < neighbors; k++ )
            {
                E[k] = _alpha * Econt[k] + _beta * Ecurv[k] + _gamma * Eimg[k];
            }
            Emin = _CV_SNAKE_BIG;
            for( j = -upper; j <= bottom; j++ )
            {
                for( k = -left; k <= right; k++ )
                {

                    if( E[(j + centery) * win.width + k + centerx] < Emin )
                    {
                        Emin = E[(j + centery) * win.width + k + centerx];
                        offsetx = k;
                        offsety = j;
                    }
                }
            }

            if( offsetx || offsety )
            {
                pt[i].x += offsetx;
                pt[i].y += offsety;
                moved++;
            }
        }




        drawContours(image, contours, 2, Scalar::all(200), 5,8);


/*        cv::Point2d ptnew;
        ptnew.x=pt->x;
        ptnew.y=pt->y;

        cv::circle(image,ptnew,300,Scalar::all(200),7);

        cv::imshow("view5", image);

*/






        converged = (moved == 0);
        if( (criteria.type & CV_TERMCRIT_ITER) && (iteration >= criteria.max_iter) )
            converged = 1;
        if( (criteria.type & CV_TERMCRIT_EPS) && (moved <= criteria.epsilon) )
            converged = 1;
    }

    cvFree( &Econt );
    cvFree( &Ecurv );
    cvFree( &Eimg );
    cvFree( &E );

    if( scheme == _CV_SNAKE_GRAD )
    {
        cvFree( &gradient );
        cvFree( &map );
    }
    return CV_OK;
}






void ImageProcessing::cvSnakeImageLMM( cv::Mat image,vector<vector<Point> > contours, const IplImage* src, CvPoint* points,
              int length, float *alpha,
              float *beta, float *gamma,
              int coeffUsage, CvSize win,
              CvTermCriteria criteria, int calcGradient )
{
    uchar *data;
    CvSize size;
    int step;

    if( src->nChannels != 1 )
        CV_Error( CV_BadNumChannels, "input image has more than one channel" );

    if( src->depth != IPL_DEPTH_8U )
        CV_Error( CV_BadDepth, "Unsupported format" );

    cvGetRawData( src, &data, &step, &size );

    IPPI_CALL( icvSnake8uC1RLMM(image, contours, data, step, size, points, length, alpha, beta, gamma, coeffUsage, win, criteria, calcGradient ? _CV_SNAKE_GRAD : _CV_SNAKE_IMAGE ));
}


