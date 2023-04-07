/*
 * image_processing.h
 *
 *  Created on: Nov 15, 2012
 *      Author: mende
 */

#ifndef IMAGE_PROCESSING_H_
#define IMAGE_PROCESSING_H_


#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <tubedetect/tubedetect_paramsConfig.h>

using namespace cv;
using namespace std;


const int MIN_CONTOUR_LENGTH=700;
const int MIN_CONTOUR_SIZE=400;
const int MIN_CONTOUR_AREA=40000;
const int DETECTION_LINE_LENGTH=40;
const int DETECTION_STEP_SIZE=25;
const double MAX_AVERAGE_ANGLE_DEVIATION=2.2;

//std::vector<cv::Point> points;


class ImageProcessing //: public cv::Mat
{

	public:
		ImageProcessing();
		~ImageProcessing();



		void ColorPixel(cv::Mat image, int row, int col, int r=0, int g=0, int b=0 );
		void ColorPixel(cv::Mat image, cv::Point p, int r=0, int g=0, int b=0 );
		void ColorPixelBig(cv::Mat image, cv::Point p, int r=0, int g=0, int b=0 );

		void drawDetectedPolygon(Mat image,int r, int g, int b );

		double CalculateAverageAngles(vector<Point> contours);

		cv::Mat GetThresholdedImage(cv::Mat imgHSV);
		
		void RemoveSpecularity(cv::Mat image);

		cv::Point DetectCentrePoint(cv::Mat image, cv::Mat bw, cv::Point p1, cv::Point p2);

		cv::Point WalkTheLine(cv::Mat image, cv::Mat bw, cv::Point p1, cv::Point offset);

		cv::Mat DetectTubeCentre (cv::Mat image, cv::Mat bw, vector<vector<Point> > contours);
//		cv::Mat DetectTubeCentre (cv::Mat image, vector<vector<Point> > contours, vector<Vec4i> hierarchy);

		void configCallback(tubedetect::tubedetect_paramsConfig &config, uint32_t level);


		static bool polygonSortEvalFunction (cv::Point pointi,cv::Point pointj);

		cv::Mat ProcessImage(cv::Mat image);

		std::vector<cv::Point> points;
		std::vector<float> slopes;

		void Erode(cv::Mat image,int erosion_type,int erosion_size);
		void Dilate(cv::Mat image,int dilation_type,int dilation_size);

		void printDetectedPolygon();

	private:

		double	lastSlope;	//only for internal use to track rapid changes of slope

		void cvSnakeImageLMM(cv::Mat image, vector<vector<Point> > contours,const IplImage* src, CvPoint* points, int length, float *alpha, float *beta, float *gamma, int coeffUsage, CvSize win, CvTermCriteria criteria, int calcGradient );

};

static CvStatus icvSnake8uC1RLMM(cv::Mat image,vector<vector<Point> > contours, unsigned char *src, int srcStep, CvSize roi, CvPoint * pt, int n, float *alpha, float *beta, float *gamma, int coeffUsage, CvSize win, CvTermCriteria criteria, int scheme );



#endif /* IMAGE_PROCESSING_H_ */
