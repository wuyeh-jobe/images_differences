#ifndef SpotDifferences_H_
#define SpotDifferences_H_

#include <string>
#include <iostream>
#include <string.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include <cmath>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

class SpotDifferences{

public:
	static void spotImageDifferenceWithDistortion(string inputImage1, string inputImage2, string diffImagePath);
	static void alignImages(Mat &im1, Mat &im2, Mat &im1Reg, Mat &h);
	static void alignImages2(Mat &im1, Mat &im2, Mat &im1Reg);
	static void correlateImg(Mat &inImage, Mat &kernelTemplate, Mat &mask);
	static void erodeImage(Mat srcImage, int structuring_elem, int structuring_size);
	static void modifyImageForCorrelation(Mat& inImage){
//		if (true) //Do not convert to grayscale, only convert to floating point
			inImage.convertTo(inImage, CV_32FC1);
			// The image values are mean-corrected.
			inImage = inImage - mean(inImage);
			return;
	}

	static Mat getStructuringElementForMorphology(int structuring_elem, int structuring_size){
		  int morph_type;
		  if( structuring_elem == 0 ){ morph_type = MORPH_RECT; }
		  else if( structuring_elem == 1 ){ morph_type = MORPH_CROSS; }
		  else if( structuring_elem == 2) { morph_type = MORPH_ELLIPSE; }
		  else {
			  std::cout << "Invalid input type requested for morphological structuring element \n\n";
//			  return NULL;
		  }

		  if (structuring_size <= 0){
			  std::string message("Invalid size requested for morphological structuring element");
			  std::cout << message << " \n\n";
			  throw std::logic_error(message);
		  }
		  Mat element = getStructuringElement( morph_type,
											   Size( 2*structuring_size + 1, 2*structuring_size+1 ),
											   Point( structuring_size, structuring_size ) );
		  return element;
	};

};


#endif
