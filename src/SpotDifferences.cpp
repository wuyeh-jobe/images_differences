#include "SpotDifferences.h"

void SpotDifferences::spotImageDifferenceWithDistortion(string inputImage1, string inputImage2, string diffImagePath){
	// These two lines will read the input images. Do not change these lines. 
	Mat srcImage1 = imread(inputImage1.c_str());
	Mat srcImage2 = imread(inputImage2.c_str());

	// Registered image will be resotred in imReg.
	// The estimated homography will be stored in h.
	Mat imReg, h;

	// Align images
	cout << "Aligning images ..." << endl;

//	cvtColor(srcImage1, srcImage1, COLOR_BGR2HSV);
//	cvtColor(srcImage2, srcImage2, COLOR_BGR2HSV);
	alignImages(srcImage1, srcImage2, imReg, h);
//	alignImages2(srcImage1, srcImage2, imReg);

	// Write aligned image to disk.
	imwrite(diffImagePath + "aligned.png", imReg);
	cout << "Saving aligned image : " << diffImagePath + "aligned.png" << endl;

	// Print estimated homography
	cout << "Estimated homography : \n" << h << endl;


	Mat diff, diff2,diff3,diff4,diff5,diff6, diff7,norm_img;

		cvtColor(imReg, imReg, COLOR_BGR2HSV);
		cvtColor(srcImage2, srcImage2, COLOR_BGR2HSV);

	modifyImageForCorrelation(imReg);
	modifyImageForCorrelation(srcImage2);
//



	absdiff(imReg, srcImage2, diff);
	imwrite("/tmp/diff_img.png",diff);

	Mat mask = Mat::zeros(imReg.rows, imReg.cols, CV_8UC1);
	int img_rows = imReg.rows;
	int img_cols = imReg.cols;
	int width = 4;
	int height = 4;
	Mat result;

	cout << "Finding differences" << endl;
	for (int row =0; row < img_rows - height; row = row + height){
		if (row > img_rows){
			row = row - img_rows;
		}
		for (int col = 0; col < img_cols - width; col = col + width){
			if (col > img_cols){
				col = col - img_cols;
			}
			Rect roi(col,row,width,height);

			matchTemplate( imReg(roi), srcImage2(roi), result, TM_CCORR_NORMED);

			if (result.at<float>(0,0) < 0.977){
//				cout << "result.at<float>(%d,%d) : \n" << result.at<float>(0,0) << " "<< col<<" "<<row << endl;
				rectangle(mask, Point(col,row), Point(col+width, row+height), Scalar(255), FILLED);
			}
		}
	}

		erodeImage(mask,0,3);

//		imwrite("/tmp/mask.png",mask);

		cout << "Finding objects" << endl;

		 // Write your code BELOW this line.

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours( mask, contours, hierarchy ,RETR_EXTERNAL , CHAIN_APPROX_SIMPLE );

		// This data structure must contain coordinates of the differences.
		vector < vector<Point> > contours_poly(contours.size());
		vector<Rect> boundRect( contours.size() );
		for( size_t i = 0; i < contours_poly.size(); i++ )
		{
			approxPolyDP( contours[i], contours_poly[i], 3, true );
			boundRect[i] = boundingRect( contours_poly[i] );
		}

		// Write your code ABOVE this line.



		cout << "Writing Results" << endl;

		// The following lines of code generate the required output. Do not change these.
		Mat drawing = imread(inputImage1.c_str());
		Mat contourMask(drawing.rows, drawing.cols, drawing.type(), Scalar(0,0,0));

		ofstream myfileStream;
		myfileStream.open(diffImagePath + "_differences.txt");

		for (size_t i = 0; i < contours_poly.size(); i++) {
			drawContours(contourMask, contours_poly, (int) i, Scalar(255, 255, 255), cv::FILLED);
			drawContours(drawing, contours_poly, (int) i, Scalar(255, 255, 255), cv::FILLED);
			drawContours(drawing, contours_poly, (int) i, Scalar(0, 0, 255), 5);

			myfileStream << "[ Contour no:" << i << ", TopLeft:" << boundRect[i].tl() << ", BottomRight:"
					<< boundRect[i].br() << "], Contour coordinates:[";
			for (int contourIndx = 0; contourIndx < contours_poly[i].size(); contourIndx++)
				myfileStream << contours_poly[i][contourIndx];

			myfileStream << "]\n";
		}
		imwrite(diffImagePath + "_diffMaskAndImage.png", drawing);
		imwrite(diffImagePath + "_diffMask.png", contourMask);
		myfileStream.close();

		myfileStream.open(diffImagePath + "_differenceCount.txt");
		myfileStream << contours_poly.size();
		myfileStream.close();
}


void SpotDifferences::erodeImage(Mat srcImage, int structuring_elem, int structuring_size) {
	try{
		Mat element = getStructuringElementForMorphology(structuring_elem, structuring_size);
		erode(srcImage, srcImage, element);
	}catch(std::logic_error& errThrown ){
		std::cout << "--error start-- \n\n" << errThrown.what() << "\n\n--error end-- \n\n";
	}
}


void SpotDifferences::alignImages(Mat &im1, Mat &im2, Mat &im1Reg, Mat &h)
{

	const int MAX_FEATURES = 500;
	const float GOOD_MATCH_PERCENT = 0.15f;
  // Convert images to grayscale
  Mat im1Gray, im2Gray;
  cvtColor(im1, im1Gray, COLOR_BGR2GRAY);
  cvtColor(im2, im2Gray, COLOR_BGR2GRAY);

  // Variables to store keypoints and descriptors
  vector<KeyPoint> keypoints1, keypoints2;
  Mat descriptors1, descriptors2;

  // Detect ORB features and compute descriptors.
  Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
  orb->detectAndCompute(im1Gray, Mat(), keypoints1, descriptors1);
  orb->detectAndCompute(im2Gray, Mat(), keypoints2, descriptors2);

  // Match features.
  vector<DMatch> matches;
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  matcher->match(descriptors1, descriptors2, matches, Mat());

  // Sort matches by score
  sort(matches.begin(), matches.end());

  // Remove not so good matches
  const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
  matches.erase(matches.begin()+numGoodMatches, matches.end());
//
//  // Draw top matches
  Mat imMatches;
//  drawMatches(im1, keypoints1, im2, keypoints2, matches, imMatches);
  drawMatches( im1, keypoints1, im2, keypoints2, matches, imMatches, Scalar::all(-1),
                  Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//  drawKeypoints(im1, keypoints1, imMatches);
//  drawKeypoints(im2, keypoints2, imMatches);
  imwrite("/tmp/matches.jpg", imMatches);

  // Extract location of good matches
  vector<Point2f> points1, points2;

  for( size_t i = 0; i < matches.size(); i++ )
  {
    points1.push_back( keypoints1[ matches[i].queryIdx ].pt );
    points2.push_back( keypoints2[ matches[i].trainIdx ].pt );
  }

  // Find homography
  h = findHomography( points1, points2, RANSAC );

  // Use homography to warp image
  warpPerspective(im1, im1Reg, h, im2.size());
}




