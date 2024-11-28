/********************************************************************************
 *
 * PluginMarkerDetectorArrows
 *
 * Copyright (c) 2021
 * All rights reserved.
 *
 * Davide Brugali, Università degli Studi di Bergamo
 *
 * -------------------------------------------------------------------------------
 *
 * File: PluginMarkerDetectorArrows.cpp
 * Created: February 14, 2021
 *
 * Supervised by: <A HREF="mailto:brugali@unibg.it">Davide Brugali</A>
 *
 * -------------------------------------------------------------------------------
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * -------------------------------------------------------------------------------
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of the University of Bergamo nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 *******************************************************************************/
#include <stresa/core/runtime/VProperty.hpp>

#include "aurora/functionality/perception/MarkerDetectorInterface.hpp"

#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>

using namespace stresa;
using namespace cv;
using namespace std;

namespace aurora {
namespace functionality {
namespace perception {

class PluginMarkerDetectorArrows : public MarkerDetectorInterface {
public:
	PluginMarkerDetectorArrows() {
		registerProperty(marker_size, "MarkerSize");
		registerProperty(show_marker, "ShowMarker");
	}
	void setup() {
	}

//	RNG rng(12345);

	void blobDetection(cv::Mat &im){
		Mat im_with_keypoints,gray_image;

		cv::cvtColor(im, gray_image, COLOR_BGR2GRAY);

		std::vector<KeyPoint> keypoints;

		cv::SimpleBlobDetector::Params params;
		// Change thresholds
		params.minThreshold = 10;
		params.maxThreshold = 200;

		params.filterByColor = true;
		params.blobColor = 255;

		// Filter by Area.
		params.filterByArea = true;
		params.minArea = 150;

		// Filter by Circularity
		params.filterByCircularity = false;
		params.minCircularity = 0.785;

		// Filter by Convexity
		params.filterByConvexity = true;
		params.minConvexity = 0.00;

		// Filter by Inertia
		params.filterByInertia = false;
		params.minInertiaRatio = 0.00;

	#if CV_MAJOR_VERSION < 3   // If you are using OpenCV 2

		// Set up detector with params
		SimpleBlobDetector detector(params);

		// Detect blobs
		detector.detect(gray_image, keypoints);
	#else

		// Set up detector with params
		Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

		// Detect blobs
		detector->detect(gray_image, keypoints);
	#endif

		drawKeypoints( gray_image, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

		imshow("keypoints", im_with_keypoints );// Show blobs
	}

	void enhaceImageQuality(cv::Mat &image) {
		cv::Mat ycrcb_image, y_channel_stretched, y_channel_enhanced, enhanced_ycrcb_image, enhanced_image;
		cv::Mat ycrcb_channels[3];
		vector<Mat> channels;

		cv::GaussianBlur(image, image, Size(5, 5), 1);

		/*cv::dilate( image, image, Mat());
		cv::erode( image, image, Mat());*/

		cv::cvtColor(image, ycrcb_image, COLOR_BGR2YCrCb);
		cv::split(ycrcb_image, ycrcb_channels);

		cv::normalize(ycrcb_channels[0], y_channel_stretched, 0, 255, NORM_MINMAX);
		cv::equalizeHist(y_channel_stretched, y_channel_enhanced);
		ycrcb_channels[0] = y_channel_enhanced;

		channels.push_back(ycrcb_channels[0]);
		channels.push_back(ycrcb_channels[1]);
		channels.push_back(ycrcb_channels[2]);
		cv::merge(channels, enhanced_ycrcb_image);
		cv::cvtColor(enhanced_ycrcb_image, enhanced_image, COLOR_YCrCb2BGR);
		imshow("Enhanced Image", enhanced_image );
		image = enhanced_image;
	}
/*
	void colorQuantization(cv::Mat &image){
		int height = image.rows;
		int width = image.cols;

		cv::cvtColor(image, lab_image, COLOR_BGR2LAB);
		lab_image.reshape(3, height * width);
		Mat labels;
		std::vector<Point2f> centers; // 2D vector of float values
		cv::kmeans(lab_image, 10, labels, TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);



	}*/

	void colorFilter(cv::Mat &frame){

		Mat mask, output_image;

		//BGR
		cv::inRange(frame, cv::Scalar(0, 0, 0),
				                      cv::Scalar(80, 60, 60), mask);
        /*
		cv::inRange(frame, cv::Scalar(0, 0, 80),
						                        cv::Scalar(10, 20, 255), mask);
        */
		Mat whiteImage(frame.rows, frame.cols, CV_8UC3, cv::Scalar(255, 255, 255));
		cv::bitwise_and(whiteImage, whiteImage, output_image, mask);

		imshow( "Color filter", output_image );
		frame = output_image;
	}

	void arrowDetection(cv::Mat &frame){

		Mat edges, output_image;

		//	Canny( src_gray, edges, thresh, thresh*2 );
		cv::Canny(frame, edges, 50, 200, 3);

		/*
		 * Contour detection
		 */
		vector<pair<Point, Point>>  arrow_vector;
		vector<vector<Point> > contours, approx;
		vector<Vec4i> hierarchy;
		findContours( edges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
		Mat drawing = Mat::zeros( edges.size(), CV_8UC3 );

		for( size_t i = 0; i< contours.size(); i++ ) {
			vector<Point> tempApprox, hull;
			double epsilon = cv::arcLength(contours[i], true) * 0.02;

			cv::approxPolyDP(contours[i], tempApprox, epsilon, true);
			if (tempApprox.size() == 7) {

				cv::convexHull(tempApprox, hull, true);

				if (7 - hull.size() == 2) { // It is an arrow, two inner points in the convex hull of 5 sides

					double max1 =0; double max2 = 0;
					std::pair<int, int> segment1, segment2;

					for (size_t y = 0; y < hull.size(); y++){

						cv::circle( drawing, hull[y], 10, (255,255,255), -1); // Show keypoints

						double distance = cv::norm(hull[(y+1)%5] - hull[(y)]);
						if (distance > max1 || distance > max2)
						{
							max2 = max1;
							max1 = distance;

							segment2 = segment1;
							segment1 = make_pair(y, (y+1)%5);
						}
					}
					int tip_index;
					if((segment1.second + 2)% 5 == segment2.first) { // Find the tip of the arrow
						tip_index = (segment1.second + 1) % 5;

					}else if ((segment2.second + 2)% 5 == segment1.first){
						tip_index = (segment2.second + 1)% 5;

					}

					// Calculate the first point of the vector that points to the direction of the tip
					Point point1 = hull[(tip_index + 2) % 5];
					Point point2 = hull[(tip_index + 3) % 5];

					// Point middle_point
					Point middle_point((point1.x + point2.x)/2, (point1.y + point2.y)/2);

					arrow_vector.push_back({hull[tip_index], middle_point});
					approx.push_back(tempApprox);
				}
			}
		}


		for( size_t i = 0; i< approx.size(); i++ ) {
			Scalar color = Scalar( 105, 105, 255 );
			drawContours( drawing, approx, (int)i, color, 2, LINE_8, hierarchy, 0 );
		}

		std::cout<<">>>>Tips: "<<arrow_vector.size()<<std::endl;

		for( size_t i = 0; i< arrow_vector.size(); i++ ) {
			Scalar color = Scalar( 255, 0, 255 );
			// cv::circle( drawing, arrow_vector[i].first, 5, color, -1);
			cv::arrowedLine(drawing, arrow_vector[i].second, arrow_vector[i].first, color, 2);

		}
		imshow( "Polygon approximation", drawing );


	}

	bool detectMarkers(cv::Mat &frame, std::vector<int> &ids, std::vector<SceneTransform> &poses) {
		std::cout << "[PluginMarkerDetectorArrows]::detectMarkers()" << std::endl;

		Mat output_image;

		enhaceImageQuality(frame);
		colorFilter(frame);
		//blobDetection(frame);
		arrowDetection(frame);

		output_image =frame;

		Mat edges;
	//	Canny( src_gray, edges, thresh, thresh*2 );
		cv::Canny(output_image, edges, 50, 200, 3);

		/*
		 * Contour detection
		 */
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours( edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
		std::cout<<"Contour size: "<<contours.size()<<std::endl;

		Mat drawing = Mat::zeros( edges.size(), CV_8UC3 );
		for( size_t i = 0; i< contours.size(); i++ ) {
			Scalar color = Scalar( 105, 105, 255 );
//			Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
			drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
		}
		imshow( "Contours", drawing );


		/*
		 * Line detection
		 * https://stackoverflow.com/questions/32476410/opencv-c-line-detection-with-houghlinesp
		 */
		// Copy edges to the images that will display the results in BGR
		cv::Mat edges_color, edges_image;
		cv::cvtColor(edges, edges_color, cv::COLOR_GRAY2BGR);
		edges_image = edges_color.clone();


		// Probabilistic Line Transform
		std::vector<Vec4i> linesP; // will hold the results of the detection
		HoughLinesP(edges, linesP, 1, CV_PI/180, 25, 15, 3 ); // runs the actual detection
		// Draw the lines
		for( size_t i = 0; i < linesP.size(); i++ ) {
			Vec4i l = linesP[i];
			line( edges_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA);
		}

	    cv::imshow("global edges", edges_image);
	    cv::waitKey(60);


		return true;
	}

protected:
	VProperty<double> marker_size;	// [cm]
	VProperty<bool> show_marker;

private:

};

extern "C" BOOST_SYMBOL_EXPORT PluginMarkerDetectorArrows marker_detector_arrows;
PluginMarkerDetectorArrows marker_detector_arrows;

} // namespace perception
} // namespace functionality
} // namespace aurora

