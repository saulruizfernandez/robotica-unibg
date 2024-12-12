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
#include <math.h>
#include <algorithm>

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
	/*
		cv::Mat ycrcb_image, y_channel_stretched, y_channel_enhanced, enhanced_ycrcb_image, enhanced_image;
		cv::Mat ycrcb_channels[3];
		vector<Mat> channels;

		//cv::GaussianBlur(image, image, Size(5, 5), 1);

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
        */
        
        	// Apply bilateral filtering, for smoothing image, reduce noise, while preserving the borders
        	cv::Mat enhanced_image1, enhanced_image2;
        	cv::convertScaleAbs(image, enhanced_image1, 2, 10);
        	cv::bilateralFilter(enhanced_image1, enhanced_image2, 15, 75, 75);
        	
		imshow("Enhanced Image", enhanced_image2 );
		image = enhanced_image2;
	}

	void colorQuantization(cv::Mat &image){
		Mat data;
		image.convertTo(data, CV_32F); // Convert to float32
		data = data.reshape(1, data.total());
		Mat labels, centers;
		int k = 8; // Color number
		cv::kmeans(data, k, labels, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);
		centers = centers.reshape(3,centers.rows);
		data = data.reshape(3,data.rows);

		Vec3f *p = data.ptr<Vec3f>();
		for (size_t i=0; i<data.rows; i++) {
		   int center_id = labels.at<int>(i);
		   p[i] = centers.at<Vec3f>(center_id);
		}

		image = data.reshape(3, image.rows);
		image.convertTo(image, CV_8U);

		imshow("Color Quantization", image );

	}

	void colorFilter(cv::Mat &frame){

		Mat mask, output_image;

		//BGR
//		cv::inRange(frame, cv::Scalar(0, 0, 0),
//				                      cv::Scalar(80, 60, 60), mask);
		//Black Filter
		/*cv::inRange(frame, cv::Scalar(0, 0, 0),
						                      cv::Scalar(128, 128, 128), mask);*/
        //Green Filter
		cv::inRange(frame, cv::Scalar(0, 0, 160),
						                        cv::Scalar(160, 123, 255), mask);
        

		Mat whiteImage(frame.rows, frame.cols, CV_8UC3, cv::Scalar(255, 255, 255));
		cv::bitwise_and(whiteImage, whiteImage, output_image, mask);

		imshow( "Color filter", output_image );
		frame = output_image;
	}

	void arrowDetection(cv::Mat &frame, std::vector<aurora::perception::VisualObject3D> &objects){
		std::cout << ">>>>>>>> ARROW DETECTION\n";
		Mat edges, output_image;

		//	Canny( src_gray, edges, thresh, thresh*2 );
		cv::Canny(frame, edges, 100, 150, 3, true);
		
		imshow( "Canny", edges );

		/*
		 * Contour detection
		 */
		Mat drawing = Mat::zeros( edges.size(), CV_8UC3 );
		vector<pair<Point, Point>>  arrow_vector;
		vector<vector<Point> > contours, approx;
		vector<Vec4i> hierarchy;
		findContours( edges, contours, hierarchy, RETR_EXTERNAL , CHAIN_APPROX_SIMPLE );
		
		for( size_t i = 0; i< contours.size(); i++ ) {
			Scalar color = Scalar( 105, 105, 255 );
//			Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
			drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
		}

		for( size_t i = 0; i< contours.size(); i++ ) {
			std::cout << "ping\n";
			vector<Point> tempApprox;
			vector<int> hull;
			double perimeter = cv::arcLength(contours[i], true);

			double epsilon =  perimeter * 0.025;

			cv::approxPolyDP(contours[i], tempApprox, epsilon, true);
			
			for( size_t y = 0; y< tempApprox.size(); y++ ) {
				Scalar color = Scalar( 255, 255, 255 );
				//drawContours( drawing, tempApprox, (int)y, color, 2, LINE_8, hierarchy, 0 );
				cv::circle(drawing, tempApprox[y], 4, color, -1);
			}			

			cv::convexHull(tempApprox, hull, false, false);
			// Sort hull
			std::sort(hull.begin(), hull.end());
			std::cout << "pong1\n";

			if ((hull.size()==5 ||  hull.size()==4) && (tempApprox.size() - hull.size()) == 2) { // It is an arrow, two inner points in the convex hull of 5 or 4 sides
				
				for (size_t y = 0; y < hull.size(); y++){
					std::cout << hull[y]<<", ";
				}
				std::cout<<endl;
				vector<int> indexes_tempApprox;
				for (int i{0}; i < tempApprox.size(); ++i) indexes_tempApprox.push_back(i);
				vector<int> indexes;
				
				std::set_difference(indexes_tempApprox.begin(), indexes_tempApprox.end(), hull.begin(), hull.end(), std::inserter(indexes, indexes.begin()));
								
				std::cout << "indexes: " << indexes[0] << ", " << indexes[1] << "\n";
				
				cv::circle(drawing, tempApprox[indexes[0]], 4, Scalar( 255, 0, 0 ), -1);
				cv::circle(drawing, tempApprox[indexes[1]], 4, Scalar( 255, 0, 0 ), -1);
				
				std::cout << "pong2\n";
				int tip_index = -1;
				for (int it{0}; it < 2; ++it) {
					// int x = (indexes[it] + 2) % hull.size();
					int x = (indexes[it] + 2) % tempApprox.size();
					//int j = (indexes[(it+1)%2] - 2) < 0 ?  (hull.size() - abs(indexes[(it+1)%2]  - 2) % hull.size()) : (indexes[(it+1)%2] - 2);
					int j = ((indexes[(it + 1) % 2] - 2) < 0)? (tempApprox.size() + (indexes[(it + 1) % 2] - 2)) : (indexes[(it + 1) % 2] - 2);
					std::cout << "X: "<<x<<", J:"<<j<<"\n";
					if (j == x){
						tip_index = j;						
					}
				}
				if (tip_index == -1){
					std::cout<<"Tip not found\n";
					continue;
				}
				std::cout << "pong3\n";
				
				// Point middle_point
				Point middle_point;
				if (hull.size() == 5){
					// Calculate the first point of the vector that points to the direction of the tip
					Point point1 = tempApprox[(tip_index + 3) % tempApprox.size()];
					Point point2 = tempApprox[(tip_index + 4) % tempApprox.size()];
	
					// Point middle_point
					middle_point.x = (point1.x + point2.x)/2;
					middle_point.y = (point1.y + point2.y)/2;
				}else {
					middle_point = tempApprox[(tip_index + 3) % tempApprox.size()];
				}
				std::cout << "pong4\n";					
				
				// Add new arrow vector
				arrow_vector.push_back({tempApprox[tip_index], middle_point});
				std::cout << "pong5\n";	
				
				approx.push_back(tempApprox);
				std::cout << "pong6\n";	
				
				// Calculate the center of the arrow
				Point center_of_arrow;
				Point point_tip = tempApprox[tip_index];
				Point point3 = tempApprox[(tip_index + 1) % tempApprox.size()];
				Point point4 = tempApprox[(tip_index + 5) % tempApprox.size()];
				
				// Represent lines as ax + by = c
				double a1 = point_tip.y - middle_point.y;
				double b1 = middle_point.x - point_tip.x;
				double c1 = a1 * (middle_point.x) + b1 * (middle_point.y);
				
				double a2 = point4.y - point3.y;
				double b2 = point3.x - point4.x;
				double c2 = a2 * (point3.x) + b2 * (point3.y);
				
				if ((a1 * b2 - a2 * b1) != 0) { // determinant != 0
					center_of_arrow.x = (b2*c1 - b1*c2) / (a1 * b2 - a2 * b1);
					center_of_arrow.y = (a1*c2 - a2*c1) / (a1 * b2 - a2 * b1);
					// Draw circle in the intersection point
					cv::circle(drawing, center_of_arrow, 10, Scalar(230, 237, 21), -1);
				}
				
				// Calculate size of marker (height and width)
				double height = sqrt((middle_point.x - point_tip.x) * (middle_point.x - point_tip.x) + (middle_point.y - point_tip.y) * (middle_point.y - point_tip.y));
				double width = sqrt((point3.x - point4.x) * (point3.x - point4.x) + (point3.y - point4.y) * (point3.y - point4.y));
				
				std::cout << "SIZE OF MARKER:\n";
				std::cout << "height: " << height << ", width: " << width << std::endl;
				
				// Calculate orientation of marker (yaw -> positive clockwise)
				double slope_arrow_vector = (a1 / b1);
				
				//double yaw_arrow = (atan(fabs(slope_arrow_vector)) * 180) / M_PI;
				double yaw_arrow = -1 * atan2(point_tip.y - middle_point.y, point_tip.x - middle_point.x)* 180 / M_PI;
				if (yaw_arrow < 0){
					yaw_arrow = 360 + yaw_arrow;
				}
				std::cout << "orientation: " << yaw_arrow << "\n";
			}
		}
		std::cout << "pong\n";

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

	bool detectMarkers(cv::Mat &frame, std::vector<aurora::perception::VisualObject3D> &objects) {
		std::cout << "[PluginMarkerDetectorArrows]::detectMarkers()" << std::endl;

		Mat output_image;

		enhaceImageQuality(frame);
		// colorQuantization(frame);
		colorFilter(frame);
		//blobDetection(frame);
		arrowDetection(frame, objects);

		output_image =frame;

//		Mat edges;
//	//	Canny( src_gray, edges, thresh, thresh*2 );
//		cv::Canny(output_image, edges, 50, 200, 3);
//
//		/*
//		 * Contour detection
//		 */
//		vector<vector<Point> > contours;
//		vector<Vec4i> hierarchy;
//		findContours( edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
//		std::cout<<"Contour size: "<<contours.size()<<std::endl;
//
//		Mat drawing = Mat::zeros( edges.size(), CV_8UC3 );
//		for( size_t i = 0; i< contours.size(); i++ ) {
//			Scalar color = Scalar( 105, 105, 255 );
////			Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
//			drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
//		}
//		imshow( "Contours", drawing );
//
//
//		/*
//		 * Line detection
//		 * https://stackoverflow.com/questions/32476410/opencv-c-line-detection-with-houghlinesp
//		 */
//		// Copy edges to the images that will display the results in BGR
//		cv::Mat edges_color, edges_image;
//		cv::cvtColor(edges, edges_color, cv::COLOR_GRAY2BGR);
//		edges_image = edges_color.clone();
//
//
//		// Probabilistic Line Transform
//		std::vector<Vec4i> linesP; // will hold the results of the detection
//		HoughLinesP(edges, linesP, 1, CV_PI/180, 25, 15, 3 ); // runs the actual detection
//		// Draw the lines
//		for( size_t i = 0; i < linesP.size(); i++ ) {
//			Vec4i l = linesP[i];
//			line( edges_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA);
//		}
//
//	    cv::imshow("global edges", edges_image);
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

