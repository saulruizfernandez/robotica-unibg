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
		
		registerProperty(marker_red_min, "MarkerRedMin");
		registerProperty(marker_red_max, "MarkerRedMax");
		registerProperty(marker_green_min, "MarkerGreenMin");
		registerProperty(marker_green_max, "MarkerGreenMax");
		registerProperty(marker_blue_min, "MarkerBlueMin");
		registerProperty(marker_blue_max, "MarkerBlueMax");
	}
	void setup() {
	}

//	RNG rng(12345);
	
	/*
	 * Apply bilateral filtering, for smoothing image (noise reduction) while keeping the borders unblurred.
	 */
	void enhaceImageQuality(cv::Mat &image) {
        	cv::Mat enhanced_image1, enhanced_image2;
		enhanced_image1 = image;
		cv::cvtColor(enhanced_image1, enhanced_image1, COLOR_BGRA2BGR);
        	cv::bilateralFilter(enhanced_image1, enhanced_image2, 15, 75, 75);      	
		// imshow("Enhanced Image", enhanced_image2 );
		image = enhanced_image2;
	}

	/*
	 * It creates a mask (binary matrix where 1s correspond to the pixels within a certain color range). It
	 * multiplies the mask by a white image, so in the result we will have a pure black & white image where
	 * white represents the filtered colour, and black the background.
	 */
	void colorFilter(cv::Mat &frame){
		Mat mask, output_image;
        	//BGR
		cv::inRange(frame, cv::Scalar(marker_blue_min, marker_green_min, marker_red_min),
				    cv::Scalar(marker_blue_max, marker_green_max, marker_red_max), mask);
		Mat whiteImage(frame.rows, frame.cols, CV_8UC3, cv::Scalar(255, 255, 255));
		cv::bitwise_and(whiteImage, whiteImage, output_image, mask);
		// imshow( "Color filter", output_image );
		frame = output_image;
	}

	/*
	 * 
	 */
	void arrowDetection(cv::Mat &frame, std::vector<aurora::perception::VisualObject3D> &objects){
		Mat edges, output_image;

		cv::Canny(frame, edges, 100, 150, 3, true);		
		// imshow( "Canny", edges );

		Mat drawing = Mat::zeros( edges.size(), CV_8UC3 );
		vector<pair<Point, Point>>  arrow_vector;
		vector<vector<Point> > contours, approx;
		vector<Vec4i> hierarchy;
		findContours( edges, contours, hierarchy, RETR_EXTERNAL , CHAIN_APPROX_SIMPLE );
		
		for( size_t i = 0; i < contours.size(); i++ ) {
			vector<Point> tempApprox; // Contains the points of the polygon
			vector<int> hull; // Contains the points of the convex hull calculated out from tempApprox
			
			double perimeter = cv::arcLength(contours[i], true);
			double epsilon =  perimeter * 0.025;
			
			cv::approxPolyDP(contours[i], tempApprox, epsilon, true);
			
			// Draw white points to check the key-points of the arrow are detected correctly
			for( size_t y = 0; y < tempApprox.size(); y++ ) {
				Scalar color = Scalar( 255, 255, 255 );
				cv::circle(drawing, tempApprox[y], 4, color, -1);
			}			

			/*
			 * Create convex hull: the method returns the indexes in the tempApprox vector that correspond to the convex hull.
			 * The indexes are sorted to ensure they are always processed in ascending order
			 */
			cv::convexHull(tempApprox, hull, false, false);
			std::sort(hull.begin(), hull.end());

			// 1st ARROW CONDITION CHECK
			if ((hull.size()==5 ||  hull.size()==4) && (tempApprox.size() - hull.size()) == 2) {
				vector<int> indexes_tempApprox;
				for (int i{0}; i < tempApprox.size(); ++i) indexes_tempApprox.push_back(i);
			
				vector<int> indexes; // Stores the indexes of the two inner points of the arrow	
				std::set_difference(indexes_tempApprox.begin(), indexes_tempApprox.end(), hull.begin(), hull.end(), std::inserter(indexes, indexes.begin()));
				
				// Draw the two inner points with a different color than white
				cv::circle(drawing, tempApprox[indexes[0]], 4, Scalar( 255, 0, 0 ), -1);
				cv::circle(drawing, tempApprox[indexes[1]], 4, Scalar( 255, 0, 0 ), -1);
				
				int tip_index = -1;
				for (int it{0}; it < 2; ++it) {
					int x = (indexes[it] + 2) % tempApprox.size();
					int j = ((indexes[(it + 1) % 2] - 2) < 0)? (tempApprox.size() + (indexes[(it + 1) % 2] - 2)) : (indexes[(it + 1) % 2] - 2);
					if (j == x){
						tip_index = j;						
					}
				}
				if (tip_index == -1){
					std::cout<<"Tip not found\n";
					continue;
				}
				
				// Point middle_point
				Point middle_point;
				if (hull.size() == 5){
					// Calculate the first point of the vector that points to the direction of the tip
					Point point1 = tempApprox[(tip_index + 3) % tempApprox.size()];
					Point point2 = tempApprox[(tip_index + 4) % tempApprox.size()];
	
					// Point middle_point
					middle_point.x = (point1.x + point2.x)/2;
					middle_point.y = (point1.y + point2.y)/2;
				} else {
					middle_point = tempApprox[(tip_index + 3) % tempApprox.size()];
				}
				
				approx.push_back(tempApprox);
				
				// Calculate the center of the arrow
				Point center_of_arrow;
				Point point_tip = tempApprox[tip_index];
				Point point3 = tempApprox[(tip_index + 1) % tempApprox.size()];
				Point point4 = tempApprox[(tip_index + 6) % tempApprox.size()];
				
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
					cv::circle(drawing, center_of_arrow, 4, Scalar(230, 237, 21), -1);
				}
				
				// Calculate the intersection to check if it is an arrow
				Point point5 = tempApprox[(tip_index + 2) % tempApprox.size()];
				Point point6 = tempApprox[(tip_index + 5) % tempApprox.size()];
				
				double a3 = point6.y - point5.y;
				double b3 = point5.x - point6.x;
				double c3 = a3 * (point5.x) + b3 * (point5.y);
				
				Point intersection_check;
				
				if ((a1 * b3 - a3 * b1) != 0) { // determinant != 0
					intersection_check.x = (b3*c1 - b1*c3) / (a1 * b3 - a3 * b1);
					intersection_check.y = (a1*c3 - a3*c1) / (a1 * b3 - a3 * b1);
					// Draw circle in the intersection point
					cv::circle(drawing, intersection_check, 4, Scalar(0, 237, 21), -1);
				}
				
				// Calculate size of marker (depth and width)
				double depth = sqrt((middle_point.x - point_tip.x) * (middle_point.x - point_tip.x) + (middle_point.y - point_tip.y) * (middle_point.y - point_tip.y));
				double width = sqrt((point3.x - point4.x) * (point3.x - point4.x) + (point3.y - point4.y) * (point3.y - point4.y));
				
				std::cout << ">>>> SIZE OF MARKER:\n";
				std::cout << "depth: " << depth << ", width: " << width << std::endl;
				
			        // Calculate distance between center and intersection_check and see if it is in a threshold
				if (cv::norm(center_of_arrow - intersection_check) > (depth * 0.25)) continue;
				
				// Calculate orientation of marker (yaw -> positive clockwise)
				double slope_arrow_vector = (a1 / b1);
				
				//double yaw_arrow = (atan(fabs(slope_arrow_vector)) * 180) / M_PI;
				double yaw_arrow = -1 * atan2(point_tip.y - middle_point.y, point_tip.x - middle_point.x)* 180 / M_PI;
				if (yaw_arrow < 0){
					yaw_arrow = 360 + yaw_arrow;
				}
				std::cout << ">>>> ORIENTATION: " << yaw_arrow << "\n";
				
				// Add new arrow vector
				arrow_vector.push_back({tempApprox[tip_index], middle_point});
				
				aurora::perception::VisualObject3D tempObject;
				
				tempObject.width = width;
				tempObject.depth = depth;		
				tempObject.height = 0;	
				
				tempObject.rototras.setTranslation(center_of_arrow.x, center_of_arrow.y, 0);
				tempObject.rototras.setRPY(0, 0, yaw_arrow * M_PI / 180);
				
				objects.push_back(tempObject);
				
			}
		}
		
		for( size_t i = 0; i< approx.size(); i++ ) {
			Scalar color = Scalar( 105, 105, 255 );
			drawContours( drawing, approx, (int)i, color, 2, LINE_8, hierarchy, 0 );
		}
		
		std::cout << ">>>> TIPS: " << arrow_vector.size() << std::endl;

		for( size_t i = 0; i< arrow_vector.size(); i++ ) {
			Scalar color = Scalar( 255, 0, 255 );
			// cv::circle( drawing, arrow_vector[i].first, 5, color, -1);
			cv::arrowedLine(drawing, arrow_vector[i].second, arrow_vector[i].first, color, 2);
		}
		imshow( "Polygon approximation", drawing );
	}

	bool detectMarkers(cv::Mat &frame, std::vector<aurora::perception::VisualObject3D> &objects) {
		std::cout << "[PluginMarkerDetectorArrows]::detectMarkers()" << std::endl;
		enhaceImageQuality(frame);				
		colorFilter(frame);						
		arrowDetection(frame, objects);
		return true;
	}

protected:
	VProperty<double> marker_size;	// [cm]
	VProperty<bool> show_marker;
	VProperty<int> marker_red_min;
	VProperty<int> marker_red_max;
	VProperty<int> marker_green_min;
	VProperty<int> marker_green_max;
	VProperty<int> marker_blue_min;
	VProperty<int> marker_blue_max;
	double focalLenght = 0.0021;
	double cameraPixelSize = 2e-6;
	double baseline = 0.12;
private:	

};

extern "C" BOOST_SYMBOL_EXPORT PluginMarkerDetectorArrows marker_detector_arrows;
PluginMarkerDetectorArrows marker_detector_arrows;

} // namespace perception
} // namespace functionality
} // namespace aurora

