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


	bool detectMarkers(cv::Mat &frame, std::vector<int> &ids, std::vector<SceneTransform> &poses) {
		std::cout << "[PluginMarkerDetectorArrows]::detectMarkers()" << std::endl;

		Mat hsv, mask, output_image;

		cv::cvtColor(frame, hsv, COLOR_BGR2HSV);

		//Method 1: Detect darkness by splitting into hsv channels

		std::vector<Mat> hsv_channels(3);
		cv::split(hsv, hsv_channels);
		Mat value_channel = hsv_channels[2];

		cv::inRange(value_channel, 0, 100, mask);
		frame.copyTo(output_image, mask);

		//Method 2: Detect color by range of BGR value
		/*
		cv::inRange(hsv, cv::Scalar(0, 0, 0),
		                        cv::Scalar(120, 120, 120), mask);

		cv::bitwise_and(frame, frame, output_image, mask);
		*/
		imshow( "Black filter", output_image );

		Mat edges;
	//	Canny( src_gray, edges, thresh, thresh*2 );
		cv::Canny(output_image, edges, 50, 200, 3);

		/*
		 * Contour detection
		 */
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours( edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
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

