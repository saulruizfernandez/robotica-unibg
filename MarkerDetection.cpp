/********************************************************************************
 *
 * MarkerDetection
 *
 * Copyright (c) 2021
 * All rights reserved.
 *
 * Davide Brugali, Università degli Studi di Bergamo
 *
 * -------------------------------------------------------------------------------
 * File: MarkerDetection.cpp
 * Created: February 14, 2021
 * Author: <A HREF="mailto:brugali@unibg.it">Davide Brugali</A>
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
 ******************************************************************************
 */

#include "MarkerDetection.hpp"

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string>
#include <vector>

using namespace stresa;

namespace aurora {
namespace components {

MarkerDetection::MarkerDetection(CameraRGBServer* server) : VActivity() {
	this->server = server;
	scene_graph = init_SceneGraph();
	registerSubscriber(&imageSub, "ImageSub", "image", nullptr, imageConnectionCallback);
	registerPublisher(&markersPub, "MarkersPub", "markers", markersConnectionCallback);

	registerFunctionality(&marker_detector, "MarkerDetector");
}

MarkerDetection::~MarkerDetection() {
}

void MarkerDetection::init() {
	std::cout << "[MarkerDetection]::init" << std::endl;
}

void MarkerDetection::reconfigure() {
	std::cout << "[MarkerDetection]::reconfigure" << std::endl;
}
void MarkerDetection::skip() {
	std::cout << "[MarkerDetection]::skip" << std::endl;
}
void MarkerDetection::missed() {
	std::cout << "[MarkerDetection]::missed" << std::endl;
}

void MarkerDetection::task() {
	std::cout << "[MarkerDetection]::task" << std::endl;

	imageCallback(this);
}

void MarkerDetection::quit() {
	std::cout << "[MarkerDetection]::quit()" << std::endl;
}

void MarkerDetection::imageCallback(VariantActivity* va) {
	MarkerDetection* activity = (MarkerDetection*) va;

	// ################ USER DEFINED BEGIN ##################
		std::cout << "[MarkerDetection] imageCallback" << std::endl;
		// Take data
		sensor_msgs::msg::dds_::Image_ image_msg;

		SampleInfo m_info;

		if(activity->imageSub.takeNextData(&image_msg, &m_info)) {
		    // *********************************
		    // *** get the image properties ****
		    // *********************************
			timespec time_stamp;
			time_stamp.tv_sec = image_msg.header().stamp().sec();
			time_stamp.tv_nsec = image_msg.header().stamp().nanosec();

			int img_height = image_msg.height();
			int img_width  = image_msg.width();
			int image_step = image_msg.step();
			bool is_bigendian = image_msg.is_bigendian();

		    // ******************************
		    // ******* get the image ********
		    // ******************************
		    slot<cv::Mat> *slot = activity->server->image_cab->getmes();

		    // ******************************
		    // ***** display the image ******
		    // ******************************
		    namedWindow("[MarkerDetection] Received Image", WINDOW_AUTOSIZE );
		    waitKey(100);
	    	imshow("Display Received Image", *(slot->data));


		    // ******************************
		    // ***** process the image ******
		    // ******************************
		    std::vector<aurora::perception::VisualObject3D> objects;
		    double roll, pitch, yaw;
			bool result = activity->marker_detector->detectMarkers(*(slot->data), objects);

			std::cout << "[MarkerDetection] Number of detected markers = "<<objects.size()<<std::endl;

			if(result) {
				// create the marker message with the list of detected markers
				double qx, qy, qz, qw;

				aurora_msgs::msg::dds_::Object3D_ marker_msg;
				aurora_msgs::msg::dds_::Object3DArray_ marker_set_msg;

				for(int i=0; i < objects.size(); i++) {
					marker_msg.header().stamp().sec(time_stamp.tv_sec);
					marker_msg.header().stamp().nanosec(time_stamp.tv_sec);

					marker_msg.header().frame_id(image_msg.header().frame_id());

					marker_msg.type("marker_aruco");
					marker_msg.id(objects[i].id);

					marker_msg.size().x(objects[i].width); // width
					marker_msg.size().y(objects[i].depth); // length
					marker_msg.size().z(objects[i].height); // height

					marker_msg.confidence(objects[i].confidence);

					marker_msg.pose().pose().position().x(objects[i].rototras.tra[0]);
					marker_msg.pose().pose().position().y(objects[i].rototras.tra[1]);
					marker_msg.pose().pose().position().z(objects[i].rototras.tra[2]);

					objects[i].rototras.getQuaternion(qx, qy, qz, qw);
					marker_msg.pose().pose().orientation().x(qx);
					marker_msg.pose().pose().orientation().y(qy);
					marker_msg.pose().pose().orientation().z(qz);
					marker_msg.pose().pose().orientation().w(qw);

					marker_set_msg.objects().push_back(marker_msg);

					std::cout<<"[MarkerDetection] Marker " << objects[i].id << " Pose("<< objects[i].rototras.tra[0] << ", " << objects[i].rototras.tra[1] << ", " << objects[i].rototras.getYaw() << ")"<<std::endl;
				}
				activity->markersPub.publish(&marker_set_msg);
			}

		    // ******************************
		    // ****** release the slot ******
		    // ******************************
		    activity->server->image_cab->release(slot);

		}
	// ################ USER DEFINED END ####################
}


void MarkerDetection::imageConnectionCallback(VariantActivity* va, std::string port, bool matched, int num_connections) {
	MarkerDetection* activity = (MarkerDetection*) va;
	std::cout << "[MarkerDetection]::" << port;
	if(matched) {
		std::cout << "  CONNECTED #=" << num_connections << std::endl;
	}
	else
		std::cout << "  DISCONNECTED #=" << num_connections << std::endl;
}

void MarkerDetection::markersConnectionCallback(VariantActivity* va, std::string port, bool matched, int num_connections) {
	MarkerDetection* activity = (MarkerDetection*) va;
	std::cout << "[MarkerDetection]::" << port;
	if(matched) {
		std::cout << "  CONNECTED #=" << num_connections << std::endl;
	}
	else
		std::cout << "  DISCONNECTED #=" << num_connections << std::endl;
}

} // namespace components
} // namespace aurora
