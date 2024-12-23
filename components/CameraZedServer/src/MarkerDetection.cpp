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

MarkerDetection::MarkerDetection(CameraZedServer* server) : VActivity() {
	this->server = server;
	scene_graph = init_SceneGraph();
	
    	registerProperty(process_left_image, "ProcessLeftImage");
    	registerProperty(process_right_image, "ProcessRightImage");
	
    	registerSubscriber(&cameraInfoSub, "CameraInfoSub", "camera_info", cameraInfoCallback);
    
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

	cameraInfoCallback(this);
}

void MarkerDetection::quit() {
	std::cout << "[MarkerDetection]::quit()" << std::endl;
}

void MarkerDetection::cameraInfoCallback(VariantActivity* va) {
	MarkerDetection* activity = (MarkerDetection*) va;

	// ################ USER DEFINED BEGIN ##################

	// Take data
    	sensor_msgs::msg::dds_::CameraInfo_ camera_info_msg;
	SampleInfo m_info;

	if(activity->cameraInfoSub.takeNextData(&camera_info_msg, &m_info)) {
	std::cout << "[MarkerDetection] imageCallback *** *** ***\n" << std::endl;
	    	// *********************************
	    	// *** get the image properties ****
	    	// *********************************
		timespec time_stamp;
		time_stamp.tv_sec = camera_info_msg.header().stamp().sec();
		time_stamp.tv_nsec = camera_info_msg.header().stamp().nanosec();

		int img_height = camera_info_msg.height();
		int img_width  = camera_info_msg.width();
//		int image_step = camera_info_msg.step();
//		bool is_bigendian = camera_info_msg.is_bigendian();

	    	// ***********************************
	    	// ******* process the images ********
	    	// ***********************************
   		slot<aurora::functionality::perception::CameraStereoData> *slot_camera_data = activity->server->camera_data_cab->getmes();
	      	if (slot_camera_data == NULL)
            		return;
      		aurora::functionality::perception::CameraStereoData *camera_data = slot_camera_data->data;
	    	std::vector<aurora::perception::VisualObject3D> left_objects;
	    	std::vector<aurora::perception::VisualObject3D> right_objects;
		    	
		// process the left image
		bool left_detected = false;
	        if (activity->process_left_image && !camera_data->left_image.empty()) {
			std::cout << "[MarkerDetection] process left image\n" << std::endl;
	            	cv::imshow("Left Image", camera_data->left_image);
			left_detected = activity->marker_detector->detectMarkers(camera_data->left_image, left_objects);
	        }

		// process the right image
	    	bool right_detected = false;
      		if (activity->process_right_image && !camera_data->right_image.empty()) {
			std::cout << "[MarkerDetection] process right image\n" << std::endl;
	            	cv::imshow("Right Image", camera_data->right_image);
			right_detected = activity->marker_detector->detectMarkers(camera_data->right_image, right_objects);
	        }
		waitKey(100);
        
      		// release the cab
      		activity->server->camera_data_cab->release(slot_camera_data);
        
		// process the marker pair
		bool pair_detected;
		if(left_detected && right_detected) {
			pair_detected = true;
		}


	    	// *****************************************
	    	// ***** publish the detected markers ******
	    	// *****************************************
		if(pair_detected) {
			double focalLenght = 0.00212;
			double cameraPixelSize = 4e-6;
			double baseline = 0.12;
			if (right_objects.size() == left_objects.size()){
				for (int i = 0; i< right_objects.size();i++){
					int disparity = left_objects[i].rototras.tra[0] - right_objects[i].rototras.tra[0];
					double distance = (focalLenght/cameraPixelSize) * (baseline/disparity);
					std::cout<<"Disctance: "<<distance<<std::endl;
				}
			}
/*			
			// create the marker message with the list of detected markers
			double qx, qy, qz, qw;

			aurora_msgs::msg::dds_::Object3D_ marker_msg;
			aurora_msgs::msg::dds_::Object3DArray_ marker_set_msg;

			for(int i=0; i < objects.size(); i++) {
				marker_msg.header().stamp().sec(time_stamp.tv_sec);
				marker_msg.header().stamp().nanosec(time_stamp.tv_sec);
				marker_msg.header().frame_id(camera_info_msg.header().frame_id());

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

				std::cout<<"[MarkerDetection] Marker " << objects[i].id << " Pose("<< objects[i].rototras.tra[0] << ", " << 
				objects[i].rototras.tra[1] << ", " << objects[i].rototras.getYaw() << ")"<<std::endl;
			}
			activity->markersPub.publish(&marker_set_msg);
*/				
		}
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
