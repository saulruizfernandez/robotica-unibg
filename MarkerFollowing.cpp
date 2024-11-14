/********************************************************************************
 *
 * MarkerFollowing
 *
 * Copyright (c) 2021
 * All rights reserved.
 *
 * Davide Brugali, Università degli Studi di Bergamo
 *
 * -------------------------------------------------------------------------------
 * File: MarkerFollowing.cpp
 * Created: November 05, 2021
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

#include "MarkerFollowing.hpp"

#include <iostream>
#include <sstream>


using namespace stresa;

namespace aurora {
namespace components {

MarkerFollowing::MarkerFollowing(VisualNavigator *map_explorer) : VActivity() {
	std::cout << "[MarkerFollowing]::MarkerFollowing" << std::endl;

	this->map_explorer = map_explorer;

	scene_graph = init_SceneGraph();

	registerSubscriber(&roverInfoSub, "RoverInfoSub", "rt/rover_info"); //, roverInfoCallback, roverInfoConnectionCallback);
	registerSubscriber(&odometrySub, "OdometrySub", "rt/odom"); //, nullptr, roverOdomConnectionCallback);
	registerSubscriber(&pathEventSub, "PathEventSub", "rt/path_status"); //, nullptr, pathEventConnectionCallback);
	registerSubscriber(&visual_objectsSub, "VisualObjectsSub", "visual_objects"); //, nullptr, visualObjectsConnectionCallback);

	registerPublisher(&markersPub, "MarkersPub", "rt/markers");

	registerPublisher(&targetPub, "TargetPub", "target", targetConnectionCallback);

	registerFunctionality(&marker_tracker, "MarkerTracker");

	registerProperty(max_speed, "MaxSpeed");

	follower_initialized = false;
	rover_initialized = false;
    odometry_initialized = false;

    trajectory_done = false;
}

MarkerFollowing::~MarkerFollowing() {
}


void MarkerFollowing::init() {
	std::cout << "[MarkerFollowing]::init" << std::endl;
}

void MarkerFollowing::reconfigure() {
	std::cout << "[MarkerFollowing]::setup" << std::endl;
}

void MarkerFollowing::task() {
	std::cout << "\n----------[MarkerFollowing]::task----------" << std::endl;

	// check if the rover parameters have been updated
	roverInfoCallback(this);

	// check if the odometry has been updated
	odometryCallback(this);

	if(! follower_initialized) {
		if(rover_initialized && odometry_initialized) {
			follower_initialized = true;
			std::cout << "follower_initialized" << std::endl;
		}
		else {
			std::cout << "*** FOLLOWER NOT INITIALIZED:";
			if(! rover_initialized)
					std::cout << " ! rover_initialized" << std::endl;
			if(! odometry_initialized)
				std::cout << " ! odometry_initialized" << std::endl;
			return;
		}
	}

	//MODIFICHE
	pathEventCallback(this);
	//MODIFICHE

	// receive the visual markers
	visualObjectsCallback(this);


	// check if there is a new target
	timespec target_time_stamp;
	new_target = marker_tracker->getTarget(robot_to_marker, target_time_stamp);

	// get the robot pose in the map reference frame
	SceneTransform map_to_robot;
	std::string map_frame_id = "slam_gridmap";

	bool result = scene_graph->getTransform(map_frame_id, "base_link", map_to_robot, target_time_stamp);
	if(!result) {
		std::cout << "[MarkerFollowing] ERROR : cannot get the " << map_frame_id << " to base_link" << " transformation" << std::endl;
		return;
	}

	if(new_target) {
		// compute the map to target transformation
		map_to_target = map_to_robot * robot_to_marker;

		// publish the new target
		publishTarget(map_to_target, map_frame_id);
	}

	// transform the marker poses in the map reference frame
	SceneTransform map_to_marker;
	for(int i = 0; i<marker_poses.size(); i++){
		map_to_marker = map_to_robot * marker_poses[i];
		marker_poses[i].setTransform(map_to_marker);
	}
	// publish the marker poses
	publishMarkerPoses(map_frame_id);


	// ################ USER DEFINED END ####################
}

void MarkerFollowing::skip() {
	std::cout << "[MarkerFollowing]::skip()" << std::endl;
}
void MarkerFollowing::missed() {
	std::cout << "[MarkerFollowing]::missed()" << std::endl;
}

void MarkerFollowing::quit() {
	std::cout << "[MarkerFollowing]::quit()" << std::endl;
}


void MarkerFollowing::publishTarget(SceneTransform &target, std::string &frame_id) {

	geometry_msgs::msg::dds_::PoseStamped_ target_msg;

	target_msg.header().frame_id(frame_id);

	target_msg.pose().position().x(target.tra[0]);
	target_msg.pose().position().y(target.tra[1]);
	target_msg.pose().position().z(target.tra[2]);

	double qx, qy, qz, qw;
	target.getQuaternion(qx, qy, qz, qw);
	target_msg.pose().orientation().x(qx);
	target_msg.pose().orientation().y(qy);
	target_msg.pose().orientation().z(qz);
	target_msg.pose().orientation().w(qw);

	targetPub.publish(&target_msg);
	std::cout << "\n*******************************" << std::endl;
	std::cout << "***************** TARGET (" << target.tra[0] << ", " << target.tra[1] << ", " << target.getYaw() << ") ****"<<std::endl;
	std::cout << "\n*******************************\n" << std::endl;
}


void MarkerFollowing::publishMarkerPoses(std::string map_frame_id) {
	array_marker_msg.header().frame_id(map_frame_id);
	array_marker_msg.poses().clear();

	double qx, qy, qz, qw;
	for(int i=0; i < marker_poses.size(); i++) {
		// add the marker pose to the pose message
		geometry_msgs::msg::dds_::Pose_ marker_pose;
		marker_pose.position().x(marker_poses[i].tra[0]);
		marker_pose.position().y(marker_poses[i].tra[1]);
		marker_pose.position().z(0.0);

		marker_poses[i].getQuaternion(qx, qy, qz, qw);
		marker_pose.orientation().x(qx);
		marker_pose.orientation().y(qy);
		marker_pose.orientation().z(qz);
		marker_pose.orientation().w(qw);

		array_marker_msg.poses().push_back(marker_pose);
	}

	markersPub.publish(&array_marker_msg);

}



void MarkerFollowing::roverInfoCallback(VariantActivity* va) {
	MarkerFollowing* activity = (MarkerFollowing*) va;

// ################ USER DEFINED BEGIN ##################
	// Take data
	rover_msgs::msg::dds_::RoverInfo_ rover_info_msg;
	SampleInfo m_info;
	if(activity->roverInfoSub.takeNextData(&rover_info_msg, &m_info)) {
		aurora::navigation::RoverInfo rover_info;
		rover_info.fp_min_x = rover_info_msg.footprint()[0];
		rover_info.fp_max_x = rover_info_msg.footprint()[1];
		rover_info.fp_min_y = rover_info_msg.footprint()[2];
		rover_info.fp_max_y = rover_info_msg.footprint()[3];
		rover_info.fp_min_z = rover_info_msg.footprint()[4];
		rover_info.fp_max_z = rover_info_msg.footprint()[5];

		rover_info.min_lin_vel = rover_info_msg.min_lin_vel();
		rover_info.max_lin_vel = rover_info_msg.max_lin_vel();
		rover_info.min_ang_vel = rover_info_msg.min_ang_vel();
		rover_info.max_ang_vel = rover_info_msg.max_ang_vel();
		rover_info.max_lin_acc = rover_info_msg.max_lin_acc();
		rover_info.max_ang_acc = rover_info_msg.max_ang_acc();
		rover_info.max_cen_acc = rover_info_msg.max_cen_acc();
		rover_info.lin_res = rover_info_msg.lin_resolution();
		rover_info.ang_res = rover_info_msg.ang_resolution();

		activity->rover_initialized = true;
		activity->marker_tracker->setRoverInfo(activity->rover_info);
//		rover_info.print();
	}
// ################ USER DEFINED END ####################
}

void MarkerFollowing::odometryCallback(VariantActivity* va) {
	MarkerFollowing* activity = (MarkerFollowing*) va;

// ################ USER DEFINED BEGIN ##################
	nav_msgs::msg::dds_::Odometry_ odo_msg;
	SampleInfo m_info;

	SceneTransform odom_transform;
	odom_transform.setQuaternion(odo_msg.pose().pose().orientation().x(), odo_msg.pose().pose().orientation().y(),
			odo_msg.pose().pose().orientation().z(), odo_msg.pose().pose().orientation().w());

	if(activity->odometrySub.takeNextData(&odo_msg, &m_info)) {
		activity->odometry_initialized = true;
		activity->marker_tracker->setOdometry(odo_msg.pose().pose().position().x(),
		 		odo_msg.pose().pose().position().y(), odom_transform.getYaw(),
		 		odo_msg.twist().twist().linear().x(),
		 		odo_msg.twist().twist().linear().y(),
		 		odo_msg.twist().twist().angular().z());

		// timespec time_stamp;
		// time_stamp.tv_sec = odo_msg.header().stamp().sec();
		// time_stamp.tv_nsec = odo_msg.header().stamp().nanosec();
		std::string parent_frame = "odom";
		std::string child_frame = "base_link";
	}
// ################ USER DEFINED END ####################
}


void MarkerFollowing::visualObjectsCallback(VariantActivity* va) {
	MarkerFollowing* activity = (MarkerFollowing*) va;

// ################ USER DEFINED BEGIN ##################
	SampleInfo m_info;
	aurora_msgs::msg::dds_::Object3DArray_ object3D_array_msg;
	aurora_msgs::msg::dds_::Object3D_ object3D_msg;

	std::vector<aurora::perception::VisualObject3D> objects;
	aurora::perception::VisualObject3D object;

	if(activity->visual_objectsSub.takeNextData(&object3D_array_msg, &m_info)) {
		// reset the message of the marker poses
		activity->marker_poses.clear();
		std::cout << "\n************ Marker received ************\n" << std::endl;

		// get the markers from the received message
		for(unsigned int i=0; i < object3D_array_msg.objects().size(); i++) {
			object3D_msg = object3D_array_msg.objects()[i];

			//time_stamp = object3D_msg.time_stamp();

			object.type = object3D_msg.type();
			object.id = object3D_msg.id();



			object.confidence = object3D_msg.confidence();

			object.rototras.tra[0] = object3D_msg.pose().pose().position().x();
			object.rototras.tra[1] = object3D_msg.pose().pose().position().y();
			object.rototras.tra[2] = object3D_msg.pose().pose().position().z();

			double qx, qy, qz, qw;
			qx = object3D_msg.pose().pose().orientation().x();
			qy = object3D_msg.pose().pose().orientation().y();
			qz = object3D_msg.pose().pose().orientation().z();
			qw = object3D_msg.pose().pose().orientation().w();

			object.rototras.setQuaternion(qx, qy, qz, qw);
			/*

			std::string type = object3D_msg.type();
			std::string id = object3D_msg.id();

			double size_x = object3D_msg.size().x();
			double size_y = object3D_msg.size().y();
			double size_z = object3D_msg.size().z();

			double confidence = object3D_msg.confidence();

			SceneTransform robot2marker;

			robot2marker.tra[0] = object3D_msg.pose().pose().position().x();
			robot2marker.tra[1] = object3D_msg.pose().pose().position().y();
			robot2marker.tra[2] = object3D_msg.pose().pose().position().z();

			double qx, qy, qz, qw;
			qx = object3D_msg.pose().pose().orientation().x();
			qy = object3D_msg.pose().pose().orientation().y();
			qz = object3D_msg.pose().pose().orientation().z();
			qw = object3D_msg.pose().pose().orientation().w();

			robot2marker.setQuaternion(qx, qy, qz, qw);

			// reference frame (e.g. camera_link) where the object position is defined
			std::string  object_frame_id = object3D_msg.header().frame_id();


			std::string  map_frame_id;

			timespec time_stamp = activity->now();
			*/

			objects.push_back(object);
			std::cout << "\n************ Marker "<<object3D_msg.id()<<" added to the list ************\n" << std::endl;
//			activity->marker_tracker->addVisualObject(type, id, size_x, size_y, size_z, robot2marker, confidence, time_stamp);

			// add the marker pose to the array of marker poses
			//activity->marker_poses.push_back(robot2marker);
			activity->marker_poses.push_back(object.rototras);
		}
		std::cout << "\n************ Elaborating Markers************\n" << std::endl;
		activity->marker_tracker->addVisualObject(objects);
		std::cout << "\n************ Elaborated Markers************\n" << std::endl;
	}
// ################ USER DEFINED END ####################
}


void MarkerFollowing::pathEventCallback(VariantActivity* va) {
	MarkerFollowing* activity = (MarkerFollowing*) va;

// ################ USER DEFINED BEGIN ##################
	// Take data
	nav_msgs::msg::dds_::PathEvent_ status_msg;

	SampleInfo m_info;
	if(activity->pathEventSub.takeNextData(&status_msg, &m_info)) {
		// get the trajectory status
		//std::string status_data = status_msg.data();

		if(status_msg.data().compare("Navigator_ARRIVED") == 0) {
			activity->trajectory_done = true;
			activity->marker_tracker->setNavigationStatus(0);
		}
		else if(status_msg.data().compare("Navigator_BLOCKED") == 0) {
			activity->marker_tracker->setNavigationStatus(-1);
		}
		else if(status_msg.data().compare("Navigator_MOVING") == 0) {
			activity->marker_tracker->setNavigationStatus(1);
		}
	}

// ################ USER DEFINED END ####################
}


void MarkerFollowing::roverInfoConnectionCallback(VariantActivity* va, std::string port, bool matched, int num_connections) {
	std::cout << "MarkerFollowing::" << port;
	if(matched)
		std::cout << "  CONNECTED #=" << num_connections << std::endl;
	else
		std::cout << "  DISCONNECTED #=" << num_connections << std::endl;
}
void MarkerFollowing::roverOdomConnectionCallback(VariantActivity* va, std::string port, bool matched, int num_connections) {
	std::cout << "MarkerFollowing::" << port;
	if(matched)
		std::cout << "  CONNECTED #=" << num_connections << std::endl;
	else
		std::cout << "  DISCONNECTED #=" << num_connections << std::endl;
}
void MarkerFollowing::visualObjectsConnectionCallback(VariantActivity* va, std::string port, bool matched, int num_connections) {
	std::cout << "MarkerFollowing::" << port;
	if(matched)
		std::cout << "  CONNECTED #=" << num_connections << std::endl;
	else
		std::cout << "  DISCONNECTED #=" << num_connections << std::endl;
}
void MarkerFollowing::pathEventConnectionCallback(VariantActivity* va, std::string port, bool matched, int num_connections) {
	std::cout << "MarkerFollowing::" << port;
	if(matched)
		std::cout << "  CONNECTED #=" << num_connections << std::endl;
	else
		std::cout << "  DISCONNECTED #=" << num_connections << std::endl;
}
void MarkerFollowing::targetConnectionCallback(VariantActivity* va, std::string port, bool matched, int num_connections) {
	std::cout << "MarkerFollowing::" << port;
	if(matched)
		std::cout << "  CONNECTED #=" << num_connections << std::endl;
	else
		std::cout << "  DISCONNECTED #=" << num_connections << std::endl;
}

} // namespace components
} // namespace aurora
