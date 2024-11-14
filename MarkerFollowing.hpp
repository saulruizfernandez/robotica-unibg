/********************************************************************************
 *
 * Exploration
 *
 * Copyright (c) 2021
 * All rights reserved.
 *
 * Davide Brugali, Università degli Studi di Bergamo
 *
 * -------------------------------------------------------------------------------
 * File: Exploration.hpp
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
#ifndef AURORA_MARKER_FOLLOWING_H
#define AURORA_MARKER_FOLLOWING_H

#include <stresa/core/runtime/VActivity.hpp>
#include <stresa/core/runtime/VariantActivity.hpp>
#include <stresa/core/runtime/VFunctionality.hpp>
#include <stresa/core/runtime/VProperty.hpp>
#include <stresa/core/runtime/VPublisher.hpp>
#include <stresa/core/runtime/VSubscriber.hpp>

#include <stresa/core/runtime/SceneGraph.hpp>
#include <stresa/core/scenegraph/SceneTransform.hpp>

#include "aurora/rover_msgsPubSubTypes.h"		// Rover_info
#include "aurora/nav_msgsPubSubTypes.h"			// Odometry, Path
#include "aurora/aurora_msgsPubSubTypes.h" 		// Object3D

#include "aurora/nav2d_core/RoverInfo.hpp"
#include "aurora/perception_core/VisualObject3D.hpp"

#include "aurora/functionality/navigation/TargetTrackerInterface.hpp"

#include "VisualNavigator.hpp"

#include <stdint.h>  	// uint_64t:          Data type for unsigned integer 64 bits.
#include <limits>
#include <vector>
#include <boost/thread/mutex.hpp>

using namespace stresa;

namespace aurora {
namespace components {

class MarkerFollowing : public VActivity {
public:
	MarkerFollowing(VisualNavigator *map_explorer);
	~MarkerFollowing();
	static void roverInfoCallback(VariantActivity* va);
	static void odometryCallback(VariantActivity* va);
	static void pathEventCallback(VariantActivity* va);
	static void visualObjectsCallback(VariantActivity* va);

	static void roverInfoConnectionCallback(VariantActivity* va, std::string port, bool matched, int num_connections);
	static void roverOdomConnectionCallback(VariantActivity* va, std::string port, bool matched, int num_connections);
	static void pathEventConnectionCallback(VariantActivity* va, std::string port, bool matched, int num_connections);
	static void visualObjectsConnectionCallback(VariantActivity* va, std::string port, bool matched, int num_connections);

	static void targetConnectionCallback(VariantActivity* va, std::string port, bool matched, int num_connections);

protected:
	void init();
	void reconfigure();
	void skip();
	void missed();
	void task();
	void quit();

private:
	VisualNavigator *map_explorer;

	VSubscriber<rover_msgs::msg::dds_::RoverInfo_PubSubType> roverInfoSub;
	VSubscriber<nav_msgs::msg::dds_::Odometry_PubSubType> odometrySub;
	VSubscriber<nav_msgs::msg::dds_::PathEvent_PubSubType> pathEventSub;
	VSubscriber<aurora_msgs::msg::dds_::Object3DArray_PubSubType> visual_objectsSub;

	VPublisher<geometry_msgs::msg::dds_::PoseStamped_PubSubType> targetPub;

	VPublisher<geometry_msgs::msg::dds_::PoseArray_PubSubType>  markersPub;


	VFunctionality<aurora::functionality::navigation::TargetTrackerInterface> marker_tracker;

	VProperty<double> max_speed;

	SceneGraph *scene_graph;

	// ################ USER DEFINED BEGIN ##################
	aurora::navigation::RoverInfo rover_info;
	bool rover_initialized;
    bool odometry_initialized;	// true when the first odometry transformation has been received
	bool follower_initialized;
	bool trajectory_done;

	SceneTransform robot_to_marker;
	SceneTransform map_to_target;
	bool new_target;

	double ox, oy, ot;

	SceneTransform odom2base;
	SceneTransform map2odom;
	SceneTransform map2base;

	void publishTarget(SceneTransform &target, std::string &frame_id);

	std::vector<SceneTransform> marker_poses;
	geometry_msgs::msg::dds_::PoseArray_ array_marker_msg;
	void publishMarkerPoses(std::string map_frame_id);

// ################ USER DEFINED END ####################
};
} // namespace components
} // namespace aurora
  

#endif
