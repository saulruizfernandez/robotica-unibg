/********************************************************************************
 *
 * PluginMarkerTracker
 *
 * Copyright (c) 2023
 * All rights reserved.
 *
 * Davide Brugali, Università degli Studi di Bergamo
 *
 * -------------------------------------------------------------------------------
 *
 * File: PluginMarkerTrackerAruco.cpp
 * Created: Sep. 19, 2021
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
#ifndef AURORA_PLUGIN_MARKER_TRACKER
#define AURORA_PLUGIN_MARKER_TRACKER

#include <math.h>
#include <stdlib.h> /* atoi */

#include <fstream>
#include <iostream>
#include <string>

#include "aurora/functionality/navigation/TargetTrackerInterface.hpp"
#include "aurora/geometry/Transform3D.hpp"
#include "aurora/nav2d_control/PoseControlDifferentialDrive.hpp"
#include "aurora/nav2d_core/RoverInfo.hpp"
#include "aurora/nav2d_core/WayPoint2D.hpp"

using namespace stresa;

namespace aurora {
namespace functionality {
namespace navigation {

class PluginMarkerTracker : public TargetTrackerInterface {
 public:
  PluginMarkerTracker() { registerProperty(min_length, "MinFrontierLength"); }

  ~PluginMarkerTracker() {
    delete robot_to_marker;
    delete next_robot_to_marker;
  }

  void setup() {
    TargetTrackerInterface::setup();
    current_marker_id = "NULL";
    first_marker = true;
    trajectory_done = false;
    robot_to_marker = NULL;
    next_robot_to_marker = new SceneTransform();
  }

  void dismiss() { TargetTrackerInterface::dismiss(); }

  void setRoverInfo(aurora::navigation::RoverInfo &rover_info) {}

  void setOdometry(double px, double py, double oz, double vx, double vy,
                   double wz) {
    if (!first_marker && vx == 0 && vy == 0) {
      std::cout << "Odometry : TRAJECTORY DONE" << std::endl;
      trajectory_done = true;
    }
  }

  void setNavigationStatus(int status) {
    std::cout << "Navigation Status: " << status << std::endl;

    if (status == 0) {
      std::cout << "Navigation Status : TRAJECTORY DONE" << std::endl;
      trajectory_done = true;
    }
  }

  /**
   * @brief Checks the closest marker to the robot and sets it as the new target.
   * 
   * @param objects 
   */
  void addVisualObject(
      std::vector<aurora::perception::VisualObject3D> &objects) {
    double minimiun = std::numeric_limits<double>::max();
    std::string idTemp = "";
    SceneTransform *robot2marker = NULL;
    for (int i = 0; i < objects.size(); i++) {
      if (objects[i].type.compare("marker") == 0 &&
          current_marker_id.compare(objects[i].id) != 0) {
        double distanceTemp =
            sqrt(objects[i].rototras.tra[0] * objects[i].rototras.tra[0] +
                 objects[i].rototras.tra[1] * objects[i].rototras.tra[1]);
        std::cout << "Marker " << objects[i].id << " distance " << distanceTemp
                  << std::endl;

        if (distanceTemp <= minimiun) {
          minimiun = distanceTemp;
          idTemp = objects[i].id;
          if (robot2marker != nullptr) delete robot2marker;
          robot2marker = new SceneTransform();
          robot2marker->tra[0] = objects[i].rototras.tra[0];
          robot2marker->tra[1] = objects[i].rototras.tra[1];
          robot2marker->tra[2] = objects[i].rototras.tra[2];
        }
      }
    }

    if (robot_to_marker == NULL && robot2marker != NULL) {
      std::cout << "FIRST TARGET: " << idTemp << std::endl;
      robot_to_marker = new SceneTransform();
      *robot_to_marker = *robot2marker;  // The first target
      current_marker_id = idTemp;
    } else if (robot2marker != NULL) {
      std::cout << "NEXT TARGET FOUND: " << idTemp
                << " -- Yaw: " << robot2marker->getYaw() << std::endl;
      *next_robot_to_marker = *robot2marker;
      next_robot_to_marker_id = idTemp;
    }

    if (robot2marker != nullptr) delete robot2marker;
  }

  bool getTarget(SceneTransform &target, timespec &time_stamp) {
    if (trajectory_done) {
      std::cout << "TRAJECTORY DONE : New target" << std::endl;
      target = *next_robot_to_marker;
      *robot_to_marker = *next_robot_to_marker;
      trajectory_done = false;
      current_marker_id = next_robot_to_marker_id;
    } else if (first_marker == true) {
      target = *robot_to_marker;
      first_marker = false;
    } else {
      std::cout << "No new target" << std::endl;
      return false;
    }
    time_stamp = marker_time_stamp;
    return true;
  }

 protected:
  VProperty<int> min_length;

 private:
  std::string current_marker_id;
  std::string next_robot_to_marker_id;
  bool first_marker;
  bool trajectory_done;
  SceneTransform *robot_to_marker;
  SceneTransform *next_robot_to_marker;

  timespec marker_time_stamp;
};

extern "C" BOOST_SYMBOL_EXPORT PluginMarkerTracker marker_tracker;
PluginMarkerTracker marker_tracker;

}  // namespace navigation
}  // namespace functionality
}  // namespace aurora
#endif
