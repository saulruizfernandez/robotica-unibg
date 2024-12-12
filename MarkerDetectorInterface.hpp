/********************************************************************************
 *
 * MarkerDetectorInterface
 *
 * Copyright (c) 2021
 * All rights reserved.
 *
 * Davide Brugali, Università degli Studi di Bergamo
 *
 * -------------------------------------------------------------------------------
 * File: MarkerDetectorInterface.hpp
 * Created: February 20, 2021
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
#ifndef AURORA_PERCEPTION_MARKER_DETECTOR_INTERFACE_H
#define AURORA_PERCEPTION_MARKER_DETECTOR_INTERFACE_H

#include "aurora/perception_core/VisualObject3D.hpp"

#include <stresa/core/runtime/StresaFunctionality.hpp>
#include <stresa/core/runtime/VProperty.hpp>

#include <stresa/core/scenegraph/SceneTransform.hpp>

#include <opencv2/opencv.hpp>

using namespace stresa;

namespace aurora {
namespace functionality {
namespace perception {

class MarkerDetectorInterface : public StresaFunctionality {
public:
	void setup() {
		std::cout << "[MarkerDetectorInterface] setup." << std::endl;
	};

	void dismiss() {
		std::cout << "[MarkerDetectorInterface] dismiss." << std::endl;
	}

	virtual bool detectMarkers(cv::Mat &frame, std::vector<aurora::perception::VisualObject3D> &objects) = 0;

protected:
};

} // namespace perception
} // namespace functionality
} // namespace aurora

#endif
