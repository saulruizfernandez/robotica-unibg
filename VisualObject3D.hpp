/********************************************************************************
 *
 * VisualObject3D
 *
 * Copyright (c) 2024
 * All rights reserved.
 *
 * Davide Brugali, Università degli Studi di Bergamo
 *
 * -------------------------------------------------------------------------------
 * File: VisualObject.hpp
 * Created: October 15, 2024
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
#ifndef AURORA_PERCEPTION_CORE_VISUAL_OBJECT_3D_HPP
#define AURORA_PERCEPTION_CORE_VISUAL_OBJECT_3D_HPP

#include <stresa/core/scenegraph/SceneTransform.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <string.h>
#include <sstream>
//#include <stdint.h>  	// uint64_t:          Data type for unsigned integer 64 bits.
//#include <vector>

#define VISUAL_OBJECT "VisualObject"

namespace aurora {
namespace perception {

class VisualObject3D {
public:
	VisualObject3D() {
		type = VISUAL_OBJECT;
		is_velocity_set = false;
	}

    std::string id;			// generated id
    std::string type;		// object type: e.g. "Door", "Person"
	double confidence;		// detection uncertainty
	int bbox[4];    		// boundig box [pixel] Top left corner (x, y) width, height

	std::string frame;		// the reference frame where the object position is defined
	SceneTransform rototras;// object position and orientation in the "frame" reference frame
	double width;			// x [m]
	double depth;			// y [m]
	double height;			// z [m]

	double vx, vy, vz;		// object velocity with respect to the "frame" reference frame
	bool is_velocity_set;	// true if velocity has been computed

	void set(VisualObject3D &obj) {
		this->id = obj.id;
		this->type = obj.type;
		this->confidence = obj.confidence;
		this->bbox[0] = obj.bbox[0];
		this->bbox[1] = obj.bbox[1];
		this->bbox[2] = obj.bbox[2];
		this->bbox[3] = obj.bbox[3];
		this->rototras.setTransform(obj.rototras);
		this->rototras.stamp = obj.rototras.stamp;
		this->width = obj.width;
		this->depth = obj.depth;
		this->height = obj.height;
		this->vx = obj.vx;
		this->vy = obj.vy;
		this->vz = obj.vz;
	}
};

} // namespace perception
} // namespace aurora

#endif
