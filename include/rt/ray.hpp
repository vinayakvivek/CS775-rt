/*
    This file is part of rt.

    rt is a simple ray tracer meant to be used for teaching ray tracing.

    Copyright (c) 2018 by Parag Chaudhuri

	Some parts of rt are derived from Nori by Wenzel Jacob.
	https://github.com/wjakob/nori/

    rt is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    rt is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <utils.hpp>

namespace rt
{
	/**
	 * \brief Class to represent a ray in rt.
	 **/
	class ray_t
	{
	public:
		/// Ray origin.
		Vector3f origin;
		/// Ray direction.
		Vector3f direction;

		/// Max and min t values on the ray - an intersection is valid if it lies between these two values.
		float mint, maxt;

		/// Constructor.
		ray_t();
		/// Constructor.
		ray_t(const Vector3f& _o, const Vector3f& _d);
		/// Constructor.
		ray_t(const Vector3f& _o, const Vector3f& _d, const float _mint, const float _maxt);
		/// Copy Constructor.
		ray_t(const ray_t &_ray);
		/// Copy Constructor, with modifications.
		ray_t(const ray_t &_ray, float _mint, float _maxt);

		/// Destructions
		virtual ~ray_t();

		/// Returns a point along the ray at _t
		const Vector3f operator()(const float _t) const;
	};
}