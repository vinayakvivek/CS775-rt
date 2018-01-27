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

#include <material.hpp>
#include <ray.hpp>
#include <utils.hpp>

namespace rt
{
	/// Forward declaration
	class object_t;

	/// Convenience typedef to reprent a hit (intersection) point
	// typedef std::pair<const object_t*, float> hit_t;

  struct hit_t {
    const object_t *obj;
    float t;
    Vector3f normal;

    hit_t() {}
    hit_t(const object_t *o, float _t, Vector3f _n) {
      obj = o;
      t = _t;
      normal = _n;
    }
  };

	/**
	 * \brief This is an abstract base class to represent objects in rt.
	 **/
	class object_t
	{
	public:
		/// Constructor
		object_t();

		/// Destructor
		virtual ~object_t();

		/// Returns a name for the object
		virtual std::string get_name(void) const = 0;

		/**
		* Returns true if the _ray hits this object. The hit information is returned in result.
		* This is not valid if there is no intersection and the function returns false.
		**/
		virtual bool intersect(hit_t& result, const ray_t& _ray) const = 0;

		/// Returns the normal to the surface at point _p.
		virtual Vector3f get_normal(Vector3f& _p) const = 0;

    /// Returns the texture color at the point surface _p.
    virtual color_t get_texture(Vector3f &_p) const = 0;

		/// Returns the material for the object.
		virtual material_t* get_material(void) const = 0;

		/// Prints information about the object to stream.
		virtual void print(std::ostream &stream) const = 0;
	};
}