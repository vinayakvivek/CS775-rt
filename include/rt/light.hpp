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

#include <color.hpp>
#include <material.hpp>
#include <scene.hpp>
#include <utils.hpp>
#include <object.hpp>
#include <mesh.hpp>

namespace rt
{
	/// Forward Declaration.
	class scene_t;
	class light_t;

	struct light_hit_t {
    const light_t *light;
    float t;
    Vector3f normal;

    light_hit_t() {}
    light_hit_t(const light_t *_l, float _t, Vector3f _n) {
      light = _l;
      t = _t;
      normal = _n;
    }
  };

	/**
	 * \brief  This is the abstract base class for lights.
	 * The light is responsible for returning its direct illumination estimate to a point.
	 **/
	class light_t
	{
	protected:

    /// Color of the light. This can be thought of as radiance emitted by the light source.
    Vector3f col;
    /// An ambient coefficient. Modulate col with ka to get ambient component of illumination.
    float ka;
    /// number of shadowrays
    int num_shadowrays;

	public:

		/// Constructor
		light_t(const Vector3f& _col, const float _ka, int _nshadowrays = 1);

		/// Destructon
		virtual ~light_t();

		/**
		* Returns the direct illumination estimate for the point hitpt, where the surface normal is normal, material is mat.
		* Scene is passed so that the camera position, and the objects can be used for computing the specular component
		* of direct illumination and the shadow computations.
		**/
		color_t direct(const Vector3f& hitpt, const ray_t &view_ray, const Vector3f& normal, const material_t* mat, const scene_t* scn) const;

    virtual Vector3f sample_point() const = 0;

		virtual bool intersect(light_hit_t& result, const ray_t& _ray) const = 0;

		color_t get_color() const {return color_t(col.x(), col.y(), col.z());}
		float get_ka() const {return ka;}

		/// Prints information about the light to the stream.
		virtual void print(std::ostream &stream) const = 0;
	};

	/**
	 * \brief This is the class for a positional point light source.
	 **/
	class point_light_t : public light_t
	{
	private:
		/// Position of the light
		Vector3f pos;

	public:
		point_light_t(const Vector3f& _pos, const Vector3f& _col, const float _ka);
		virtual ~point_light_t();

    virtual bool intersect(light_hit_t& result, const ray_t& _ray) const;
    virtual Vector3f sample_point() const;

		virtual void print(std::ostream &stream) const;
	};

	class area_light_t : public light_t
	{
			Vector3f center;
			Vector3f normal;
			Vector3f radius;

			transform_t transform;

		public:

			area_light_t(
        const Vector3f &_center,
        const Vector3f _normal,
        const Vector3f _radius,
        const Vector3f& _col,
        float _ka,
        int _num_shadowrays);
			virtual ~area_light_t() {}

			virtual Vector3f sample_point() const;
			virtual bool intersect(light_hit_t& result, const ray_t& _ray) const;

      virtual void print(std::ostream &stream) const;
	};

  class rect_light_t : public light_t
  {
      Vector3f center;
      Vector3f normal;
      Vector3f a, b;

      transform_t transform;

    public:

      rect_light_t(
        const Vector3f &_center,
        const Vector3f _normal,
        const Vector3f _a,
        const Vector3f _b,
        const Vector3f& _col,
        float _ka,
        int _num_shadowrays);
      virtual ~rect_light_t() {}

      virtual Vector3f sample_point() const;
      virtual bool intersect(light_hit_t& result, const ray_t& _ray) const;

      virtual void print(std::ostream &stream) const;
  };
}