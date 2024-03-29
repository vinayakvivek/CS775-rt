#pragma once

#include <material.hpp>
#include <object.hpp>
#include <ray.hpp>
#include <utils.hpp>
#include <texture.hpp>
#include <transform.hpp>

#include <tiny_obj_loader.hpp>

namespace rt
{
  struct triangle_t {
    Vector3f A, B, C;
    Vector3f nA, nB, nC;
    Vector3f N;
    triangle_t() {}
    triangle_t(const Vector3f &a, const Vector3f &b, const Vector3f &c) {
      A = a; B = b; C = c;
      N = ((B-A).cross(C-A)).normalized();
      nA = nB = nC = N;
    }
    triangle_t(
      const Vector3f &a, const Vector3f &b, const Vector3f &c,
      const Vector3f &na, const Vector3f &nb, const Vector3f &nc);
    bool intersect(const ray_t &ray, float &t, Vector3f &normal) const;
  };

  struct plane_t {
    // Ax + By + Cz + D = 0
    Vector3f N;  // normal (A, B, C)
    float D;
    plane_t() {}
    plane_t(const Vector3f &n, const Vector3f &p) {
      N = n.normalized();
      D = -N.dot(p);
    }
    bool intersect(const ray_t &ray, float &t, bool &entering) const;
  };

  struct bounding_box_t {
    plane_t faces[6];
    bounding_box_t() {}
    bounding_box_t(
      const Vector3f &rft,  // right-front-top
      const Vector3f &lbb   // left-back-bottom)
    );
    bounding_box_t(
      const Vector3f &rft,  // right-front-top
      const Vector3f &lbb,   // left-back-bottom)
      const transform_t &transform
    );
    bool intersect(const ray_t &ray, float &t) const;
  };

  class mesh_t : public object_t {

    std::string inputfile;
    std::vector<triangle_t> triangles;
    int num_triangles;

    Vector3f center;
    bounding_box_t bounding_box;
    transform_t transform;

    bool show_bounding_box;

   public:

    mesh_t(
      material_t* _mat,
      const color_t &_color,
      std::string texture_file,
      std::string obj_file,
      Vector3f _center,
      Vector3f _scale,
      Vector3f _rot,
      bool show_bounding_box = false);

    virtual ~mesh_t();

    /// Returns the mandatory object name
    std::string get_name(void) const { return std::string("mesh"); }

    /**
    * Returns true if the _ray hits this object. The hit information is returned in result.
    * This is not valid if there is no intersection and the function returns false.
    **/
    bool intersect(hit_t& result, const ray_t& _ray) const;

    /// Returns the normal to the surface at point _p.
    Eigen::Vector3f get_normal(Eigen::Vector3f& _p) const;

    color_t get_texture(Vector3f &_p) const;

    /// Returns the material for the sphere.
    material_t* get_material(void) const;

    color_t get_color() const;

    /// Prints information about the sphere. to stream.
    virtual void print(std::ostream &stream) const;
  };
}