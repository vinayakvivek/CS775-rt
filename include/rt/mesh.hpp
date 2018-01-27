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
    Vector3f N;
    triangle_t() {}
    triangle_t(const Vector3f &a, const Vector3f &b, const Vector3f &c) {
      A = a; B = b; C = c;
      N = ((B-A).cross(C-B)).normalized();
    }
  };

  class mesh_t : public object_t {

    std::string inputfile;
    std::vector<triangle_t> triangles;
    int num_triangles;

    material_t* mat;
    // texture_t* tex;

    Vector3f center;
    transform_t transform;

   public:

    mesh_t(material_t* _mat, std::string file_name, Vector3f c);
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

    /// Prints information about the sphere. to stream.
    virtual void print(std::ostream &stream) const;
  };
}