#pragma once

#include <color.hpp>
#include <object.hpp>
#include <ray.hpp>
#include <scene.hpp>
#include <material.hpp>

namespace rt
{
  class BRDF {
   public:
    static color_t specular(
      const ray_t &iray,
      const Vector3f &n,
      ray_t &rray,
      material_t *mat,
      float &pdf);
  };
}