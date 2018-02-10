#include <brdf.hpp>
using namespace rt;

Vector3f cosineSampleHemisphere(float u1, float u2){
  const float r = sqrt(u1);
  const float theta = 2 * M_PI * u2;

  const float x = r * cos(theta);
  const float y = r * sin(theta);

  return Vector3f(x, y, sqrt(fmax(0.0f, 1 - u1)));
}


void BRDF::specular(
  const ray_t &iray,
  const Vector3f &n,
  ray_t &rray,
  material_t *mat,
  float &pdf)
{
  Vector3f r = (iray.direction - 2.0 * n.dot(iray.direction) * n).normalized();

  Vector3f w = r;
  Vector3f u = Vector3f(0.00424, 1, 0.00764).cross(w);
  u.normalize();
  Vector3f v = u.cross(w);

  Vector3f sp = cosineSampleHemisphere(erand48(), erand48())
  Vector3f wi = sp.x() * u + sp.y() * v + sp.z() * w;

  if (n.dot(wi) < 0.0)  // reflected ray is below surface
    wi = -sp.x() * u - sp.y() * v + sp.z() * w;

  float phong_lobe = pow(r.dot(wi), mat->get_shininess());
  pdf = phong_lobe * (n.dot(wi));

  return mat->get_specular() * phong_lobe;
}