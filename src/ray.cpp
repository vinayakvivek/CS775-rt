#include <ray.hpp>

using namespace rt;

ray_t::ray_t():origin(Vector3f(0.0f,0.0f,0.0f)), direction(Vector3f(0.0f,0.0f,1.0f)), mint(EPSILON), maxt(std::numeric_limits<float>::infinity())
{ }

ray_t::ray_t(const Vector3f& _o, const Vector3f& _d):origin(_o), direction(_d), mint(EPSILON), maxt(std::numeric_limits<float>::infinity())
{ }

ray_t::ray_t(const Vector3f& _o, const Vector3f& _d, const float _mint, const float _maxt):origin(_o), direction(_d), mint(_mint), maxt(_maxt)
{ }

ray_t::ray_t(const ray_t &_ray): origin(_ray.origin), direction(_ray.direction),
       mint(_ray.mint), maxt(_ray.maxt)
{ }

ray_t::ray_t(const ray_t &_ray, float _mint, float _maxt): origin(_ray.origin), direction(_ray.direction),
     	mint(_mint), maxt(_maxt)
{ }

ray_t::~ray_t()
{ }

const Vector3f ray_t::operator()(const float _t) const {return (origin + _t*direction);}


bool ray_t::rayTriangleIntersect(
      const Vector3f &v0, const Vector3f &v1, const Vector3f &v2,
      float &t, float &u, float &v) const
{
  const float kEpsilon = 1e-8;

  Vector3f dir = direction;
  Vector3f orig = origin;

  // ----------------- Muller method

  Vector3f v0v1 = v1 - v0;
  Vector3f v0v2 = v2 - v0;

  Vector3f pvec = dir.cross(v0v2);
  float det = v0v1.dot(pvec);

  // if the determinant is negative the triangle is backfacing
  // if the determinant is close to 0, the ray misses the triangle
  if (det < kEpsilon) return false;

  // ray and triangle are parallel if det is close to 0
  if (fabs(det) < kEpsilon) return false;

  float invDet = 1 / det;

  Vector3f tvec = orig - v0;
  u = tvec.dot(pvec) * invDet;
  if (u < 0 || u > 1) return false;

  Vector3f qvec = tvec.cross(v0v1);
  v = dir.dot(qvec) * invDet;
  if (v < 0 || u + v > 1) return false;

  t = v0v2.dot(qvec) * invDet;

  return true;

  // ----------------- normal method

  // // compute plane's normal
  // Vector3f v0v1 = v1 - v0;
  // Vector3f v0v2 = v2 - v0;
  // // no need to normalize
  // Vector3f N = v0v1.cross(v0v2); // N
  // float denom = N.dot(N);

  // // Step 1: finding P

  // // check if ray and plane are parallel ?
  // float NdotRayDirection = N.dot(dir);
  // if (fabs(NdotRayDirection) < kEpsilon) // almost 0
  //     return false; // they are parallel so they don't intersect !

  // // compute d parameter using equation 2
  // float d = N.dot(v0);

  // // compute t (equation 3)
  // t = (N.dot(orig) + d) / NdotRayDirection;
  // // check if the triangle is in behind the ray
  // if (t < 0) return false; // the triangle is behind

  // // compute the intersection point using equation 1
  // Vector3f P = orig + t * dir;

  // // Step 2: inside-outside test
  // Vector3f C; // vector perpendicular to triangle's plane

  // // edge 0
  // Vector3f edge0 = v1 - v0;
  // Vector3f vp0 = P - v0;
  // C = edge0.cross(vp0);
  // if (N.dot(C) < 0) return false; // P is on the right side

  // // edge 1
  // Vector3f edge1 = v2 - v1;
  // Vector3f vp1 = P - v1;
  // C = edge1.cross(vp1);
  // if ((u = N.dot(C)) < 0)  return false; // P is on the right side

  // // edge 2
  // Vector3f edge2 = v0 - v2;
  // Vector3f vp2 = P - v2;
  // C = edge2.cross(vp2);
  // if ((v = N.dot(C)) < 0) return false; // P is on the right side;

  // u /= denom;
  // v /= denom;

  // return true; // this ray hits the triangle
}