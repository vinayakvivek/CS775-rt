#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include <mesh.hpp>

using namespace rt;

const float kEpsilon = 1e-8;

bool rayTriangleIntersect(
    const ray_t &ray,
    const Vector3f &v0, const Vector3f &v1, const Vector3f &v2,
    float &t, float &u, float &v)
{
    Vector3f dir = ray.direction;
    Vector3f orig = ray.origin;

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

mesh_t::mesh_t(material_t* _mat, std::string file_name, Vector3f _center,
      Vector3f _scale, Vector3f _rot) {

  std::cout << file_name << "\n";

  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;

  std::string err;
  bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, file_name.c_str());

  if (!err.empty()) { // `err` may contain warning message.
    std::cerr << err << std::endl;
  }

  if (!ret) {
    std::cout << "failed to load obj file.\n";
    exit(1);
  }

  std::cout << "loaded successfully.\n";

  float scale = 1.0;
  Matrix4f scale_matrix, trans_matrix, rot_matrix;
  scale_matrix <<
    _scale.x(), 0.0, 0.0, 0.0,
    0.0, _scale.y(), 0.0, 0.0,
    0.0, 0.0, _scale.z(), 0.0,
    0.0, 0.0, 0.0, 1.0;
  trans_matrix <<
    1.0, 0.0, 0.0, _center.x(),
    0.0, 1.0, 0.0, _center.y(),
    0.0, 0.0, 1.0, _center.z(),
    0.0, 0.0, 0.0, 1.0;
  rot_matrix <<
    0.36, 0.48, -0.8, 0.0,
    -0.8, 0.60, 0.0, 0.0,
    0.48, 0.64, 0.60, 0.0,
    0.0, 0.0, 0.0, 1.0;

  Matrix3f m;
  m = AngleAxisf(_rot.z(), Vector3f::UnitZ()) *
      AngleAxisf(_rot.y(), Vector3f::UnitY()) *
      AngleAxisf(_rot.x(), Vector3f::UnitZ());
  rot_matrix.topLeftCorner<3,3>() = m;


  transform = transform_t(trans_matrix * scale_matrix * rot_matrix);
  num_triangles = 0;
  this->center = _center;
  mat = _mat;

  std::vector<Vector3f> face;

  // Loop over shapes
  for (size_t s = 0; s < shapes.size(); s++) {
    // Loop over faces(polygon)
    size_t index_offset = 0;
    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {

      int fv = shapes[s].mesh.num_face_vertices[f];
      face.resize(0);

      // Loop over vertices in the face.
      for (size_t v = 0; v < fv; v++) {
        // access to vertex
        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
        float vx = attrib.vertices[3*idx.vertex_index+0];
        float vy = attrib.vertices[3*idx.vertex_index+1];
        float vz = attrib.vertices[3*idx.vertex_index+2];
        face.push_back(transform.transform_point(Vector3f(vx, vy, vz)));
      }
      index_offset += fv;

      // std::cout << fv << "\n";

      for (int v = 0; v < fv-2; ++v) {
        triangles.push_back(triangle_t(
            face[0], face[v+1], face[v+2]
          ));
        num_triangles++;
      }

      // per-face material
      shapes[s].mesh.material_ids[f];
    }
  }

  std::cout << "preprocessing done.\n";
}

mesh_t::~mesh_t() {}

bool mesh_t::intersect(hit_t& result, const ray_t& _ray) const {
  float curr_t = _ray.maxt;
  Vector3f curr_normal;
  float t, u, v;
  Vector3f a, b;

  bool found_intersection = false;

  for (triangle_t tri : triangles) {
    if (rayTriangleIntersect(_ray, tri.A, tri.B, tri.C, t, u, v)) {
      if (t < curr_t && t > _ray.mint) {
        curr_t = t;
        curr_normal = tri.N;
        a = tri.A; b = tri.B;
        found_intersection = true;
      }
    }
  }

  if (found_intersection) {
    result.obj = this;
    result.t = curr_t;
    result.normal = curr_normal;
    return true;
  }

  return false;
}

Eigen::Vector3f mesh_t::get_normal(Eigen::Vector3f& _p) const {
  Eigen::Vector3f normal;
  return normal;
}

color_t mesh_t::get_texture(Vector3f &_p) const {
  color_t col(1.0, 1.0, 1.0);
  return col;
}

material_t* mesh_t::get_material(void) const
{
  return mat;
}

void mesh_t::print(std::ostream &stream) const
{
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[ ", " ]");

  stream<<"Object Properties: -------------------------------"<<std::endl;
  stream<<"Type: Mesh"<<std::endl;
}