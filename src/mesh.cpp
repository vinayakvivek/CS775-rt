#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include <mesh.hpp>

using namespace rt;

const float kEpsilon = 1e-8;

bool triangle_t::intersect(const ray_t &ray, float &t) const {
  Vector3f AB = B - A;
  Vector3f AC = C - A;

  float u, v;

  Vector3f pvec = ray.direction.cross(AC);
  float det = AB.dot(pvec);

  // if the determinant is negative the triangle is backfacing
  // if the determinant is close to 0, the ray misses the triangle
  if (det < kEpsilon) return false;

  // ray and triangle are parallel if det is close to 0
  if (fabs(det) < kEpsilon) return false;

  float invDet = 1 / det;

  Vector3f tvec = ray.origin - A;
  u = tvec.dot(pvec) * invDet;
  if (u < 0 || u > 1) return false;

  Vector3f qvec = tvec.cross(AB);
  v = ray.direction.dot(qvec) * invDet;
  if (v < 0 || u + v > 1) return false;

  t = AC.dot(qvec) * invDet;

  return true;
}

bool plane_t::intersect(const ray_t &ray, float &t, bool &entering) const {
  double denom = N.dot(ray.direction);
  if (fabs(denom) < kEpsilon) return false;

  entering = (denom < 0);
  t = -(N.dot(ray.origin) + D) / denom;
  return true;
}

bounding_box_t::bounding_box_t(
  const Vector3f &rft,  // right-front-top
  const Vector3f &lbb   // left-back-bottom
) {
  faces[0] = plane_t(Vector3f(1.0, 0.0, 0.0), rft);  // front
  faces[1] = plane_t(Vector3f(-1.0, 0.0, 0.0), lbb);  // back

  faces[2] = plane_t(Vector3f(0.0, 1.0, 0.0), rft);  // top
  faces[3] = plane_t(Vector3f(0.0, -1.0, 0.0), lbb);  // bottom

  faces[4] = plane_t(Vector3f(0.0, 0.0, 1.0), rft);  // right
  faces[5] = plane_t(Vector3f(0.0, 0.0, -1.0), lbb);  // left
}

bounding_box_t::bounding_box_t(
  const Vector3f &_rft,  // right-front-top
  const Vector3f &_lbb,   // left-back-bottom)
  const transform_t &transform
) {
  // TODO: transform normals and vertex points
  Vector3f rft = transform.transform_point(_rft);
  Vector3f lbb = transform.transform_point(_lbb);

  faces[0] = plane_t(transform.transform_normal(Vector3f(1.0, 0.0, 0.0)), rft);  // front
  faces[1] = plane_t(transform.transform_normal(Vector3f(-1.0, 0.0, 0.0)), lbb);  // back

  faces[2] = plane_t(transform.transform_normal(Vector3f(0.0, 1.0, 0.0)), rft);  // top
  faces[3] = plane_t(transform.transform_normal(Vector3f(0.0, -1.0, 0.0)), lbb);  // bottom

  faces[4] = plane_t(transform.transform_normal(Vector3f(0.0, 0.0, 1.0)), rft);  // right
  faces[5] = plane_t(transform.transform_normal(Vector3f(0.0, 0.0, -1.0)), lbb);  // left
}

bool bounding_box_t::intersect(const ray_t &ray, float &t) const {
  float te = kEpsilon, tl = std::numeric_limits<float>::infinity();
  bool is_entering = false;

  for (int i = 0; i < 6; ++i) {
    if (faces[i].intersect(ray, t, is_entering)) {
      if (is_entering) te = fmax(te, t);
      else tl = fmin(tl, t);
    }
  }

  if (te > tl) return false;

  return te > ray.mint && te < ray.maxt;
}

mesh_t::mesh_t(
  material_t* _mat,
  const color_t &_color,
  std::string texture_file,
  std::string obj_file,
  Vector3f _center,
  Vector3f _scale,
  Vector3f _rot,
  bool show_bounding_box
): object_t(_mat, _color) {

  std::cout << "texture: " << texture_file << "\n";
  tex = new texture_t(texture_file);

  std::cout << obj_file << "\n";

  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;

  std::string err;
  bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, obj_file.c_str());

  if (!err.empty()) { // `err` may contain warning message.
    std::cerr << err << std::endl;
  }

  if (!ret) {
    std::cout << "failed to load obj file.\n";
    exit(1);
  }

  std::cout << "loaded successfully.\n";

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

  Vector3f rft(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
  Vector3f lbb(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());

  transform = transform_t(trans_matrix * scale_matrix * rot_matrix);
  num_triangles = 0;
  this->center = _center;
  this->show_bounding_box = show_bounding_box;
  // mat = _mat;

  std::vector<Vector3f> face;

  // Loop over shapes
  for (size_t s = 0; s < shapes.size(); s++) {
    // Loop over faces(polygon)
    size_t index_offset = 0;
    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {

      int fv = shapes[s].mesh.num_face_vertices[f];
      face.resize(0);

      // Loop over vertices in the face.
      for (int v = 0; v < fv; v++) {
        // access to vertex
        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
        float vx = attrib.vertices[3*idx.vertex_index+0];
        float vy = attrib.vertices[3*idx.vertex_index+1];
        float vz = attrib.vertices[3*idx.vertex_index+2];
        Vector3f t_point = transform.transform_point(Vector3f(vx, vy, vz));
        face.push_back(t_point);

        rft.x() = fmax(vx, rft.x());
        rft.y() = fmax(vy, rft.y());
        rft.z() = fmax(vz, rft.z());

        lbb.x() = fmin(vx, lbb.x());
        lbb.y() = fmin(vy, lbb.y());
        lbb.z() = fmin(vz, lbb.z());
      }
      index_offset += fv;

      for (int v = 0; v < fv-2; ++v) {
        triangles.push_back(triangle_t(
            face[0], face[v+1], face[v+2]
          ));
        num_triangles++;
      }

      // per-face material
      // shapes[s].mesh.material_ids[f];
    }
  }

  bounding_box = bounding_box_t(rft, lbb, transform);
  std::cout << "preprocessing done.\n";
}

color_t mesh_t::get_color() const {
  return color;
}

mesh_t::~mesh_t() {}

bool mesh_t::intersect(hit_t& result, const ray_t& _ray) const {
  float curr_t = _ray.maxt;
  Vector3f curr_normal = Vector3f(0.0, 1.0, 0.0);
  float t;

  bool found_intersection = false;

  if (bounding_box.intersect(_ray, t)) {
    if (show_bounding_box) {
      found_intersection = true;
      curr_t = t;
    } else {
      for (triangle_t tri : triangles) {
        if (tri.intersect(_ray, t)) {
          if (t < curr_t && t > _ray.mint) {
            curr_t = t;
            curr_normal = tri.N;
            found_intersection = true;
          }
        }
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
  Vector3f p = (_p - center).normalized();
  float tx1 = atan2(p.x(), p.z()) / (2. * M_PI) + 0.5;
  float ty1 = asin(p.y()) / M_PI + .5;

  return tex->get_color(tx1, ty1);
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