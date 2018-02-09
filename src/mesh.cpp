#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include <mesh.hpp>

using namespace rt;



mesh_t::mesh_t(
  material_t* _mat,
  const color_t &_color,
    std::string texture_file,
    std::string obj_file,
  Vector3f _center,
  Vector3f _scale,
  Vector3f _rot
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


  transform = transform_t(trans_matrix * scale_matrix * rot_matrix);
  num_triangles = 0;
  this->center = _center;
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
        face.push_back(transform.transform_point(Vector3f(vx, vy, vz)));
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

  std::cout << "preprocessing done.\n";
}

color_t mesh_t::get_color() const {
  return color;
}

mesh_t::~mesh_t() {}

bool mesh_t::intersect(hit_t& result, const ray_t& _ray) const {
  float curr_t = _ray.maxt;
  Vector3f curr_normal;
  float t, u, v;
  Vector3f a, b;

  bool found_intersection = false;

  for (triangle_t tri : triangles) {
    if (_ray.rayTriangleIntersect(tri.A, tri.B, tri.C, t, u, v)) {
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