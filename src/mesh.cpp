#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include <mesh.hpp>

using namespace rt;

mesh_t::mesh_t(material_t* _mat, std::string file_name, Vector3f center) {
  std::cout << file_name << "\n";

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

  // Loop over shapes
  for (size_t s = 0; s < shapes.size(); s++) {
    // Loop over faces(polygon)
    size_t index_offset = 0;
    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
      int fv = shapes[s].mesh.num_face_vertices[f];

      // Loop over vertices in the face.
      for (size_t v = 0; v < fv; v++) {
        // access to vertex
        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
        // tinyobj::real_t vx = attrib.vertices[3*idx.vertex_index+0];
        // tinyobj::real_t vy = attrib.vertices[3*idx.vertex_index+1];
        // tinyobj::real_t vz = attrib.vertices[3*idx.vertex_index+2];
        tinyobj::real_t nx = attrib.normals[3*idx.normal_index+0];
        tinyobj::real_t ny = attrib.normals[3*idx.normal_index+1];
        tinyobj::real_t nz = attrib.normals[3*idx.normal_index+2];
        // tinyobj::real_t tx = attrib.texcoords[2*idx.texcoord_index+0];
        // tinyobj::real_t ty = attrib.texcoords[2*idx.texcoord_index+1];
        // Optional: vertex colors
        // tinyobj::real_t red = attrib.colors[3*idx.vertex_index+0];
        // tinyobj::real_t green = attrib.colors[3*idx.vertex_index+1];
        // tinyobj::real_t blue = attrib.colors[3*idx.vertex_index+2];

        std::cout << "normal: " << nx << ", " << ny << ", " << nz << "\n";
      }
      index_offset += fv;

      // per-face material
      shapes[s].mesh.material_ids[f];
    }
  }
}

mesh_t::~mesh_t() {}

bool mesh_t::intersect(hit_t& result, const ray_t& _ray) const {
  return false;
}

Eigen::Vector3f mesh_t::get_normal(Eigen::Vector3f& _p) const {
  Eigen::Vector3f normal;
  return normal;
}

color_t mesh_t::get_texture(Vector3f &_p) const {
  color_t col;
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