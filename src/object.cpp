#include <object.hpp>

using namespace rt;

object_t::object_t() {
  tex = NULL;
  mat = NULL;
  color = color_t(0.0, 0.0, 0.0);
}

object_t::object_t(material_t* _mat) {
  mat = _mat;
  tex = NULL;
  color = color_t(0.0, 0.0, 0.0);
}

object_t::object_t(material_t* _mat, const color_t &_color) {
  mat = _mat;
  tex = NULL;
  color = _color;
}

object_t::~object_t() {
  delete tex;
  delete mat;
}