
#pragma once

#include <utils.hpp>
#include <string>
#include <fstream>
#include <streambuf>
#include <color.hpp>

namespace rt {

  class texture_t {

    std::string image_file;
    unsigned char* data;
    int width, height;
    bool has_loaded;

   public:

    texture_t(const std::string &file_name);
    ~texture_t();
    color_t get_color(float u, float v) const;
    bool exists() {return has_loaded;}
  };
}