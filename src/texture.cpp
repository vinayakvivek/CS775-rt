#include <texture.hpp>

using namespace rt;

texture_t::texture_t(const std::string &file_name) {
  image_file = file_name;

  std::ifstream f(file_name);
  if (!f.is_open()) {
    std::cout << "file '" <<  file_name << "' does not exist.\n";
  } else {
    std::string str;

    f.seekg(0, std::ios::end);
    str.reserve(f.tellg());
    f.seekg(0, std::ios::beg);

    str.assign((std::istreambuf_iterator<char>(f)),
                std::istreambuf_iterator<char>());

    // extract image height and width from header
    width = *(int*)&str[18];
    height = *(int*)&str[22];

    int size = 3 * width * height;
    data = new unsigned char[size]; // allocate 3 bytes per pixel

    for (int i = 0; i < size; ++i) {
      data[i] = *(char*)&str[54 + i];
    }
  }
}

color_t texture_t::get_color(float u, float v) const {
  int x = (int)(u * width);
  int y = (int)(v * height);

  int pixel_pos = 3 * ((y * width) + x);

  if (pixel_pos + 3 >= 3*width*height)
    return color_t(1.0, 1.0, 1.0);

  float r = (float)((int)data[pixel_pos]) / 255.0;
  float g = (float)((int)data[pixel_pos + 1]) / 255.0;
  float b = (float)((int)data[pixel_pos + 2]) / 255.0;

  color_t col(r, g, b);
  return col;
}