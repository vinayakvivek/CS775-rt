#include <resolver.hpp>

#include <rt.hpp>


using namespace rt;

void rt::render(const scene_t* scn, std::string file_name)
{
  unsigned int w=scn->img->get_width();
  unsigned int h=scn->img->get_height();

  // scn->cam->print(std::cout);
  int nspp = scn->intg->nspp;
  float steps = w*h, step = 0;

  for (int j = h-1; j >= 0; --j)
  {
    for (int i = 0; i < w; ++i)
    {
      ray_t ray;
      color_t col(0.0);

      fprintf(stderr,"\rRendering (%d nspp) %5.2f%%", nspp, 100.0 * step / steps);
      step += 1;

      if (nspp == 1) {
        ray = scn->cam->get_ray(float(i)/float(w), float(j)/float(w));
        col += scn->intg->radiance(scn, ray, 0);
      } else {
        for (int s = 0; s < nspp; ++s) {
          ray = scn->cam->get_ray(float(i + erand48())/float(w), float(j + erand48())/float(w));
          col += scn->intg->radiance(scn, ray, 0);
        }
        col /= nspp;
      }

      scn->img->set_pixel(i, j, col);
    }

    scn->img->write(file_name);
  }

  std::cout << "\n";
}

int main(int argc, char **argv)
{
  if (argc != 2)
    {
          std::cerr << "Syntax: " << argv[0] << " <scene.xml>" << std::endl;
          return -1;
    }

  filesystem::path path(argv[1]);

  try
  {
    if (path.extension() == "xml")
    {
      std::string scene_filename(argv[1]);
        rt::scene_t scn(scene_filename);

        unsigned int w = scn.img->get_width();
        unsigned int h = scn.img->get_height();

        Vector3f eye = scn.cam->get_eye();
        Vector3f lookat = scn.cam->get_lookat();
        Vector3f vup = scn.cam->get_up();

        float vfov = 40.0;
        float dist_to_focus = 1.0;
        float aperture = 0.1;
        scn.cam = new camera_t(eye, lookat, vup, vfov, float(w)/float(h), aperture, dist_to_focus);

        std::string img_filename = scene_filename;
        size_t lastdot = img_filename.find_last_of(".");
        if (lastdot != std::string::npos)
          img_filename.erase(lastdot, std::string::npos);
        img_filename += ".ppm";

        rt::render(&scn, img_filename);
      }
      else
      {
        std::cerr<<"Error: Unknown file type."<<std::endl;
        return -1;
      }
  }
  catch (const std::exception &e)
  {
    std::cerr<<"Error: "<<e.what()<<std::endl;
    return -1;
  }

  return 0;
}
