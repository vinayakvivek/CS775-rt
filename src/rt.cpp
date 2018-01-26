#include <resolver.hpp>

#include <rt.hpp>


using namespace rt;

void rt::render(const scene_t* scn)
{
	unsigned int w=scn->img->get_width();
	unsigned int h=scn->img->get_height();

  scn->cam->print(std::cout);

	for (int j = h-1; j >= 0; --j)
	{
		for (int i = 0; i < w; ++i)
		{
			ray_t ray;
      color_t col(1.0);
			int d;

      // Eigen::Vector2f psample=scn->img->sample_pixel(i,j);
      // color_t col = scn->cam->sample_ray(ray, psample);

      ray = scn->cam->get_ray(float(i)/float(w), float(j)/float(w));

			col *= scn->intg->radiance(scn, ray, d);

			scn->img->set_pixel(i, j, col);
		}

	}
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
        // Vector3f eye(0.0, -5.0, 10.0);
        // Vector3f lookat(0.0, 1.0, 0.0);
        // Vector3f vup(0.0, 1.0, 0.0);
        // float vfov = 20.0;

        Vector3f eye = scn.cam->get_eye();
        Vector3f lookat = scn.cam->get_lookat();
        Vector3f vup = scn.cam->get_up();

        float vfov = 40.0;
        float dist_to_focus = 1.0;
        float aperture = 0.1;
        scn.cam = new camera_t(eye, lookat, vup, vfov, float(w)/float(h), aperture, dist_to_focus);

  			rt::render(&scn);

			  std::string img_filename = scene_filename;
    		size_t lastdot = img_filename.find_last_of(".");
    		if (lastdot != std::string::npos)
     			img_filename.erase(lastdot, std::string::npos);
   			img_filename += ".ppm";

  			scn.img->write(img_filename);
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
