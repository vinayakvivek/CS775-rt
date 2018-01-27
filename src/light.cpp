#include <light.hpp>

using namespace rt;

light_t::light_t() { }
light_t::~light_t() { }


point_light_t::point_light_t(const Vector3f& _pos, const Vector3f& _col, const float _ka): pos(_pos), col(_col), ka(_ka)
{ }

point_light_t::~point_light_t()
{ }

color_t point_light_t::direct(const Vector3f& hitpt, const ray_t &view_ray, const Vector3f& normal, const material_t* mat, const scene_t* scn) const
{

	Vector3f eps_hitpt = hitpt + EPSILON * normal;
	Vector3f direction = (pos - eps_hitpt).normalized();
	ray_t shadow_ray(eps_hitpt, direction);
	shadow_ray.maxt = (pos - eps_hitpt).norm();

	color_t shade(0.0, 0.0, 0.0);

	bool found_intersection=false;
	std::vector<object_t*>::const_iterator oit;
	hit_t hit;

	for (oit=scn->objs.begin(); oit!=scn->objs.end(); oit++)
	{
		if ((*oit)->intersect(hit, shadow_ray))
		{
		  found_intersection = true;
		  break;
		}
	}

	if (!found_intersection && !mat->get_is_transmit()) {

		float d = direction.norm();
		float cos = normal.dot(direction);
		float attenuation = fmin(1, 1.0 / (0.1 + 0.00001 * d + 0.00001 * d*d));

		Vector3f h = (-view_ray.direction + direction).normalized();

		shade = mat->get_diffuse() * fmax(0.0, cos) +
					  mat->get_specular() * pow(normal.dot(h), mat->get_shininess());

		shade *= attenuation;
	}

	return shade.array() * col.array();
}


void point_light_t::print(std::ostream &stream) const
{
	Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[ ", " ]");

	stream<<"Light Properties: -------------------------------"<<std::endl;
	stream<<"Type: Point Light"<<std::endl;
	stream<<"Position: "<<pos.format(CommaInitFmt)<<std::endl;
	stream<<"Color: "<<col.format(CommaInitFmt)<<std::endl;
	stream<<"Ambient Coefficient: "<<ka<<std::endl<<std::endl;
}