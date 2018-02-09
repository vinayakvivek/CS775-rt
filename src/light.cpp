#include <light.hpp>

using namespace rt;

light_t::light_t(const Vector3f& _col, const float _ka, int _nshadowrays)
	: col(_col), ka(_ka), num_shadowrays(_nshadowrays) {}

light_t::~light_t() {}

color_t light_t::direct(const Vector3f& hitpt, const ray_t &view_ray, const Vector3f& normal, const material_t* mat, const scene_t* scn) const
{
	Vector3f eps_hitpt = hitpt + EPSILON * normal;

	color_t total_shade(0.0, 0.0, 0.0);
	for (int i = 0; i < num_shadowrays; ++i) {
		Vector3f pos = sample_point();

		Vector3f direction = (pos - eps_hitpt).normalized();
		ray_t shadow_ray(eps_hitpt, direction);
		shadow_ray.maxt = (pos - eps_hitpt).norm();

		bool found_intersection=false;
		std::vector<object_t*>::const_iterator oit;
		hit_t hit;

		for (oit=scn->objs.begin(); oit!=scn->objs.end(); oit++) {
			if ((*oit)->intersect(hit, shadow_ray)) {
			  found_intersection = true;
			  break;
			}
		}

		color_t shade(0.0, 0.0, 0.0);

		if (!found_intersection) {

			// float d = direction.norm();
			float cos = normal.dot(direction);
			// float attenuation = fmin(1, 1.0 / (1.0 + 0.001 * d + 0.0001 * d*d));
			Vector3f h = (-view_ray.direction + direction).normalized();

			shade = mat->get_diffuse() * fmax(0.0, cos) +
						  mat->get_specular() * pow(normal.dot(h), mat->get_shininess());

			// shade *= attenuation;
		}

		total_shade += shade;
	}

	total_shade /= num_shadowrays;

	return total_shade.array() * col.array() * ka;
}

// point-light -----------------------

point_light_t::point_light_t(const Vector3f& _pos, const Vector3f& _col, const float _ka)
	: light_t(_col, _ka, 1), pos(_pos) {}

point_light_t::~point_light_t() {}

Vector3f point_light_t::sample_point() const {
	return pos;
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

// area-light ------------------------

area_light_t::area_light_t(
	const Vector3f &_center,
	const Vector3f _normal,
	const Vector3f _radius,
	const Vector3f& _col,
	float _ka,
	int _num_shadowrays
): light_t(_col, _ka, _num_shadowrays) {

	center = _center;
	normal = _normal.normalized();
	radius = _radius;

  Vector3f w = normal;
  Vector3f v = radius.normalized();
  Vector3f u = v.cross(w);

  Matrix4f to_light_coords;
  to_light_coords <<
  	u.x(), u.y(), u.z(), -center.dot(u),
  	v.x(), v.y(), v.z(), -center.dot(v),
  	w.x(), w.y(), w.z(), -center.dot(w),
  	0.0,   0.0,   0.0,   1.0;

  transform = transform_t(to_light_coords.inverse());
}

Vector3f area_light_t::sample_point() const {
	return transform.transform_point(randomInUnitDisk() * radius.norm());
}

bool area_light_t::intersect(light_hit_t& result, const ray_t& _ray) const {
	float D = -normal.dot(center);
	float t = - (normal.dot(_ray.origin) + D) / normal.dot(_ray.direction);

	Vector3f point = _ray(t);
	if ((point - center).norm() < radius.norm()) {
		if (t < _ray.maxt && t > _ray.mint) {
			result.light = this;
			result.t = t;
			result.normal = normal;
			return true;
		}
	}

	return false;
}

void area_light_t::print(std::ostream &stream) const
{
	Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[ ", " ]");

	stream<<"Light Properties: -------------------------------"<<std::endl;
	stream<<"Type: Point Light"<<std::endl;
	stream<<"Center: "<<center.format(CommaInitFmt)<<std::endl;
	stream<<"Color: "<<col.format(CommaInitFmt)<<std::endl;
	stream<<"Ambient Coefficient: "<<ka<<std::endl;
	stream<<"num_shadowrays: " << num_shadowrays << "\n\n";
}