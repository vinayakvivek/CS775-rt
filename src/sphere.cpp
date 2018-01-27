#include <sphere.hpp>

using namespace rt;

sphere_t::sphere_t(material_t* _mat):center(0.0,0.0,0.0),radius(1.0),mat(_mat) { }
sphere_t::sphere_t(material_t* _mat, Eigen::Vector3f _c, float _r, std::string texture_file): center(_c), radius(_r), mat(_mat) {
	std::cout << "texture: " << texture_file << "\n";
	tex = new texture_t(texture_file);
}

sphere_t::~sphere_t() { }

bool sphere_t::intersect(hit_t& result, const ray_t& _ray) const
{
	Vector3f oc = _ray.origin - center;
	float a = _ray.direction.dot(_ray.direction);
	float _b = _ray.direction.dot(oc);
	float c = oc.dot(oc) - radius * radius;
	float discriminant = _b*_b - a*c;

	if (discriminant > 0) {
		float sqr_d = sqrt(discriminant);
		float temp = (-_b - sqr_d) / a;
		if (temp < _ray.maxt && temp > _ray.mint) {
			result.obj = this;
			result.t = temp;
			result.normal = (_ray(temp) - center) / radius;
			return true;
		}
		temp = (-_b + sqr_d) / a;
		if (temp < _ray.maxt && temp > _ray.mint) {
			result.obj = this;
			result.t = temp;
			result.normal = (_ray(temp) - center) / radius;
			return true;
		}
		return false;
	}
	return false;
}

Eigen::Vector3f sphere_t::get_normal(Eigen::Vector3f& _p) const
{
	Eigen::Vector3f normal = _p - center;
	normal.normalize();

	return normal;
}

color_t sphere_t::get_texture(Vector3f &_p) const {
	Vector3f p = _p.normalized();
	float tx1 = atan2(p.x(), p.z()) / (2. * M_PI) + 0.5;
	float ty1 = asin(p.y()) / M_PI + .5;
	// float tx1 = asin(p.x()) / M_PI + 0.5;

	// std::cout << tx1 << " " << ty1 << "\n";

	return tex->get_color(tx1, ty1);
}

material_t* sphere_t::get_material(void) const
{
	return mat;
}

void sphere_t::print(std::ostream &stream) const
{
	Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[ ", " ]");

	stream<<"Object Properties: -------------------------------"<<std::endl;
	stream<<"Type: Sphere"<<std::endl;
	stream<<"Center: "<<center.format(CommaInitFmt)<<std::endl;
	stream<<"Radius: "<<radius<<std::endl<<std::endl;
}