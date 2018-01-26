#include <iostream>

#include <integrator.hpp>

using namespace rt;

color_t whitted_integrator_t::radiance(const scene_t* _scn, ray_t& _ray, int& d) const
{
	bool found_intersection=false;
	std::vector<object_t*>::const_iterator oit;
	hit_t hit, minhit;
	Eigen::Vector3f hitpt, normal;

	for (oit=_scn->objs.begin(); oit!=_scn->objs.end(); oit++)
	{
		if ((*oit)->intersect(hit, _ray))
		{
		  _ray.maxt = hit.second;
		  minhit = hit;

		  hitpt = _ray.origin+_ray.maxt*_ray.direction;
		  normal = (*oit)->get_normal(hitpt);

		  found_intersection = true;
		}
	}

	if (!found_intersection) {
		// std::cout << "no intersection!\n";
		Vector3f unit_direction = _ray.direction.normalized();
	  float t = 0.5f * (unit_direction.y() + 1.0f);
	  return float(1.0 - t) * color_t(1.0, 1.0, 1.0) + t * color_t(0.5, 0.7, 1.0);
	}

	color_t d_col(0.0);
	std::list<light_t*>::const_iterator lit;
	for(lit=_scn->lits.begin(); lit!=_scn->lits.end(); lit++)
	{
		d_col += (*lit)->direct(hitpt, _ray, normal, minhit.first->get_material(), _scn);
	}

	return d_col;
}