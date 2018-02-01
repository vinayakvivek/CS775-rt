#include <iostream>

#include <integrator.hpp>

using namespace rt;

void reflect(const ray_t &incident_ray, const Vector3f &N, ray_t &reflected_ray) {
	Vector3f I = incident_ray.direction;
	reflected_ray.direction = (I - 2.0 * N.dot(I) * N).normalized();
}

// return: false ==> total internal reflection
bool refract(const ray_t &incident_ray, const Vector3f &N, float eta, ray_t &refracted_ray) {
	Vector3f I = incident_ray.direction.normalized();
  float dt = I.dot(N);
  float discriminant = 1.0 - eta*eta*(1 - dt*dt);
  if (discriminant > 0) {
    refracted_ray.direction = (eta * (I - N*dt) - N*sqrt(discriminant)).normalized();
    return true;
  }
  return false;
}

double schlick(double cosine, double eta) {
  double r0 = (1 - eta) / (1 + eta);
  r0 = r0 * r0;
  double c = 1 - cosine;
  return r0 + (1 - r0)*c*c*c*c*c;
}

color_t whitted_integrator_t::radiance(const scene_t* _scn, ray_t& _ray, int d) const
{
	bool found_intersection=false;
	std::vector<object_t*>::const_iterator oit;
	hit_t hit, minhit;
	color_t ambient_color(0.05, 0.05, 0.05);

	for (oit=_scn->objs.begin(); oit!=_scn->objs.end(); oit++) {
		if ((*oit)->intersect(hit, _ray)) {
		  _ray.maxt = hit.t;
		  minhit = hit;
		  found_intersection = true;
		}
	}

	if (!found_intersection) {
		// std::cout << "no intersection!\n";
		// Vector3f unit_direction = _ray.direction.normalized();
	 //  float t = 0.5f * (unit_direction.y() + 1.0f);
	 //  return float(1.0 - t) * color_t(1.0, 1.0, 1.0) + t * color_t(0.5, 0.7, 1.0);
		return _scn->img->get_bgcolor();
	}

	Vector3f hitpt = _ray.origin+_ray.maxt*_ray.direction;
	Vector3f normal = minhit.normal;

	color_t d_col(0.0);
	std::list<light_t*>::const_iterator lit;
	for(lit=_scn->lits.begin(); lit!=_scn->lits.end(); lit++) {
		d_col += (*lit)->direct(hitpt, _ray, normal, minhit.obj->get_material(), _scn);
	}

	if (d > _scn->intg->depth) {
		d_col = _scn->img->get_bgcolor();
	} else {
		bool can_transmit = minhit.obj->get_material()->get_is_transmit();
		bool can_reflect = minhit.obj->get_material()->get_is_reflect();
		ray_t scattered_ray;
		scattered_ray.origin = hitpt + normal * EPSILON;

		if (can_transmit && can_reflect) {
			float cosine = _ray.direction.normalized().dot(normal);
			float ni_over_nt;
			float eta = minhit.obj->get_material()->get_eta();

	    if (cosine > 0) {
	      normal = -normal;
	      ni_over_nt = eta;
	    } else {
	      ni_over_nt = 1.0 / eta;
	    }

	    color_t kr = minhit.obj->get_material()->get_reflect();
	    color_t kt = minhit.obj->get_material()->get_transmit();

			if (refract(_ray, normal, ni_over_nt, scattered_ray)) {
				// refract
				d_col += kt * _scn->intg->radiance(_scn, scattered_ray, d + 1);
			} else {
				// total internal reflection
				kr += kt;
			}

			// reflect
			reflect(_ray, normal, scattered_ray);
			d_col += kr * _scn->intg->radiance(_scn, scattered_ray, d + 1);


		} else if (can_reflect) {
			// reflection only
			reflect(_ray, normal, scattered_ray);
			d_col += minhit.obj->get_material()->get_reflect() * _scn->intg->radiance(_scn, scattered_ray, d + 1);

		} else {

			d_col += minhit.obj->get_material()->get_diffuse() * ambient_color;
		}
	}

	// d_col += minhit.obj->get_material()->get_diffuse() * 0.1;

	d_col *= minhit.obj->get_texture(normal);

	return d_col;
}