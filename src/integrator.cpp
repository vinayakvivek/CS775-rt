#include <iostream>

#include <integrator.hpp>

using namespace rt;

void reflect(const ray_t &incident_ray, const Vector3f &N, ray_t &reflected_ray) {
	Vector3f I = incident_ray.direction;
	reflected_ray.direction = (I - 2.0 * N.dot(I) * N).normalized();
}

// return: false ==> total internal reflection
bool refract(const ray_t &incident_ray, const Vector3f &N, float eta, ray_t &refracted_ray) {
	Vector3f I = incident_ray.direction;
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
		// Vector3f unit_direction = _ray.direction;
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
			float cosine = _ray.direction.dot(normal);
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

			// double nc=1, nt=eta, c=1-fabs(cosine);
			// double a=nt-nc, b=nt+nc, R0=a*a/(b*b);
			// double Re=R0+(1-R0)*c*c*c*c*c,Tr=1-Re;

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

color_t path_integrator_t::radiance(const scene_t* _scn, ray_t& _ray, int d) const {

	if (d >= _scn->intg->depth) {
		return _scn->img->get_bgcolor();
	}

	bool found_intersection = false;
	std::vector<object_t*>::const_iterator oit;
	hit_t hit, minhit;

	for (oit=_scn->objs.begin(); oit!=_scn->objs.end(); oit++) {
		if ((*oit)->intersect(hit, _ray)) {
		  _ray.maxt = hit.t;
		  minhit = hit;
		  found_intersection = true;
		}
	}

	bool found_light_intersection = false;
	std::list<light_t*>::const_iterator lit;
	light_hit_t lhit, minlhit;

	color_t d_col(0.0);
	for(lit=_scn->lits.begin(); lit!=_scn->lits.end(); lit++) {
		if ((*lit)->intersect(lhit, _ray)) {
			_ray.maxt = lhit.t;
			minlhit = lhit;
			found_light_intersection = true;
		}
	}

	if (found_light_intersection) {
		return (minlhit.light->get_color());
	}

	if (!found_intersection) {
		return _scn->img->get_bgcolor();
	}

	Vector3f hitpt = _ray.origin + _ray.maxt * _ray.direction;
	Vector3f n = minhit.normal;
	Vector3f nl = (n.dot(_ray.direction) < 0) ? n : -1*n;

	color_t f = minhit.obj->get_color();

	double p = fmax(f.x(), fmax(f.y(), f.z()));

	if (depth > 5) {
		if (erand48() < p)
			f = f / p;
		else
			return _scn->img->get_bgcolor();
	}

	f = f * minhit.obj->get_texture(hitpt);

	for(lit=_scn->lits.begin(); lit!=_scn->lits.end(); lit++) {
		d_col += 0.2 * f * (*lit)->direct(hitpt, _ray, nl, minhit.obj->get_material(), _scn);
	}

	bool can_transmit = minhit.obj->get_material()->get_is_transmit();
	bool can_reflect = minhit.obj->get_material()->get_is_reflect();

	if (!can_reflect && !can_transmit) {
		// DIFFUSE
    ray_t scattered_ray;
		scattered_ray.origin = hitpt;
   	scattered_ray.direction = (n + randomInUnitSphere()).normalized();
    d_col += f * _scn->intg->radiance(_scn, scattered_ray, d+1);
    return d_col;

	} else if (can_reflect && !can_transmit) {
		// SPECULAR
		double fuzz = 0.05;

		ray_t reflected_ray;
		reflected_ray.origin = hitpt + EPSILON * n;
		reflect(_ray, n, reflected_ray);

		Vector3f specular_dir = (reflected_ray.direction + fuzz * randomInUnitSphere()).normalized();
		double cosa = specular_dir.dot(reflected_ray.direction);
		float shininess = minhit.obj->get_material()->get_shininess();
		color_t specular = minhit.obj->get_material()->get_specular();

		reflected_ray.direction = specular_dir;

		d_col += f * specular * pow(cosa, shininess) * _scn->intg->radiance(_scn, reflected_ray, d+1);
		return d_col;

		// ray_t reflected_ray;
		// reflected_ray.origin = hitpt;
		// float pdf = 1.0;

		// color_t fr = BRDF::specular(_ray, n, reflected_ray, minhit.obj->get_material(), pdf);
		// // std::cout << fr.x() << ", " << fr.y() << ", " << fr.z() << "\n";
		// d_col += f * fr * _scn->intg->radiance(_scn, reflected_ray, d+1) * (n.dot(reflected_ray.direction)) / pdf;
		// return d_col;

	} else {
		// DIELECTRIC
		ray_t reflected_ray, transmitted_ray;
		reflected_ray.origin = hitpt;
		transmitted_ray.origin = hitpt;

		bool into = n.dot(nl) > 0;                // Ray from outside going in?
	  double nc = 1, nt = minhit.obj->get_material()->get_eta();
	  double nnt = into ? (nc / nt):(nt / nc);
	  double ddn = _ray.direction.dot(nl);

	  reflect(_ray, n, reflected_ray);

	  if (!refract(_ray, nl, nnt, transmitted_ray)) {
	  	d_col += f * _scn->intg->radiance(_scn, reflected_ray, d+1);
	  	return d_col;
	  }

	  Vector3f tdir = transmitted_ray.direction;

	  double a = nt-nc, b = nt+nc;
	  double R0 = a*a/(b*b), c = 1 - (into ? -ddn : tdir.dot(n));
	  double Re = R0 + (1 - R0)*c*c*c*c*c;
	  double Tr = 1 - Re, P = .25 + .5 * Re;
	  double RP = Re/P, TP = Tr/(1 - P);

	  if (d > 2) {
	  	if (erand48() < P)
	  		d_col += f * RP * _scn->intg->radiance(_scn, reflected_ray, d+1);
	  	else
	  		d_col += f * TP * _scn->intg->radiance(_scn, transmitted_ray, d+1);
	  } else {
	  	d_col += f * (Re * _scn->intg->radiance(_scn, reflected_ray, d+1) +
	  				 Tr * _scn->intg->radiance(_scn, transmitted_ray, d+1));
	  }

	  return d_col;
	}

	return _scn->img->get_bgcolor();
}


color_t smallpt_integrator_t::radiance(const scene_t* _scn, ray_t& _ray, int d) const {
	return color_t(0.0, 0.0, 0.0);
}