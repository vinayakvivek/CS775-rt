/*
    This file is part of rt.

    rt is a simple ray tracer meant to be used for teaching ray tracing.

    Copyright (c) 2018 by Parag Chaudhuri

	Some parts of rt are derived from Nori by Wenzel Jacob.
	https://github.com/wjakob/nori/

    rt is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    rt is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <camera.hpp>
#include <utils.hpp>

using namespace rt;

/**
 * The image2cam transformation is similar to the modeling-viewing pipeline transformation that opengl
 * does to get from the VCS to the CCS.
 *
 * The cam2world transformation is set to identity by default. This is the WCS to VCS transform.
 *
 * Must ensure that this is called at least once with valid parameters, before the camera can
 * act as a producer of primary rays.
*/
void camera_t::init(void)
{
	float recip = 1.0f / (far - near);
    float cot = 1.0f / std::tan(deg2rad(fov / 2.0f));

    Eigen::Matrix4f perspective;
    perspective <<
        cot, 0,   0,   0,
        0, cot,   0,   0,
        0,   0,  far * recip, -near * far * recip,
        0,   0,   1,   0;



    image2cam = transform_t(
        Eigen::DiagonalMatrix<float, 3>(Vector3f(-0.5f, -0.5f * aspect, 1.0f)) *
        Eigen::Translation<float, 3>(-1.0f, -1.0f/aspect, 0.0f) * perspective).inverse();

    cam2world = transform_t();
}

camera_t::camera_t()
{
	lookat=Vector3f(0.0,0.0,1.0);
	eye=Vector3f(0.0,0.0,0.0);
	up=Vector3f(0.0,1.0,0.0);
	fov=60.0;
	near=1e-4f;
	far=1e4f;
	aspect=640.0f/480.0f;

	init();
}

camera_t::camera_t(const Vector3f _lat, const Vector3f _eye, const Vector3f _up, float _fov, float _near, float _far):
	lookat(_lat), eye(_eye), up(_up), fov(_fov), near(_near), far(_far)
{ aspect=640.0f/480.0f; init(); }

camera_t::camera_t(const camera_t &_cam):
	lookat(_cam.lookat), eye(_cam.eye), up(_cam.up),fov(_cam.fov), near(_cam.near), far(_cam.far)
{ aspect=640.0f/480.0f; init(); }

camera_t::~camera_t()
{ }

camera_t::camera_t(Vector3f eye, Vector3f lookat, Vector3f vup, float vfov, float aspect, float aperture, float focus_dist) {
    std::cout << "initializing camera..\n";

    lens_radius = aperture / 2;
    float theta = vfov * M_PI / 180.0;
    float half_height = tan(theta / 2.0);
    float half_width = aspect * half_height;

    w = (eye - lookat).normalized();
    u = vup.cross(w).normalized();
    v = w.cross(u);

    lower_left_corner = eye - half_width*focus_dist*u - half_height*focus_dist*v - focus_dist*w;
    horizontal = 2 * half_width * focus_dist * u;
    vertical = 2 * half_height * focus_dist * v;
    origin = eye;
}

const Vector3f camera_t::get_lookat(void) {return lookat;}
const Vector3f camera_t::get_eye(void) {return eye;}
const Vector3f camera_t::get_up(void) {return up;}
const float camera_t::get_fov(void) {return fov;}

color_t camera_t::sample_ray(ray_t &ray, const Vector2f& _pixelpos) const
{
	Vector3f pt(_pixelpos.x(), _pixelpos.y(), 0.0);
 	Vector3f near_p= image2cam.transform_point(pt);
 	Vector3f d = near_p.normalized();
	float onebyz = 1.0f/d.z();

    ray.origin = cam2world.transform_point(Vector3f(0.0f,0.0f,0.0f));
    ray.direction = cam2world * d;

    ray.mint = near * onebyz;
    ray.maxt = far * onebyz;

    return color_t(1.0);
}

ray_t camera_t::get_ray(float s, float t) const {
    ray_t primary_ray(origin, lower_left_corner + s*horizontal + t*vertical - origin);
    return primary_ray;
}

void camera_t::print(std::ostream &stream)
{
	IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", "[ ", " ]");

	stream<<"Camera Properties: -------------------------------"<<std::endl;
	stream<<"Lookat: "<<lookat.format(CommaInitFmt)<<std::endl;
	stream<<"Eye: "<<eye.format(CommaInitFmt)<<std::endl;
	stream<<"Up: "<<up.format(CommaInitFmt)<<std::endl;
	stream<<"FoV: "<<fov<<std::endl;
	stream<<"Near:"<<near<<std::endl;
	stream<<"Far:"<<far<<std::endl<<std::endl;

    stream << "u: " << u.format(CommaInitFmt) << "\n";
    stream << "v: " << v.format(CommaInitFmt) << "\n";
    stream << "w: " << w.format(CommaInitFmt) << "\n";
    stream << "origin: " << origin.format(CommaInitFmt) << "\n";
    stream << "lower_left_corner: " << lower_left_corner.format(CommaInitFmt) << "\n";
    stream << "horizontal: " << horizontal.format(CommaInitFmt) << "\n";
    stream << "vertical: " << vertical.format(CommaInitFmt) << "\n";
}