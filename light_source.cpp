/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements light_source.h

***********************************************************/

#include <cmath>
#include <cstdio>
#include "light_source.h"
#include "raytracer.h"

/* These precomputed sin and cos tables are used to greatly speed
 * up the get_normal calculations of spherical lights */
static const double sin_table[] = {0.0, 0.38268342617627565,
0.707106771713121, 0.9238795248208143, 1.0, 0.9238795453287401,
0.7071068096068267, 0.3826834756867882, 0.0, -0.3826833766657622,
-0.7071067338194131, -0.923879504312886, -1.0, -0.923879565836663, 
-0.70710684750053, -0.3826835251972993};
static const double cos_table[] = {1.0, 0.9238795350747775,
0.707106790659974, 0.3826834509315322, 0.0, -0.3826834014210188,
-0.7071067527662672, -0.9238795145668506, -1.0, -0.9238795555827017,
-0.7071068285536788, -0.3826835004420441, 0.0, 0.3826833519105045,
0.7071067148725588, 0.9238794940589211};

void PointLight::shade( Ray3D& ray, Raytracer *raytracer ) {
	// The pointlight is simplified as much as possible to perform
	// fast raytracing - doesn't actually use the raytracer object
	// or look at the scene, but it's here for satisfying the parameter
	// prototype
	if (ray.intersection.none) {
		return;
	}

	Intersection i = ray.intersection;
	Material *m = i.mat;	// material properties

	Ray3D r;
	double transmission;

	Vector3D n = i.normal;
	n.normalize();
	Vector3D d = -ray.dir;
	d.normalize();
	Vector3D c = _pos - i.point;	// direction to light source
	c.normalize();

	// calculate each phong term
	float diffuse = n.dot(c);
	
	// specular requires finding the theoretical reflection, t
	Vector3D t = (2*diffuse)*n - c;
	float specular = d.dot(t);

	// clamp the lower bounds
	if (diffuse < 0) { diffuse = 0; }
	if (specular < 0) { specular = 0; }
	
	// figure out the shadow term
	r = Ray3D(i.point + 0.05*c, c);
	r.intersection.t_value = (_pos - i.point).length();
	transmission = raytracer->getLightTransmission(r);
	
	// apply the phong shading formula
	Colour base_ambient = m->ambient * _col;
	Colour base_diffuse = m->diffuse * _col;
	Colour base_specular = m->specular * _col;

	
	//ray.col = ray.col + basic_colour;
	ray.col = base_ambient + transmission * diffuse*base_diffuse +
			transmission * pow(specular, m->specular_exp)*base_specular;
	// clamp the upper bounds
	ray.col.clamp();
}

/**
 * The ball light is split up into 128 integrative units to perform
 * a numerical integration. This allows precomputing the sine and
 * cosine values (as done above) to speed up calculations
 */
Vector3D BallLight::get_normal(int integrativeElement) {
	int minor_axis = integrativeElement % 8;
	int major_axis = integrativeElement / 8;
	
	double xcomponent = sin_table[minor_axis] * sin_table[major_axis];
	double ycomponent = sin_table[minor_axis] * cos_table[major_axis];
	double zcomponent = cos_table[minor_axis];
	
	Vector3D n = Vector3D(xcomponent, ycomponent, zcomponent);
	
	return n;
}
Point3D BallLight::get_position(int integrativeElement) {
	int minor_axis = integrativeElement % 8;
	int major_axis = integrativeElement / 8;
	
	double xcomponent = sin_table[minor_axis] * sin_table[major_axis];
	double ycomponent = sin_table[minor_axis] * cos_table[major_axis];
	double zcomponent = cos_table[minor_axis];
	
	xcomponent = _pos[0] + _radius * xcomponent;
	ycomponent = _pos[1] + _radius * ycomponent;
	zcomponent = _pos[2] + _radius * zcomponent;
	
	return Point3D(xcomponent, ycomponent, zcomponent);
}

/**
 * A ball light will perform a numerical integration of all its positions
 * and its foreshortening/shadow effects on the intersecting ray
 * This is very computationally complex, but looks amazing!
 */
void BallLight::shade( Ray3D& ray, Raytracer *raytracer ) {
	if (ray.intersection.none) {
		return;
	}
	
	Intersection i = ray.intersection;
	Material *m = i.mat;	// material properties
	
	// apply the phong shading formula
	Colour base_ambient = m->ambient * _col;
	Colour base_diffuse = m->diffuse * _col;
	Colour base_specular = m->specular * _col;
	
	// Now integrate over each flux component
	static const int num_elems = 128;
	
	// n is the normal at the intersection, d is the ray direction
	Vector3D n = i.normal;
	n.normalize();
	Vector3D d = ray.dir;
	d.normalize();
	
	// c is the light source direction, t is the theoretical specular direction
	Vector3D c, t;
	float diffuse, specular;
	Colour current_colour;
	float foreshortening_term;
	Ray3D r;
	double transmission;
	double effective_flux;
	
	#ifdef USE_FINERFLUX
	effective_flux = _flux;
	for (int dflux = 0; dflux < num_elems; dflux++) {
	#else
	effective_flux = _flux * 4 * 2;
	int dflux;
	for (int innerflux = 0; innerflux < 4; innerflux++) {
	for (int outerflux = 0; outerflux < num_elems/8; outerflux += 4) {
		dflux = 8*outerflux + 2*innerflux;
	#endif
		c = get_position(dflux) - i.point;
		c.normalize();
		t = d - (2*d.dot(n))*n;
		
		diffuse = n.dot(c);
		specular = t.dot(c);
		
		r = Ray3D(i.point + 0.05*c, c);
		r.intersection.t_value = (get_position(dflux) - i.point).length();
		
		transmission = raytracer->getLightTransmission(r);
		
		// clamp the lower bounds
		if (diffuse < 0) { diffuse = 0; }
		if (specular < 0) { specular = 0; }
		
		foreshortening_term = -get_normal(dflux).dot(t);
		if (foreshortening_term > 1) {
			foreshortening_term = 1;
		}
		if (foreshortening_term <= 0) {
			continue;
		}
		
		current_colour = base_ambient + transmission * diffuse*base_diffuse +
				transmission * pow(specular, m->specular_exp)*base_specular;
		
		ray.col = ray.col + ((foreshortening_term * effective_flux / double(num_elems)) *
				current_colour);
	#ifndef USE_FINERFLUX
	}
	#endif
	}
	ray.col.clamp();
}
