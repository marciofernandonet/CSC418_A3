/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements light_source.h

***********************************************************/

#include <cmath>
#include <cstdio>
#include "light_source.h"

	
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

Vector3D PointLight::get_normal(int integrativeElement) {
	int minor_axis = integrativeElement % 8;
	int major_axis = integrativeElement / 8;
	
	double xcomponent = sin_table[minor_axis] * sin_table[major_axis];
	double ycomponent = sin_table[minor_axis] * cos_table[major_axis];
	double zcomponent = cos_table[minor_axis];
	
	Vector3D n = Vector3D(xcomponent, ycomponent, zcomponent);
	
	return n;
}

void PointLight::shade( Ray3D& ray ) {
	// TODO: implement this function to fill in values for ray.col 
	// using phong shading.  Make sure your vectors are normalized, and
	// clamp colour values to 1.0.
	//
	// It is assumed at this point that the intersection information in ray 
	// is available.  So be sure that traverseScene() is called on the ray 
	// before this function.
	
	// When reaching this point, the ray intersection field is assumed to exist
	//  but it's checked anyway... just in case
	if (ray.intersection.none) {
		return;
	}
	
	Intersection i = ray.intersection;
	Material *m = i.mat;	// material properties
	
	// Now integrate over each flux component
	static const int num_elems = 128;
	
	Vector3D n = i.normal;
	n.normalize();
	Vector3D d = ray.dir;
	d.normalize();
	Vector3D c = _pos - i.point;	// camera relative coord
	c.normalize();
	Vector3D t = d - (2*d.dot(n))*n;	// theoretical specular direction
	
	// calculate each phong term
	float diffuse = n.dot(c);
	float specular = t.dot(c);
	
	// clamp the lower bounds
	if (diffuse < 0) { diffuse = 0; }
	if (specular < 0) { specular = 0; }
	
	// apply the phong shading formula
	Colour base_ambient = m->ambient * _col;
	Colour base_diffuse = m->diffuse * _col;
	Colour base_specular = m->specular * _col;
	
	Colour basic_colour = base_ambient + diffuse*base_diffuse +
			pow(specular, m->specular_exp)*base_specular;
	double foreshortening_term;
	for (int dflux = 0; dflux < num_elems; dflux++) {
		foreshortening_term = get_normal(dflux).dot(d);
		if (foreshortening_term > 1) {
			// this probably won't happen, but just in case
			foreshortening_term = 1;
		}
		if (foreshortening_term > 0) {
			ray.col = ray.col + ((foreshortening_term * _flux / double(num_elems)) *
					basic_colour);
		}
	}
	ray.col.clamp();
}

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

void BallLight::shade( Ray3D& ray ) {
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
	
	Vector3D n = i.normal;
	n.normalize();
	Vector3D d = ray.dir;
	d.normalize();
	
	Vector3D c, t;
	float diffuse, specular;
	Colour current_colour;
	float foreshortening_term;
	
	for (int dflux = 0; dflux < num_elems; dflux++) {
		c = get_position(dflux) - i.point;
		c.normalize();
		t = d - (2*d.dot(n))*n;
		
		diffuse = n.dot(c);
		specular = t.dot(c);
		
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
		
		current_colour = base_ambient + diffuse*base_diffuse +
				pow(specular, m->specular_exp)*base_specular;
		
		ray.col = ray.col + ((foreshortening_term * _flux / double(num_elems)) *
				current_colour);
	}
	ray.col.clamp();
}
