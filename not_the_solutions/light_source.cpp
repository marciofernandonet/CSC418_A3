/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements light_source.h

***********************************************************/

#include <cmath>
#include "light_source.h"

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
	
	Vector3D n = i.normal;
	n.normalize();
	Vector3D d = -ray.dir;
	d.normalize();
	Vector3D c = _pos - i.point;	// camera relative coord
	c.normalize();
	
	// calculate each phong term
	float diffuse = n.dot(c);
	// specular requires finding the theoretical reflection, t
	Vector3D t = (2*diffuse)*n - c;
	float specular = d.dot(t);
	
	// clamp the lower bounds
	if (diffuse < 0) { diffuse = 0; }
	if (specular < 0) { specular = 0; }
	
	// apply the phong shading formula
	Colour base_ambient = m->ambient * _col_ambient;
	Colour base_diffuse = m->diffuse * _col_diffuse;
	Colour base_specular = m->specular * _col_specular;
	ray.col = base_ambient + diffuse*base_diffuse +
			pow(specular, m->specular_exp)*base_specular;
	// clamp the upper bounds
	ray.col.clamp();

}

