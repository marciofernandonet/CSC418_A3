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
  //new code
  Intersection i = ray.intersection;
  Material mat = *(i.mat);

  Vector3D n = i.normal;
  n.normalize();

  Vector3D c = -1 * ray.dir;
  c.normalize();

  Vector3D s = _pos - i.point;
  s.normalize();

  //diffuse coefficient
  double ndots = n.dot(s);
  //specular coefficient
  double cdotm = c.dot(2 * ndots * n - s);
  if (ndots < 0.0)
    ndots = 0.0;
  if (cdotm < 0.0)
    cdotm = 0.0;
  ray.col = mat.ambient * _col_ambient + 
            ndots * mat.diffuse * _col_diffuse + 
            pow(cdotm, mat.specular_exp) * mat.specular * _col_specular;
  ray.col.clamp();
}

