/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		   light source classes

***********************************************************/

#ifndef _LIGHT_SOURCE_H_
#define _LIGHT_SOURCE_H_

#include "util.h"

class Raytracer;

// Light sources are given a shade method that accepts the raytracer
// object itself
// This is required for performing shadow computations
class LightSource {
public:
	virtual void shade( Ray3D& ray, Raytracer *raytracer ) = 0;
};

// A point light is defined by its position in world space and its
// colour.
class PointLight : public LightSource {
public:
	PointLight( Point3D pos, Colour col ) : _pos(pos), _col(col) {}
	void shade( Ray3D& ray, Raytracer *raytracer );
private:
	Point3D _pos;
	Colour _col;
};

// A very simple extended light source, but it demonstrates the effects
// of foreshortening and will produce soft shadows
class BallLight : public LightSource {
public:
	BallLight( Point3D pos, double radius, Colour col) : _pos(pos),
			_radius(radius), _col(col), _flux(4.0) {}
	BallLight( Point3D pos, double radius, Colour col, double flux) :
			_pos(pos), _radius(radius), _col(col), _flux(flux) {}
	void shade( Ray3D& ray, Raytracer *raytracer );
	Vector3D get_normal( int integrativeElement );
	Point3D get_position( int integrativeElement );
private:
	Point3D _pos;
	double _radius;
	Colour _col;
	double _flux;
};

#endif	// _LIGHT_SOURCE_H_
