/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		   light source classes

***********************************************************/

#include "util.h"

// Base class for a light source.  You could define different types
// of lights here, but point light is sufficient for most scenes you
// might want to render.  Different light sources shade the ray 
// differently.
class LightSource {
public:
	virtual void shade( Ray3D& ray ) = 0;
};

// A point light is defined by its position in world space and its
// colour.
class PointLight : public LightSource {
public:
	PointLight( Point3D pos, Colour col ) : _pos(pos), _col(col), _flux(2.0) {}
	PointLight( Point3D pos, Colour col, double flux) :
			_pos(pos), _col(col), _flux(flux) {}
	void shade( Ray3D& ray );
	Vector3D get_normal( int integrativeElement );
private:
	Point3D _pos;
	Colour _col;
	double _flux;
};

class BallLight : public LightSource {
public:
	BallLight( Point3D pos, double radius, Colour col) : _pos(pos),
			_radius(radius), _col(col), _flux(2) {}
	BallLight( Point3D pos, double radius, Colour col, double flux) :
			_pos(pos), _radius(radius), _col(col), _flux(flux) {}
	void shade( Ray3D& ray );
	Vector3D get_normal( int integrativeElement );
	Point3D get_position( int integrativeElement );
private:
	Point3D _pos;
	double _radius;
	Colour _col;
	double _flux;
};
