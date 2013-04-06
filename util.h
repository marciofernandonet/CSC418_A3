/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		utility functions and structures
		(based on code from CGL, University of Waterloo), 
		modify this file as you see fit.

***********************************************************/

#ifndef _UTIL_
#define _UTIL_

#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI	(3.14159265358979323846)
#endif

#ifndef MAX_REFLECTIONS
#define MAX_REFLECTIONS	(3)
#endif

#ifndef MAX_REFRACTIONS 
#define MAX_REFRACTIONS (5)
#endif

#define USE_EXTENDEDLIGHTS
#define USE_REFRACTIONS
#define USE_TRANSMISSIONSHADOWS
//~ #define USE_REFLECTIONS
//~ #define USE_FINERFLUX

class Point3D {
public:
	Point3D(); 
	Point3D(double x, double y, double z);  
	Point3D(const Point3D& other); 

	Point3D& operator =(const Point3D& other); 
	double& operator[](int i); 
	double operator[](int i) const; 

private:
	double m_data[3];
};

class Vector3D {
public:
	Vector3D(); 
	Vector3D(double x, double y, double z); 
	Vector3D(const Vector3D& other); 

	Vector3D& operator =(const Vector3D& other); 
	double& operator[](int i);  
	double operator[](int i) const;  

	double length() const; 
	double normalize();
	double dot(const Vector3D& other) const; 
	Vector3D cross(const Vector3D& other) const; 

private:
	double m_data[3];
};

// standard operators on points and vectors
Vector3D operator *(double s, const Vector3D& v); 
Vector3D operator +(const Vector3D& u, const Vector3D& v); 
Point3D operator +(const Point3D& u, const Vector3D& v); 
Vector3D operator -(const Point3D& u, const Point3D& v); 
Vector3D operator -(const Vector3D& u, const Vector3D& v); 
Vector3D operator -(const Vector3D& u); 
Point3D operator -(const Point3D& u, const Vector3D& v); 
Vector3D cross(const Vector3D& u, const Vector3D& v); 
std::ostream& operator <<(std::ostream& o, const Point3D& p); 
std::ostream& operator <<(std::ostream& o, const Vector3D& v); 

class Vector4D {
public:
	Vector4D(); 
	Vector4D(double w, double x, double y, double z); 
	Vector4D(const Vector4D& other); 

	Vector4D& operator =(const Vector4D& other); 
	double& operator[](int i);  
	double operator[](int i) const;  

private:
	double m_data[4];
};

class Matrix4x4 {
public:
  Matrix4x4(); 
  Matrix4x4(const Matrix4x4& other); 
  Matrix4x4& operator=(const Matrix4x4& other); 

  Vector4D getRow(int row) const; 
  double *getRow(int row); 
  Vector4D getColumn(int col) const; 

  Vector4D operator[](int row) const; 
  double *operator[](int row); 

  Matrix4x4 transpose() const; 
		
private:
  double m_data[16];
};

Matrix4x4 operator *(const Matrix4x4& M, const Matrix4x4& N); 
Vector3D operator *(const Matrix4x4& M, const Vector3D& v); 
Point3D operator *(const Matrix4x4& M, const Point3D& p);
// Multiply n by the transpose of M, useful for transforming normals.  
// Recall that normals should be transformed by the inverse transpose 
// of the matrix.  
Vector3D transNorm(const Matrix4x4& M, const Vector3D& n); 
std::ostream& operator <<(std::ostream& os, const Matrix4x4& M); 

class Colour {
public:
	Colour(); 
	Colour(double r, double g, double b); 
	Colour(const Colour& other); 

	Colour& operator =(const Colour& other); 
	Colour operator *(const Colour& other); 
	double& operator[](int i);  
	double operator[](int i) const; 
    
	void clamp(); 	

private:
	double m_data[3];
};

Colour operator *(double s, const Colour& c); 
Colour operator +(const Colour& u, const Colour& v); 
std::ostream& operator <<(std::ostream& o, const Colour& c); 

struct Material {
	Material( Colour ambient, Colour diffuse, Colour specular, double exp ) :
		ambient(ambient), diffuse(diffuse), specular(specular), 
		specular_exp(exp), reflectivity(0.05), transitivity(0.0), cLight(1.0) {}
	Material( Colour ambient, Colour diffuse, Colour specular, double exp, double ref) :
		ambient(ambient), diffuse(diffuse), specular(specular), 
		specular_exp(exp), reflectivity(ref),transitivity(0.0), cLight(1.0) {}
	Material( Colour ambient, Colour diffuse, Colour specular, double exp, double ref, double transitivity, double cLight) :
		ambient(ambient), diffuse(diffuse), specular(specular), 
		specular_exp(exp), reflectivity(ref) , transitivity(transitivity), cLight(cLight){}
	// Ambient components for Phong shading.
	Colour ambient; 
	// Diffuse components for Phong shading.
	Colour diffuse;
	// Specular components for Phong shading.
	Colour specular;
	// Specular exponent.
	double specular_exp;
	// Reflectivity ratio
	double reflectivity;
	// Transitivity; 0 for opaque, 1.0 for completely transparetn
	double transitivity;
	//Speed of light inside material
	double cLight;
};

struct Intersection {
	// Location of intersection.
	Point3D point;
	// Normal at the intersection.
	Vector3D normal;
	// Material at the intersection.
	Material* mat;
	// Position of the intersection point on your ray.
	// (i.e. point = ray.origin + t_value * ray.dir)
	// This is used when you need to intersect multiply objects and
	// only want to keep the nearest intersection.
	double t_value;	
	// Set to true when no intersection has occured.
	bool none;
};

// Ray structure. 
struct Ray3D {
	Ray3D() : reflections(0), refractions(0) {
		intersection.none = true; 
	}
	Ray3D( Point3D p, Vector3D v ) : origin(p), dir(v), reflections(0),refractions(0), cLight(1.0) {
		intersection.none = true;
	}
	Ray3D( Point3D p, Vector3D v, int ref) : origin(p), dir(v), reflections(ref), refractions(0), cLight(1.0) {
		intersection.none = true;
	}
	Ray3D( Point3D p, Vector3D v, int ref, int refract, double cLight) : origin(p), dir(v), reflections(ref), refractions(refract), cLight(cLight) {
		intersection.none = true;
	}
	// Origin and direction of the ray.
	Point3D origin;
	Vector3D dir;
	// Intersection status, should be computed by the intersection
	// function.
	Intersection intersection;
	// Current colour of the ray, should be computed by the shading
	// function.
	Colour col;
	// A counter for number of times the ray has been reflected so it can be
	// reflected only a certain number of times
	int reflections;
	//Refraction counter
	int refractions;
	// Ratio of Speed of light in current medium vs vacuum (always <=1)
	// Used for computing refraction angles
	double cLight;
};
#endif





