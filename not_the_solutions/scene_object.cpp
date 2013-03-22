/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements scene_object.h

***********************************************************/

#include <cmath>
#include <iostream>
#include "scene_object.h"

bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	// TODO: implement intersection code for UnitSquare, which is
	// defined on the xy-plane, with vertices (0.5, 0.5, 0), 
	// (-0.5, 0.5, 0), (-0.5, -0.5, 0), (0.5, -0.5, 0), and normal
	// (0, 0, 1).
	//
	// Your goal here is to fill ray.intersection with correct values
	// should an intersection occur.  This includes intersection.point,
	// intersection.normal, intersection.none, intersection.t_value.
	//
	// HINT: Remember to first transform the ray into object space
	// to simplify the intersection test.

	Point3D  tpoint;
	Vector3D tvector;

	tpoint = worldToModel * ray.origin;
	Vector3D e = Vector3D(tpoint[0], tpoint[1], tpoint[2]);
	Vector3D d = worldToModel * ray.dir;

	Vector3D n        = Vector3D(0, 0, 1);
	float denominator = d.dot(n);
	if (denominator != 0)
	{
		float t = -e.dot(n)/denominator;
		if (t >= 0) 
		{
			Vector3D p = e + t*d;
			if (!( (p[0] < -0.5) || (p[0] > 0.5) 
                            || (p[1] < -0.5) || (p[1] > 0.5)) && 
                               (ray.intersection.none || ray.intersection.t_value > t) )
			{
				ray.intersection.none    = false;
				ray.intersection.t_value = t;
				ray.intersection.normal  = transNorm(worldToModel, n);
				//tvector                  = modelToWorld*p;
				//ray.intersection.point   = Point3D(tvector[0], tvector[1], tvector[2]);
				ray.intersection.point   = modelToWorld*Point3D(p[0], p[1], p[2]);	
				
				return true;
			}
		}
	}
	return false;
}

bool UnitSphere::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	// TODO: implement intersection code for UnitSphere, which is centred 
	// on the origin.  
	//
	// Your goal here is to fill ray.intersection with correct values
	// should an intersection occur.  This includes intersection.point, 
	// intersection.normal, intersection.none, intersection.t_value.   
	//
	// HINT: Remember to first transform the ray into object space  
	// to simplify the intersection test.
	
	////////////////////////// OUR WORK ///////////////////////////////////
	// Convert the ray to model view.
	//

	// From the book, if a ray intersects a circle, 
        // then it must satisfy a discrimant...
	// (d*((e-c))**2-(d*d)((e-c)*(e-c)-(R)(R))) >= 0, 
	// Where * is the dot product, and
	// Where e is the vector origin, d is the direction,
	// c is the centre of the circle, and R is the radius of the circle
	// Since the circle is of unit radius and centred at the origin,
	// c = 0, R = 1, which implies an intersection occurs when...
	// (d*e)(d*e)-(d*d)(e*e-1)) >= 0

	Point3D  tpoint;
	Vector3D tvector;

	tpoint = worldToModel * ray.origin;
	Vector3D e = Vector3D(tpoint[0], tpoint[1], tpoint[2]);
	Vector3D d = worldToModel * ray.dir;
	
	
	float A = d.dot(d);
	float B = e.dot(d);
	float C = e.dot(e) - 1;

	float discriminant = 4*B*B - 4*A*C;

	if (discriminant >= 0) 
	{
		// Otherwise, for some t (which is the co-efficient term in front
		// of 'd', t >= 0 implies an intersection occured.
	
		//float tpos = ((-2*d).dot(e) + discriminant)/(d.dot(d));
		//float tneg  = ((-2*d).dot(e) - discriminant)/(d.dot(d));
		
		float tpos = ( -2*B + pow(discriminant, 0.5) ) / (2*A);
		float tneg = ( -2*B - pow(discriminant, 0.5) ) / (2*A);
		

		float t = -1;
		if (tneg >= 0)
		{
			t = tneg;
		}
		else if (tpos >= 0)
		{
			t = tpos;
		}
			
		
		if ( t >= 0 && (ray.intersection.none || ray.intersection.t_value > t))
		
		// An intersection occured! Let's find out where.
		// t_plus tells us a what time the intersection occured.
		// Let's plug that into the ray vector.
		// The normal of a surface is given by (2x, 2y, 2z) = 2*p
	
		{
			ray.intersection.none = false;
			ray.intersection.t_value = t;
	
			Vector3D v               = e + ray.intersection.t_value*d;
			v                        = transNorm(worldToModel, v);
	
			
			tvector                  = e + ray.intersection.t_value*d;
			ray.intersection.point   = modelToWorld*Point3D(tvector[0], tvector[1], tvector[2]);
			

			ray.intersection.normal  = v;
			return true;
		}
	}
	return false;
}

