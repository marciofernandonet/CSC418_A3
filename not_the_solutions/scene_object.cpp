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
	
	// First Transform ray into object space
	// Note that the 4x4 matrix class has operator * overloaded so we can multiply 4x4 matricies
	// with 3D-vectors and 3D points; whether point or vector, the appropriate fourth homogeneous 
	// coordinate will be added and transformation will be applied. Returned result will be a 
	// 3D vector or point

	Point3D  startPoint;
	Vector3D dirVector;

	startPoint = worldToModel * ray.origin;
	Vector3D a = Vector3D(startPoint[0], startPoint[1], startPoint[2]); // Ray origin in object coords
	Vector3D d = worldToModel * ray.dir; // Ray direction in object coords

	Vector3D n        = Vector3D(0, 0, 1); //unit surface normal
	float dn = d.dot(n); // Dot product of ray direction with unit surface normal
	if (d.dot(n) != 0) //If d.dot(n)==0, ray is in plane of square; don't compute intersection
	{
		float lambdaStar = -a.dot(n)/dn; // Ray parameter at intersection with plane of square
		if (lambdaStar >= 0) 
		{
			Vector3D p = a + lambdaStar*d; // Point of intesection with plane of square
			if ( ( (std::abs(p[0])<=0.5) && (std::abs(p[1])<=0.5) ) && 
                               (ray.intersection.none || ray.intersection.t_value > lambdaStar) )
				// The above checks to make sure ray intersects square and also
				// checks if there is an intersection in front of ray 
			{
				ray.intersection.point   = modelToWorld*Point3D(p[0], p[1], p[2]);	
				ray.intersection.normal  = transNorm(worldToModel, n);
				ray.intersection.none    = false;
				ray.intersection.t_value = lambdaStar;
			
				
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
	
	return false;
}

