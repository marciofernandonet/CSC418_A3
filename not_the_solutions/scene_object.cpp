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
	// to simplify the intersection test.ssss
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
	// First convert the ray to the model view
	Vector3D modelRayOrigin;
	Vector3D modelRayDirection;
	
	Point3D temp = worldToModel * ray.origin;	// convert to vector
	modelRayOrigin = Vector3D(temp[0], temp[1], temp[2]);	// to do dot product
	modelRayDirection = worldToModel * ray.dir;
	
	/* Now intersect using the following information:
	 * Given the ray's origin (z) and direction (d), the sphere's center (c),
	 *  and radius (R), let the difference between the ray's origin and sphere's
	 *  center be (r), so then r = z - c
	 * Then there is an intersection iff
	 * (d * r)^2 - |d|^2(|r|^2 - R^2) >= 0
	 * This requires computing the dot product between d and r, d and d, and
	 *  r and r (where |d|^2 = d * d and |r|^2 = r*r)
	 *  This formula is taken from the textbook
	 * But in the model's coordinates, we can assume two things:
	 *     (1)  The model is already centered at the origin, so c = 0
	 *     (2)  The model is already normalized, so R = 1
	 * Then the expression above simplifies to intersection iff
	 * (d*z)^2 - |d|^2(|z|^2 - 1) >= 0
	 * which is computed as
	 * (d.dot(z)) * (d.dot(z)) - (d.dot(d)) * (z.dot(z) - 1) >= 0
	 */
	float ddotz = modelRayDirection.dot(modelRayOrigin);
	float ddotd = modelRayDirection.dot(modelRayDirection);
	float zdotz = modelRayOrigin.dot(modelRayOrigin);
	float square_descriminant = ddotz*ddotz - ddotd*(zdotz - 1);
	
	if (square_descriminant >= 0) {
		// A collision may have happened, but on which hemisphere?
		float descriminant = pow(square_descriminant, 0.5);
		
		float t_front = (-2*ddotz + descriminant) / (2 * ddotd);
		float t_back = (-2*ddotz - descriminant) / (2 * ddotd);
		
		float t = t_front;
		if (t_back >= 0) {
			t = t_back;
		}
		
		if (t >= 0) {
			// A collision happened, now check if the ray collided anywhere
			//  before this point
			if (ray.intersection.none || ray.intersection.t_value > t) {
				// Nope, so set this as the nearest intersection so far
				ray.intersection.none = false;
				ray.intersection.t_value = t;
				
				// Now find the intersection point and 
				//  normal (an expensive calculation, saved for the end)
				Vector3D i = modelRayOrigin + t*modelRayDirection;	// The raw model intersection coord
				
				ray.intersection.point = modelToWorld * Point3D(i[0], i[1], i[2]);
				ray.intersection.normal = transNorm(worldToModel, i);
				
				return true;
			}
		}
	}
	
	return false;
}

