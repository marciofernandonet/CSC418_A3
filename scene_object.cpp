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

	// Determine unit normal, pointing outwards from square
	Vector3D n = Vector3D(0, 0, 1); //unit surface normal
	if(a[2] <0){
		n = Vector3D(0, 0, -1);
	}

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
		
		//float t_front = (-2*ddotz + descriminant) / (2 * ddotd);
		//float t_back = (-2*ddotz - descriminant) / (2 * ddotd);
		float t_front = (-ddotz + descriminant) / (ddotd);
		float t_back = (-ddotz - descriminant) / (ddotd);
		
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
//==============================================================================================

bool UnitCylinder::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	// In object coords, the cylinder has a circular base centered at x=y=0 and
	// has its axis aligned with z-axis, running from z=-1 to z=+1.
	Point3D  modelRayOrigin;

	modelRayOrigin = worldToModel * ray.origin;
	Vector3D a = Vector3D(modelRayOrigin[0], modelRayOrigin[1], modelRayOrigin[2]); // Ray origin in object coords
	Vector3D d = worldToModel * ray.dir; // Ray direction in object coords
	

	float ddotd_xy = d.dot(d) - d[2]*d[2]; 
	float adota_xy = a.dot(a) - a[2]*a[2];
	float ddota_xy = d.dot(a) - a[2]*d[2];
	
	//First check for intersection with infinite cylinder, disreagarding z coord
	float discriminant_xy  = (a[0]*d[0] + a[1]*d[1])*(a[0]*d[0] + a[1]*d[1]) - (d[0]*d[0] + d[1]*d[1])*(a[0]*a[0] + a[1]*a[1] -1);

	if( discriminant_xy >= 0){ //Ray intersects infinite cylinder
		
		if(ddotd_xy >0.00001){
			// If ddotd_xy <= 0.00001, ray is nearly (or exactly) parallel to z-axis
			// so skip checking sides and check below to see it it intersects caps
			// at z=-1 or z=+1
	
			float sqt_discriminant_xy = pow(discriminant_xy, 0.5);
			float t_front = (-ddota_xy + sqt_discriminant_xy) / ddotd_xy;
			float t_back = (-ddota_xy - sqt_discriminant_xy) / ddotd_xy;
		
			float t_cylWall = t_front; 

			// Check if ray is pointng away from cylinder
			if(t_front < 0){
				return false;
			}

			// Which is the near face?
			if (t_back >= 0) {
				t_cylWall = t_back;
			}
			//Check for intersections in front of near cylinder wall
			if (ray.intersection.none || ray.intersection.t_value > t_cylWall) {
				// Nope, so set this as the nearest intersection so far
				Vector3D infCylInt = a + t_cylWall*d; //intersection with infinite cylinder

				if (std::abs(infCylInt[2]) <= 1){ // Ray hits side of cylinder between z=-1 and z=+1
			
					ray.intersection.none = false;
					ray.intersection.t_value = t_cylWall;
					ray.intersection.point = modelToWorld * Point3D(infCylInt[0], infCylInt[1], infCylInt[2]);
					Vector3D nInt =  Vector3D(infCylInt[0], infCylInt[1], 0);//surface unit normal at intersection
					//nInt.normalize();
					ray.intersection.normal = transNorm(worldToModel, nInt);
					return true;
				}//Ray missed side of cylinder between z=-1, z=+1
			} //Something blocking ray and cylinder wall,
			  //but whatever it is might not block the rays 
			  //intersections with caps, which is checked below
		}

			// At this point, know that ray intersects infinite cylinder
			// but not on walls between z=-1 and z=+1.
			// We are interested in the cases where the ray 
			// might still hit the top or bottom caps, which are flat circles
			// of radius 1.
			// Need to check intersections with the planes z=1, z=-1,
			// then look at x^2+y^2 for those intersection points.
		
			// First a special case: if d[2]==0, ray moves parallel to planes of
			// caps, but we know it did not intersect cylinder between z=-1
			// and z=+1, so it misses the cylinder.
			if(d[2]==0){
				// Ray moves parallel to planes of caps
				// but does not hit walls
				return false;
			}

			// t-values for intersection with planes z=+1, z=-1 respectively
			float t1 = ( 1 - a[2])/d[2]; // int with z=+1 plane
			float t2 = (-1 - a[2])/d[2]; // int with z=-1 plane

			if((t1 > 0) && (t2 > 0)){ 
				//Ray is outside sandwich formed by two planes 
				float t_nearPlane = std::min(t1,t2);

				if( (!ray.intersection.none) && (ray.intersection.t_value <= t_nearPlane) ){
					// Ray has an intersection in front of the near plane
					// Ray does not intersect near plane
					return false;
				}
				// Now we know ray has no intersection in front of near plane
				// So perform computations to determine if cap is intersected and where
				Vector3D zPlaneInt = a + t_nearPlane*d;
				// xy coords of intersection with near plane in front of ray
				float r_xy = zPlaneInt[0]*zPlaneInt[0]+zPlaneInt[1]*zPlaneInt[1];

				if( r_xy < 1){
					// Intersection with near plane hits cap
					// Determine unit normal at correct cap
					Vector3D n = Vector3D(0,0,1);
					if(zPlaneInt[2] < 0){ // Ray intersects z=-1 plane, otherwise z=+1
						n[2] = -1;
					}
					ray.intersection.none = false;
					ray.intersection.t_value = t_nearPlane;
					ray.intersection.point = modelToWorld * Point3D(zPlaneInt[0], zPlaneInt[1], zPlaneInt[2]);
					ray.intersection.normal = transNorm(worldToModel, n);
					return true;
					
				}
				// Otherwise ray misses cap
				return false;
			
			}else if( ((t1 > 0) && (t2 < 0)) || ((t1 < 0) && (t2 > 0)) ){ 
				//Ray is inside sandwich
				float t_nearPlane = std::max(t1,t2); // note difference from case above

				if( (!ray.intersection.none) && (ray.intersection.t_value <= t_nearPlane) ){
					// Ray has an intersection in front of the near plane
					// Ray does not intersect near plane
					return false;
				}
				
				// Now we know ray has no intersection in front of near plane
				// So perform computations to determine if cap is intersected and where
				Vector3D zPlaneInt = a + t_nearPlane*d;
				// xy coords of intersection with near plane in front of ray
				float r_xy = zPlaneInt[0]*zPlaneInt[0]+zPlaneInt[1]*zPlaneInt[1];

				if( r_xy < 1){
					// Intersection with near plane hits cap
					// Determine unit normal at correct cap
					Vector3D n = Vector3D(0,0,1);
					if(zPlaneInt[2] < 0){ // Ray intersects z=-1 plane, otherwise z=+1
						n[2] = -1;
					}
					ray.intersection.none = false;
					ray.intersection.t_value = t_nearPlane;
					ray.intersection.point = modelToWorld * Point3D(zPlaneInt[0], zPlaneInt[1], zPlaneInt[2]);
					ray.intersection.normal = transNorm(worldToModel, n);
					return true;
					
				}
				// Otherwise ray misses cap
				return false;

			}else{  // (t1 < 0) && (t2<0)
				// Ray is outside sandwich and pointing away from it
				return false;
			}
		
	}//end of if(discriminant)
	
	// If we get his far the ray has missed infinite cylinder entirely...
	return false;
} //End Unit Cylinder

