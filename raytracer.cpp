/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		Implementations of functions in raytracer.h, 
		and the main function which specifies the 
		scene to be rendered.	

***********************************************************/


#include "raytracer.h"
#include "bmp_io.h"
#include <cmath>
#include <iostream>
#include <cstdlib>

Raytracer::Raytracer() : _lightSource(NULL) {
	_root = new SceneDagNode();
}

Raytracer::~Raytracer() {
	delete _root;
}

SceneDagNode* Raytracer::addObject( SceneDagNode* parent, 
		SceneObject* obj, Material* mat ) {
	SceneDagNode* node = new SceneDagNode( obj, mat );
	node->parent = parent;
	node->next = NULL;
	node->child = NULL;
	
	// Add the object to the parent's child list, this means
	// whatever transformation applied to the parent will also
	// be applied to the child.
	if (parent->child == NULL) {
		parent->child = node;
	}
	else {
		parent = parent->child;
		while (parent->next != NULL) {
			parent = parent->next;
		}
		parent->next = node;
	}
	
	return node;;
}

LightListNode* Raytracer::addLightSource( LightSource* light ) {
	LightListNode* tmp = _lightSource;
	_lightSource = new LightListNode( light, tmp );
	return _lightSource;
}

void Raytracer::rotate( SceneDagNode* node, char axis, double angle ) {
	Matrix4x4 rotation;
	double toRadian = 2*M_PI/360.0;
	int i;
	
	for (i = 0; i < 2; i++) {
		switch(axis) {
			case 'x':
				rotation[0][0] = 1;
				rotation[1][1] = cos(angle*toRadian);
				rotation[1][2] = -sin(angle*toRadian);
				rotation[2][1] = sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'y':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][2] = sin(angle*toRadian);
				rotation[1][1] = 1;
				rotation[2][0] = -sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'z':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][1] = -sin(angle*toRadian);
				rotation[1][0] = sin(angle*toRadian);
				rotation[1][1] = cos(angle*toRadian);
				rotation[2][2] = 1;
				rotation[3][3] = 1;
			break;
		}
		if (i == 0) {
		    node->trans = node->trans*rotation; 	
			angle = -angle;
		} 
		else {
			node->invtrans = rotation*node->invtrans; 
		}	
	}
}

void Raytracer::translate( SceneDagNode* node, Vector3D trans ) {
	Matrix4x4 translation;
	
	translation[0][3] = trans[0];
	translation[1][3] = trans[1];
	translation[2][3] = trans[2];
	node->trans = node->trans*translation; 	
	translation[0][3] = -trans[0];
	translation[1][3] = -trans[1];
	translation[2][3] = -trans[2];
	node->invtrans = translation*node->invtrans; 
}

void Raytracer::scale( SceneDagNode* node, Point3D origin, double factor[3] ) {
	Matrix4x4 scale;
	
	scale[0][0] = factor[0];
	scale[0][3] = origin[0] - factor[0] * origin[0];
	scale[1][1] = factor[1];
	scale[1][3] = origin[1] - factor[1] * origin[1];
	scale[2][2] = factor[2];
	scale[2][3] = origin[2] - factor[2] * origin[2];
	node->trans = node->trans*scale; 	
	scale[0][0] = 1/factor[0];
	scale[0][3] = origin[0] - 1/factor[0] * origin[0];
	scale[1][1] = 1/factor[1];
	scale[1][3] = origin[1] - 1/factor[1] * origin[1];
	scale[2][2] = 1/factor[2];
	scale[2][3] = origin[2] - 1/factor[2] * origin[2];
	node->invtrans = scale*node->invtrans; 
}

Matrix4x4 Raytracer::initInvViewMatrix( Point3D eye, Vector3D view, 
		Vector3D up ) {
	Matrix4x4 mat; 
	Vector3D w;
	view.normalize();
	up = up - up.dot(view)*view;
	up.normalize();
	w = view.cross(up);

	mat[0][0] = w[0];
	mat[1][0] = w[1];
	mat[2][0] = w[2];
	mat[0][1] = up[0];
	mat[1][1] = up[1];
	mat[2][1] = up[2];
	mat[0][2] = -view[0];
	mat[1][2] = -view[1];
	mat[2][2] = -view[2];
	mat[0][3] = eye[0];
	mat[1][3] = eye[1];
	mat[2][3] = eye[2];

	return mat; 
}

void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray ) {
	SceneDagNode *childPtr;

	// Applies transformation of the current node to the global
	// transformation matrices.
	_modelToWorld = _modelToWorld*node->trans;
	_worldToModel = node->invtrans*_worldToModel; 
	if (node->obj) {
		// Perform intersection.
		if (node->obj->intersect(ray, _worldToModel, _modelToWorld)) {
			ray.intersection.mat = node->mat;
		}
	}
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		traverseScene(childPtr, ray);
		childPtr = childPtr->next;
	}

	// Removes transformation of the current node from the global
	// transformation matrices.
	_worldToModel = node->trans*_worldToModel;
	_modelToWorld = _modelToWorld*node->invtrans;
}

void Raytracer::computeShading( Ray3D& ray ) {
	if (ray.intersection.none) { return; }	// don't bother checking
	LightListNode* curLight = _lightSource;
	for (;;) {
		if (curLight == NULL) break;
		// Each lightSource provides its own shading function.

		// Implement shadows here if needed.
		
		curLight->light->shade(ray);
		curLight = curLight->next;
	}
}

/**
 * This method is provided to light sources so they can send rays from the
 * material to the light source
 * The returned value is a number from 0.0 (meaning completely in shadow)
 * to 1.0 (meaning light is unimpeded in that path)
 * The ray is expected to have its intersection.t_value set to the light
 * source (so no t >= t_value will be considered for shadows)
 */
double Raytracer::getLightTransmission( Ray3D& ray ) {
	double transmission = 1.0;
	double max_t_value = ray.intersection.t_value;
	double dot_product;
	
	// This is pretty much traverse scene, but resets the t_value
	// and intersection as it goes along
	SceneDagNode *node = _root;
	SceneDagNode *childPtr;
	
	_modelToWorld = _modelToWorld*node->trans;
	_worldToModel = node->invtrans*_worldToModel;
	if (node->obj) {
		// Imagine there is no intersection and perform intersection
		ray.intersection.none = true;
		if (node->obj->intersect(ray, _worldToModel, _modelToWorld)) {
			if (ray.intersection.t_value < max_t_value) {
				ray.dir.normalize();
				ray.intersection.normal.normalize();
				dot_product = ray.dir.dot(ray.intersection.normal);
				transmission = transmission * dot_product *
						dot_product * node->mat->transitivity;
			}
		}
	}
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		transmission = transmission * getLightTransmission(ray);
		childPtr = childPtr->next;
	}

	// Removes transformation of the current node from the global
	// transformation matrices
	_worldToModel = node->trans*_worldToModel;
	_modelToWorld = _modelToWorld*node->invtrans;
	
	return transmission;
}

void Raytracer::initPixelBuffer() {
	int numbytes = _scrWidth * _scrHeight * sizeof(unsigned char);
	_rbuffer = new unsigned char[numbytes];
	_gbuffer = new unsigned char[numbytes];
	_bbuffer = new unsigned char[numbytes];
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			_rbuffer[i*_scrWidth+j] = 0;
			_gbuffer[i*_scrWidth+j] = 0;
			_bbuffer[i*_scrWidth+j] = 0;
		}
	}
}

void Raytracer::initSuperPixelBuffer(){
	int numbytes = _scrWidth * _aaLevel * _scrHeight * _aaLevel * sizeof(unsigned char);
	_superrbuffer = new unsigned char[numbytes];
	_supergbuffer = new unsigned char[numbytes];
	_superbbuffer = new unsigned char[numbytes];
	for (int i = 0; i < _aaLevel * _scrHeight; i++) {
		for (int j = 0; j < _aaLevel * _scrWidth; j++) {
			_superrbuffer[i*(_aaLevel*_scrWidth)+j] = 0;
			_supergbuffer[i*(_aaLevel*_scrWidth)+j] = 0;
			_superbbuffer[i*(_aaLevel*_scrWidth)+j] = 0;
		}
	}
}

void Raytracer::flushPixelBuffer( char *file_name ) {
	bmp_write( file_name, _scrWidth, _scrHeight, _rbuffer, _gbuffer, _bbuffer );
	delete _rbuffer;
	delete _gbuffer;
	delete _bbuffer;
}

Colour Raytracer::shadeRay( Ray3D& ray ) {
	Colour col(0.0, 0.0, 0.0); 
	traverseScene(_root, ray); 
	
	// Don't bother shading if the ray didn't hit 
	// anything.
	if (!ray.intersection.none) {
		computeShading(ray);
		
		if ((ray.reflections < MAX_REFLECTIONS) /*&& (ray.refractions <3)*/) {
			// emit another ray
			Vector3D n = ray.intersection.normal;
			n.normalize();
			Vector3D d = ray.dir;
			d.normalize();
			
			double dot = n.dot(d);
			Vector3D newdir = d - (2 * dot * n);
			
			Ray3D newRay = Ray3D(ray.intersection.point + 0.01*newdir,
					newdir, ray.reflections+1);
			Colour secondaryColour = shadeRay(newRay);
			
			double ref = ray.intersection.mat->reflectivity;
			col = (1-ref)*ray.col + ref*secondaryColour;
		} else {
			col = ray.col;
		}
		// Check for refractions		
		// Don't check for refractions of reflected rays
		
		if((ray.intersection.mat->transitivity >= 0.1) && (ray.refractions < MAX_REFRACTIONS) ){ //i.e., don't refract reflected rays
			double c1 = ray.cLight;
			double c2 = ray.intersection.mat->cLight;
			if (ray.cLight < 0.999){//Ray leaves object to air/vacuum
				c2= 1;}
			
		
			Vector3D n = ray.intersection.normal;
			n.normalize();
			Vector3D d = ray.dir;
			d.normalize();

			double dot = n.dot(d);
			Vector3D reflDir = d - (2 * dot * n);
			reflDir.normalize();

			//Now determine refraction direction
			//Depends on reflDir, c1, c2, n, as specified in the relation below
			double theta1 = acos( n.dot(-d) );
			if(dot > 0 ){ //Ray is leaving object
				theta1 = acos( n.dot(d) ); 
			}
			double theta2 = asin(c2*sin(theta1)/c1);

			//Check for critical angle
			
			// Compute refraction direction 
			Vector3D refractDir = (c2/c1)*ray.dir + ( (c2/c1)*cos(theta1) - cos(theta2))*n;
			if(dot > 0 ){ //Ray is leaving object
				refractDir = (c2/c1)*ray.dir - ( (c2/c1)*cos(theta1) - cos(theta2))*n;}
			
			refractDir.normalize();
			
			Ray3D refractRay = Ray3D(ray.intersection.point + 0.001*refractDir, refractDir,ray.reflections, ray.refractions+1, c2 );

			Colour colRefract = shadeRay(refractRay);
			double matTran = ray.intersection.mat->transitivity;
			if(!refractRay.intersection.none){ //Refracted ray does not go off into space
				col = (1-matTran)*col + matTran*colRefract;
			}
		}
		
	}
	
	return col; 
}	

void Raytracer::render( int width, int height, Point3D eye, Vector3D view, 
		Vector3D up, double fov, int AA_level, char* fileName ) {
	Matrix4x4 viewToWorld;
	_scrWidth = width;
	_scrHeight = height;
	double factor = (double(_scrHeight)/2)/tan(fov*M_PI/360.0);

	initPixelBuffer();
	viewToWorld = initInvViewMatrix(eye, view, up);
	
	/**
	 * Extension: Anti-aliasing level via supersampling method
	 * Algorithm:
	 * Generate an image of dimensions equal to a multiple of the original
	 * dimensions (as specified by the AA level)
	 * Then sample individual pixels and average them to compose a pixel
	 * on the actual screen
	 */
	 _aaLevel = AA_level;
	initSuperPixelBuffer();
	
	/// A little print for the user
	fprintf(stderr, "Rendering %dx%d scene, AA-level %d\n", _scrWidth,
			_scrHeight, _aaLevel);
	
	int superi, superj;
	double offi, offj;
	// Construct a ray for each pixel.
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			// Prepare to supersample for anti aliasing
			for (int m = 0; m < _aaLevel; m++) {
			for (int n = 0; n < _aaLevel; n++) {
			
			// Sets up ray origin and direction in view space, 
			// image plane is at z = -1.
			Point3D origin(0, 0, 0);
			Point3D imagePlane;
			offj = double(1)/(0.5 * _aaLevel) + double(n)/_aaLevel;
			offi = double(1)/(0.5 * _aaLevel) + double(m)/_aaLevel;
			imagePlane[0] = (-double((_scrWidth))/2 + offj + j)/factor;
			imagePlane[1] = (-double((_scrHeight))/2 + offi + i)/factor;
			imagePlane[2] = -1;

			// Create the transformed ray and shoot it out (shadeRay)
			Vector3D pixelVector = Vector3D(imagePlane[0], imagePlane[1],
					imagePlane[2]);
			Vector3D transformedPixelVector = viewToWorld * pixelVector;
			Point3D transformedOrigin = viewToWorld * origin;
			
			Ray3D ray = Ray3D(transformedOrigin, transformedPixelVector);

			Colour col = shadeRay(ray); 
			
			superi = i*_aaLevel + m;
			superj = j*_aaLevel + n;
			
			_superrbuffer[superi*(_aaLevel*_scrWidth)+superj] = int(col[0]*255);
			_supergbuffer[superi*(_aaLevel*_scrWidth)+superj] = int(col[1]*255);
			_superbbuffer[superi*(_aaLevel*_scrWidth)+superj] = int(col[2]*255);
			
			}
			}
		}
	}
	
	// Now average out the pixels from the super buffer (supersampling)
	factor = double(1)/(_aaLevel * _aaLevel);
	unsigned long rtemp;
	unsigned long gtemp;
	unsigned long btemp;
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			rtemp = 0;
			gtemp = 0;
			btemp = 0;
			for (int m = 0; m < _aaLevel; m++) {
				superi = i*_aaLevel + m;
				for (int n = 0; n < _aaLevel; n++) {
					superj = j*_aaLevel + n;
					
					rtemp += _superrbuffer[superi*(_aaLevel*_scrWidth)+superj];
					gtemp += _supergbuffer[superi*(_aaLevel*_scrWidth)+superj];
					btemp += _superbbuffer[superi*(_aaLevel*_scrWidth)+superj];
				}
			}
			_rbuffer[i*_scrWidth+j] = factor * rtemp;
			_gbuffer[i*_scrWidth+j] = factor * gtemp;
			_bbuffer[i*_scrWidth+j] = factor * btemp;
		}
	}

	flushPixelBuffer(fileName);
}

int main(int argc, char* argv[])
{	
	// Build your scene and setup your camera here, by calling 
	// functions from Raytracer.  The code here sets up an example
	// scene and renders it from two different view points, DO NOT
	// change this if you're just implementing part one of the 
	// assignment.  
	Raytracer raytracer;
	int width = 320; 
	int height = 240; 
	int aa = 2;

	if (argc == 3) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
	} else if (argc == 4) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
		aa = atoi(argv[3]);
	}

	// Camera parameters.
	Point3D eye(0, 0, 1);
	Vector3D view(0, 0, -1);
	Vector3D up(0, 1, 0);
	double fov = 60;

	// Defines a material for shading.
	Material gold( Colour(0.3, 0.3, 0.3), Colour(0.75164, 0.60648, 0.22648), 
			Colour(0.628281, 0.555802, 0.366065), 
			51.2, 0.01, 0.0, 1/2.4 );
	Material jade( Colour(0.22, 0.38, 0.33), Colour(0.52, 0.73, 0.57), 
			Colour(0.316228, 0.316228, 0.316228), 
			12.8, 0.2 , 0.0, 0.0 );
	Material polishedGold( Colour(0.24725, 0.2245, 0.0645), Colour(0.34615, 0.3143, 0.0903),
			Colour(0.797357, 0.723991, 0.208006), 83.2, 0.01,0.0,0.0);

	Material glass( Colour(0.15, 0.15, 0.15), Colour(0.08, 0.08, 0.08), Colour(0.2, 0.2, 0.2), 10.1,0.05,0.9,1/1.5 );

	// Defines a point light source.
	//~ raytracer.addLightSource( new PointLight(Point3D(0, 0, 5), 
				//~ Colour(0.9, 0.9, 0.9) ) );
	// Defines a ball light source
	raytracer.addLightSource( new BallLight(Point3D(-5, 1, 10),
			10.0, Colour(0.9, 0.9, 0.9), 0.666) );
	raytracer.addLightSource( new BallLight(Point3D(5, -1, 10),
			10.0, Colour(0.9, 0.9, 0.9), 0.666) );
	raytracer.addLightSource( new BallLight(Point3D(0, 0, 16),
			10.0, Colour(0.9, 0.9, 0.9), 0.666) );
	raytracer.addLightSource( new BallLight(Point3D(0, 0, -10),
			5.0, Colour(0.9, 0.9, 0.9), 1.666) );

	// Add a unit square into the scene with material mat.
	SceneDagNode* sphere = raytracer.addObject( new UnitSphere(), &glass);
	SceneDagNode* plane = raytracer.addObject( new UnitSquare(), &jade);
	SceneDagNode* cylinder = raytracer.addObject( new UnitCylinder(), &gold);
	
	// Apply some transformations to the unit square.
	double factor1[3] = { 1.0, 2.0, 1.0 };
	double factor2[3] = { 6.0, 6.0, 6.0 };
	double factor3[3] = { 1.0, 1.0, 2.0 };
	raytracer.translate(sphere, Vector3D(0, 0, -5));	
	raytracer.rotate(sphere, 'x', -45); 
	raytracer.rotate(sphere, 'z', 45); 
	raytracer.scale(sphere, Point3D(0, 0, 0), factor1);

	raytracer.translate(plane, Vector3D(0, 0, -7));	
	raytracer.rotate(plane, 'z', 45); 
	raytracer.scale(plane, Point3D(0, 0, 0), factor2);
	
	
	raytracer.translate(cylinder, Vector3D(4, 0, -5));
	raytracer.rotate(cylinder, 'x', -75); 
	raytracer.scale(cylinder, Point3D(0, 0, 0), factor3);

	


	// Render the scene, feel free to make the image smaller for
	// testing purposes.	
	raytracer.render(width, height, eye, view, up, fov, aa, "view1.bmp");
	
	// Render it from a different point of view.
	Point3D eye2(4, 2, 1);
	Vector3D view2(-4, -2, -6);
	raytracer.render(width, height, eye2, view2, up, fov, aa, "view2.bmp");
	
	
	Point3D eye3(-4, -2, 1);
	Vector3D view3(4, 2, -6);
	raytracer.render(width, height, eye3, view3, up, fov, aa, "view3.bmp");
	
	return 0;
}

