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
#include "util.h"
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

void Raytracer::computeShading( Ray3D& ray, char renderStyle) {
	if (ray.intersection.none) { return; }	// don't bother checking
	LightListNode* curLight = _lightSource;
	for (;;) {
		if (curLight == NULL) break;
		// Each lightSource provides its own shading function.

		// Implement shadows here if needed.
		
		curLight->light->shade(ray, this, renderStyle);
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
	#ifdef IGNORE_SHADOWS
	return 1.0;
	#endif

	#ifdef USE_TRANSMISSIONSHADOWS
	return getLightTransmissionRecurse(_root, ray);
	#else
	// less expensive algorithm
	double max_t_value = ray.intersection.t_value;
	ray.intersection.none = true;
	traverseScene(_root, ray);
	if (ray.intersection.t_value < max_t_value) {
		return 0.1;	// looks more natural
	}
	return 1.0;
	#endif
}

double Raytracer::getLightTransmissionRecurse( SceneDagNode* node, Ray3D& ray ) {
	double transmission = 1.0;
	double max_t_value = ray.intersection.t_value;
	double dot_product;
	
	// This is pretty much traverse scene, but resets the t_value
	// and intersection as it goes along
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
		transmission = transmission *
				getLightTransmissionRecurse(childPtr, ray);
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

Colour Raytracer::shadeRay( Ray3D& ray, char renderStyle ) {
	Colour col(0.0, 0.0, 0.0); 
	traverseScene(_root, ray); 
	
	// Don't bother shading if the ray didn't hit 
	// anything.
	if (!ray.intersection.none) {
		//computeShading(ray);
		if (renderStyle != 's'){
			computeShading(ray, renderStyle);
			col = ray.col;
			   
			#ifdef USE_REFLECTIONS
			if ((ray.intersection.mat->reflectivity >= 0.01) && (ray.reflections < MAX_REFLECTIONS) && (renderStyle != 'd')) {
				// emit another ray
				Vector3D n = ray.intersection.normal;
				n.normalize();
				Vector3D d = ray.dir;
				d.normalize();
	

			
				double dot = n.dot(d);
				Vector3D newdir = d - (2 * dot * n);
			
				Ray3D newRay = Ray3D(ray.intersection.point + 0.01*newdir,
						newdir, ray.reflections+1, ray.refractions, ray.cLight);
				Colour secondaryColour = shadeRay(newRay, renderStyle);
			

				double ref = ray.intersection.mat->reflectivity;
				col = (1-ref)*ray.col + ref*secondaryColour;
			} else {
				col = ray.col;
			}
			#else
			col = ray.col;
			#endif
			// Check for refractions		
			// Don't check for refractions of reflected rays
			#ifdef USE_REFRACTIONS
			if((ray.intersection.mat->transitivity >= 0.1) && (ray.refractions < MAX_REFRACTIONS) && (renderStyle != 'd')){ 
				double c1 = ray.cLight;
				double c2 = ray.intersection.mat->cLight;
				if (ray.cLight < 0.99){//Ray leaves object to air/vacuum
					c2= 1;
				}

			
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
				if(dot > 0 ){ //Ray is leaving object     =====================changed sign
					refractDir = (c2/c1)*ray.dir - ( (c2/c1)*cos(theta1) - cos(theta2))*n;
				}
			
				refractDir.normalize();
			
				Ray3D refractRay = Ray3D(ray.intersection.point + 0.0001*refractDir, refractDir,ray.reflections, ray.refractions+1, c2 );


				Colour colRefract = shadeRay(refractRay, renderStyle);
				double matTran = ray.intersection.mat->transitivity;
				if(!refractRay.intersection.none){ //Refracted ray does not go off into space
					col = (1-matTran)*col + matTran*colRefract;
				}
			}//end of refractions
			#endif

		}//End of check if(renderStyle != 's')
		else{ //renderStyle == 's'
			col = (*(ray.intersection.mat)).diffuse;
		}
	}//End of check if (!ray.intersection.none) 
	
	return col; 
}// End of shadeRay	

void Raytracer::render( int width, int height, Point3D eye, Vector3D view, 
		Vector3D up, double fov, int AA_level, char* fileName, char renderStyle ) {
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
			
			//Check for scene render style
			Colour col = shadeRay(ray, renderStyle); 
			
			superi = i*_aaLevel + m;
			superj = j*_aaLevel + n;
			
			_superrbuffer[superi*(_aaLevel*_scrWidth)+superj] = int(col[0]*255);
			_supergbuffer[superi*(_aaLevel*_scrWidth)+superj] = int(col[1]*255);
			_superbbuffer[superi*(_aaLevel*_scrWidth)+superj] = int(col[2]*255);
			
			}
			}//End supersampling loop
		}
	}//finsihed pixel loop
	
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
	int sceneNum = 0;

	double toRadian = 2*M_PI/360.0;

	fprintf(stderr, "Using options:\n");
	
	#ifdef USE_EXTENDEDLIGHTS
	fprintf(stderr, "\tExtended light sources\n");
	#else
	fprintf(stderr, "\tPoint light sources\n");
	#endif
	
	#ifdef USE_REFRACTIONS
	fprintf(stderr, "\tRefractions\n");
	#else
	fprintf(stderr, "\tNo refractions\n");
	#endif
	
	#ifdef USE_REFLECTIONS
	fprintf(stderr, "\tReflections\n");
	#else
	fprintf(stderr, "\tNo reflections\n");
	#endif
	 
	#ifdef IGNORE_SHADOWS
		fprintf(stderr, "\tNo shadows\n");
	#else{
		#ifdef USE_TRANSMISSIONSHADOWS
		fprintf(stderr, "\tTransmission-based shadows\n");
		#else
		fprintf(stderr, "\tSimple shadows\n");
		#endif
	 }
	#endif
	
	#ifdef USE_FINERFLUX
	fprintf(stderr, "\tFiner numerical flux intergrations\n");
	#else
	fprintf(stderr, "\tCoarser numerical flux intergrations\n");
	#endif


	if (argc == 3) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
	} else if (argc == 4) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
		aa = atoi(argv[3]);
	} else if (argc == 5) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
		aa = atoi(argv[3]);
		sceneNum = atoi(argv[4]);
	}
	// SceneNum should not exceed total scenes
	if ((sceneNum > 3)|| (sceneNum <0)){
		sceneNum = 0;
	}
	// Camera parameters.
	Point3D eye(0, 0, 1);
	Vector3D view(0, 0, -1);
	Vector3D up(0, 1, 0);
	double fov = 60;
	
	
	// Defines materials for shading.
	Material gold( Colour(0.3, 0.3, 0.3), Colour(0.75164, 0.60648, 0.22648), 
			Colour(0.628281, 0.555802, 0.366065), 
			51.2, 0.001, 0.0, 1/2.4 );
	Material jade( Colour(0.22, 0.38, 0.33), Colour(0.52, 0.73, 0.57), 
			Colour(0.316228, 0.316228, 0.316228), 
			12.8, 0.2 , 0.0, 0.0 );
	Material polishedGold( Colour(0.24725, 0.2245, 0.0645), Colour(0.34615, 0.3143, 0.0903),
			Colour(0.797357, 0.723991, 0.208006), 83.2, 0.01,0.0,0.0);

	Material glass( Colour(0.15, 0.15, 0.15), Colour(0.08, 0.08, 0.08), 
			Colour(0.2, 0.2, 0.2), 10.1,0.08,0.9,0.6667 );
	
	Material glass1( Colour(0.2, 0.2, 0.2), Colour(0.2, 0.2, 0.2), 
			Colour(0.7, 0.7, 0.7), 25.1,0.0,0.9,0.6667 );
	
	
	Material steel( Colour(0.1, 0.1, 0.1), Colour(0.1, 0.1, 0.1), 
			Colour(0.8, 0.8, 0.8), 80, 0.03, 0.0, 1.0 );

	Material blueSolid( Colour(0, 0, 1), Colour(0, 0, 1), 
			Colour(0, 0, 0), 0, 0.0, 0.0, 1.0 );

	Material redSolid( Colour(1, 0, 0), Colour(1, 0, 0), 
			Colour(0, 0, 0), 0, 0.0, 0.0, 1.0 );

	Material chrome( Colour(0.25, 0.25, 0.25), Colour(0.4,0.4,0.4), 
			Colour(0.7746, 0.7746, 0.7746), 77, 0.42, 0.0, 1.0);
	
	Material ruby( Colour(0.1745, 0.01175, 0.01175), Colour(0.61424, 0.04136, 0.04136),
			Colour(0.727811, 0.626959, 0.626959) , 76.8, 0.01, 0.45, 0.565);

	Material pearl( Colour(0.25, 0.20725, 0.20725), Colour(1, 0.829, 0.829), 
			Colour(0.296648, 0.296648, 0.296648), 11.264, 0.1,0.0,1.0 );

	Material silver(Colour(0.23125, 0.23125, 0.23125), Colour(0.2775, 0.2775, 0.2775),
			Colour(0.773911, 0.773911, 0.773911), 89.6, 0.4,0.0, 1.0);

	Material emerald(Colour(0.0215, 0.1745, 0.0215),Colour(0.07568, 0.61424, 0.07568),
			Colour(0.633, 0.727811, 0.633), 76.8, 0.1, 0.25, 0.637);
	
	Material brass(Colour(0.329412, 0.223529,  0.027451),Colour(0.780392, 0.568627, 0.113725),
			Colour(0.992157, 0.941176, 0.807843),27.8974, 0.3, 0.0, 1.0 );

	Material bronze(Colour(0.2125, 0.1275, 0.054), Colour(0.714, 0.4284, 0.18144), 
		Colour(0.393548, 0.271906, 0.166721), 25.6, 0.1, 0.0, 1.0 );
	
	Material bronzeShiny(Colour(0.25, 0.148, 0.06475), Colour(0.4, 0.2368, 0.1036), 
		Colour(0.774597, 0.458561, 0.200621), 76.86, 0.15, 0.0, 1.0 );
	
	Material turquoise(Colour(0.1, 0.18725, 0.1745), Colour(0.396, 0.74151, 0.69102),
			Colour(0.297254, 0.30829, 0.306678), 12.8, 0.01, 0.2, 0.9);

	Material obsidian(Colour(0.05375, 0.05, 0.06625), Colour(0.18275, 0.17, 0.22525),
			Colour(0.332741, 0.328634, 0.346435), 38.4, 0.05, 0.18, 0.413);

	Material copper(Colour(0.19125, 0.0735, 0.0225), Colour(0.7038, 0.27048, 0.0828), 
			Colour(0.256777, 0.137622, 0.086014), 12.8, 0.1, 0.0, 1.0 );
	
	Material copperPolished(Colour(0.2295, 0.08825, 0.0275), Colour(0.5508, 0.2118, 0.066), 
			Colour(0.580594, 0.223257, 0.0695701), 51.2, 0.15, 0.0, 1.0 );
	
	Material pewter(Colour(0.105882, 0.058824, 0.113725), Colour(0.427451, 0.470588, 0.541176), 
			Colour(0.333333, 0.333333, 0.521569), 9.84615, 0.0, 0.0, 1.0 );


	// Light Sources
	//=====================	
	//raytracer.addLightSource( new PointLight(Point3D(1, 1, 2),Colour(0.5, 0.5, 0.5)) );

	#ifdef USE_EXTENDEDLIGHTS
	// Defines a ball light source
	raytracer.addLightSource( new BallLight(Point3D(0, 0, 7),
			2.0, Colour(0.9, 0.9, 0.9), 3) );
	#else
	// Defines a point light source.
	raytracer.addLightSource( new PointLight(Point3D(0, 0, 5), 
				Colour(0.9, 0.9, 0.9) ) );
	#endif


	if (sceneNum==0){

		// Defines a point light source.
		//raytracer.addLightSource( new PointLight(Point3D(0, 0, 5), 
		//			Colour(0.9, 0.9, 0.9) ) );

		// Add a unit square into the scene with material mat.
		SceneDagNode* sphere = raytracer.addObject( new UnitSphere(), &turquoise);
		SceneDagNode* plane = raytracer.addObject( new UnitSquare(), &brass );
	
		// Apply some transformations to the unit square.
		double factor1[3] = { 1.0, 2.0, 1.0 };
		double factor2[3] = { 6.0, 6.0, 1.0 };
		double factor3[3] = { 4.0, 4.0, 4.0 };
		double factor4[3] = { 3.7, 3.7, 3.7 };
		raytracer.translate(sphere, Vector3D(0, 0, -5));	
		raytracer.rotate(sphere, 'x', -45); 
		raytracer.rotate(sphere, 'z', 45); 
		raytracer.scale(sphere, Point3D(0, 0, 0), factor1);

		raytracer.translate(plane, Vector3D(0, 0, -7));	
		raytracer.rotate(plane, 'z', 45); 
		raytracer.scale(plane, Point3D(0, 0, 0), factor2);
		/*
		SceneDagNode* bigSphere = raytracer.addObject( new UnitSphere(), &glass1);
		raytracer.scale(bigSphere, Point3D(0, 0, 0), factor3);
		raytracer.translate(bigSphere, Vector3D(0, 0, -7));

		SceneDagNode* bigSphere2 = raytracer.addObject( new UnitSphere(), &glass1);
		raytracer.scale(bigSphere2, Point3D(0, 0, 0), factor4);
		raytracer.translate(bigSphere2, Vector3D(0, 0, -7));
		*/

	}// end of scene 0

	if (sceneNum==1){
		/*
		raytracer.addLightSource( new BallLight(Point3D(-1, 1, 1),
			5.0, Colour(0.9, 0.9, 0.9), 0.888) );
		raytracer.addLightSource( new PointLight(Point3D(0, 0, 2),Colour(0.5, 0.5, 0.5)) );
		*/

		// Add a unit square into the scene with material mat.
		SceneDagNode* sphere = raytracer.addObject( new UnitSphere(), &glass);
		SceneDagNode* sphere1 = raytracer.addObject( new UnitSphere(), &gold);
		SceneDagNode* plane = raytracer.addObject( new UnitSquare(), &jade);
		SceneDagNode* cylinder = raytracer.addObject( new UnitCone(), &gold);

	
		// Apply some transformations to the unit square.
		double factor1[3] = { 1.0, 2.0, 1.0 };
		double factor2[3] = { 6.0, 6.0, 1.0 };
		double factor3[3] = { 2.0, 2.0, 3.0 };
		raytracer.translate(sphere, Vector3D(0, 0, -5));	
		raytracer.rotate(sphere, 'x', -45); 
		raytracer.rotate(sphere, 'z', 45); 
		raytracer.scale(sphere, Point3D(0, 0, 0), factor1);

		raytracer.translate(sphere1, Vector3D(-2, 0, -5));	
	
		raytracer.translate(plane, Vector3D(0, 0, -7));	
		raytracer.rotate(plane, 'z', 45); 
		raytracer.scale(plane, Point3D(0, 0, 0), factor2);
	
	
		raytracer.translate(cylinder, Vector3D(3, 0, -5));
		//raytracer.rotate(cylinder, 'y', -20); 
		raytracer.rotate(cylinder, 'x', -90); 
		raytracer.scale(cylinder, Point3D(0, 0, 0), factor3);

	}// end of scene1


	//=============== Scene 2 ==============================
	//=====================================================

	if(sceneNum == 2){
		/*
		raytracer.addLightSource( new BallLight(Point3D(-1, 1, 1),
			5.0, Colour(0.9, 0.9, 0.9), 0.888) );
		raytracer.addLightSource( new PointLight(Point3D(0, 0, 2),Colour(0.5, 0.5, 0.5)) );
		*/
		//Set up walls
		//========================================================

		SceneDagNode* planeBack = raytracer.addObject( new UnitSquare(), &brass);
		SceneDagNode* planeBottom = raytracer.addObject( new UnitSquare(), &chrome);
		SceneDagNode* planeTop = raytracer.addObject( new UnitSquare(), &copperPolished);
		SceneDagNode* planeLeft = raytracer.addObject( new UnitSquare(), &bronzeShiny);
		SceneDagNode* planeRight = raytracer.addObject( new UnitSquare(), &brass);
		SceneDagNode* planeRear = raytracer.addObject( new UnitSquare(), &brass);

		double scaleFactor[3] = {8.0,8.0,1.0};
		double scaleFactor1[3] = {20.01,20.01,1.0};

		raytracer.translate(planeBottom, Vector3D(0, -10, 0));
		raytracer.translate(planeTop, Vector3D(0, 10, 0));
		raytracer.translate(planeLeft, Vector3D(-10, 0, 0));

		raytracer.translate(planeRight, Vector3D(10, 0, 0));
	
		raytracer.translate(planeBack, Vector3D(0, 0, -19.9));
		raytracer.translate(planeBottom, Vector3D(0, 0, -10));
		raytracer.translate(planeTop, Vector3D(0, 0, -10));
		raytracer.translate(planeLeft, Vector3D(0, 0, -10));
		raytracer.translate(planeRight, Vector3D(0, 0, -10));
		raytracer.translate(planeRear, Vector3D(0, 0, 20));
	
		raytracer.rotate(planeTop, 'x', 90); 
		raytracer.rotate(planeBottom, 'x',-90); 
		raytracer.rotate(planeLeft, 'y', -90); 
		raytracer.rotate(planeRight, 'y', 90); 
		raytracer.rotate(planeRear, 'x', 180);

		raytracer.scale(planeBack, Point3D(0, 0, 0), scaleFactor1);
		raytracer.scale(planeBottom, Point3D(0, 0, 0), scaleFactor1);
		raytracer.scale(planeTop, Point3D(0, 0, 0), scaleFactor1);
		raytracer.scale(planeLeft, Point3D(0, 0, 0), scaleFactor1);
		raytracer.scale(planeRight, Point3D(0, 0, 0), scaleFactor1);
		raytracer.scale(planeRear, Point3D(0, 0, 0), scaleFactor1);
		//===========================================================
	
		double scaleBall[3] = {2,2,2};
		SceneDagNode* sphere = raytracer.addObject( new UnitSphere(), &glass1);
		SceneDagNode* sphere1 = raytracer.addObject( new UnitSphere(), &ruby);
		SceneDagNode* sphere2 = raytracer.addObject( new UnitSphere(), &chrome);
		//SceneDagNode* cone = raytracer.addObject(sphere, new UnitCone(), &brass);

		//raytracer.translate(cone, Vector3D(0,0,-2));
		raytracer.translate(sphere, Vector3D(-1,-1,-11));
		raytracer.scale(sphere, Point3D(0,0,0), scaleBall);
	
		raytracer.translate(sphere1, Vector3D(2,-1,-11));
		raytracer.translate(sphere2, Vector3D(2,3,-11));
		//raytracer.translate(cone, Vector3D(-1,-1,-12));
		//raytracer.rotate(cone, 'x', -90);

	}//end of scene 2
	

	//==================== Scene 3 =================
	//===============================================
	if(sceneNum == 3){
		/*
		raytracer.addLightSource( new BallLight(Point3D(-1, 1, 1),
			5.0, Colour(0.9, 0.9, 0.9), 0.888) );
		raytracer.addLightSource( new PointLight(Point3D(0, 0, 2),Colour(0.5, 0.5, 0.5)) );
		//raytracer.addLightSource( new PointLight(Point3D(0, 0, 2),Colour(0.5, 0.5, 0.5)) );
		*/

		double factor1[3] = { 1.0, 1.0, 3.0 };
		double factor2[3] = { 6.0, 6.0, 1.0 };
		double factor3[3] = { 2.0, 2.0, 3.0 };

		SceneDagNode* plane1 = raytracer.addObject( new UnitSquare(), &chrome);
		SceneDagNode* plane2 = raytracer.addObject( new UnitSquare(), &brass);
		SceneDagNode* plane3 = raytracer.addObject( new UnitSquare(), &brass);
		SceneDagNode* cone  = raytracer.addObject( new UnitCone(), &ruby);
		SceneDagNode* cylinder = raytracer.addObject( new UnitCylinder(), &turquoise );


		
		raytracer.translate(cylinder, Vector3D(0,-3,-7));
		raytracer.rotate(cylinder, 'x', -90);
		raytracer.scale(cylinder, Point3D(0,0,0), factor1);
		
		raytracer.translate(cone, Vector3D(0,3,-7));
		raytracer.rotate(cone, 'x', 90);
		raytracer.scale(cone, Point3D(0,0,0), factor1);
		

		raytracer.translate(plane1, Vector3D(0,0,-10));
		raytracer.translate(plane2, Vector3D(4,0,-7));
		raytracer.translate(plane3, Vector3D(-4,0,-7));

		raytracer.rotate(plane2, 'y', -75);
		raytracer.rotate(plane3, 'y', 75);

		raytracer.scale(plane1, Point3D(0,0,0), factor2);
		raytracer.scale(plane2, Point3D(0,0,0), factor2);
		raytracer.scale(plane3, Point3D(0,0,0), factor2);
		
		//==== face ====
		SceneDagNode* eyeLfront = raytracer.addObject( new UnitSquare(), &pearl);
		SceneDagNode* eyeRfront = raytracer.addObject( new UnitSquare(), &emerald);
		
		SceneDagNode* eyeLback = raytracer.addObject( new UnitSquare(), &emerald);
		SceneDagNode* eyeRback = raytracer.addObject( new UnitSquare(), &emerald);
		
		double eyeScale[3] = {0.4, 0.4, 1};


		raytracer.translate(eyeLfront, Vector3D(0.5,-0.5,-5));
		raytracer.translate(eyeRfront, Vector3D(-0.5,-0.5,-5));
		raytracer.translate(eyeLback, Vector3D(-0.5,-0.5,-9));
		raytracer.translate(eyeRback, Vector3D(0.5,-0.5,-9));

		raytracer.rotate(eyeLfront, 'z', 45);
		raytracer.rotate(eyeRfront, 'z', 45);
		raytracer.rotate(eyeLback, 'z', 45);
		raytracer.rotate(eyeRback, 'z', 45);
		
		raytracer.scale(eyeLfront, Point3D(0,0,0), eyeScale);
		raytracer.scale(eyeRfront, Point3D(0,0,0), eyeScale);
		raytracer.scale(eyeLback, Point3D(0,0,0), eyeScale);
		raytracer.scale(eyeRback, Point3D(0,0,0), eyeScale);
	
	}//end of scene 3


	// Render the scene, feel free to make the image smaller for
	// testing purposes.	

	raytracer.render(width, height, eye, view, up, fov, aa,  "sig1.bmp", 's');
	raytracer.render(width, height, eye, view, up, fov, aa, "diffuse1.bmp",'d');
	raytracer.render(width, height, eye, view, up, fov, aa, "view1.bmp",'p');
	
	
	

	
	// Render it from a different point of view.
	Point3D eye2(4, 2, 1);
	Vector3D view2(-4, -2, -6);
	
	raytracer.render(width, height, eye2, view2, up, fov, aa, "sig2.bmp", 's');
	raytracer.render(width, height, eye2, view2, up, fov, aa, "diffuse2.bmp",'d');
	raytracer.render(width, height, eye2, view2, up, fov, aa, "view2.bmp",'p');
	
	
	
	Point3D eye3(-4, -2, 1);
	Vector3D view3(4, 2, -6);

	raytracer.render(width, height, eye3, view3, up, fov, aa, "sig3.bmp", 's');
	raytracer.render(width, height, eye3, view3, up, fov, aa, "diffuse3.bmp",'d');
	raytracer.render(width, height, eye3, view3, up, fov, aa, "view3.bmp",'p');
	
	
	
	
	return 0;
}

