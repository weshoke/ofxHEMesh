#pragma once
#include "ofMain.h"
#include "ofxHEMesh.h"

struct aiScene;


class ofxHEMeshAssimpLoader{
public:
	ofxHEMeshAssimpLoader();
	~ofxHEMeshAssimpLoader();
	
	bool loadModel(string modelName, bool triangulate=false, bool optimize=false);
	void addToHemesh(ofxHEMesh& hemesh, int idx);
	
protected:
	void clear();
	bool loadModel(ofBuffer& buffer, bool triangulate, bool optimize, const char * extension);


	const aiScene *scene;
};


