#pragma once
#include "ofxHEMeshNode.h"

struct ofxHEMeshVertexAdjacency {
	ofxHEMeshVertexAdjacency() : he() {}
	
	ofxHEMeshHalfedge he;
};

struct ofxHEMeshHalfedgeAdjacency {
	ofxHEMeshHalfedgeAdjacency() :
		v(), f(), prev(), next()
	{}
	
	ofxHEMeshVertex v;
	ofxHEMeshFace f;
	ofxHEMeshHalfedge prev;
	ofxHEMeshHalfedge next;
};

struct ofxHEMeshFaceAdjacency {
	ofxHEMeshFaceAdjacency() : he() {}
	
	ofxHEMeshHalfedge he;
};