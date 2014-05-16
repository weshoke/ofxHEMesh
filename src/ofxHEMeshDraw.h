#pragma once
#include "ofxHEMesh.h"

namespace hemesh {

	void faceIndices(const ofxHEMesh& hemesh, vector<ofIndexType>& indices);
	void edgeIndices(const ofxHEMesh& hemesh, vector<ofIndexType>& indices);
	void vertexNormalVectors(const ofxHEMesh& hemesh, vector<ofVec3f> &points, ofxHEMesh::Scalar scale=1.);
	void borderEdges(const ofxHEMesh& hemesh, vector<ofIndexType>& indices);

} // hemesh::