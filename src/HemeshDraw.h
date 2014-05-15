#pragma once
#include "Hemesh.h"

namespace hemesh {

	void faceIndices(const Hemesh& hemesh, vector<ofIndexType>& indices);
	void edgeIndices(const Hemesh& hemesh, vector<ofIndexType>& indices);
	void vertexNormalVectors(const Hemesh& hemesh, vector<ofVec3f> &points, Hemesh::Scalar scale=1.);
	void borderEdges(const Hemesh& hemesh, vector<ofIndexType>& indices);

} // hemesh::