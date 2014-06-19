#pragma once
#include "ofxHEMeshAdaptive.h"

namespace hemesh {

template<unsigned N>
ofxHEMesh::Scalar powN(ofxHEMesh::Scalar v) {
	return v*powN<N-1>(v);
}

template<> ofxHEMesh::Scalar powN<1>(ofxHEMesh::Scalar v);
template<> ofxHEMesh::Scalar powN<0>(ofxHEMesh::Scalar v);

template<unsigned N>
ofxHEMesh::Scalar blend(ofxHEMesh::Scalar v) {
	return ofxHEMesh::Scalar(N-1)*powN<N>(v) - ofxHEMesh::Scalar(N)*powN<N-1>(v) + 1.;
}


void planarTangents(const ofxHEMesh::Direction &n, ofxHEMesh::Direction& t1, ofxHEMesh::Direction& t2);
void drawCircle(const ofxHEMesh::Point &center, const ofxHEMesh::Direction& t1, const ofxHEMesh::Direction& t2, ofxHEMesh::Scalar radius);


class SweepTool {
public:
	SweepTool(ofxHEMeshAdaptive& hemesh, ofxHEMesh::Scalar radius);
	
	ofxHEMesh::Scalar getRadius() const { return radius; }
	ofxHEMesh::Point getCenter() const { return center; }
	void setCenter(const ofxHEMesh::Point& v) { center = v; }
	
protected:
	ofxHEMeshAdaptive& hemesh;
	ofxHEMesh::Scalar radius;
	ofxHEMesh::Point center;
};


} // hemesh::