#include "ofxHEMeshAdaptiveTools.h"


namespace hemesh {

template<> ofxHEMesh::Scalar powN<1>(ofxHEMesh::Scalar v) {
	return v;
}

template<> ofxHEMesh::Scalar powN<0>(ofxHEMesh::Scalar v) {
	return 1;
}

void planarTangents(const ofxHEMesh::Direction &n, ofxHEMesh::Direction& t1, ofxHEMesh::Direction& t2) {
	ofxHEMesh::Direction probe = n;
	if(probe.y == 0 && probe.z == 0) {
		probe.y += 1;
	}
	else {
		probe.x += 1;
	}

	ofxHEMesh::Scalar along = n.dot(probe);
	t1 = (probe-n*along).normalize();
	t2 = n.crossed(t1);
}

void drawCircle(const ofxHEMesh::Point &center, const ofxHEMesh::Direction& t1, const ofxHEMesh::Direction& t2, ofxHEMesh::Scalar radius) {
	ofxHEMesh::Scalar dtheta = 2.*M_PI/40.;
	ofxHEMesh::Scalar theta = 0.;
	glBegin(GL_LINE_LOOP);
	for(int i=0; i < 40.; ++i) {
		ofxHEMesh::Scalar ca = cos(theta);
		ofxHEMesh::Scalar sa = sin(theta);
		ofxHEMesh::Point p = center + radius*(ca*t1+sa*t2);
		glVertex3fv(p.getPtr());
		theta += dtheta;
	}
	glEnd();
}

SweepTool::SweepTool(ofxHEMeshAdaptive& hemesh, ofxHEMesh::Scalar radius)
:	hemesh(hemesh),
	radius(radius)
{}


} // hemesh::