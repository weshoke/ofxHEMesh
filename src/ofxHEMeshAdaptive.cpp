#include "ofxHEMeshAdaptive.h"


ofxHEMeshAdaptive::ofxHEMeshAdaptive(Scalar detail)
: detail(detail)
{
	detail2 = detail*detail;
	edgeLength = detail/2.15;
	edgeLength2 = edgeLength*edgeLength;
	thickness = detail*0.6;
	thickness2 = thickness*thickness;
	maxMove = sqrt((thickness*thickness - detail*detail/3.)/4.);
}

void ofxHEMeshAdaptive::initializeMesh() {
	std::cout << meanEdgeLength() << " dev: " << sqrt(edgeLengthVariance(meanEdgeLength())) << "\n";
	centroidTriangulation();
	Scalar mu = meanEdgeLength();
	Scalar sigma = sqrt(edgeLengthVariance(mu));
	while((mu+sigma) > detail) {
		remeshLoop();
		mu *= 0.5;
		sigma *= 0.5;
	}
}

void ofxHEMeshAdaptive::splitLongEdges() {
	ofxHEMeshEdgeIterator eit = edgesBegin();
	ofxHEMeshEdgeIterator eite = edgesEnd();
	for(; eit != eite; ++eit) {
		if(halfedgeShouldBeSplit(*eit)) {
			splitHalfedgeAndTriangulate(*eit);
		}
	}
}

bool ofxHEMeshAdaptive::halfedgeShouldBeSplit(ofxHEMeshHalfedge h) {
	return halfedgeLengthSquared(h) > detail2;
}

void ofxHEMeshAdaptive::splitHalfedgeAndTriangulate(ofxHEMeshHalfedge h) {
	ofxHEMeshVertex v = splitHalfedge(h);
	ofxHEMeshHalfedge hn1 = vertexHalfedge(v);
	ofxHEMeshHalfedge hn2 = halfedgeSinkCCW(hn1);
	connectHalfedgesCofacial(hn1, halfedgeNext(halfedgeNext(hn1)));
	connectHalfedgesCofacial(hn2, halfedgeNext(halfedgeNext(hn2)));
}

void ofxHEMeshAdaptive::collapseShortEdges() {
	ofxHEMeshEdgeIterator eit = edgesBegin();
	ofxHEMeshEdgeIterator eite = edgesEnd();
	for(; eit != eite; ++eit) {
		if(halfedgeShouldBeCollapsed(*eit)) {
			collapseHalfedge(*eit);
		}
	}
}

bool ofxHEMeshAdaptive::halfedgeShouldBeCollapsed(ofxHEMeshHalfedge h) {
	return halfedgeLengthSquared(h) < edgeLength2;
}