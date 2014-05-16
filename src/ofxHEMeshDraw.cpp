#include "ofxHEMeshDraw.h"

namespace hemesh {

void faceIndices(const ofxHEMesh& hemesh, vector<ofIndexType>& indices) {
	ofxHEMeshFaceIterator fit = hemesh.facesBegin();
	ofxHEMeshFaceIterator fite = hemesh.facesEnd();
	for(; fit != fite; ++fit) {
		ofxHEMeshFaceCirculator fc = hemesh.faceCirculate(*fit);
		ofxHEMeshFaceCirculator fce = fc;
		do {
			indices.push_back(hemesh.halfedgeVertex(*fc).idx);
			++fc;
		} while(fc != fce);
	}
}

void edgeIndices(const ofxHEMesh& hemesh, vector<ofIndexType>& indices) {
	ofxHEMeshEdgeIterator eit = hemesh.edgesBegin();
	ofxHEMeshEdgeIterator eite = hemesh.edgesEnd();
	for(; eit != eite; ++eit) {
		indices.push_back(hemesh.halfedgeSource(*eit).idx);
		indices.push_back(hemesh.halfedgeSink(*eit).idx);
	}
}

void vertexNormalVectors(const ofxHEMesh& hemesh, vector<ofVec3f> &points, ofxHEMesh::Scalar scale) {
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		ofxHEMesh::Point pt = hemesh.vertexPoint(*vit);
		ofxHEMesh::Point pt2 = pt + hemesh.angleWeightedVertexNormal(*vit)*scale;
		points.push_back(pt);
		points.push_back(pt2);
	}
}

void borderEdges(const ofxHEMesh& hemesh, vector<ofIndexType>& indices) {
	ofxHEMeshEdgeIterator eit = hemesh.edgesBegin();
	ofxHEMeshEdgeIterator eite = hemesh.edgesEnd();
	for(; eit != eite; ++eit) {
		ofxHEMeshHalfedge h = *eit;
		ofxHEMeshHalfedge ho = hemesh.halfedgeOpposite(h);
		if(!hemesh.halfedgeFace(h).isValid()) {
			indices.push_back(hemesh.halfedgeSource(h).idx);
			indices.push_back(hemesh.halfedgeSink(h).idx);
		}
		if(!hemesh.halfedgeFace(ho).isValid()) {
			indices.push_back(hemesh.halfedgeSource(ho).idx);
			indices.push_back(hemesh.halfedgeSink(ho).idx);
		}
	}
}

} // hemesh::