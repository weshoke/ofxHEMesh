#include "HemeshDraw.h"

namespace hemesh {

void faceIndices(const Hemesh& hemesh, vector<ofIndexType>& indices) {
	FaceIterator fit = hemesh.facesBegin();
	FaceIterator fite = hemesh.facesEnd();
	for(; fit != fite; ++fit) {
		FaceCirculator fc = hemesh.faceCirculate(*fit);
		FaceCirculator fce = fc;
		do {
			indices.push_back(hemesh.halfedgeVertex(*fc).idx);
			++fc;
		} while(fc != fce);
	}
}

void edgeIndices(const Hemesh& hemesh, vector<ofIndexType>& indices) {
	EdgeIterator eit = hemesh.edgesBegin();
	EdgeIterator eite = hemesh.edgesEnd();
	for(; eit != eite; ++eit) {
		indices.push_back(hemesh.halfedgeSource(*eit).idx);
		indices.push_back(hemesh.halfedgeSink(*eit).idx);
	}
}

void vertexNormalVectors(const Hemesh& hemesh, vector<ofVec3f> &points, Hemesh::Scalar scale) {
	VertexIterator vit = hemesh.verticesBegin();
	VertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		Hemesh::Point pt = hemesh.vertexPoint(*vit);
		Hemesh::Point pt2 = pt + hemesh.angleWeightedVertexNormal(*vit)*scale;
		points.push_back(pt);
		points.push_back(pt2);
	}
}

void borderEdges(const Hemesh& hemesh, vector<ofIndexType>& indices) {
	EdgeIterator eit = hemesh.edgesBegin();
	EdgeIterator eite = hemesh.edgesEnd();
	for(; eit != eite; ++eit) {
		Halfedge h = *eit;
		Halfedge ho = hemesh.halfedgeOpposite(h);
		if(!hemesh.halfedgeFace(h).isValid()) {
			indices.push_back(hemesh.halfedgeSource(h).idx);
			indices.push_back(hemesh.halfedgeSink(h).idx);
			//std::cout << hemesh.vertexPoint(hemesh.halfedgeSource(h)) << " - " << hemesh.vertexPoint(hemesh.halfedgeSink(h)) << "\n";
		}
		if(!hemesh.halfedgeFace(ho).isValid()) {
			indices.push_back(hemesh.halfedgeSource(ho).idx);
			indices.push_back(hemesh.halfedgeSink(ho).idx);
			//std::cout << hemesh.vertexPoint(hemesh.halfedgeSource(ho)) << " - " << hemesh.vertexPoint(hemesh.halfedgeSink(ho)) << "\n";
		}
	}
}

} // hemesh::