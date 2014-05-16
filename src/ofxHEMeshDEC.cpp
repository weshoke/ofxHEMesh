#include "ofxHEMeshDEC.h"

namespace hemesh {

void hodgeStar0Form(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& star0) {
	int n = hemesh.getNumVertices();
	star0.resize(n, n);
	
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		ofxHEMeshVertex v = *vit;
		star0.insert(v.idx, v.idx) = hemesh.vertexArea(v);
	}
}

void hodgeStar1Form(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& star1) {
	int n = hemesh.getNumEdges();
	star1.resize(n, n);
	
	ofxHEMeshEdgeIterator eit = hemesh.edgesBegin();
	ofxHEMeshEdgeIterator eite = hemesh.edgesEnd();
	for(; eit != eite; ++eit) {
		ofxHEMeshHalfedge h = *eit;
		ofxHEMeshHalfedge ho = hemesh.halfedgeOpposite(h);
		ofxHEMesh::Scalar cotAlpha = hemesh.halfedgeCotan(h);
		ofxHEMesh::Scalar cotBeta  = hemesh.halfedgeCotan(ho);
		int eidx = h.idx/2;
		star1.insert(eidx, eidx) = (cotAlpha + cotBeta)*0.5;
	}
}

void exteriorDerivative0Form(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& d0) {
	int nV = hemesh.getNumVertices();
	int nE = hemesh.getNumEdges();
	d0.resize(nE, nV);
	
	ofxHEMeshEdgeIterator eit = hemesh.edgesBegin();
	ofxHEMeshEdgeIterator eite = hemesh.edgesEnd();
	for(; eit != eite; ++eit) {
		ofxHEMeshHalfedge h = *eit;
		ofxHEMeshVertex v1 = hemesh.halfedgeVertex(h);
		ofxHEMeshVertex v2 = hemesh.halfedgeSource(h);
		int eidx = h.idx/2;
		d0.insert(eidx, v1.idx) = 1.;
		d0.insert(eidx, v2.idx) = -1.;
	}
}

void laplacian(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& L) {
	Eigen::SparseMatrix<double> d0;
	Eigen::SparseMatrix<double> star1;
	exteriorDerivative0Form(hemesh, d0);
	hodgeStar1Form(hemesh, star1);
	L = d0.transpose()*star1*d0;
}



MeanCurvatureNormals::MeanCurvatureNormals(ofxHEMesh& hemesh)
:	hemesh(hemesh)
{}

void MeanCurvatureNormals::build() {
	laplacian(hemesh, L);
	getPositions();
	normals = L*positions;
}

ofxHEMesh::Direction MeanCurvatureNormals::getNormal(ofxHEMeshVertex v) {
	return ofxHEMesh::Direction(normals(v.idx, 0), normals(v.idx, 1), normals(v.idx, 2));
}

void MeanCurvatureNormals::getPositions() {
	positions.resize(L.rows(), 3);
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		ofxHEMeshVertex v = *vit;
		ofxHEMesh::Point pos = hemesh.vertexPoint(v);
		positions(v.idx, 0) = (double)pos[0];
		positions(v.idx, 1) = (double)pos[1];
		positions(v.idx, 2) = (double)pos[2];
	}
}

} // hemesh::