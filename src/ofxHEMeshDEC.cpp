#include "ofxHEMeshDEC.h"

namespace hemesh {

void hodgeStar0Form(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& star0) {
	int n = hemesh.getNumVertices();
	star0.resize(n, n);
	star0.reserve(n);
	
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
	star1.reserve(n);
	
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

typedef Eigen::Triplet<double> Tripletd;

void exteriorDerivative0Form(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& d0) {
	int nV = hemesh.getNumVertices();
	int nE = hemesh.getNumEdges();
	d0.resize(nE, nV);
	
	//d0.reserve(nE*2);
	//d0.reserve(Eigen::VectorXi::Constant(nV, 4));
	vector<Tripletd> entries;
	entries.reserve(nE*2);
	
	ofxHEMeshEdgeIterator eit = hemesh.edgesBegin();
	ofxHEMeshEdgeIterator eite = hemesh.edgesEnd();
	for(; eit != eite; ++eit) {
		ofxHEMeshHalfedge h = *eit;
		ofxHEMeshVertex v1 = hemesh.halfedgeSink(h);
		ofxHEMeshVertex v2 = hemesh.halfedgeSource(h);
		int eidx = h.idx/2;
		//d0.insert(eidx, v1.idx) = 1.;
		//d0.insert(eidx, v2.idx) = -1.;
		entries.push_back(Tripletd(eidx, v1.idx, 1));
		entries.push_back(Tripletd(eidx, v2.idx, -1));
	}
	d0.setFromTriplets(entries.begin(), entries.end());
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

void MeanCurvatureNormals::getNormals(vector<ofxHEMesh::Direction>& normals) {
	normals.resize(hemesh.getNumVertices());
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		normals[(*vit).idx] = getNormal(*vit);
	}
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


MeanCurvatureFlow::MeanCurvatureFlow(ofxHEMesh& hemesh)
: hemesh(hemesh)
{}
		
void MeanCurvatureFlow::step(double amt) {
	laplacian(hemesh, L);
	hodgeStar0Form(hemesh, star0);
	getPositions();
	
	Eigen::SparseMatrix<double> A = star0 + amt*L;
	Eigen::Matrix<double, Eigen::Dynamic, 3> rhs = star0 * positions;
	
	Eigen::SimplicialLDLT< Eigen::SparseMatrix<double> > solver;
	solver.compute(A);
	if(solver.info()!=Eigen::Success) {
		// decomposition failed
		return false;
	}
	
	Eigen::Matrix<double, Eigen::Dynamic, 3> newPositions = solver.solve(rhs);
	if(solver.info()!=Eigen::Success) {
		// solving failed
		return false;
	}
	
	setPositions(newPositions);
}

void MeanCurvatureFlow::getPositions() {
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

void MeanCurvatureFlow::setPositions(Eigen::Matrix<double, Eigen::Dynamic, 3> &newPositions) {
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		ofxHEMeshVertex v = *vit;
		hemesh.vertexMoveTo(v, ofxHEMesh::Point(
			newPositions(v.idx, 0),
			newPositions(v.idx, 1),
			newPositions(v.idx, 2)
		));
	}
}

} // hemesh::