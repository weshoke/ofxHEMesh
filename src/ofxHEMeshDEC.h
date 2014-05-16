#pragma once
#include "ofxHEMesh.h"
#include "Eigen/Sparse"

namespace hemesh {

	void hodgeStar0Form(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& star0);
	void hodgeStar1Form(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& star1);
	void exteriorDerivative0Form(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& d0);
	void laplacian(const ofxHEMesh& hemesh, Eigen::SparseMatrix<double>& L);

	class MeanCurvatureNormals{
	public:
		MeanCurvatureNormals(ofxHEMesh& hemesh);
		ofxHEMesh::Direction getNormal(ofxHEMeshVertex v);
		ofxHEMesh::Scalar getMeanCurvature(ofxHEMeshVertex v);
		
	protected:
		void build();
		void getPositions();

		const ofxHEMesh& hemesh;
		Eigen::SparseMatrix<double> L;
		Eigen::Matrix<double, Eigen::Dynamic, 3> positions;
		Eigen::Matrix<double, Eigen::Dynamic, 3> normals;
	};

} // hemesh::