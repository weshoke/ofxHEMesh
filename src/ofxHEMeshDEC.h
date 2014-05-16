#pragma once
#include "Hemesh.h"
#include "Eigen/Sparse"

namespace hemesh {

	void hodgeStar0Form(const Hemesh& hemesh, Eigen::SparseMatrix<double>& star0);
	void hodgeStar1Form(const Hemesh& hemesh, Eigen::SparseMatrix<double>& star1);
	void exteriorDerivative0Form(const Hemesh& hemesh, Eigen::SparseMatrix<double>& d0);
	void laplacian(const Hemesh& hemesh, Eigen::SparseMatrix<double>& L);

	class MeanCurvatureNormals{
	public:
		MeanCurvatureNormals(Hemesh& hemesh);
		Hemesh::Direction getNormal(Vertex v);
		Hemesh::Scalar getMeanCurvature(Vertex v);
		
	protected:
		void build();
		void getPositions();

		const Hemesh& hemesh;
		Eigen::SparseMatrix<double> L;
		Eigen::Matrix<double, Eigen::Dynamic, 3> positions;
		Eigen::Matrix<double, Eigen::Dynamic, 3> normals;
	};

} // hemesh::