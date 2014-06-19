#pragma once
#include "ofxHEMeshAdaptive.h"
#include "ofxOpenVDB.h"

class ofxHEMeshAdaptiveGrid :
	public ofxHEMeshAdaptive,
	public ofxHEMesh::GeometryListener
{
public:
	typedef IndexMeshRayIntersector<Int64Grid, MarkedLinearSearchImpl<Int64Grid> > RayIntersector;
	
	struct RayIntersectionData{
		RayIntersectionData()
		{}
	
		RayIntersectionData(const RayIntersectionData& src)
		: t(src.t), ijk(src.ijk), xyz(src.xyz), face(src.face)
		{
			visited.reserve(src.visited.size());
			for(int i=0; i < src.visited.size(); ++i) {
				visited.push_back(src.visited[i]);
			}
			
			visitedFaces.reserve(src.visitedFaces.size());
			for(int i=0; i < src.visitedFaces.size(); ++i) {
				visitedFaces.push_back(src.visitedFaces[i]);
			}
			
			points.reserve(src.points.size());
			for(int i=0; i < src.points.size(); ++i) {
				points.push_back(src.points[i]);
			}
		}
	
		double t;
		openvdb::Coord ijk;
		Point xyz;
		ofxHEMeshFace face;
		vector<ofxHEMeshVertex> visited;
		vector<ofxHEMeshFace> visitedFaces;
		vector<ofxHEMesh::Point> points;
	};

	ofxHEMeshAdaptiveGrid(Scalar detail);
	virtual ~ofxHEMeshAdaptiveGrid() {}
	
	bool loadOBJModel(string modelName);
	bool castRay(const Point& eye, const Direction& dir, RayIntersectionData& data);
	
	void verticesCleared(ofxHEMeshVertex v);
	void vertexAdded(ofxHEMeshVertex v);
	void vertexWillBeMovedTo(ofxHEMeshVertex v, Point p);
	void vertexWillBeRemoved(ofxHEMeshVertex v);
	
	ofxHEMeshVertex vertexAt(Coord c) const;
	const Int64Grid& getGrid() const { return grid; }
	
	Coord pointToCoord(Point p);
	
protected:
	bool testRayOnFace(Ray<>& ray, const Point& p, ofxHEMeshFace f, RayIntersectionData& data);
	bool testRayOnVertexHood(Ray<>& ray, ofxHEMeshVertex v, RayIntersectionData& data, set<ofxHEMeshVertex>& visitedVertices);

	void addVertexToGrid(ofxHEMeshVertex v, Coord c);
	void removeVertexFromGrid(ofxHEMeshVertex v, Coord c);
	Coord vertexToCoord(ofxHEMeshVertex v);
	

	Int64Grid grid;
};