#pragma once
#include "HemeshPropertySet.h"
#include "HemeshAdjacency.h"
#include "HemeshIterators.h"
#include "ofMain.h"
#include <vector>
#include <map>
#include <set>
#include <string>
#include <functional>
#include <stdexcept>
#include <queue>
#include <sstream>

using std::vector;
using std::map;
using std::set;
using std::string;


class Hemesh{
public:
	friend class FaceIterator;
	friend class EdgeIterator;
	friend class VertexIterator;
	typedef std::pair<Vertex, Vertex> ExplicitEdge;
	typedef vector<Vertex> ExplicitFace;


	Hemesh();
	~Hemesh() {}
	

	/////////////////////////////////////////////////////////
	// Geometric quantities
	typedef float Scalar;
	typedef ofVec3f Point;
	typedef ofVec3f Direction;
	/////////////////////////////////////////////////////////
	
	/////////////////////////////////////////////////////////
	// Mesh-level modifications
	void subdivideLoop();
	void subdivideCatmullClark();
	/////////////////////////////////////////////////////////
	
	
	/////////////////////////////////////////////////////////
	// Add combinatorial elements
	void addMesh(const ofMesh& mesh);
	Vertex addVertex(const Point& p);
	Halfedge addEdge();
	void addFaces(const vector<ExplicitFace>& faces);
	Face addFace(const vector<Vertex>& vertices);
	
	// Remove combinatorial elements
	void removeVertex(Vertex v);
	bool removeHalfedge(Halfedge h);
	void removeFace(Face f);
	
	// Number of combinatorial elements (some could be inactive)
	int getNumVertices() const;
	int getNumEdges() const;
	int getNumHalfedges() const;
	int getNumFaces() const;
	
	// Iterate combinatorial elements
	FaceIterator facesBegin() const;
	FaceIterator facesEnd() const;
	EdgeIterator edgesBegin() const;
	EdgeIterator edgesEnd() const;
	VertexIterator verticesBegin() const;
	VertexIterator verticesEnd() const;
	FaceCirculator faceCirculate(const Face& f) const;
	VertexCirculator vertexCirculate(const Vertex& v) const;
	
	// Set + get connectivity
	Halfedge vertexHalfedge(Vertex v) const;
	void setVertexHalfedge(Vertex v, Halfedge h);
	Halfedge faceHalfedge(Face f) const;
	void setFaceHalfedge(Face f, Halfedge h);
	
	Halfedge halfedgeOpposite(Halfedge h) const;
	Vertex halfedgeVertex(Halfedge h) const;
	void setHalfedgeVertex(Halfedge h, Vertex v);
	Face halfedgeFace(Halfedge h) const;
	void setHalfedgeFace(Halfedge h, Face f);
	Vertex halfedgeSource(Halfedge h) const;
	Vertex halfedgeSink(Halfedge h) const;
	Halfedge halfedgeNext(Halfedge h) const;
	void setHalfedgeNext(Halfedge h, Halfedge next);
	Halfedge halfedgePrev(Halfedge h) const;
	void setHalfedgePrev(Halfedge h, Halfedge prev);
	Halfedge halfedgeSourceCW(Halfedge h) const;
	Halfedge halfedgeSourceCCW(Halfedge h) const;
	Halfedge halfedgeSinkCW(Halfedge h) const;
	Halfedge halfedgeSinkCCW(Halfedge h) const;
	Halfedge findHalfedge(Vertex v1, Vertex v2) const;
	bool halfedgeIsOnBoundary(Halfedge h) const;
	
	// Combinatorial Properties
	int vertexValence(Vertex v) const;
	
	// Geometric modification
	void vertexMoveTo(Vertex v, const Point& p);
	
	// Geometric properties
	Point centroid() const;
	
	Point vertexPoint(Vertex v) const;
	Direction angleWeightedVertexNormal(Vertex v) const;
	
	Scalar faceArea(Face f) const;
	Point faceCentroid(Face f) const;
	Direction faceNormal(Face f) const;
	
	Point halfedgeLerp(Halfedge h, Scalar t) const;
	Point halfedgeMidpoint(Halfedge h) const;
	Scalar halfedgeCotan(Halfedge h) const;
	Scalar halfedgeLengthSquared(Halfedge h) const;
	Direction halfedgeDirection(Halfedge h) const;
	Scalar angleAtVertex(Halfedge h) const;
	Scalar halfedgeAngle(Halfedge h) const;
	
	// Combinatorial element properties
	template<typename T>
	Property<T> * addVertexProperty(const string &name, T def=T()) {
		return vertexProperties.add(name, def);
	}
	
	template<typename T>
	Property<T> * addHalfedgeProperty(const string &name, T def=T()) {
		return halfedgeProperties.add(name, def);
	}
	
	template<typename T>
	Property<T> * addEdgeProperty(const string &name, T def=T()) {
		return edgeProperties.add(name, def);
	}
	
	template<typename T>
	Property<T> * addFaceProperty(const string &name, T def=T()) {
		return faceProperties.add(name, def);
	}
	/////////////////////////////////////////////////////////
	

	/////////////////////////////////////////////////////////
	// Geometric elements
	const Property<Point>& getPoints() const { return *points; }
	/////////////////////////////////////////////////////////
	
	/////////////////////////////////////////////////////////
	// Debugging
	string halfedgeString(Halfedge h) const;
	void printFace(Face f) const;
	void printVertexOneHood(Vertex v) const;
	/////////////////////////////////////////////////////////

protected:

	HemeshPropertySet vertexProperties;
	HemeshPropertySet halfedgeProperties;
	HemeshPropertySet edgeProperties;
	HemeshPropertySet faceProperties;
	
	Property<VertexAdjacency> *vertexAdjacency;
	Property<HalfedgeAdjacency> *halfedgeAdjacency;
	Property<FaceAdjacency> *faceAdjacency;
	
	Property<Point>* points;
};
