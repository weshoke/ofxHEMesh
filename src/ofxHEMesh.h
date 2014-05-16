#pragma once
#include "ofxHEMeshPropertySet.h"
#include "ofxHEMeshAdjacency.h"
#include "ofxHEMeshIterators.h"
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


class ofxHEMesh{
public:
	friend class ofxHEMeshFaceIterator;
	friend class ofxHEMeshEdgeIterator;
	friend class ofxHEMeshVertexIterator;
	typedef std::pair<ofxHEMeshVertex, ofxHEMeshVertex> ExplicitEdge;
	typedef vector<ofxHEMeshVertex> ExplicitFace;


	ofxHEMesh();
	~ofxHEMesh() {}
	

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
	ofxHEMeshVertex addVertex(const Point& p);
	ofxHEMeshHalfedge addEdge();
	void addFaces(const vector<ExplicitFace>& faces);
	ofxHEMeshFace addFace(const vector<ofxHEMeshVertex>& vertices);
	
	// Remove combinatorial elements
	void removeVertex(ofxHEMeshVertex v);
	bool removeHalfedge(ofxHEMeshHalfedge h);
	void removeFace(ofxHEMeshFace f);
	
	// Number of combinatorial elements (some could be inactive)
	int getNumVertices() const;
	int getNumEdges() const;
	int getNumHalfedges() const;
	int getNumFaces() const;
	
	// Iterate combinatorial elements
	ofxHEMeshFaceIterator facesBegin() const;
	ofxHEMeshFaceIterator facesEnd() const;
	ofxHEMeshEdgeIterator edgesBegin() const;
	ofxHEMeshEdgeIterator edgesEnd() const;
	ofxHEMeshVertexIterator verticesBegin() const;
	ofxHEMeshVertexIterator verticesEnd() const;
	ofxHEMeshFaceCirculator faceCirculate(const ofxHEMeshFace& f) const;
	ofxHEMeshVertexCirculator vertexCirculate(const ofxHEMeshVertex& v) const;
	
	// Set + get connectivity
	ofxHEMeshHalfedge vertexHalfedge(ofxHEMeshVertex v) const;
	void setVertexHalfedge(ofxHEMeshVertex v, ofxHEMeshHalfedge h);
	ofxHEMeshHalfedge faceHalfedge(ofxHEMeshFace f) const;
	void setFaceHalfedge(ofxHEMeshFace f, ofxHEMeshHalfedge h);
	
	ofxHEMeshHalfedge halfedgeOpposite(ofxHEMeshHalfedge h) const;
	ofxHEMeshVertex halfedgeVertex(ofxHEMeshHalfedge h) const;
	void setHalfedgeVertex(ofxHEMeshHalfedge h, ofxHEMeshVertex v);
	ofxHEMeshFace halfedgeFace(ofxHEMeshHalfedge h) const;
	void setHalfedgeFace(ofxHEMeshHalfedge h, ofxHEMeshFace f);
	ofxHEMeshVertex halfedgeSource(ofxHEMeshHalfedge h) const;
	ofxHEMeshVertex halfedgeSink(ofxHEMeshHalfedge h) const;
	ofxHEMeshHalfedge halfedgeNext(ofxHEMeshHalfedge h) const;
	void setHalfedgeNext(ofxHEMeshHalfedge h, ofxHEMeshHalfedge next);
	ofxHEMeshHalfedge halfedgePrev(ofxHEMeshHalfedge h) const;
	void setHalfedgePrev(ofxHEMeshHalfedge h, ofxHEMeshHalfedge prev);
	ofxHEMeshHalfedge halfedgeSourceCW(ofxHEMeshHalfedge h) const;
	ofxHEMeshHalfedge halfedgeSourceCCW(ofxHEMeshHalfedge h) const;
	ofxHEMeshHalfedge halfedgeSinkCW(ofxHEMeshHalfedge h) const;
	ofxHEMeshHalfedge halfedgeSinkCCW(ofxHEMeshHalfedge h) const;
	ofxHEMeshHalfedge findHalfedge(ofxHEMeshVertex v1, ofxHEMeshVertex v2) const;
	bool halfedgeIsOnBoundary(ofxHEMeshHalfedge h) const;
	
	// Combinatorial Properties
	int vertexValence(ofxHEMeshVertex v) const;
	
	// Geometric modification
	void vertexMoveTo(ofxHEMeshVertex v, const Point& p);
	
	// Geometric properties
	Point centroid() const;
	
	Point vertexPoint(ofxHEMeshVertex v) const;
	Direction angleWeightedVertexNormal(ofxHEMeshVertex v) const;
	Scalar vertexArea(ofxHEMeshVertex v) const;
	
	Scalar faceArea(ofxHEMeshFace f) const;
	Point faceCentroid(ofxHEMeshFace f) const;
	Direction faceNormal(ofxHEMeshFace f) const;
	
	Point halfedgeLerp(ofxHEMeshHalfedge h, Scalar t) const;
	Point halfedgeMidpoint(ofxHEMeshHalfedge h) const;
	Scalar halfedgeCotan(ofxHEMeshHalfedge h) const;
	Scalar halfedgeLengthSquared(ofxHEMeshHalfedge h) const;
	Direction halfedgeDirection(ofxHEMeshHalfedge h) const;
	Scalar angleAtVertex(ofxHEMeshHalfedge h) const;
	Scalar halfedgeAngle(ofxHEMeshHalfedge h) const;
	
	// Combinatorial element properties
	template<typename T>
	ofxHEMeshProperty<T> * addVertexProperty(const string &name, T def=T()) {
		return vertexProperties.add(name, def);
	}
	
	template<typename T>
	ofxHEMeshProperty<T> * addHalfedgeProperty(const string &name, T def=T()) {
		return halfedgeProperties.add(name, def);
	}
	
	template<typename T>
	ofxHEMeshProperty<T> * addEdgeProperty(const string &name, T def=T()) {
		return edgeProperties.add(name, def);
	}
	
	template<typename T>
	ofxHEMeshProperty<T> * addFaceProperty(const string &name, T def=T()) {
		return faceProperties.add(name, def);
	}
	/////////////////////////////////////////////////////////
	

	/////////////////////////////////////////////////////////
	// Geometric elements
	const ofxHEMeshProperty<Point>& getPoints() const { return *points; }
	/////////////////////////////////////////////////////////
	
	/////////////////////////////////////////////////////////
	// Flags
	bool getTopologyDirty() const { return topologyDirty; }
	void setTopologyDirty(bool v) { topologyDirty = v; }
	bool getGeometryDirty() const { return geometryDirty; }
	void setGeometryDirty(bool v) { geometryDirty = v; }
	/////////////////////////////////////////////////////////
	
	/////////////////////////////////////////////////////////
	// Debugging
	string halfedgeString(ofxHEMeshHalfedge h) const;
	void printFace(ofxHEMeshFace f) const;
	void printVertexOneHood(ofxHEMeshVertex v) const;
	/////////////////////////////////////////////////////////

protected:

	ofxHEMeshPropertySet vertexProperties;
	ofxHEMeshPropertySet halfedgeProperties;
	ofxHEMeshPropertySet edgeProperties;
	ofxHEMeshPropertySet faceProperties;
	
	ofxHEMeshProperty<ofxHEMeshVertexAdjacency> *vertexAdjacency;
	ofxHEMeshProperty<ofxHEMeshHalfedgeAdjacency> *halfedgeAdjacency;
	ofxHEMeshProperty<ofxHEMeshFaceAdjacency> *faceAdjacency;
	
	ofxHEMeshProperty<Point>* points;
	bool topologyDirty;
	bool geometryDirty;
};
