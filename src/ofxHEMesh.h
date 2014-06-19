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
	
	ofxHEMesh& operator=(const ofxHEMesh& src);
	

	/////////////////////////////////////////////////////////
	// Geometric quantities
	typedef float Scalar;
	typedef ofVec3f Point;
	typedef ofVec3f Direction;
	/////////////////////////////////////////////////////////

	class GeometryListener{
	public:
		typedef void (GeometryListener::*MemberFn)(ofxHEMeshVertex v);
		typedef void (GeometryListener::*MemberFnPt)(ofxHEMeshVertex v, Point p);
	
		virtual void verticesCleared(ofxHEMeshVertex v) = 0;
		virtual void vertexAdded(ofxHEMeshVertex v) = 0;
		virtual void vertexWillBeMovedTo(ofxHEMeshVertex v, Point p) = 0;
		virtual void vertexWillBeRemoved(ofxHEMeshVertex v) = 0;
	};


	/////////////////////////////////////////////////////////
	// Mesh-level modifications
	void remeshLoop();
	void subdivideLoop();
	void subdivideCatmullClark();
	void subdivideDooSabin();
	void subdivideModifiedCornerCut(Scalar tension);
	void facePeel(Scalar thickness);
	void dual();
	void triangulate();
	void centroidTriangulation();
	void reverseFaces();
	void translate(Direction dir);
	/////////////////////////////////////////////////////////
	
	/////////////////////////////////////////////////////////
	// Component-level modifications
	void connectFacesSimple(ofxHEMeshHalfedge h1, ofxHEMeshHalfedge h2);
	void connectHalfedgesCofacial(ofxHEMeshHalfedge h1, ofxHEMeshHalfedge h2);
	ofxHEMeshVertex splitHalfedgeQuadraticFit(ofxHEMeshHalfedge h);
	ofxHEMeshVertex splitHalfedge(ofxHEMeshHalfedge h, Scalar t=0.5);
	ofxHEMeshVertex splitHalfedge(ofxHEMeshHalfedge h, Point pt);
	
	ofxHEMeshVertex collapseHalfedgeQuadraticFit(ofxHEMeshHalfedge h);
	ofxHEMeshVertex collapseHalfedge(ofxHEMeshHalfedge h, Scalar t=0.5);
	ofxHEMeshVertex collapseHalfedge(ofxHEMeshHalfedge h, Point pt);
	/////////////////////////////////////////////////////////
	
	/////////////////////////////////////////////////////////
	// Queries
	ofxHEMeshHalfedge nearestVertexInFaceToPoint(const Point& pt, ofxHEMeshFace f) const;
	/////////////////////////////////////////////////////////
	
	
	/////////////////////////////////////////////////////////
	// Add combinatorial elements
	bool loadOBJModel(string modelName);
	void addMesh(const ofMesh& mesh);
	void addMesh(const ofxHEMesh& hemesh);
	ofxHEMeshVertex addVertex(const Point& p);
	ofxHEMeshHalfedge addEdge();
	void addFaces(const vector<ExplicitFace>& faces);
	ofxHEMeshFace addFace(const ExplicitFace& vertices);
	
	// Remove combinatorial elements
	void removeVertex(ofxHEMeshVertex v);
	void removeAllVertices();
	bool removeHalfedge(ofxHEMeshHalfedge h);
	void eraseHalfedge(ofxHEMeshHalfedge h);
	void removeFace(ofxHEMeshFace f);
	bool removeFaceIfDegenerate(ofxHEMeshFace f);
	
	
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
	ofxHEMeshPolygonSplitter splitPolygon(const ofxHEMeshFace& f) const;
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
	void linkHalfedges(ofxHEMeshHalfedge prev, ofxHEMeshHalfedge next);
	void swapHalfedgeAdjacency(ofxHEMeshHalfedge src, ofxHEMeshHalfedge dst);
	
	// Combinatorial Properties
	int faceSize(ofxHEMeshFace f) const;
	bool faceIsDegenerate(ofxHEMeshFace f) const;
	bool halfedgeEndPointsShareOneRing(ofxHEMeshHalfedge h) const;
	int vertexValence(ofxHEMeshVertex v) const;
	void vertexOneRing(ofxHEMeshVertex v, set<ofxHEMeshVertex>& oneRing) const;
	bool verticesShareOneRing(ofxHEMeshVertex v1, ofxHEMeshVertex v2) const;
	bool halfedgeIsInFace(ofxHEMeshFace f, ofxHEMeshHalfedge h) const;
	bool halfedgeLinksToVertex(ofxHEMeshVertex v, ofxHEMeshHalfedge h) const;
	
	// Geometric modification
	void vertexMove(ofxHEMeshVertex v, const Direction& dir);
	void vertexMoveTo(ofxHEMeshVertex v, const Point& p);
	
	// Geometric properties
	Point centroid() const;
	Scalar meanEdgeLength() const;
	Scalar edgeLengthVariance(Scalar expected) const;
	
	Point vertexPoint(ofxHEMeshVertex v) const;
	Direction angleWeightedVertexNormal(ofxHEMeshVertex v) const;
	Scalar vertexArea(ofxHEMeshVertex v) const;
	void vertexQuadric(ofxHEMeshVertex v, ofMatrix4x4& Q) const;
	
	void facePoints(ofxHEMeshFace f, vector<Point>& points) const;
	Scalar faceArea(ofxHEMeshFace f) const;
	Point faceCentroid(ofxHEMeshFace f) const;
	Direction faceNormal(ofxHEMeshFace f) const;
	void faceQuadric(ofxHEMeshFace f, ofMatrix4x4& Q) const;
	bool withinFace(ofxHEMeshFace f, const Point& pt) const;
	
	Point halfedgeLerp(ofxHEMeshHalfedge h, Scalar t) const;
	Point halfedgeMidpoint(ofxHEMeshHalfedge h) const;
	Scalar halfedgeCotan(ofxHEMeshHalfedge h) const;
	Scalar halfedgeLengthSquared(ofxHEMeshHalfedge h) const;
	Scalar halfedgeLength(ofxHEMeshHalfedge h) const;
	Direction halfedgeDirection(ofxHEMeshHalfedge h) const;
	Scalar angleAtVertex(ofxHEMeshHalfedge h) const;
	Scalar halfedgeAngle(ofxHEMeshHalfedge h) const;
	Direction halfedgeRotated(ofxHEMeshHalfedge h) const;
	Point halfedgeQuadraticFit(ofxHEMeshHalfedge h) const;
	
	Direction triangleNormal(const ofxHEMeshTriangle& tri) const;
	bool withinTriangle(const ofxHEMeshTriangle& tri, const Point& pt) const;
	
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
	
	void clearVertices();
	void clearHalfedges();
	void clearFaces();
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
	void printVertexOneHoodFaces(ofxHEMeshVertex v) const;
	void print() const;
	bool verifyConnectivity() const;
	/////////////////////////////////////////////////////////

	void addGeometryListener(GeometryListener *listener) {
		geometryListeners.push_back(listener);
	}
	
	void notifyGeometryListeners(ofxHEMeshVertex v, GeometryListener::MemberFn f) {
		for(int i=0; i < geometryListeners.size(); ++i) {
			GeometryListener& listener = *geometryListeners[i];
			((listener).*(f))(v);
		}
	}
	
	void notifyGeometryListeners(ofxHEMeshVertex v, Point p, GeometryListener::MemberFnPt f) {
		for(int i=0; i < geometryListeners.size(); ++i) {
			GeometryListener& listener = *geometryListeners[i];
			((listener).*(f))(v, p);
		}
	}


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
	
	vector<GeometryListener *> geometryListeners;
};

void printExplicitFace(const ofxHEMesh::ExplicitFace& face);
void printExplicitFaces(const vector<ofxHEMesh::ExplicitFace>& faces);
