#include "ofxHEMesh.h"
#include <sstream>


#define WRAP_NEXT(idx, n) (((idx)+1)%(n))
#define WRAP_PREV(idx, n) (((idx)+(n)-1)%(n))

using std::ifstream;
using std::ofstream;
using std::queue;
using std::set;

ofxHEMesh::ofxHEMesh()
:	points(0)
{
	vertexAdjacency = addVertexProperty<ofxHEMeshVertexAdjacency>("vertex-adjacency", ofxHEMeshVertexAdjacency());
	halfedgeAdjacency = addHalfedgeProperty<ofxHEMeshHalfedgeAdjacency>("halfedge-adjacency", ofxHEMeshHalfedgeAdjacency());
	faceAdjacency = addFaceProperty<ofxHEMeshFaceAdjacency>("face-adjacency", ofxHEMeshFaceAdjacency());
	points = addVertexProperty<Point>("points", Point());
}

// Assumes the mesh is a triangulation
void ofxHEMesh::subdivideLoop() {

	// Get the new location of existing vertices in the subd mesh
	ofxHEMeshVertexIterator vit = verticesBegin();
	ofxHEMeshVertexIterator vite = verticesEnd();
	map<ofxHEMeshVertex, Point> newPositions;
	for(; vit != vite; ++vit) {
		ofxHEMeshVertexCirculator vc = vertexCirculate(*vit);
		ofxHEMeshVertexCirculator vce = vc;
		int valence = vertexValence(*vit);
		Scalar beta;
		if(valence == 3) {
			beta = 3./16.*valence;
		}
		else {
			beta = 3./(8.*valence);
		}
		
		Point nPt = vertexPoint(*vit)*(1.-valence*beta);
		do {
			nPt += vertexPoint(halfedgeSource(*vc))*beta;
			++vc;
		} while(vc != vce);
		
		newPositions.insert(std::pair<ofxHEMeshVertex, Point>(*vit, nPt));
	}

	// Get the location of new vertices dividing edges in the subd mesh
	map<ofxHEMeshHalfedge, ofxHEMeshVertex> edgeVertices;
	ofxHEMeshEdgeIterator eit = edgesBegin();
	ofxHEMeshEdgeIterator eite = edgesEnd();
	for(; eit != eite; ++eit) {
		ofxHEMeshHalfedge h = *eit;
		ofxHEMeshHalfedge hn = halfedgeNext(h);
		ofxHEMeshHalfedge hon = halfedgeNext(halfedgeOpposite(h));
		Point pt =
			(vertexPoint(halfedgeSource(h)) + vertexPoint(halfedgeSink(h)))*0.375 +
			(vertexPoint(halfedgeSink(hn)) + vertexPoint(halfedgeSink(hon)))*0.125;
		ofxHEMeshVertex v = addVertex(pt);
		edgeVertices.insert(std::pair<ofxHEMeshHalfedge, ofxHEMeshVertex>(h, v));
		edgeVertices.insert(std::pair<ofxHEMeshHalfedge, ofxHEMeshVertex>(halfedgeOpposite(h), v));
	}
	
	// Move the existing vertices to their subd mesh positions
	map<ofxHEMeshVertex, Point>::const_iterator npt_it = newPositions.begin();
	map<ofxHEMeshVertex, Point>::const_iterator npt_ite = newPositions.end();
	for(; npt_it != npt_ite; ++npt_it) {
		vertexMoveTo(npt_it->first, npt_it->second);
	}
	
	// Calculate the subd mesh faces
	vector<ExplicitFace> faces;
	faces.reserve(faceAdjacency->size()*4);
	ofxHEMeshFaceIterator fit = facesBegin();
	ofxHEMeshFaceIterator fite = facesEnd();
	for(; fit != fite; ++fit) {
		ofxHEMeshFaceCirculator fc = faceCirculate(*fit);
		ofxHEMeshFaceCirculator fce = fc;
		
		ExplicitFace innerFace(3);
		int i=0;
		do {
			ExplicitFace cornerFace(3);
			ofxHEMeshHalfedge h = *fc;
			cornerFace[0] = halfedgeSink(h);
			cornerFace[1] = edgeVertices[halfedgeNext(h)];
			cornerFace[2] = edgeVertices[h];
			faces.push_back(cornerFace);
			innerFace[i] = edgeVertices[h];
			++i;
			++fc;
		} while(fc != fce);
		faces.push_back(innerFace);
	}
	
	faceAdjacency->clear();
	addFaces(faces);
}

// Assumes no boundaries
void ofxHEMesh::subdivideCatmullClark() {
	map<ofxHEMeshFace, Point> facePoints;
	ofxHEMeshFaceIterator fit = facesBegin();
	ofxHEMeshFaceIterator fite = facesEnd();
	for(; fit != fite; ++fit) {
		facePoints.insert(std::pair<ofxHEMeshFace, Point>(*fit, faceCentroid(*fit)));
	}
	
	map<ofxHEMeshHalfedge, Point> edgePoints;
	ofxHEMeshEdgeIterator eit = edgesBegin();
	ofxHEMeshEdgeIterator eite = edgesEnd();
	for(; eit != eite; ++eit) {
		ofxHEMeshHalfedge h = *eit;
		ofxHEMeshHalfedge ho = halfedgeOpposite(h);
		Point pt = 0.25*(
			vertexPoint(halfedgeSink(h)) + vertexPoint(halfedgeSink(ho)) +
			facePoints[halfedgeFace(h)] + facePoints[halfedgeFace(ho)]
		);
		edgePoints.insert(std::pair<ofxHEMeshHalfedge, Point>(h, pt));
		edgePoints.insert(std::pair<ofxHEMeshHalfedge, Point>(ho, pt));
	}
	
	ofxHEMeshVertexIterator vit = verticesBegin();
	ofxHEMeshVertexIterator vite = verticesEnd();
	for(; vit != vite; ++vit) {
		Point Q(0, 0, 0);
		Point R(0, 0, 0);
		Scalar valence = 0;
		ofxHEMeshVertexCirculator vc = vertexCirculate(*vit);
		ofxHEMeshVertexCirculator vce = vc;
		do {
			ofxHEMeshHalfedge h = *vc;
			Q += facePoints[halfedgeFace(h)];
			R += edgePoints[h];
			++valence;
		} while(vc != vce);
	}
}

void ofxHEMesh::addMesh(const ofMesh& mesh) {
	ofxHEMeshVertex vstart(vertexAdjacency->size());
	
	const vector<ofVec3f>& vertices = mesh.getVertices();
	vector<ofVec3f>::const_iterator vit = vertices.begin();
	vector<ofVec3f>::const_iterator vite = vertices.end();

	for(; vit != vite; ++vit) {
		addVertex(*vit);
	}
	
	
	// Assume triangles for now
	// TOOD: use mesh.getMode()
	vector< vector<ofxHEMeshVertex> > faces;
	const vector<ofIndexType>& indices = mesh.getIndices();
	vector<ofIndexType>::const_iterator iit = indices.begin();
	vector<ofIndexType>::const_iterator iite = indices.end();
	
	faces.reserve(indices.size()/3);
	while(iit != iite) {
		vector<ofxHEMeshVertex> face(3);
		for(int i=0; i < 3; ++i) {
			ofxHEMeshVertex v = ofxHEMeshVertex((*iit)+vstart.idx);
			face[i] = v;
			++iit;
		}
		faces.push_back(face);
	}
	
	addFaces(faces);
}

ofxHEMeshVertex ofxHEMesh::addVertex(const Point& p) {
	int idx = vertexProperties.size();
	vertexProperties.extend();
	points->set(idx, p);
	ofxHEMeshVertex v(idx);
	topologyDirty = true;
	return v;
}

ofxHEMeshHalfedge ofxHEMesh::addEdge() {
	halfedgeProperties.extend();
	halfedgeProperties.extend();
	return ofxHEMeshHalfedge(halfedgeProperties.size()-2);
}

static bool orderVertices(ofxHEMeshVertex& v1, ofxHEMeshVertex& v2) {
	bool swap = v1 > v2;
	if(swap) {
		ofxHEMeshVertex v = v2;
		v2 = v1;
		v1 = v;
	}
	return swap;
}

void ofxHEMesh::addFaces(const vector<ExplicitFace>& faces) {
	map<ExplicitEdge, ofxHEMeshHalfedge> explicitEdgeMap;
	int i, j;
	
	// Create any edges that don't yet exist
	set<ofxHEMeshHalfedge> allHalfedges;
	for(i=0; i < faces.size(); ++i) {
		const vector<ofxHEMeshVertex>& face = faces[i];
		int nv = int(face.size());
		for(j=0; j < nv; ++j) {
			ofxHEMeshVertex v1 = face[j];
			ofxHEMeshVertex v2 = face[WRAP_NEXT(j, nv)];
			orderVertices(v1, v2);
			if(explicitEdgeMap.find(ExplicitEdge(v1, v2)) == explicitEdgeMap.end()) {
				ofxHEMeshHalfedge h1 = addEdge();
				ofxHEMeshHalfedge h1o = halfedgeOpposite(h1);
				setHalfedgeVertex(h1, v2);
				setHalfedgeVertex(h1o, v1);
				setVertexHalfedge(v2, h1);
				setVertexHalfedge(v1, h1o);
				explicitEdgeMap.insert(std::pair<ExplicitEdge, ofxHEMeshHalfedge>(ExplicitEdge(v1, v2), h1));
				allHalfedges.insert(h1);
				allHalfedges.insert(h1o);
			}
		}
	}
	
	// Link Halfedges around faces
	for(i=0; i < faces.size(); ++i) {
		ofxHEMeshFace f(faceProperties.size());
		faceAdjacency->extend();
	
		const vector<ofxHEMeshVertex>& face = faces[i];
		int nv = int(face.size());
		vector<ofxHEMeshHalfedge> halfedges;
		for(j=0; j < nv; ++j) {
			ofxHEMeshVertex v1 = face[j];
			ofxHEMeshVertex v2 = face[WRAP_NEXT(j, nv)];
			bool didSwap = orderVertices(v1, v2);
			auto iter = explicitEdgeMap.find(ExplicitEdge(v1, v2));
			if(didSwap) halfedges.push_back(halfedgeOpposite(iter->second));
			else halfedges.push_back(iter->second);
		}
		
		setFaceHalfedge(f, halfedges[0]);
		for(j=0; j < nv; ++j) {
			ofxHEMeshHalfedge h1 = halfedges[j];
			ofxHEMeshHalfedge h2 = halfedges[WRAP_NEXT(j, nv)];
			allHalfedges.erase(allHalfedges.find(h1));
			setHalfedgeNext(h1, h2);
			setHalfedgePrev(h2, h1);
			setHalfedgeFace(h1, f);
		}
	}
	
	set<ofxHEMeshHalfedge>::const_iterator ahit = allHalfedges.begin();
	set<ofxHEMeshHalfedge>::const_iterator ahite = allHalfedges.end();
	map<ofxHEMeshVertex, ofxHEMeshHalfedge> sinks;
	map<ofxHEMeshVertex, ofxHEMeshHalfedge> sources;
	for(; ahit != ahite; ++ahit) {
		ofxHEMeshHalfedge h = *ahit;
		sinks.insert(std::pair<ofxHEMeshVertex, ofxHEMeshHalfedge>(halfedgeSink(h), h));
		sources.insert(std::pair<ofxHEMeshVertex, ofxHEMeshHalfedge>(halfedgeSource(h), h));
	}
	
	map<ofxHEMeshVertex, ofxHEMeshHalfedge>::const_iterator sink_it = sinks.begin();
	map<ofxHEMeshVertex, ofxHEMeshHalfedge>::const_iterator sink_ite = sinks.end();
	map<ofxHEMeshVertex, ofxHEMeshHalfedge>::const_iterator source_ite = sources.end();
	for(; sink_it != sink_ite; ++sink_it) {
		map<ofxHEMeshVertex, ofxHEMeshHalfedge>::const_iterator source_it = sources.find(sink_it->first);
		if(source_it == source_ite) {
			std::cout << halfedgeString(sink_it->second) << " doesn't have a source\n";
			continue;
		}
		setHalfedgeNext(sink_it->second, source_it->second);
		setHalfedgePrev(source_it->second, sink_it->second);
	}
	topologyDirty = true;
}


ofxHEMeshFace ofxHEMesh::addFace(const vector<ofxHEMeshVertex>& vertices) {
	vector<ofxHEMeshHalfedge> halfedges;
	vector<bool> exists;
	
	size_t nv = vertices.size();
	size_t i;
	for(i=0; i < nv; ++i) {
		ofxHEMeshVertex v1 = vertices[i];
		ofxHEMeshVertex v2 = vertices[WRAP_NEXT(i, nv)];
		
		ofxHEMeshHalfedge h = findHalfedge(v1, v2);
		bool hExists = h.isValid();

		if(hExists && !halfedgeIsOnBoundary(h)) {
			std::stringstream ss;
			ss << "attempt to add Face where Halfedge " << v1.idx << " " << v2.idx << " isn't on a boundary\n";
			ss << "Face: ";
			for(int j=0; j < nv; ++j) {
				ss << vertices[j].idx << "-";
			}
			throw std::invalid_argument(ss.str());
			return ofxHEMeshFace();
		}
		
		halfedges.push_back(h);
		exists.push_back(hExists);
	}
	
	ofxHEMeshFace f(faceProperties.size());
	faceAdjacency->extend();
	topologyDirty = true;

	// set the face of halfedges and create any new ones necessary
	vector<ofxHEMeshHalfedge> halfedgesPrev;
	vector<ofxHEMeshHalfedge> halfedgesNext;
	for(i=0; i < nv; ++i) {
		if(exists[i]) {
			setHalfedgeFace(halfedges[i], f);
		}
		else {
			ofxHEMeshHalfedge h1 = addEdge();
			ofxHEMeshHalfedge h2 = ofxHEMeshHalfedge(h1.idx+1);
			
			ofxHEMeshVertex v1 = vertices[i];
			ofxHEMeshVertex v2 = vertices[WRAP_NEXT(i, nv)];
			setHalfedgeVertex(h1, v2);
			setHalfedgeVertex(h2, v1);
			setHalfedgeFace(h1, f);
			setVertexHalfedge(v2, h1);
			halfedges[i] = h1;
		}
		
		// cache halfedge adjacency information
		halfedgesPrev.push_back(halfedgePrev(halfedges[i]));
		halfedgesNext.push_back(halfedgeNext(halfedges[i]));
	}
	
	// link halfedges together
	for(i=0; i < nv; ++i) {
		ofxHEMeshHalfedge h = halfedges[i];
		
		// inner loop
		auto pidx = WRAP_PREV(i, nv);
		auto nidx = WRAP_NEXT(i, nv);
		ofxHEMeshHalfedge p = halfedges[pidx];
		ofxHEMeshHalfedge n = halfedges[nidx];
		setHalfedgePrev(h, p);
		setHalfedgeNext(h, n);
		
		if(!exists[i]) {
			// outer loop
			ofxHEMeshHalfedge ho = halfedgeOpposite(h);
			if(exists[nidx]) {
				ofxHEMeshHalfedge np = halfedgesPrev[nidx];
				setHalfedgePrev(ho, np);
				setHalfedgeNext(np, ho);
			}
			else {
				setHalfedgePrev(ho, halfedgeOpposite(n));
			}
			
			if(exists[pidx]) {
				ofxHEMeshHalfedge nn = halfedgesNext[pidx];
				setHalfedgeNext(ho, nn);
				setHalfedgePrev(nn, ho);
			}
			else {
				setHalfedgeNext(ho, halfedgeOpposite(p));
			}
		}
	}
	
	setFaceHalfedge(f, halfedges[0]);
	return f;
}


void ofxHEMesh::removeVertex(ofxHEMeshVertex v) {
	if(!v.isValid()) throw std::invalid_argument("attempting to remove invalid vertex");
	
	ofxHEMeshHalfedge h = vertexHalfedge(v);
	if(h.isValid()) {
		do {
			ofxHEMeshHalfedge hh = halfedgeSinkCCW(h);
			if(removeHalfedge(h)) {
				break;
			}
			h = hh;
		}
		while(h.isValid());
	}
	setVertexHalfedge(v, ofxHEMeshHalfedge());
	topologyDirty = true;
}


bool ofxHEMesh::removeHalfedge(ofxHEMeshHalfedge h) {
	ofxHEMeshFace f = halfedgeFace(h);
	ofxHEMeshHalfedge ho = halfedgeOpposite(h);
	ofxHEMeshFace f2 = halfedgeFace(ho);

	ofxHEMeshVertex v = halfedgeVertex(h);
	ofxHEMeshVertex v2 = halfedgeVertex(ho);

	ofxHEMeshHalfedge h1p = halfedgePrev(h);
	ofxHEMeshHalfedge h1n = halfedgeSourceCW(h);
	ofxHEMeshHalfedge h2p = halfedgePrev(ho);
	ofxHEMeshHalfedge h2n = halfedgeSourceCW(ho);
	
	ofxHEMeshVertex vrem;
	if(h1p == ho) vrem = v2;
	else vrem = v;
	
	setHalfedgePrev(h1n, h1p);
	setHalfedgeNext(h1p, h1n);
	
	setHalfedgePrev(h2n, h2p);
	setHalfedgeNext(h2p, h2n);
	
	halfedgeAdjacency->set(h.idx, ofxHEMeshHalfedgeAdjacency());
	halfedgeAdjacency->set(ho.idx, ofxHEMeshHalfedgeAdjacency());
	topologyDirty = true;

	if(f.idx == f2.idx) {
		// belong to the same face, last edge linking vertex to mesh
		setVertexHalfedge(vrem, ofxHEMeshHalfedge());
		if(vrem != v2) {
			setVertexHalfedge(v2, h1p);
		}
		if(f.isValid()) {
			setFaceHalfedge(f, h1p);
		}
		return true;
	}
	else {
		setVertexHalfedge(v, h2p);
		setVertexHalfedge(v2, h1p);
		setFaceHalfedge(f, ofxHEMeshHalfedge());
		setFaceHalfedge(f2, h1p);
	
		// set all halfedges to the common face
		ofxHEMeshHalfedge hh = h1p;
		do {
			setHalfedgeFace(hh, f2);
			hh = halfedgeNext(hh);
		} while(hh != h1p);
		return false;
	}
}


void ofxHEMesh::removeFace(ofxHEMeshFace f) {
	ofxHEMeshHalfedge h = faceHalfedge(f);
	ofxHEMeshHalfedge hstart = h;
	do {
		setHalfedgeFace(h, ofxHEMeshFace());
		h = halfedgeNext(h);
	}
	while(h != hstart);
	setFaceHalfedge(f, ofxHEMeshHalfedge());
	topologyDirty = true;
}

int ofxHEMesh::getNumVertices() const {
	return (int)vertexAdjacency->size();
}

int ofxHEMesh::getNumEdges() const {
	return ((int)halfedgeAdjacency->size())/2;
}

int ofxHEMesh::getNumHalfedges() const {
	return (int)halfedgeAdjacency->size();
}

int ofxHEMesh::getNumFaces() const {
	return (int)faceAdjacency->size();
}

ofxHEMeshFaceIterator ofxHEMesh::facesBegin() const {
	ofxHEMeshFace f(0);
	if(faceAdjacency->size() > 0) {
		while(!faceHalfedge(f).isValid()) {
			++f.idx;
		}
	}
	return ofxHEMeshFaceIterator(this, f);
}

ofxHEMeshFaceIterator ofxHEMesh::facesEnd() const {
	return ofxHEMeshFaceIterator(this, ofxHEMeshFace(faceAdjacency->size()));
}

ofxHEMeshEdgeIterator ofxHEMesh::edgesBegin() const {
	ofxHEMeshHalfedge h(0);
	if(halfedgeAdjacency->size() > 0) {
		while(!halfedgeFace(h).isValid()) {
			++h.idx;
		}
	}
	return ofxHEMeshEdgeIterator(this, h);
}

ofxHEMeshEdgeIterator ofxHEMesh::edgesEnd() const {
	return ofxHEMeshEdgeIterator(this, ofxHEMeshHalfedge(halfedgeAdjacency->size()));
}

ofxHEMeshVertexIterator ofxHEMesh::verticesBegin() const {
	ofxHEMeshVertex v(0);
	if(vertexAdjacency->size() > 0) {
		while(!vertexHalfedge(v).isValid()) {
			++v.idx;
		}
	}
	return ofxHEMeshVertexIterator(this, v);
}

ofxHEMeshVertexIterator ofxHEMesh::verticesEnd() const {
	return ofxHEMeshVertexIterator(this, ofxHEMeshVertex(vertexAdjacency->size()));
}

ofxHEMeshFaceCirculator ofxHEMesh::faceCirculate(const ofxHEMeshFace& f) const {
	return ofxHEMeshFaceCirculator(this, faceHalfedge(f));
}

ofxHEMeshVertexCirculator ofxHEMesh::vertexCirculate(const ofxHEMeshVertex& v) const {
	return ofxHEMeshVertexCirculator(this, vertexHalfedge(v));
}

ofxHEMeshHalfedge ofxHEMesh::vertexHalfedge(ofxHEMeshVertex v) const {
	return vertexAdjacency->get(v.idx).he;
}


void ofxHEMesh::setVertexHalfedge(ofxHEMeshVertex v, ofxHEMeshHalfedge h) {
	vertexAdjacency->get(v.idx).he = h;
}


ofxHEMeshHalfedge ofxHEMesh::faceHalfedge(ofxHEMeshFace f) const {
	return faceAdjacency->get(f.idx).he;
}


void ofxHEMesh::setFaceHalfedge(ofxHEMeshFace f, ofxHEMeshHalfedge h) {
	faceAdjacency->get(f.idx).he = h;
}


ofxHEMeshHalfedge ofxHEMesh::halfedgeOpposite(ofxHEMeshHalfedge h) const {
	if((h.idx & 1) == 1) return ofxHEMeshHalfedge(h.idx-1);
	else return ofxHEMeshHalfedge(h.idx+1);
}


ofxHEMeshVertex ofxHEMesh::halfedgeVertex(ofxHEMeshHalfedge h) const {
	return halfedgeAdjacency->get(h.idx).v;
}


void ofxHEMesh::setHalfedgeVertex(ofxHEMeshHalfedge h, ofxHEMeshVertex v) {
	halfedgeAdjacency->get(h.idx).v = v;
}


ofxHEMeshFace ofxHEMesh::halfedgeFace(ofxHEMeshHalfedge h) const {
	return halfedgeAdjacency->get(h.idx).f;
}


void ofxHEMesh::setHalfedgeFace(ofxHEMeshHalfedge h, ofxHEMeshFace f) {
	halfedgeAdjacency->get(h.idx).f = f;
}


ofxHEMeshVertex ofxHEMesh::halfedgeSource(ofxHEMeshHalfedge h) const {
	return halfedgeVertex(halfedgeOpposite(h));
}


ofxHEMeshVertex ofxHEMesh::halfedgeSink(ofxHEMeshHalfedge h) const {
	return halfedgeVertex(h);
}


ofxHEMeshHalfedge ofxHEMesh::halfedgeNext(ofxHEMeshHalfedge h) const {
	return halfedgeAdjacency->get(h.idx).next;
}


void ofxHEMesh::setHalfedgeNext(ofxHEMeshHalfedge h, ofxHEMeshHalfedge next) {
	halfedgeAdjacency->get(h.idx).next = next;
}


ofxHEMeshHalfedge ofxHEMesh::halfedgePrev(ofxHEMeshHalfedge h) const {
	return halfedgeAdjacency->get(h.idx).prev;
}


void ofxHEMesh::setHalfedgePrev(ofxHEMeshHalfedge h, ofxHEMeshHalfedge prev) {
	halfedgeAdjacency->get(h.idx).prev = prev;
}


ofxHEMeshHalfedge ofxHEMesh::halfedgeSourceCW(ofxHEMeshHalfedge h) const {
	return halfedgeNext(halfedgeOpposite(h));
}


ofxHEMeshHalfedge ofxHEMesh::halfedgeSourceCCW(ofxHEMeshHalfedge h) const {
	return halfedgeOpposite(halfedgePrev(h));
}


ofxHEMeshHalfedge ofxHEMesh::halfedgeSinkCW(ofxHEMeshHalfedge h) const {
	return halfedgeOpposite(halfedgeNext(h));
}


ofxHEMeshHalfedge ofxHEMesh::halfedgeSinkCCW(ofxHEMeshHalfedge h) const {
	return halfedgePrev(halfedgeOpposite(h));
}



ofxHEMeshHalfedge ofxHEMesh::findHalfedge(ofxHEMeshVertex v1, ofxHEMeshVertex v2) const {
	ofxHEMeshHalfedge h1 = vertexHalfedge(v1);
	ofxHEMeshHalfedge h2 = vertexHalfedge(v2);
	if(h1.isValid() && h2.isValid()) {
		ofxHEMeshHalfedge hStart = h2;
		do {
			if(halfedgeSource(h2) == v1) {
				return h2;
			}
			h2 = halfedgeSinkCCW(h2);
		} while(h2 != hStart);
	}
	return ofxHEMeshHalfedge();
}

bool ofxHEMesh::halfedgeIsOnBoundary(ofxHEMeshHalfedge h) const {
	return !halfedgeFace(h).isValid();
}

int ofxHEMesh::vertexValence(ofxHEMeshVertex v) const {
	int n=0;
	ofxHEMeshVertexCirculator vc = vertexCirculate(v);
	ofxHEMeshVertexCirculator vce = vc;
	do {
		++n;
		++vc;
	} while(vc != vce);
	return n;
}

void ofxHEMesh::vertexMoveTo(ofxHEMeshVertex v, const Point& p) {
	points->set(v.idx, p);
	geometryDirty = true;
}

ofxHEMesh::Point ofxHEMesh::centroid() const {
	Point c(0, 0, 0);
	Scalar n = 0;
	ofxHEMeshVertexIterator vit = verticesBegin();
	ofxHEMeshVertexIterator vite = verticesEnd();
	for(; vit != vite; ++vit) {
		c += vertexPoint(*vit);
		++n;
	}
	c *= 1./n;
	return c;
}

ofxHEMesh::Point ofxHEMesh::vertexPoint(ofxHEMeshVertex v) const {
	return points->get(v.idx);
}

ofxHEMesh::Direction ofxHEMesh::angleWeightedVertexNormal(ofxHEMeshVertex v) const {
	ofxHEMeshVertexCirculator vc = vertexCirculate(v);
	ofxHEMeshVertexCirculator vce = vc;
	
	Direction n(0, 0, 0);
	do {
		ofxHEMeshHalfedge& h = *vc;
		
		Scalar theta = angleAtVertex(h);
		ofxHEMeshFace f = halfedgeFace(h);
		if(f.isValid()) {
			n += faceNormal(f)*theta;
		}
		++vc;
	} while(vc != vce);
	// TODO: check for NaNs
	n.normalize();
	return n;
}

ofxHEMesh::Scalar ofxHEMesh::vertexArea(ofxHEMeshVertex v) const {
	Scalar A = 0;
	ofxHEMeshVertexCirculator vc = vertexCirculate(v);
	ofxHEMeshVertexCirculator vce = vc;
	do {
		ofxHEMeshHalfedge h = *vc;
		ofxHEMeshFace f = halfedgeFace(h);
		A += faceArea(f);
		h = halfedgeSinkCCW(h);
	} while(vc != vce);
	// TODO: check for NaN
	return A*0.333333333333333;
}

ofxHEMesh::Scalar ofxHEMesh::faceArea(ofxHEMeshFace f) const {
	// TODO: general case vs. Triangle case
	ofxHEMeshHalfedge h1 = faceHalfedge(f);
	ofxHEMeshHalfedge h2 = halfedgeNext(h1);
	Direction v1 = halfedgeDirection(h1);
	Direction v2 = halfedgeDirection(h2);
	Direction n = v1.cross(v2);
	return 0.5*n.length();
}

ofxHEMesh::Point ofxHEMesh::faceCentroid(ofxHEMeshFace f) const {
	Point centroid(0, 0, 0);
	float n = 0;
	ofxHEMeshFaceCirculator fc = faceCirculate(f);
	ofxHEMeshFaceCirculator fce = fc;
	do {
		centroid += vertexPoint(halfedgeVertex(*fc));
		++n;
		++fc;
	} while(fc != fce);
	
	return centroid*(1/n);
}

ofxHEMesh::Direction ofxHEMesh::faceNormal(ofxHEMeshFace f) const {
	ofxHEMeshHalfedge h1 = faceHalfedge(f);
	ofxHEMeshHalfedge h2 = halfedgeNext(h1);
	Direction v1 = halfedgeDirection(h1);
	Direction v2 = halfedgeDirection(h2);
	Direction n = v1.cross(v2);
	n.normalize();
	return n;
}

ofxHEMesh::Point ofxHEMesh::halfedgeLerp(ofxHEMeshHalfedge h, Scalar t) const {
	Point p1 = vertexPoint(halfedgeSource(h));
	Point p2 = vertexPoint(halfedgeSink(h));
	return p1+(p2-p1)*t;
}

ofxHEMesh::Point ofxHEMesh::halfedgeMidpoint(ofxHEMeshHalfedge h) const {
	return halfedgeLerp(h, 0.5);
}

ofxHEMesh::Scalar ofxHEMesh::halfedgeCotan(ofxHEMeshHalfedge h) const {
	ofxHEMeshHalfedge h2 = halfedgeNext(h);
	ofxHEMeshHalfedge h3 = halfedgeNext(h2);
	ofxHEMeshVertex v1 = halfedgeVertex(h2);
	ofxHEMeshVertex v2 = halfedgeVertex(h3);
	ofxHEMeshVertex v3 = halfedgeVertex(h);
	Point p0 = vertexPoint(v1);
	Point p1 = vertexPoint(v2);
	Point p2 = vertexPoint(v3 );
	
	Direction u = p1-p0;
	Direction v = p2-p0;

	Scalar res = u.dot(v)/u.cross(v).length();
	// TODO: check for NaNs
	return res;
}

ofxHEMesh::Scalar ofxHEMesh::halfedgeLengthSquared(ofxHEMeshHalfedge h) const {
	return halfedgeDirection(h).lengthSquared();
}

ofxHEMesh::Direction ofxHEMesh::halfedgeDirection(ofxHEMeshHalfedge h) const {
	return vertexPoint(halfedgeSink(h)) - vertexPoint(halfedgeSource(h));
}

ofxHEMesh::Scalar ofxHEMesh::angleAtVertex(ofxHEMeshHalfedge h) const {
	Direction v1 = -halfedgeDirection(h);
	Direction v2 = halfedgeDirection(halfedgeNext(h));
	v1.normalize();
	v2.normalize();
	Scalar dot = MAX(MIN(1, v1.dot(v2)), -1);
	Scalar res = acos(dot);
	// TODO: check for NaNs
	return res;
}

ofxHEMesh::Scalar ofxHEMesh::halfedgeAngle(ofxHEMeshHalfedge h) const {

}

string ofxHEMesh::halfedgeString(ofxHEMeshHalfedge h) const {
	std::stringstream ss;
	ss << halfedgeSource(h).idx << "-" << halfedgeSink(h).idx;
	return ss.str();
}

void ofxHEMesh::printFace(ofxHEMeshFace f) const {
	ofxHEMeshFaceCirculator fc = faceCirculate(f);
	ofxHEMeshFaceCirculator fce = fc;
	do {
		std::cout << halfedgeVertex(*fc).idx << "-";
		++fc;
	} while(fc != fce);
	std::cout << "\n";
}

void ofxHEMesh::printVertexOneHood(ofxHEMeshVertex v) const {
	ofxHEMeshVertexCirculator vc = vertexCirculate(v);
	ofxHEMeshVertexCirculator vce = vc;
	do {
		std::cout << halfedgeSource(*vc).idx << "-";
		++vc;
	} while(vc != vce);
	std::cout << "\n";
}