#include "Hemesh.h"
#include <sstream>


#define WRAP_NEXT(idx, n) (((idx)+1)%(n))
#define WRAP_PREV(idx, n) (((idx)+(n)-1)%(n))

using std::ifstream;
using std::ofstream;
using std::queue;
using std::set;

Hemesh::Hemesh()
:	points(0)
{
	vertexAdjacency = addVertexProperty<VertexAdjacency>("vertex-adjacency", VertexAdjacency());
	halfedgeAdjacency = addHalfedgeProperty<HalfedgeAdjacency>("halfedge-adjacency", HalfedgeAdjacency());
	faceAdjacency = addFaceProperty<FaceAdjacency>("face-adjacency", FaceAdjacency());
	points = addVertexProperty<Point>("points", Point());
}

// Assumes the mesh is a triangulation
void Hemesh::subdivideLoop() {

	// Get the new location of existing vertices in the subd mesh
	VertexIterator vit = verticesBegin();
	VertexIterator vite = verticesEnd();
	map<Vertex, Point> newPositions;
	for(; vit != vite; ++vit) {
		VertexCirculator vc = vertexCirculate(*vit);
		VertexCirculator vce = vc;
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
		
		newPositions.insert(std::pair<Vertex, Point>(*vit, nPt));
	}

	// Get the location of new vertices dividing edges in the subd mesh
	map<Halfedge, Vertex> edgeVertices;
	EdgeIterator eit = edgesBegin();
	EdgeIterator eite = edgesEnd();
	for(; eit != eite; ++eit) {
		Halfedge h = *eit;
		Halfedge hn = halfedgeNext(h);
		Halfedge hon = halfedgeNext(halfedgeOpposite(h));
		Point pt =
			(vertexPoint(halfedgeSource(h)) + vertexPoint(halfedgeSink(h)))*0.375 +
			(vertexPoint(halfedgeSink(hn)) + vertexPoint(halfedgeSink(hon)))*0.125;
		Vertex v = addVertex(pt);
		edgeVertices.insert(std::pair<Halfedge, Vertex>(h, v));
		edgeVertices.insert(std::pair<Halfedge, Vertex>(halfedgeOpposite(h), v));
	}
	
	// Move the existing vertices to their subd mesh positions
	map<Vertex, Point>::const_iterator npt_it = newPositions.begin();
	map<Vertex, Point>::const_iterator npt_ite = newPositions.end();
	for(; npt_it != npt_ite; ++npt_it) {
		vertexMoveTo(npt_it->first, npt_it->second);
	}
	
	// Calculate the subd mesh faces
	vector<ExplicitFace> faces;
	faces.reserve(faceAdjacency->size()*4);
	FaceIterator fit = facesBegin();
	FaceIterator fite = facesEnd();
	for(; fit != fite; ++fit) {
		FaceCirculator fc = faceCirculate(*fit);
		FaceCirculator fce = fc;
		
		ExplicitFace innerFace(3);
		int i=0;
		do {
			ExplicitFace cornerFace(3);
			Halfedge h = *fc;
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
void Hemesh::subdivideCatmullClark() {
	map<Face, Point> facePoints;
	FaceIterator fit = facesBegin();
	FaceIterator fite = facesEnd();
	for(; fit != fite; ++fit) {
		facePoints.insert(std::pair<Face, Point>(*fit, faceCentroid(*fit)));
	}
	
	map<Halfedge, Point> edgePoints;
	EdgeIterator eit = edgesBegin();
	EdgeIterator eite = edgesEnd();
	for(; eit != eite; ++eit) {
		Halfedge h = *eit;
		Halfedge ho = halfedgeOpposite(h);
		Point pt = 0.25*(
			vertexPoint(halfedgeSink(h)) + vertexPoint(halfedgeSink(ho)) +
			facePoints[halfedgeFace(h)] + facePoints[halfedgeFace(ho)]
		);
		edgePoints.insert(std::pair<Halfedge, Point>(h, pt));
		edgePoints.insert(std::pair<Halfedge, Point>(ho, pt));
	}
	
	VertexIterator vit = verticesBegin();
	VertexIterator vite = verticesEnd();
	for(; vit != vite; ++vit) {
		Point Q(0, 0, 0);
		Point R(0, 0, 0);
		Scalar valence = 0;
		VertexCirculator vc = vertexCirculate(*vit);
		VertexCirculator vce = vc;
		do {
			Halfedge h = *vc;
			Q += facePoints[halfedgeFace(h)];
			R += edgePoints[h];
			++valence;
		} while(vc != vce);
	}
}

void Hemesh::addMesh(const ofMesh& mesh) {
	Vertex vstart(vertexAdjacency->size());
	
	const vector<ofVec3f>& vertices = mesh.getVertices();
	vector<ofVec3f>::const_iterator vit = vertices.begin();
	vector<ofVec3f>::const_iterator vite = vertices.end();

	for(; vit != vite; ++vit) {
		addVertex(*vit);
	}
	
	
	// Assume triangles for now
	// TOOD: use mesh.getMode()
	vector< vector<Vertex> > faces;
	const vector<ofIndexType>& indices = mesh.getIndices();
	vector<ofIndexType>::const_iterator iit = indices.begin();
	vector<ofIndexType>::const_iterator iite = indices.end();
	
	faces.reserve(indices.size()/3);
	while(iit != iite) {
		vector<Vertex> face(3);
		for(int i=0; i < 3; ++i) {
			Vertex v = Vertex((*iit)+vstart.idx);
			face[i] = v;
			++iit;
		}
		faces.push_back(face);
	}
	
	addFaces(faces);
}

Vertex Hemesh::addVertex(const Point& p) {
	int idx = vertexProperties.size();
	vertexProperties.extend();
	points->set(idx, p);
	Vertex v(idx);
	return v;
}

Halfedge Hemesh::addEdge() {
	halfedgeProperties.extend();
	halfedgeProperties.extend();
	return Halfedge(halfedgeProperties.size()-2);
}

static bool orderVertices(Vertex& v1, Vertex& v2) {
	bool swap = v1 > v2;
	if(swap) {
		Vertex v = v2;
		v2 = v1;
		v1 = v;
	}
	return swap;
}

void Hemesh::addFaces(const vector<ExplicitFace>& faces) {
	map<ExplicitEdge, Halfedge> explicitEdgeMap;
	int i, j;
	
	// Create any edges that don't yet exist
	set<Halfedge> allHalfedges;
	for(i=0; i < faces.size(); ++i) {
		const vector<Vertex>& face = faces[i];
		int nv = int(face.size());
		for(j=0; j < nv; ++j) {
			Vertex v1 = face[j];
			Vertex v2 = face[WRAP_NEXT(j, nv)];
			orderVertices(v1, v2);
			if(explicitEdgeMap.find(ExplicitEdge(v1, v2)) == explicitEdgeMap.end()) {
				Halfedge h1 = addEdge();
				Halfedge h1o = halfedgeOpposite(h1);
				setHalfedgeVertex(h1, v2);
				setHalfedgeVertex(h1o, v1);
				setVertexHalfedge(v2, h1);
				setVertexHalfedge(v1, h1o);
				explicitEdgeMap.insert(std::pair<ExplicitEdge, Halfedge>(ExplicitEdge(v1, v2), h1));
				allHalfedges.insert(h1);
				allHalfedges.insert(h1o);
			}
		}
	}
	
	// Link Halfedges around faces
	for(i=0; i < faces.size(); ++i) {
		Face f(faceProperties.size());
		faceAdjacency->extend();
	
		const vector<Vertex>& face = faces[i];
		int nv = int(face.size());
		vector<Halfedge> halfedges;
		for(j=0; j < nv; ++j) {
			Vertex v1 = face[j];
			Vertex v2 = face[WRAP_NEXT(j, nv)];
			bool didSwap = orderVertices(v1, v2);
			auto iter = explicitEdgeMap.find(ExplicitEdge(v1, v2));
			if(didSwap) halfedges.push_back(halfedgeOpposite(iter->second));
			else halfedges.push_back(iter->second);
		}
		
		setFaceHalfedge(f, halfedges[0]);
		for(j=0; j < nv; ++j) {
			Halfedge h1 = halfedges[j];
			Halfedge h2 = halfedges[WRAP_NEXT(j, nv)];
			allHalfedges.erase(allHalfedges.find(h1));
			setHalfedgeNext(h1, h2);
			setHalfedgePrev(h2, h1);
			setHalfedgeFace(h1, f);
		}
	}
	
	set<Halfedge>::const_iterator ahit = allHalfedges.begin();
	set<Halfedge>::const_iterator ahite = allHalfedges.end();
	map<Vertex, Halfedge> sinks;
	map<Vertex, Halfedge> sources;
	for(; ahit != ahite; ++ahit) {
		Halfedge h = *ahit;
		sinks.insert(std::pair<Vertex, Halfedge>(halfedgeSink(h), h));
		sources.insert(std::pair<Vertex, Halfedge>(halfedgeSource(h), h));
	}
	
	map<Vertex, Halfedge>::const_iterator sink_it = sinks.begin();
	map<Vertex, Halfedge>::const_iterator sink_ite = sinks.end();
	map<Vertex, Halfedge>::const_iterator source_ite = sources.end();
	for(; sink_it != sink_ite; ++sink_it) {
		map<Vertex, Halfedge>::const_iterator source_it = sources.find(sink_it->first);
		if(source_it == source_ite) {
			std::cout << halfedgeString(sink_it->second) << " doesn't have a source\n";
			continue;
		}
		setHalfedgeNext(sink_it->second, source_it->second);
		setHalfedgePrev(source_it->second, sink_it->second);
	}
}


Face Hemesh::addFace(const vector<Vertex>& vertices) {
	vector<Halfedge> halfedges;
	vector<bool> exists;
	
	size_t nv = vertices.size();
	size_t i;
	for(i=0; i < nv; ++i) {
		Vertex v1 = vertices[i];
		Vertex v2 = vertices[WRAP_NEXT(i, nv)];
		
		Halfedge h = findHalfedge(v1, v2);
		bool hExists = h.isValid();

		if(hExists && !halfedgeIsOnBoundary(h)) {
			std::stringstream ss;
			ss << "attempt to add Face where Halfedge " << v1.idx << " " << v2.idx << " isn't on a boundary\n";
			ss << "Face: ";
			for(int j=0; j < nv; ++j) {
				ss << vertices[j].idx << "-";
			}
			throw std::invalid_argument(ss.str());
			return Face();
		}
		
		halfedges.push_back(h);
		exists.push_back(hExists);
	}
	
	Face f(faceProperties.size());
	faceAdjacency->extend();
	

	// set the face of halfedges and create any new ones necessary
	vector<Halfedge> halfedgesPrev;
	vector<Halfedge> halfedgesNext;
	for(i=0; i < nv; ++i) {
		if(exists[i]) {
			setHalfedgeFace(halfedges[i], f);
		}
		else {
			Halfedge h1 = addEdge();
			Halfedge h2 = Halfedge(h1.idx+1);
			
			Vertex v1 = vertices[i];
			Vertex v2 = vertices[WRAP_NEXT(i, nv)];
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
		Halfedge h = halfedges[i];
		
		// inner loop
		auto pidx = WRAP_PREV(i, nv);
		auto nidx = WRAP_NEXT(i, nv);
		Halfedge p = halfedges[pidx];
		Halfedge n = halfedges[nidx];
		setHalfedgePrev(h, p);
		setHalfedgeNext(h, n);
		
		if(!exists[i]) {
			// outer loop
			Halfedge ho = halfedgeOpposite(h);
			if(exists[nidx]) {
				Halfedge np = halfedgesPrev[nidx];
				setHalfedgePrev(ho, np);
				setHalfedgeNext(np, ho);
			}
			else {
				setHalfedgePrev(ho, halfedgeOpposite(n));
			}
			
			if(exists[pidx]) {
				Halfedge nn = halfedgesNext[pidx];
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


void Hemesh::removeVertex(Vertex v) {
	if(!v.isValid()) throw std::invalid_argument("attempting to remove invalid vertex");
	
	Halfedge h = vertexHalfedge(v);
	if(h.isValid()) {
		do {
			Halfedge hh = halfedgeSinkCCW(h);
			if(removeHalfedge(h)) {
				break;
			}
			h = hh;
		}
		while(h.isValid());
	}
	setVertexHalfedge(v, Halfedge());
}


bool Hemesh::removeHalfedge(Halfedge h) {
	Face f = halfedgeFace(h);
	Halfedge ho = halfedgeOpposite(h);
	Face f2 = halfedgeFace(ho);

	Vertex v = halfedgeVertex(h);
	Vertex v2 = halfedgeVertex(ho);

	Halfedge h1p = halfedgePrev(h);
	Halfedge h1n = halfedgeSourceCW(h);
	Halfedge h2p = halfedgePrev(ho);
	Halfedge h2n = halfedgeSourceCW(ho);
	
	Vertex vrem;
	if(h1p == ho) vrem = v2;
	else vrem = v;
	
	setHalfedgePrev(h1n, h1p);
	setHalfedgeNext(h1p, h1n);
	
	setHalfedgePrev(h2n, h2p);
	setHalfedgeNext(h2p, h2n);
	
	halfedgeAdjacency->set(h.idx, HalfedgeAdjacency());
	halfedgeAdjacency->set(ho.idx, HalfedgeAdjacency());

	if(f.idx == f2.idx) {
		// belong to the same face, last edge linking vertex to mesh
		setVertexHalfedge(vrem, Halfedge());
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
		setFaceHalfedge(f, Halfedge());
		setFaceHalfedge(f2, h1p);
	
		// set all halfedges to the common face
		Halfedge hh = h1p;
		do {
			setHalfedgeFace(hh, f2);
			hh = halfedgeNext(hh);
		} while(hh != h1p);
		return false;
	}
}


void Hemesh::removeFace(Face f) {
	Halfedge h = faceHalfedge(f);
	Halfedge hstart = h;
	do {
		setHalfedgeFace(h, Face());
		h = halfedgeNext(h);
	}
	while(h != hstart);
	setFaceHalfedge(f, Halfedge());
}

int Hemesh::getNumVertices() const {
	return (int)vertexAdjacency->size();
}

int Hemesh::getNumEdges() const {
	return ((int)halfedgeAdjacency->size())/2;
}

int Hemesh::getNumHalfedges() const {
	return (int)halfedgeAdjacency->size();
}

int Hemesh::getNumFaces() const {
	return (int)faceAdjacency->size();
}

FaceIterator Hemesh::facesBegin() const {
	Face f(0);
	if(faceAdjacency->size() > 0) {
		while(!faceHalfedge(f).isValid()) {
			++f.idx;
		}
	}
	return FaceIterator(this, f);
}

FaceIterator Hemesh::facesEnd() const {
	return FaceIterator(this, Face(faceAdjacency->size()));
}

EdgeIterator Hemesh::edgesBegin() const {
	Halfedge h(0);
	if(halfedgeAdjacency->size() > 0) {
		while(!halfedgeFace(h).isValid()) {
			++h.idx;
		}
	}
	return EdgeIterator(this, h);
}

EdgeIterator Hemesh::edgesEnd() const {
	return EdgeIterator(this, Halfedge(halfedgeAdjacency->size()));
}

VertexIterator Hemesh::verticesBegin() const {
	Vertex v(0);
	if(vertexAdjacency->size() > 0) {
		while(!vertexHalfedge(v).isValid()) {
			++v.idx;
		}
	}
	return VertexIterator(this, v);
}

VertexIterator Hemesh::verticesEnd() const {
	return VertexIterator(this, Vertex(vertexAdjacency->size()));
}

FaceCirculator Hemesh::faceCirculate(const Face& f) const {
	return FaceCirculator(this, faceHalfedge(f));
}

VertexCirculator Hemesh::vertexCirculate(const Vertex& v) const {
	return VertexCirculator(this, vertexHalfedge(v));
}

Halfedge Hemesh::vertexHalfedge(Vertex v) const {
	return vertexAdjacency->get(v.idx).he;
}


void Hemesh::setVertexHalfedge(Vertex v, Halfedge h) {
	vertexAdjacency->get(v.idx).he = h;
}


Halfedge Hemesh::faceHalfedge(Face f) const {
	return faceAdjacency->get(f.idx).he;
}


void Hemesh::setFaceHalfedge(Face f, Halfedge h) {
	faceAdjacency->get(f.idx).he = h;
}


Halfedge Hemesh::halfedgeOpposite(Halfedge h) const {
	if((h.idx & 1) == 1) return Halfedge(h.idx-1);
	else return Halfedge(h.idx+1);
}


Vertex Hemesh::halfedgeVertex(Halfedge h) const {
	return halfedgeAdjacency->get(h.idx).v;
}


void Hemesh::setHalfedgeVertex(Halfedge h, Vertex v) {
	halfedgeAdjacency->get(h.idx).v = v;
}


Face Hemesh::halfedgeFace(Halfedge h) const {
	return halfedgeAdjacency->get(h.idx).f;
}


void Hemesh::setHalfedgeFace(Halfedge h, Face f) {
	halfedgeAdjacency->get(h.idx).f = f;
}


Vertex Hemesh::halfedgeSource(Halfedge h) const {
	return halfedgeVertex(halfedgeOpposite(h));
}


Vertex Hemesh::halfedgeSink(Halfedge h) const {
	return halfedgeVertex(h);
}


Halfedge Hemesh::halfedgeNext(Halfedge h) const {
	return halfedgeAdjacency->get(h.idx).next;
}


void Hemesh::setHalfedgeNext(Halfedge h, Halfedge next) {
	halfedgeAdjacency->get(h.idx).next = next;
}


Halfedge Hemesh::halfedgePrev(Halfedge h) const {
	return halfedgeAdjacency->get(h.idx).prev;
}


void Hemesh::setHalfedgePrev(Halfedge h, Halfedge prev) {
	halfedgeAdjacency->get(h.idx).prev = prev;
}


Halfedge Hemesh::halfedgeSourceCW(Halfedge h) const {
	return halfedgeNext(halfedgeOpposite(h));
}


Halfedge Hemesh::halfedgeSourceCCW(Halfedge h) const {
	return halfedgeOpposite(halfedgePrev(h));
}


Halfedge Hemesh::halfedgeSinkCW(Halfedge h) const {
	return halfedgeOpposite(halfedgeNext(h));
}


Halfedge Hemesh::halfedgeSinkCCW(Halfedge h) const {
	return halfedgePrev(halfedgeOpposite(h));
}



Halfedge Hemesh::findHalfedge(Vertex v1, Vertex v2) const {
	Halfedge h1 = vertexHalfedge(v1);
	Halfedge h2 = vertexHalfedge(v2);
	if(h1.isValid() && h2.isValid()) {
		Halfedge hStart = h2;
		do {
			if(halfedgeSource(h2) == v1) {
				return h2;
			}
			h2 = halfedgeSinkCCW(h2);
		} while(h2 != hStart);
	}
	return Halfedge();
}

bool Hemesh::halfedgeIsOnBoundary(Halfedge h) const {
	return !halfedgeFace(h).isValid();
}

int Hemesh::vertexValence(Vertex v) const {
	int n=0;
	VertexCirculator vc = vertexCirculate(v);
	VertexCirculator vce = vc;
	do {
		++n;
		++vc;
	} while(vc != vce);
	return n;
}

void Hemesh::vertexMoveTo(Vertex v, const Point& p) {
	points->set(v.idx, p);
}

Hemesh::Point Hemesh::centroid() const {
	Point c(0, 0, 0);
	Scalar n = 0;
	VertexIterator vit = verticesBegin();
	VertexIterator vite = verticesEnd();
	for(; vit != vite; ++vit) {
		c += vertexPoint(*vit);
		++n;
	}
	c *= 1./n;
	return c;
}

Hemesh::Point Hemesh::vertexPoint(Vertex v) const {
	return points->get(v.idx);
}

Hemesh::Direction Hemesh::angleWeightedVertexNormal(Vertex v) const {
	VertexCirculator vc = vertexCirculate(v);
	VertexCirculator vce = vc;
	
	Direction n(0, 0, 0);
	do {
		Halfedge& h = *vc;
		
		Scalar theta = angleAtVertex(h);
		Face f = halfedgeFace(h);
		if(f.isValid()) {
			n += faceNormal(f)*theta;
		}
		++vc;
	} while(vc != vce);
	// TODO: check for NaNs
	n.normalize();
	return n;
}

Hemesh::Scalar Hemesh::faceArea(Face f) const {
	// TODO: general case vs. Triangle case
}

Hemesh::Point Hemesh::faceCentroid(Face f) const {
	Point centroid(0, 0, 0);
	float n = 0;
	FaceCirculator fc = faceCirculate(f);
	FaceCirculator fce = fc;
	do {
		centroid += vertexPoint(halfedgeVertex(*fc));
		++n;
		++fc;
	} while(fc != fce);
	
	return centroid*(1/n);
}

Hemesh::Direction Hemesh::faceNormal(Face f) const {
	Halfedge h1 = faceHalfedge(f);
	Halfedge h2 = halfedgeNext(h1);
	Direction v1 = halfedgeDirection(h1);
	Direction v2 = halfedgeDirection(h2);
	Direction n = v1.cross(v2);
	n.normalize();
	return n;
}

Hemesh::Point Hemesh::halfedgeLerp(Halfedge h, Scalar t) const {
	Point p1 = vertexPoint(halfedgeSource(h));
	Point p2 = vertexPoint(halfedgeSource(h));
	return p1+(p2-p1)*t;
}

Hemesh::Point Hemesh::halfedgeMidpoint(Halfedge h) const {
	return halfedgeLerp(h, 0.5);
}

Hemesh::Scalar Hemesh::halfedgeCotan(Halfedge h) const {
	Halfedge h2 = halfedgeNext(h);
	Halfedge h3 = halfedgeNext(h2);
	Vertex v1 = halfedgeVertex(h2);
	Vertex v2 = halfedgeVertex(h3);
	Vertex v3 = halfedgeVertex(h);
	Point p0 = vertexPoint(v1);
	Point p1 = vertexPoint(v2);
	Point p2 = vertexPoint(v3 );
	
	Direction u = p1-p0;
	Direction v = p2-p0;

	Scalar res = u.dot(v)/u.cross(v).length();
	// TODO: check for NaNs
	return res;
}

Hemesh::Scalar Hemesh::halfedgeLengthSquared(Halfedge h) const {
	return halfedgeDirection(h).lengthSquared();
}

Hemesh::Direction Hemesh::halfedgeDirection(Halfedge h) const {
	return vertexPoint(halfedgeSink(h)) - vertexPoint(halfedgeSource(h));
}

Hemesh::Scalar Hemesh::angleAtVertex(Halfedge h) const {
	Direction v1 = -halfedgeDirection(h);
	Direction v2 = halfedgeDirection(halfedgeNext(h));
	v1.normalize();
	v2.normalize();
	Scalar dot = MAX(MIN(1, v1.dot(v2)), -1);
	Scalar res = acos(dot);
	// TODO: check for NaNs
	return res;
}

Hemesh::Scalar Hemesh::halfedgeAngle(Halfedge h) const {

}

string Hemesh::halfedgeString(Halfedge h) const {
	std::stringstream ss;
	ss << halfedgeSource(h).idx << "-" << halfedgeSink(h).idx;
	return ss.str();
}

void Hemesh::printFace(Face f) const {
	FaceCirculator fc = faceCirculate(f);
	FaceCirculator fce = fc;
	do {
		std::cout << halfedgeVertex(*fc).idx << "-";
		++fc;
	} while(fc != fce);
	std::cout << "\n";
}

void Hemesh::printVertexOneHood(Vertex v) const {
	VertexCirculator vc = vertexCirculate(v);
	VertexCirculator vce = vc;
	do {
		std::cout << halfedgeSource(*vc).idx << "-";
		++vc;
	} while(vc != vce);
	std::cout << "\n";
}