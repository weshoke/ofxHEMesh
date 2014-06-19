#include "ofxHEMeshAdaptiveGrid.h"

ofxHEMeshAdaptiveGrid::ofxHEMeshAdaptiveGrid(Scalar detail)
:	ofxHEMeshAdaptive(detail),
	grid(ofxHEMeshNode::Invalid)
{
	math::Transform::Ptr xform = math::Transform::createLinearTransform(detail);
	grid.setTransform(xform);
	std::cout << "detail: " << detail << "\n";
	std::cout << grid.transform().voxelSize() << "\n";
}

bool ofxHEMeshAdaptiveGrid::loadOBJModel(string modelName) {
	ofxHEMesh::loadOBJModel(modelName);
	initializeMesh();
	addGeometryListener(this);
	
	ofxHEMeshVertexIterator vit = verticesBegin();
	ofxHEMeshVertexIterator vite = verticesEnd();
	for(; vit != vite; ++vit) {
		vertexAdded(*vit);
	}
}

bool ofxHEMeshAdaptiveGrid::testRayOnFace(Ray<>& ray, const Point& p, ofxHEMeshFace f, RayIntersectionData& data) {
	Direction n = faceNormal(f);
	Ray<>::RealType t;
	if(ray.intersects(Ray<>::Vec3T(n.x, n.y, n.z), Ray<>::Vec3T(p.x, p.y, p.z), t)) {
		Ray<>::Vec3T pray = ray(t);
		Point pint(pray[0], pray[1], pray[2]);

		if(withinFace(f, pint)) {
			data.t = t;
			data.xyz = pint;
			data.face = f;
			return true;
		}
	}
	
	return false;
}

bool ofxHEMeshAdaptiveGrid::testRayOnVertexHood(Ray<>& ray, ofxHEMeshVertex v, RayIntersectionData& data, set<ofxHEMeshVertex>& visitedVertices) {
	visitedVertices.insert(v);

	Point p = vertexPoint(v);
	ofxHEMeshVertexCirculator vc = vertexCirculate(v);
	ofxHEMeshVertexCirculator vce = vc;
	do {
		ofxHEMeshVertex vsrc = halfedgeSource(*vc);
		if(vertexToCoord(vsrc) == data.ijk) {
			if(visitedVertices.find(vsrc) == visitedVertices.end()) {
				//std::cout << v.idx << " --> " << vsrc.idx << " " << data.ijk << " " << data.ijk << " other vertex\n";
				if(testRayOnVertexHood(ray, vsrc, data, visitedVertices)) {
					return true;
				}
			}
		}
	
		ofxHEMeshFace f = halfedgeFace(*vc);
		if(testRayOnFace(ray, p, f, data)) {
			return true;
		}
		++vc;
	} while(vc != vce);
	
	return false;
}

bool ofxHEMeshAdaptiveGrid::castRay(const Point& eye, const Direction& dir, RayIntersectionData& data) {
	Ray<>::Vec3T dird(dir.x, dir.y, dir.z);
	dird.normalize();
	
	openvdb::math::Ray<> ray(Ray<>::Vec3T(eye.x, eye.y, eye.z), dird);
	openvdb::math::Ray<> rIndex = ray.applyInverseMap(*(grid.transform().baseMap()));
	RayIntersector intersector(grid);
	
	set<ofxHEMeshVertex> visitedVertices;
	
	Vec3d xyz;
	while(intersector.intersectsIS(rIndex, xyz)) {
		data.ijk = Coord(xyz[0], xyz[1], xyz[2]);
	
		ofxHEMeshVertex v = vertexAt(data.ijk);
		if(!v.isValid()) throw std::invalid_argument("cast ray invalid vertex");
		
		data.visited.push_back(v);
		
		if(testRayOnVertexHood(ray, v, data, visitedVertices)) {
			return true;
		}
		intersector.ignoreCoord(data.ijk);
	}
	return false;
}

void ofxHEMeshAdaptiveGrid::verticesCleared(ofxHEMeshVertex v) {
	// not used right now
}

void ofxHEMeshAdaptiveGrid::vertexAdded(ofxHEMeshVertex v) {
	addVertexToGrid(v, vertexToCoord(v));
}

void ofxHEMeshAdaptiveGrid::vertexWillBeMovedTo(ofxHEMeshVertex v, Point p) {
	Coord c1 = vertexToCoord(v);
	Coord c = pointToCoord(p);
	if(c != c1) {
		removeVertexFromGrid(v, c1);
		addVertexToGrid(v, c);
	}
}

void ofxHEMeshAdaptiveGrid::vertexWillBeRemoved(ofxHEMeshVertex v) {;
	removeVertexFromGrid(v, vertexToCoord(v));
}

ofxHEMeshVertex ofxHEMeshAdaptiveGrid::vertexAt(Coord c) const {
	Int64Grid::ConstAccessor accessor = grid.getAccessor();
	return ofxHEMeshVertex(accessor.getValue(c));
}

void ofxHEMeshAdaptiveGrid::addVertexToGrid(ofxHEMeshVertex v, Coord c) {
	Int64Grid::Accessor accessor = grid.getAccessor();
	if(!accessor.isValueOn(c)) {
		accessor.setValueOn(c, v.idx);
	}
}

void ofxHEMeshAdaptiveGrid::removeVertexFromGrid(ofxHEMeshVertex v, Coord c) {
	Int64Grid::Accessor accessor = grid.getAccessor();
	if(int(accessor.getValue(c)) == v.idx) {
		// find another vertex to replace
		ofxHEMeshVertexCirculator vc = vertexCirculate(v);
		ofxHEMeshVertexCirculator vce = vc;
		do {
			ofxHEMeshVertex v2 = halfedgeSource(*vc);
			Coord c2 = vertexToCoord(v2);
			if(c2 == c) {
				accessor.setValue(c, v2.idx);
				return;
			}
			++vc;
		} while(vc != vce);
		
		
		// if not, erase
		accessor.setValueOff(c, ofxHEMeshNode::Invalid);
	}
}

Coord ofxHEMeshAdaptiveGrid::vertexToCoord(ofxHEMeshVertex v) {
	return pointToCoord(vertexPoint(v));
}

Coord ofxHEMeshAdaptiveGrid::pointToCoord(Point p) {
	return grid.transform().worldToIndexNodeCentered(Vec3d(p.x, p.y, p.z));
}