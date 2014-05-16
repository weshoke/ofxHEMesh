#include "ofxHEMeshDraw.h"

ofxHEMeshDraw::ofxHEMeshDraw(ofxHEMesh& hemesh, NormalType normalType)
:	hemesh(hemesh),
	normalType(normalType),
	meshVertexNormals(NULL),
	drawEdges(false),
	drawFaces(true),
	drawVertexNormals(false),
	calculateVertexNormals(false),
	normalScale(0.1)
{}

void ofxHEMeshDraw::draw() {
	if(hemesh.getTopologyDirty()) {
		if(normalType == VertexNormals) {
			calculateVertexNormals = true;
		}
		if(drawEdges.enabled) {
			updateEdges();
		}
		// Do this before updating faces
		if(normalType != NoNormals || drawVertexNormals) {
			updateNormals();
			if(drawVertexNormals) {
				updateVertexNormalVectors();
			}
		}
		if(drawFaces.enabled) {
			updateFaces();
		}
		
		hemesh.setTopologyDirty(false);
		hemesh.setGeometryDirty(false);
	}
	else if(hemesh.getGeometryDirty()) {
		if(normalType == VertexNormals) {
			calculateVertexNormals = true;
		}
		hemesh.setGeometryDirty(false);
	}
	
	if(drawFaces.enabled) {
		glEnable(GL_LIGHTING);
			glEnable(GL_LIGHT0);		
			glEnable(GL_POLYGON_OFFSET_FILL);
				glPolygonOffset(1, 1);
				faces.drawElements(GL_TRIANGLES, faces.getNumIndices());
			glDisable(GL_POLYGON_OFFSET_FILL);
		glDisable(GL_LIGHTING);
	}
	if(drawEdges.enabled) {
		ofSetColor(ofColor::black);
		edges.drawElements(GL_LINES, edges.getNumIndices());
	}
	if(drawVertexNormals) {
		ofSetColor(ofColor::red);
		vertexNormals.draw(GL_LINES, 0, vertexNormals.getNumVertices());
	}
}

bool ofxHEMeshDraw::getDrawEdges() const {
	return drawEdges.enabled;
}

ofxHEMeshDraw& ofxHEMeshDraw::setDrawEdges(bool v) {
	drawEdges.enable(v);
	return *this;
}

bool ofxHEMeshDraw::getDrawFaces() const {
	return drawFaces.enabled;
}

ofxHEMeshDraw& ofxHEMeshDraw::setDrawFaces(bool v) {
	drawFaces.enable(v);
	return *this;
}

bool ofxHEMeshDraw::getDrawVertexNormals() const {
	return drawVertexNormals;
}

ofxHEMeshDraw& ofxHEMeshDraw::setDrawVertexNormals(bool v) {
	drawVertexNormals = v;
	if(drawVertexNormals) {
		calculateVertexNormals = true;
	}
	return *this;
}

void ofxHEMeshDraw::faceIndices(vector<ofIndexType>& indices) {
	ofxHEMeshFaceIterator fit = hemesh.facesBegin();
	ofxHEMeshFaceIterator fite = hemesh.facesEnd();
	for(; fit != fite; ++fit) {
		ofxHEMeshFaceCirculator fc = hemesh.faceCirculate(*fit);
		ofxHEMeshFaceCirculator fce = fc;
		do {
			indices.push_back(hemesh.halfedgeVertex(*fc).idx);
			++fc;
		} while(fc != fce);
	}
}

void ofxHEMeshDraw::edgeIndices(vector<ofIndexType>& indices) {
	ofxHEMeshEdgeIterator eit = hemesh.edgesBegin();
	ofxHEMeshEdgeIterator eite = hemesh.edgesEnd();
	for(; eit != eite; ++eit) {
		indices.push_back(hemesh.halfedgeSource(*eit).idx);
		indices.push_back(hemesh.halfedgeSink(*eit).idx);
	}
}

void ofxHEMeshDraw::vertexNormalVectors(vector<ofVec3f> &points, ofxHEMesh::Scalar scale) {
	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		ofxHEMesh::Point pt = hemesh.vertexPoint(*vit);
		ofxHEMesh::Point pt2 = pt + meshVertexNormals->get((*vit).idx);
		points.push_back(pt);
		points.push_back(pt2);
	}
}

void ofxHEMeshDraw::borderEdges(vector<ofIndexType>& indices) {
	ofxHEMeshEdgeIterator eit = hemesh.edgesBegin();
	ofxHEMeshEdgeIterator eite = hemesh.edgesEnd();
	for(; eit != eite; ++eit) {
		ofxHEMeshHalfedge h = *eit;
		ofxHEMeshHalfedge ho = hemesh.halfedgeOpposite(h);
		if(!hemesh.halfedgeFace(h).isValid()) {
			indices.push_back(hemesh.halfedgeSource(h).idx);
			indices.push_back(hemesh.halfedgeSink(h).idx);
		}
		if(!hemesh.halfedgeFace(ho).isValid()) {
			indices.push_back(hemesh.halfedgeSource(ho).idx);
			indices.push_back(hemesh.halfedgeSink(ho).idx);
		}
	}
}

void ofxHEMeshDraw::updateEdges() {
	vector<ofIndexType> indices;
	indices.reserve(hemesh.getNumEdges()*2);
	edgeIndices(indices);
	edges.setVertexData(hemesh.getPoints().ptr(), hemesh.getPoints().size(), GL_DYNAMIC_DRAW);
	edges.setIndexData(&indices[0], indices.size(), GL_DYNAMIC_DRAW);
	drawEdges.needsUpdate = false;
}

void ofxHEMeshDraw::updateFaces() {
	vector<ofIndexType> indices;
	indices.reserve(hemesh.getNumFaces()*3);	// minimally have triangle faces
	faceIndices(indices);
	faces.setVertexData(hemesh.getPoints().ptr(), hemesh.getPoints().size(), GL_DYNAMIC_DRAW);
	if(normalType == VertexNormals) {
		faces.setNormalData(meshVertexNormals->ptr(), meshVertexNormals->size(), GL_DYNAMIC_DRAW);
	}
	faces.setIndexData(&indices[0], indices.size(), GL_DYNAMIC_DRAW);
	drawFaces.needsUpdate = false;
}

void ofxHEMeshDraw::updateNormals() {
	switch(normalType) {
		case VertexNormals:
			updateVertexNormals();
			break;
			
		case FaceNormals:
			break;
			
		default:
			break;
	}
	
	if(drawVertexNormals && calculateVertexNormals) {
		updateVertexNormals();
	}
}

void ofxHEMeshDraw::updateVertexNormals() {
	if(!meshVertexNormals) {
		meshVertexNormals = hemesh.addVertexProperty<ofxHEMesh::Direction>("vertex-normals");
	}

	ofxHEMeshVertexIterator vit = hemesh.verticesBegin();
	ofxHEMeshVertexIterator vite = hemesh.verticesEnd();
	for(; vit != vite; ++vit) {
		meshVertexNormals->set((*vit).idx, hemesh.angleWeightedVertexNormal(*vit));
	}
	calculateVertexNormals = false;
}

void ofxHEMeshDraw::updateVertexNormalVectors() {
	vector<ofVec3f> points;
	points.reserve(hemesh.getNumVertices()*2);
	vertexNormalVectors(points, normalScale);
	vertexNormals.setVertexData(&points[0], points.size(), GL_DYNAMIC_DRAW);
}