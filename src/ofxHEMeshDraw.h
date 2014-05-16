#pragma once
#include "ofxHEMesh.h"


class ofxHEMeshDraw {
public:
	enum NormalType{
		VertexNormals = 0,
		FaceNormals,
		NoNormals
	};
	
	struct Property{
		Property(bool enabled, bool needsUpdate=true)
		: enabled(enabled), needsUpdate(needsUpdate)
		{}
		
		void enable(bool v) {
			enabled = v;
			if(enabled) {
				needsUpdate = true;
			}
		}
	
		bool enabled;
		bool needsUpdate;
	};

	ofxHEMeshDraw(ofxHEMesh& hemesh, NormalType normalType=VertexNormals);
	
	void draw();

	bool getDrawEdges() const;
	ofxHEMeshDraw& setDrawEdges(bool v);
	bool getDrawFaces() const;
	ofxHEMeshDraw& setDrawFaces(bool v);
	bool getDrawVertexNormals() const;
	ofxHEMeshDraw& setDrawVertexNormals(bool v);

	void faceIndices(vector<ofIndexType>& indices);
	void edgeIndices(vector<ofIndexType>& indices);
	void vertexNormalVectors(vector<ofVec3f> &points, ofxHEMesh::Scalar scale=1.);
	void borderEdges(vector<ofIndexType>& indices);
	
protected:
	void updateEdges();
	void updateFaces();
	void updateNormals();
	void updateVertexNormals();
	void updateVertexNormalVectors();


	ofxHEMesh& hemesh;
	NormalType normalType;
	ofxHEMeshProperty<ofVec3f> *meshVertexNormals;
	Property drawEdges;
	Property drawFaces;
	bool drawVertexNormals;
	bool calculateVertexNormals;
	ofVbo edges;
	ofVbo faces;
	ofVbo vertexNormals;
	float normalScale;
};