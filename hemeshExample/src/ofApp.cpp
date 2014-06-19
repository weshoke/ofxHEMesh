#include "ofApp.h"
#include "aiMesh.h"
#include "ofxAssimpUtils.h"
#include "ofxHEMeshDEC.h"
#include "ofxHEMeshSelection.h"
#include "ofxHEMeshAdaptiveTools.h"

//--------------------------------------------------------------
ofApp::ofApp()
:	//hemesh(0.2),
	hemeshDraw(hemesh),
	didDrag(false)
{
	hemeshDraw.setDrawEdges(true);//.setDrawVertexNormals(true);
	hemeshDraw.setDrawBoundaryEdges(true);
	//hemeshDraw.setDrawFaces(false);
	//hemeshDraw.setDrawVertexNormals(true);
	hemeshDraw.setDrawVertices(true);
	//hemeshDraw.setDrawVertexLabels(true);
}


void ofApp::setup(){
	
	ofEnableAntiAliasing();
	//ofLog(OF_LOG_VERBOSE);
	camera.setupPerspective(false, 60, 0.1, 10000);
	camera.setPosition(0, 0, -10);
	camera.lookAt(ofVec3f(0, 0, 0));
	camera.setAutoDistance(false);
	
	//hemesh.loadOBJModel("soccerball.wireframe.obj");
	hemesh.loadOBJModel("cube.boundary.obj");
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
	ofEnableDepthTest();

	camera.begin();
	glLightfv(GL_LIGHT0, GL_POSITION, ofVec4f(0., 15., 0., 1.).getPtr());
	hemeshDraw.draw(camera);
	camera.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if(key == '1') {
		hemesh.subdivideCatmullClark();
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
	didDrag = true;
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	didDrag = false;
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}