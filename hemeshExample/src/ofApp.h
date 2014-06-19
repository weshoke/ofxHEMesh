#pragma once

#include "ofMain.h"
#include "ofEasyCam.h"
#include "ofVbo.h"
#include "GeometryKernel.h"
#include "ofxHEMesh.h"
#include "ofxHEMeshDraw.h"


class ofApp : public ofBaseApp{
public:
	ofApp();
	
	void setup();
	void update();
	void draw();
	
	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);
	
protected:
	
	ofEasyCam camera;
	ofxHEMesh hemesh;
	ofxHEMeshDraw hemeshDraw;
	bool didDrag;
};
