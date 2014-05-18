#include "ofxHEMeshAssimpLoader.h"
#include "assimp.h"
#include "aiScene.h"
#include "aiConfig.h"
#include "aiPostProcess.h"

ofxHEMeshAssimpLoader::ofxHEMeshAssimpLoader()
: scene(NULL)
{}

ofxHEMeshAssimpLoader::~ofxHEMeshAssimpLoader() {
	clear();
}

void ofxHEMeshAssimpLoader::clear() {
	if(scene){
		aiReleaseImport(scene);
		scene = NULL;
	}
}

bool ofxHEMeshAssimpLoader::loadModel(string modelName, bool triangulate, bool optimize) {
	clear();
	
	ofFile file;
	file.open(modelName, ofFile::ReadOnly, true); // Since it may be a binary file we should read it in binary -Ed
    if(!file.exists()) {
        ofLogVerbose("ofxHEMeshAssimpLoader") << "loadModel(): model does not exist: \"" << modelName << "\"";
        return false;
    }

    ofLogVerbose("ofxHEMeshAssimpLoader") << "loadModel(): loading \"" << file.getFileName()
		<< "\" from \"" << file.getEnclosingDirectory() << "\"";
    
    ofBuffer buffer = file.readToBuffer();
	return loadModel(buffer, triangulate, optimize, file.getExtension().c_str());
}

bool ofxHEMeshAssimpLoader::loadModel(ofBuffer& buffer, bool triangulate, bool optimize, const char * extension) {
	aiSetImportPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_LINE | aiPrimitiveType_POINT );
	aiSetImportPropertyInteger(AI_CONFIG_PP_PTV_NORMALIZE, true);
	
	unsigned int flags = 0;//aiProcess_JoinIdenticalVertices;
	if(triangulate) flags | aiProcess_Triangulate;
	if(optimize) flags |=  aiProcess_ImproveCacheLocality | aiProcess_OptimizeGraph |
			aiProcess_OptimizeMeshes | aiProcess_JoinIdenticalVertices |
			aiProcess_RemoveRedundantMaterials;

	aiEnableVerboseLogging(AI_TRUE);
	scene = aiImportFileFromMemory(buffer.getBinaryBuffer(), buffer.size(), flags, extension);
	return scene != NULL;
}

void ofxHEMeshAssimpLoader::addToHemesh(ofxHEMesh& hemesh, int idx) {
	if(!scene) return;

	if(idx >= 0 && idx < scene->mNumMeshes) {
		aiMesh *mesh = scene->mMeshes[idx];
		ofxHEMeshVertex vstart(hemesh.getNumVertices());
		
		for(int i=0; i < mesh->mNumVertices; ++i) {
			aiVector3D &v = mesh->mVertices[i];
			hemesh.addVertex(ofxHEMesh::Point(v.x, v.y, v.z));
		}
		
		vector<ofxHEMesh::ExplicitFace> faces;
		faces.reserve(mesh->mNumFaces);
		for(int i=0; i < mesh->mNumFaces; ++i) {
			aiFace &f = mesh->mFaces[i];
			ofxHEMesh::ExplicitFace explicitFace(f.mNumIndices);
			for(int j=0; j < f.mNumIndices; ++j) {
				explicitFace[j] = ofxHEMeshVertex(f.mIndices[j]+vstart.idx);
			}
			faces.push_back(explicitFace);
		}
		hemesh.addFaces(faces);
	}
}