#pragma once
#include "HemeshNode.h"

struct VertexAdjacency {
	VertexAdjacency() : he() {}
	
	Halfedge he;
};

struct HalfedgeAdjacency {
	HalfedgeAdjacency() :
		v(), f(), prev(), next()
	{}
	
	Vertex v;
	Face f;
	Halfedge prev;
	Halfedge next;
};

struct FaceAdjacency {
	FaceAdjacency() : he() {}
	
	Halfedge he;
};