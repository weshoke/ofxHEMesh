#pragma once
#include "HemeshNode.h"

class Hemesh;

struct FaceIterator {
		
	FaceIterator();
	FaceIterator(const Hemesh* hemesh);
	FaceIterator(const Hemesh* hemesh, Face f);
	FaceIterator(const FaceIterator& src);
	
	bool operator==(const FaceIterator& right);
	bool operator!=(const FaceIterator& right);
	
	Face& operator*();
	Face* operator->();
	
	FaceIterator& operator++();
	FaceIterator operator++(int);
	FaceIterator& operator--();
	FaceIterator operator--(int);
	
	const Hemesh* hemesh;
	Face f;
};

struct EdgeIterator {
		
	EdgeIterator();
	EdgeIterator(const Hemesh* hemesh);
	EdgeIterator(const Hemesh* hemesh, Halfedge h);
	EdgeIterator(const EdgeIterator& src);
	
	bool operator==(const EdgeIterator& right);
	bool operator!=(const EdgeIterator& right);
	
	Halfedge& operator*();
	Halfedge* operator->();
	
	EdgeIterator& operator++();
	EdgeIterator operator++(int);
	EdgeIterator& operator--();
	EdgeIterator operator--(int);
	
	const Hemesh* hemesh;
	Halfedge h;
};

struct VertexIterator {
		
	VertexIterator();
	VertexIterator(const Hemesh* hemesh);
	VertexIterator(const Hemesh* hemesh, Vertex v);
	VertexIterator(const VertexIterator& src);
	
	bool operator==(const VertexIterator& right);
	bool operator!=(const VertexIterator& right);
	
	Vertex& operator*();
	Vertex* operator->();
	
	VertexIterator& operator++();
	VertexIterator operator++(int);
	VertexIterator& operator--();
	VertexIterator operator--(int);
	
	const Hemesh* hemesh;
	Vertex v;
};

struct FaceCirculator {
		
	FaceCirculator();
	FaceCirculator(const Hemesh* hemesh);
	FaceCirculator(const Hemesh* hemesh, Halfedge h);
	FaceCirculator(const FaceCirculator& src);
	
	bool operator==(const FaceCirculator& right);
	bool operator!=(const FaceCirculator& right);
	
	Halfedge& operator*();
	Halfedge* operator->();
	
	FaceCirculator& operator++();
	FaceCirculator operator++(int);
	FaceCirculator& operator--();
	FaceCirculator operator--(int);
	
	const Hemesh* hemesh;
	Halfedge h;
};

struct VertexCirculator {
		
	VertexCirculator();
	VertexCirculator(const Hemesh* hemesh);
	VertexCirculator(const Hemesh* hemesh, Halfedge h);
	VertexCirculator(const VertexCirculator& src);
	
	bool operator==(const VertexCirculator& right);
	bool operator!=(const VertexCirculator& right);
	
	Halfedge& operator*();
	Halfedge* operator->();
	
	VertexCirculator& operator++();
	VertexCirculator operator++(int);
	VertexCirculator& operator--();
	VertexCirculator operator--(int);
	
	const Hemesh* hemesh;
	Halfedge h;
};