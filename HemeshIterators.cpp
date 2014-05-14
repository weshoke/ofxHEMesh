#include "HemeshIterators.h"
#include "Hemesh.h"

FaceIterator::FaceIterator()
: hemesh(NULL), f()
{}

FaceIterator::FaceIterator(const Hemesh* hemesh)
: hemesh(hemesh), f()
{}

FaceIterator::FaceIterator(const Hemesh* hemesh, Face f)
: hemesh(hemesh), f(f)
{}

FaceIterator::FaceIterator(const FaceIterator& src)
: hemesh(src.hemesh), f(src.f)
{}

bool FaceIterator::operator==(const FaceIterator& right) {
	return hemesh == right.hemesh && f == right.f;
}

bool FaceIterator::operator!=(const FaceIterator& right) {
	return !((*this) == right);
}

Face& FaceIterator::operator*() {
	return f;
}

Face* FaceIterator::operator->() {
	return &f;
}

FaceIterator& FaceIterator::operator++() {
	int n = hemesh->faceAdjacency->size();
	do {
		++f.idx;
		if(hemesh->faceHalfedge(f).isValid()) {
			break;
		}
	} while(f.idx < n);
	return *this;
}

FaceIterator FaceIterator::operator++(int) {
	FaceIterator it(*this);
	++(*this);
	return it;
}

FaceIterator& FaceIterator::operator--() {
	do {
		--f.idx;
		if(hemesh->faceHalfedge(f).isValid()) {
			break;
		}
	} while(f.idx > 0);
	return *this;
}

FaceIterator FaceIterator::operator--(int) {
	FaceIterator it(*this);
	--(*this);
	return it;
}


EdgeIterator::EdgeIterator()
: hemesh(NULL), h()
{}

EdgeIterator::EdgeIterator(const Hemesh* hemesh)
: hemesh(hemesh), h()
{}

EdgeIterator::EdgeIterator(const Hemesh* hemesh, Halfedge h)
: hemesh(hemesh), h(h)
{}

EdgeIterator::EdgeIterator(const EdgeIterator& src)
: hemesh(src.hemesh), h(src.h)
{}

bool EdgeIterator::operator==(const EdgeIterator& right) {
	return hemesh == right.hemesh && h == right.h;
}

bool EdgeIterator::operator!=(const EdgeIterator& right) {
	return !((*this) == right);
}

Halfedge& EdgeIterator::operator*() {
	return h;
}

Halfedge* EdgeIterator::operator->() {
	return &h;
}

EdgeIterator& EdgeIterator::operator++() {
	int n = hemesh->halfedgeAdjacency->size();
	do {
		h.idx += 2;
		if(hemesh->halfedgeVertex(h).isValid()) {
			break;
		}
	} while(h.idx < n);
	h.idx = MIN(h.idx, n);
	return *this;
}

EdgeIterator EdgeIterator::operator++(int) {
	EdgeIterator it(*this);
	++(*this);
	return it;
}

EdgeIterator& EdgeIterator::operator--() {
	do {
		h.idx -= 2;
		if(hemesh->halfedgeVertex(h).isValid()) {
			break;
		}
	} while(h.idx > 0);
	h.idx = MAX(h.idx, 0);
	return *this;
}

EdgeIterator EdgeIterator::operator--(int) {
	EdgeIterator it(*this);
	--(*this);
	return it;
}

VertexIterator::VertexIterator()
: hemesh(NULL), v()
{}

VertexIterator::VertexIterator(const Hemesh* hemesh)
: hemesh(hemesh), v()
{}

VertexIterator::VertexIterator(const Hemesh* hemesh, Vertex v)
: hemesh(hemesh), v(v)
{}

VertexIterator::VertexIterator(const VertexIterator& src)
: hemesh(src.hemesh), v(src.v)
{}

bool VertexIterator::operator==(const VertexIterator& right) {
	return hemesh == right.hemesh && v == right.v;
}

bool VertexIterator::operator!=(const VertexIterator& right) {
	return !((*this) == right);
}

Vertex& VertexIterator::operator*() {
	return v;
}

Vertex* VertexIterator::operator->() {
	return &v;
}

VertexIterator& VertexIterator::operator++() {
	int n = hemesh->vertexAdjacency->size();
	do {
		++v.idx;
		if(hemesh->vertexHalfedge(v).isValid()) {
			break;
		}
	} while(v.idx < n);
	return *this;
}

VertexIterator VertexIterator::operator++(int) {
	VertexIterator it(*this);
	++(*this);
	return it;
}

VertexIterator& VertexIterator::operator--() {
	do {
		--v.idx;
		if(hemesh->vertexHalfedge(v).isValid()) {
			break;
		}
	} while(v.idx > 0);
	return *this;
}

VertexIterator VertexIterator::operator--(int) {
	VertexIterator it(*this);
	--(*this);
	return it;
}


FaceCirculator::FaceCirculator()
: hemesh(NULL), h()
{}

FaceCirculator::FaceCirculator(const Hemesh* hemesh)
: hemesh(hemesh), h()
{}

FaceCirculator::FaceCirculator(const Hemesh* hemesh, Halfedge h)
: hemesh(hemesh), h(h)
{}

FaceCirculator::FaceCirculator(const FaceCirculator& src)
: hemesh(src.hemesh), h(src.h)
{}

bool FaceCirculator::operator==(const FaceCirculator& right) {
	return hemesh == right.hemesh && h == right.h;
}
bool FaceCirculator::operator!=(const FaceCirculator& right) {
	return !(*this == right);
}

Halfedge& FaceCirculator::operator*() {
	return h;
}

Halfedge* FaceCirculator::operator->() {
	return &h;
}

FaceCirculator& FaceCirculator::operator++() {
	h = hemesh->halfedgeNext(h);
	return *this;
}

FaceCirculator FaceCirculator::operator++(int) {
	FaceCirculator it(*this);
	++(*this);
	return it;
}

FaceCirculator& FaceCirculator::operator--() {
	h = hemesh->halfedgePrev(h);
	return *this;
}

FaceCirculator FaceCirculator::operator--(int) {
	FaceCirculator it(*this);
	--(*this);
	return it;
}



VertexCirculator::VertexCirculator()
: hemesh(NULL), h()
{}

VertexCirculator::VertexCirculator(const Hemesh* hemesh)
: hemesh(hemesh), h()
{}

VertexCirculator::VertexCirculator(const Hemesh* hemesh, Halfedge h)
: hemesh(hemesh), h(h)
{}

VertexCirculator::VertexCirculator(const VertexCirculator& src)
: hemesh(src.hemesh), h(src.h)
{}

bool VertexCirculator::operator==(const VertexCirculator& right) {
	return hemesh == right.hemesh && h == right.h;
}

bool VertexCirculator::operator!=(const VertexCirculator& right) {
	return !(*this == right);
}

Halfedge& VertexCirculator::operator*() {
	return h;
}

Halfedge* VertexCirculator::operator->() {
	return &h;
}

VertexCirculator& VertexCirculator::operator++() {
	h = hemesh->halfedgeSinkCCW(h);
	return *this;
}

VertexCirculator VertexCirculator::operator++(int) {
	VertexCirculator it(*this);
	++(*this);
	return it;
}

VertexCirculator& VertexCirculator::operator--() {
	h = hemesh->halfedgeSinkCW(h);
	return *this;
}

VertexCirculator VertexCirculator::operator--(int) {
	VertexCirculator it(*this);
	--(*this);
	return it;
}