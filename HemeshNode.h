#pragma once

struct Node {
	enum Indices{
		Invalid = -1
	};

	explicit Node(int idx=Invalid)
	:	idx(idx)
	{}
	
	bool isValid() const {
		return idx != Invalid;
	}
	
	bool operator==(const Node &rhs) const {
		return idx == rhs.idx;
	}
	bool operator!=(const Node &rhs) const {
		return idx != rhs.idx;
	}
	bool operator<(const Node &rhs) const {
		return idx < rhs.idx;
	}
	bool operator>(const Node &rhs) const {
		return idx > rhs.idx;
	}

	int idx;
};

struct Vertex : public Node {
	explicit Vertex(int idx=Invalid) : Node(idx)
	{}
};
struct Halfedge : public Node {
	explicit Halfedge(int idx=Invalid) : Node(idx)
	{}
};
struct Edge : public Node {
	explicit Edge(int idx=Invalid) : Node(idx)
	{}
};
struct Face : public Node {
	enum Winding{
		CW=0,
		CCW
	};

	explicit Face(int idx=Invalid) : Node(idx)
	{}
};

/*
std::ostream& operator<<(std::ostream& os, const Vertex& v) {
    os << std::string("Vertex:");
	os << v.idx;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Halfedge& h) {
    os << std::string("Halfedge:") << h.idx;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Face& f) {
    os << std::string("Face:") << f.idx;
    return os;
}
*/