#pragma once
#include <string>
#include <vector>
#include <map>

using std::string;
using std::vector;
using std::map;


/*
In order to define generic properties agnostic of type, there has to be a virtual base class
that is inherited by a templated sub-class such that all instantiations of the template can be 
stored in a single collection as pointers.
*/
// interface that doesn't require knowing the array element type
class PropertyBase {
public:
	PropertyBase() {}
	virtual ~PropertyBase() {}

	virtual int capacity() = 0;
	virtual void clear() = 0;
	virtual void extend() = 0;
	virtual void reserve(int n) = 0;
	virtual void resize(int n) = 0;
	virtual void swapItems(int idx1, int idx2) = 0;
	virtual int size() const = 0;

protected:
};


// wrapper around std::vector
template <typename T>
class Property : public PropertyBase {
public:
	Property(const string &name, T def=T())
	:	mName(name), mDef(def)
	{}
	
	~Property() {}
	
	int capacity() { return int(mValues.capacity()); }
	void clear() { mValues.clear(); }
	void extend() { mValues.push_back(mDef); }
	T& get(int idx) { return mValues[idx]; }
	const T& get(int idx) const { return mValues[idx]; }
	T& last() { return get(size()-1); }
	const T& last() const { return get(size()-1); }
	void reserve(int n) { mValues.reserve(n); }
	void resize(int n) { mValues.resize(n, mDef); }
	void set(int idx, const T &v) {
		mValues[idx] = v;
	}
	int size() const { return int(mValues.size()); }
	void swapItems(int idx1, int idx2) {
		T tmp = mValues[idx1];
		mValues[idx1] = mValues[idx2];
		mValues[idx2] = tmp;
	}
	
	T* ptr() { return &mValues[0]; }
	const T* ptr() const { return &mValues[0]; }
	
	vector<T>& getValues() { return mValues; }
	const vector<T>& getValues() const { return mValues; }

protected:
	string mName;
	vector<T> mValues;
	T mDef;
};

class HemeshPropertySet {
public:

	HemeshPropertySet() {}
	~HemeshPropertySet() {
		clear();
	}
	
	void clear() {
		map<string, PropertyBase *>::iterator it = mProperties.begin();
		map<string, PropertyBase *>::iterator ite = mProperties.end();
		for(; it != ite; ++it) {
			delete it->second;
		}
		mProperties.clear();
	}
	
	template <typename T>
	Property<T> * add(const string &name, T def) {
		Property<T> *prop = new Property<T>(name);
		mProperties.insert(std::pair<string, PropertyBase*>(name, prop));
		prop->resize(size());
		return prop;
	}
	
	void extend() {
		map<string, PropertyBase *>::iterator it = mProperties.begin();
		map<string, PropertyBase *>::iterator ite = mProperties.end();
		for(; it != ite; ++it) {
			it->second->extend();
		}
	}
	
	void resize(int n) {
		map<string, PropertyBase *>::iterator it = mProperties.begin();
		map<string, PropertyBase *>::iterator ite = mProperties.end();
		for(; it != ite; ++it) {
			it->second->resize(n);
		}
	}
	
	// assumes arrays are always synchonized after an add operation
	int size() const {
		map<string, PropertyBase *>::const_iterator it = mProperties.begin();
		if(it == mProperties.end()) return 0;
		else return it->second->size();
	}
	
	void swapItems(int idx1, int idx2) {
		map<string, PropertyBase *>::iterator it = mProperties.begin();
		map<string, PropertyBase *>::iterator ite = mProperties.end();
		for(; it != ite; ++it) {
			it->second->swapItems(idx1, idx2);
		}
	}

protected:
	map<string, PropertyBase *> mProperties;
};