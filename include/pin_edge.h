#ifndef PIN_EDGE_H
#define PIN_EDGE_H

#include <phydb/phydb.h>
#include <iostream>
using namespace phydb;
namespace pwroute {

class VEdge { 
public:
    int x = -1;
    int y1 = -1; //y1 < y2
    int y2 = -1;

    VEdge() {};
    VEdge(int x_, int y1_, int y2_) : x(x_), y1(y1_), y2(y2_) {}

    bool operator > (const VEdge e) const {
        return x > e.x;
    }

    bool operator < (const VEdge e) const {
        return x < e.x;
    }

    friend std::ostream &operator<<(std::ostream &os, const VEdge &e);
};

class EdgeRange {
public: 
    std::map<int , int> points;
    
    EdgeRange() {}

    void Push(int start, int end, bool add);

    void MergeEdge(int x, std::string component_name); //returns merged edge, i.e. metal range

    void Subtract(int x, std::string component_name, std::map<int, int>,
        std::vector<VEdge>&, std::vector<VEdge>& );
};

class HEdge {
public:
    int x1 = -1; //x1 < x2
    int x2 = -1;
    int y = -1;

    HEdge() {};
    HEdge(int x1_, int x2_, int y_) : x1(x1_), x2(x2_), y(y_) {}

    bool operator > (const HEdge e) const {
        return y > e.y;
    }

    bool operator < (const HEdge e) const {
        return y < e.y;
    }
};

class PinEdge {
public:
    std::vector<VEdge> l_edges;
    std::vector<VEdge> r_edges;
    std::vector<HEdge> t_edges;
    std::vector<HEdge> b_edges; 

    PinEdge() {};
};




} // namespace pwroute


#endif