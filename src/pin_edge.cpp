#include "pwroute.h" 
#include "pin_edge.h"
#include <math.h>
#include <algorithm>
using namespace phydb;
namespace pwroute {

bool RangeOverlap(Range<int> a, Range<int> b) {

    if(a.begin <= b.begin && a.end <= b.begin)
        return false;
    else if(b.begin <= a.begin && b.end <= a.begin)
        return false;
    else
        return true;
}

void EdgeRange::Push(int start, int end, bool add) {//add or subtract to the range
    if(points.find(start) == points.end()) {
        if(add)
           points[start] = 1;
        else 
            points[start] = -1;
    }
    else 
        points[start] = points[start] + (add? 1 : -1);

    if(points.find(end) == points.end()) {
        if(add)
            points[end] = -1;
        else
            points[end] = 1;
    }
    else 
        points[end] = points[end] + (add? -1 : 1);

}

void EdgeRange::MergeEdge(int x, std::string component_name) {
    int edge_cnt = 0;
    int start = -1;
    int start_cnt = 0;
    std::map<int, int> merged_points;
    for(auto p : points) {
        if(p.second == 0) {
            continue;
        }
        else {
            if(edge_cnt == 0) {
                start = p.first;
                start_cnt = p.second;
                edge_cnt += p.second;
                continue;
            }
            edge_cnt += p.second;

            if(edge_cnt == 0) {
                merged_points[start] = start_cnt;
                merged_points[p.first] = -start_cnt;
            }
        }
    }
    points.clear();
    points = merged_points;
}

void EdgeRange::Subtract(int x, std::string component_name, std::map<int, int> prev_points, 
    std::vector<VEdge>& l_edges, std::vector<VEdge>& r_edges) {
    EdgeRange tmp_edge_range;
    tmp_edge_range.points = this->points;
    for(auto prev_p : prev_points) {
        tmp_edge_range.points[prev_p.first] -= prev_p.second; //subtract
    }
    tmp_edge_range.MergeEdge(x, component_name);

    std::vector<Range<int>> edges;
    int start = -1;
    bool left = true;
    for(auto p : tmp_edge_range.points) {
        if(start == -1) {
            start = p.first;
            if(p.second == 1)
                left = true;
            else
                left = false;
        }
        else if(left && p.second == -1) {
            l_edges.emplace_back(x, start, p.first);
            start = -1;
        }
        else if(!left && p.second == 1) {
            r_edges.emplace_back(x, start, p.first);
            start = -1;
        }
        else {
            std::cout << "ERROR: merge edge points error in subtract." << std::endl;
            for(auto print_p : tmp_edge_range.points) {
                std::cout << x << ": ( " << print_p.first << " , " << print_p.second << " ) " << std::endl;
            }
            exit(1);
        }
    }
}

bool PWRoute::SameSignalObsBlock(int touchX, int touchY, int h_extend, int v_extend, int M2_spacing, PinEdge& pin_edges) {

    int left_x = touchX - h_extend;
    int right_x = touchX + h_extend;
    int bot_y = touchY - v_extend;
    int top_y = touchY + v_extend;

    for(auto l_edge : pin_edges.l_edges) {
        Range<int> edge1(bot_y, top_y);
        Range<int> edge2(l_edge.y1, l_edge.y2);
        if(l_edge.x > right_x && l_edge.x < right_x + M2_spacing && RangeOverlap(edge1, edge2))
            return false;
    }

    for(auto r_edge : pin_edges.r_edges) {
        Range<int> edge1(bot_y, top_y);
        Range<int> edge2(r_edge.y1, r_edge.y2);
        if(r_edge.x < left_x && r_edge.x > left_x + M2_spacing && RangeOverlap(edge1, edge2))
            return false;
    }

    return true;

}

void PWRoute::ExtractPinEdge(std::string component_name, std::vector<Rect2D<double>>& pinRect, PinEdge& pin_edge) {
    std::vector<VEdge> l_edges;
    std::vector<VEdge> r_edges;
    std::set<int> x_pos;
    std::vector<int> x_vec;

    for(auto rect : pinRect) {
        l_edges.emplace_back((int) rect.ll.x, (int) rect.ll.y, (int) rect.ur.y);
        r_edges.emplace_back((int) rect.ur.x, (int) rect.ll.y, (int) rect.ur.y);
        x_pos.insert((int) rect.ll.x);
        x_pos.insert((int) rect.ur.x);
    }

    std::sort(l_edges.begin(), l_edges.end());
    std::sort(r_edges.begin(), r_edges.end());
    for(auto x : x_pos) {
        x_vec.push_back(x);
        /*if(component_name == "tst_aelem_50_6_as__10_act_acx0") {
            std::cout << "x: " << x << std::endl;
        }*/
    }
    int l_cnt = 0;
    int r_cnt = 0;
    EdgeRange l_edge_range;
    for(auto x : x_vec) {
        std::map<int, int> prev_points = l_edge_range.points;
        for(;r_cnt < r_edges.size();r_cnt++) {
            if(r_edges[r_cnt].x > x) {
                break;
            }
            else
                l_edge_range.Push(r_edges[r_cnt].y1, r_edges[r_cnt].y2, false);
        }


        for(;l_cnt < l_edges.size();l_cnt++) {
            if(l_edges[l_cnt].x > x) {
                break;
            }
            else
                l_edge_range.Push(l_edges[l_cnt].y1, l_edges[l_cnt].y2, true);
        }
        l_edge_range.MergeEdge(x, component_name);
        l_edge_range.Subtract(x, component_name, prev_points, pin_edge.l_edges, pin_edge.r_edges);
    }
}


std::ostream &operator<<(std::ostream &os, const VEdge &e) {
  os << e.x << " : (" << e.y1 << ", " << e.y2 << ") ";
  return os;
}


} //namespace pwroute