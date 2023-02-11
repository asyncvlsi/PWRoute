#ifndef PWROUTE_H
#define PWROUTE_H

#include <set>
#include <phydb/phydb.h> 
#include "pwroute_component.h"
#include "pin_edge.h"
using namespace phydb;

#if !defined(uint)
typedef unsigned int uint;
#endif

namespace pwroute {

enum VERBOSE {
    none = 0,
    info,
    debug
};

class Wire {
public:
    double coorX[2];
    double coorY[2];
    double ext[2];
    int numPathPoint;
    std::string layerName;
    std::string viaName;
    int width;
    Wire() {
        coorX[0] = -1;
        coorY[0] = -1;
        ext[0] = -1;
        coorX[1] = -1;
        coorY[1] = -1;
        ext[1] = -1;
	numPathPoint = 0;
	width = 0;
    }

};

class PWGND {
public:
    int hmeshWidth = -1;
    int hmeshExt = -1;
    int vmeshWidth = -1;
    int vmeshExt = -1;
    
    int vmeshPitch = -1;
    int hmeshPitch = -1;
    int signalNetWidth = -1;
    
    int m2_expanded_width = -1;//expand the width to the min width of vias, to avoid DRC
    int m3_expanded_width = -1;
    phydb::Range<int> m2_expanded_range; // distance threshold to expand the wire
    phydb::Range<int> m3_expanded_range;
    
    bool need_m2_minarea = true;
    bool same_signal_obs_reserve_ = false; // if M2 via width is smaller or equal to M2 width, no need to reserve
    
    std::string hMeshLayerName;
    std::string vMeshLayerName;
    int lastHLayerID;
    std::string lastHLayerName;
    std::string upperBoundNet;
    std::string direction;

    std::vector<int> xMesh;
    std::vector<std::vector<int>> poweryMesh;
    std::vector<std::vector<int>> gndyMesh;
    
    std::vector<Wire> powerWires;
    std::vector<Wire> gndWires;
    
    std::vector<Range<int>> powerHighLayerY;
    std::vector<Range<int>> gndHighLayerY;

    std::map<int, std::set<int>> M2Fill;
    std::set<Point2D<int>> unusablePoints;
    std::set<Point2D<int>> usedConnectionPoints;
    bool use_adj_mesh_point_flag_;
    std::set<Point2D<int>> powerM2Points;
    std::set<Point2D<int>> gndM2Points;
};


class PWRoute {
  private:
    
    //The following three parameters related to width and step of mesh are tunable
    uint high_mesh_multiple_width = 8; // PW/GND wires are 8 times of the standard width on metal 6
    uint high_mesh_multiple_step = 16; // PW/GND step are 16 times of the standard pitch on metal 6
    uint cluster_mesh_multiple_width = 2; // mesh width is 2 times of standard wire width on metal2 
                                        // the actual used width is max(2 * metal2_width, length of M1-M2 via)

    //This power router assumes layer[0] == metal1
    //The following three parameters related to metal layers should not be changed
    //Reinforcement horizontal connection are on metal 6
    uint high_mesh_layer = 10; // Metal 6
    bool use_reinforcement_ = true;
    uint cluster_vertical_layer = 8; //vertical mesh between cols, metal 5
    uint cluster_horizontal_layer = 6; //horizontal mesh between rows, metal 4
    uint detailed_route_layer = 4; //vertical connection, metal 3	

    phydb::PhyDB* db_ptr_;

    std::vector<double> layer_min_length_;
    std::map<int, int> layerid_2_trackid_;
    std::map<int, int> topLayerId_2_viaId_;

    std::vector<PWRouteComponent> components_;

    PWGND pwgnd_;

    int POWER_FOUND = 0;
    int POWER_UNFOUND = 0;
    int GROUND_FOUND = 0;
    int GROUND_UNFOUND = 0;

    int verbose_ = 0;

    void ComputeMinLength();
    void LinkTrackToLayer();
    void SetDefaultVia();
    void PreprocessComponents();
    void SNetConfig();
    void InitCluster();
    bool AdjMeshPointCheck(int, int, int);
    void AdjMeshPointFlag();
    void RouteSNet();
    void M3CloseViasPad();
    void RouteHighLayerReinforcement(std::string );
    void RouteLowLayerMesh(std::string );
    void MarkUnusablePoint();
    int SafeBoundaryDistance();
    bool InHighLayerViaRange(std::string signal, int ypos);
    bool NearHighLayerViaRange(std::string signal, int ypos, int& midRange);
    bool OverlapLowLayerVia(std::string signal, int i, int j);
    bool NearLowLayerViaRange(std::string signal, int i, int j, int& y_dst);
    void MarkUnusablePointComp(PWRouteComponent& component);
    void FindRowSNet(std::string componentName, std::string pinName, Rect2D<double> rect, int& powerY, int& gndY);
    void DetailedRouteSNet();
    void DetailedRouteSNetComp(PWRouteComponent& component);
    bool M2DetailedRouteSNet(PWRouteComponent& component, std::string signal, int signalY, 
        std::vector<Wire>& Wires, Point2D<int>& powerPoint);
    void PlaceHighLayerVias(std::vector<Wire>& wires, int X, int Y, 
        std::string topLayerName, int viaIdx, int length, int viaDistance);
    bool M1M3DetailedRouteSNet(PWRouteComponent& component, std::string signal, int signalY, 
        std::vector<Wire>& Wires, Point2D<int>& powerPoint);

    void findClosestTouchPoints(std::vector<Rect2D<double>>& rects, std::map<int, int>& closestPoint, 
        Track track, int expand, int signalY);
    void findFarthestTouchPoints(std::vector<Rect2D<double>>& rects, std::map<int, int>& farthestPoint, 
        Track track, int expand, int signalY);
    void findTouchPointsNoTrack(std::vector<Rect2D<double>>& rects, std::map<int, int>& closestPoint, int expand, int signalY);
    void findTouchPointsOBSNoTrack(std::vector<Rect2D<double>>& rects, const std::map<int, int>& closestPinPoint, 
        std::map<int, int>& closestOBSPoint, int expand, int signalY);
    bool M1DetailedRouteSNet(PWRouteComponent& component, std::string signal);
    void M2MetalFill(std::string signal);
    double FitGrid(double num, double manufacturingGrid);
    phydb::SNet* FindSNet(SignalUse);

    bool DetailedRouteCloseVia(int dist, phydb::Range<int> range);
    bool OutOfComponent(PWRouteComponent& , int, int, int );
    void ExtractPinEdge(std::string , std::vector<Rect2D<double>>& , PinEdge& );
    bool SameSignalObsBlock(int touchX, int touchY, int h_extend, int v_extend, int M2_spacing, PinEdge& pin_edges);
  public: 
    PWRoute() {}
    PWRoute(phydb::PhyDB* p) : db_ptr_(p){}
    PWRoute(phydb::PhyDB* p, int verb) : db_ptr_(p), verbose_(verb) {}

    void SetDBPtr(phydb::PhyDB* p);

    void SetMeshWidthStep(int, int, int);
    void SetReinforcement(bool ); 
    void RunPWRoute();
    void ExportToPhyDB();

};

}

#endif



