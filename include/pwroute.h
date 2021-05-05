#ifndef PWROUTE_H
#define PWROUTE_H

#include <set>
#include <phydb/phydb.h> 
#include "pwroute_component.h"
using namespace phydb;
namespace pwroute {

class Wire {
public:
    double coorX[2];
    double coorY[2];
    int numPathPoint;
    string layerName;
    string viaName;
    int width;

};

class PWGND {
public:
    int pitch;
    int meshWidth;
    int signalNetWidth;
    //string meshLayerName;
    //string routeLayerName;
    string hMeshLayerName;
    string vMeshLayerName;
    int lastHLayerID;
    string lastHLayerName;
    string upperBoundNet;
    string direction;

    vector<int> xMesh;
    vector<vector<int>> poweryMesh;
    vector<vector<int>> gndyMesh;
    
    vector<Wire> powerWires;
    vector<Wire> gndWires;
    
    vector<Range<int>> powerHighLayerY;
    vector<Range<int>> gndHighLayerY;

    map<int, set<int>> M2Fill;
    set<Point2D<int>> unusablePoints;
    set<Point2D<int>> powerM2Points;
    set<Point2D<int>> gndM2Points;
};


class PWRoute {
  private:
    
    //The following three parameters related to width and step of mesh are tunable
    int high_mesh_multiple_width = 8; // PW/GND wires are 8 times of the standard width on metal 6
    int high_mesh_multiple_step = 16; // PW/GND step are 16 times of the standard pitch on metal 6
    int cluster_mesh_multiple_width = 2; // mesh width is 2 times of standard wire width on metal2 
                                        // the actual used width is max(2 * metal2_width, length of M1-M2 via)

    //This power router assumes layer[0] == metal1
    //The following three parameters related to metal layers should not be changed
    //Reinforcement horizontal connection are on metal 6
    int high_mesh_layer = 10; // Metal 6
    int cluster_horizontal_layer = 6; //horizontal mesh between rows, metal 4
    int cluster_vertical_layer = 8; //vertical mesh between cols, metal 5

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

    void ComputeMinLength();
    void LinkTrackToLayer();
    void SetDefaultVia();
    void PreprocessComponents();
    void SNetConfig();
    void InitCluster();
    void RouteSNet();
    void RouteHighLayerMesh(string );
    void RouteLowLayerMesh(string );
    void MarkUnusablePoint();
    int SafeBoundaryDistance();
    bool InHighLayerViaRange(string signal, int ypos);
    bool NearHighLayerViaRange(string signal, int ypos, int& midRange);
    void MarkUnusablePointComp(PWRouteComponent& component);
    void FindRowSNet(string componentName, string pinName, Rect2D<double> rect, int& powerY, int& gndY);
    void DetailedRouteSNet();
    void DetailedRouteSNetComp(PWRouteComponent& component);
    bool M2DetailedRouteSNet(PWRouteComponent& component, string signal, int signalY, 
        vector<Wire>& Wires, Point2D<int>& powerPoint);
    void PlaceHighLayerVias(vector<Wire>& wires, int X, int Y, 
        string topLayerName, int viaIdx, int length, int viaDistance, int layerWidth);
    bool M1M3DetailedRouteSNet(PWRouteComponent& component, string signal, int signalY, 
        vector<Wire>& Wires, Point2D<int>& powerPoint);

    void findClosestTouchPoints(vector<Rect2D<double>>& rects, map<int, int>& closestPoint, 
        Track track, int expand, int signalY);
    void findFarthestTouchPoints(vector<Rect2D<double>>& rects, map<int, int>& farthestPoint, 
        Track track, int expand, int signalY);
    void findTouchPointsNoTrack(vector<Rect2D<double>>& rects, map<int, int>& closestPoint, int expand, int signalY);
    void findTouchPointsOBSNoTrack(vector<Rect2D<double>>& rects, const map<int, int>& closestPinPoint, 
        map<int, int>& closestOBSPoint, int expand, int signalY);
    bool M1DetailedRouteSNet(PWRouteComponent& component, string signal);
    void M2MetalFill(string signal);
    phydb::SNet* FindSNet(SignalUse);

  public: 
    PWRoute() {}
    PWRoute(phydb::PhyDB* p) : db_ptr_(p) {}

    void SetDBPtr(phydb::PhyDB* p);

    void SetMeshWidthStep(int, int, int);
    void RunPWRoute();
    void ExportToPhyDB();
};

}

#endif



