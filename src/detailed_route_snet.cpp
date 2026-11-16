#include "pwroute.h" 
#include "pin_edge.h"
#include <algorithm>
#include <phydb/common/logging.h>
#include <math.h>
#include <map>
using namespace phydb;
namespace pwroute {

bool PWRoute::PinCoversRect(const std::vector<Rect2D<double>> &pinRects, const Rect2D<double> &viaMetalRect) {
    std::vector<Rect2D<double>> uncovered{viaMetalRect};

    for (const auto &pinRect : pinRects) {
        std::vector<Rect2D<double>> nextUncovered;
        for (const auto &piece : uncovered) {
            double lx = std::max(piece.ll.x, pinRect.ll.x);
            double ly = std::max(piece.ll.y, pinRect.ll.y);
            double ux = std::min(piece.ur.x, pinRect.ur.x);
            double uy = std::min(piece.ur.y, pinRect.ur.y);

            if (lx >= ux || ly >= uy) {
                nextUncovered.push_back(piece);
                continue;
            }
            if (piece.ll.y < ly) {
                nextUncovered.emplace_back(piece.ll.x, piece.ll.y, piece.ur.x, ly);
            }
            if (uy < piece.ur.y) {
                nextUncovered.emplace_back(piece.ll.x, uy, piece.ur.x, piece.ur.y);
            }
            if (piece.ll.x < lx) {
                nextUncovered.emplace_back(piece.ll.x, ly, lx, uy);
            }
            if (ux < piece.ur.x) {
                nextUncovered.emplace_back(ux, ly, piece.ur.x, uy);
            }
        }
        uncovered.swap(nextUncovered);
        if (uncovered.empty()) {
            return true;
        }
    }
    return uncovered.empty();
}



bool PWRoute::M1DetailedRouteSNet(PWRouteComponent& component, std::string signal) {
    std::vector<Rect2D<double>> pinRect;
    std::vector<Rect2D<double>> M2pinRect;
    std::vector<Rect2D<double>> obsRect;
    
    auto layers = db_ptr_->GetTechPtr()->GetLayersRef();
    for(auto pin : component.pins_) {
        for(auto layerRect : pin.layer_rects_) {
            
            int layerIdx = db_ptr_->GetTechPtr()->GetLayerId(layerRect.layer_name_);
            if(layerIdx == 0) { //M1 
                for(auto rect : layerRect.rects_) {
                    if(pin.use_ == StrToSignalUse(signal))
                        pinRect.push_back(rect);
                    else
                        obsRect.push_back(rect);
                }
            }
            else if(layerIdx == 2) { //M2
                for(auto rect : layerRect.rects_) {
                    if(pin.use_ == StrToSignalUse(signal))
                        M2pinRect.push_back(rect);
                }
            }
        }
    }
    
    for(auto layerRect : component.obs_.GetLayerRects()) {
        int layerIdx = db_ptr_->GetTechPtr()->GetLayerId(layerRect.layer_name_);
        if(layerIdx == 0) { //M1
            for(auto rect : layerRect.rects_) {
                obsRect.push_back(rect);
            }
        }
    }

    if(pinRect.size() == 0 && M2pinRect.size() == 0) // no pin
        return true;
    else if(pinRect.size() == 0) // no pin on m1
        return false;
   
    return false;

}


void PWRoute::findTouchPointsOBSNoTrack(std::vector<Rect2D<double>>& rects, const std::map<int, int>& closestPinPoint, 
        std::map<int, int>& closestOBSPoint, int expand, int signalY) {
    for(auto rect : rects) {
        //inclusive track       
        int left = rect.ll.x - expand;
        int right = rect.ur.x + expand;
        
        int midY = (rect.ll.y + rect.ur.y) / 2;
        
        for(auto touchPoint : closestPinPoint) {
            int x = touchPoint.first;
            if(left <= x && right >= x) {
                if(closestOBSPoint.count(x) == 0) 
                    closestOBSPoint[x] = (midY < signalY)? rect.ur.y : rect.ll.y;
                else {
                    if(midY < signalY && closestOBSPoint[x] < rect.ur.y) {
                        closestOBSPoint[x] = rect.ur.y;   
                    }
                    else if(midY > signalY && closestOBSPoint[x] > rect.ll.y) {
                        closestOBSPoint[x] = rect.ll.y;    
                    }
                }
            }
        }
    }
}

void PWRoute::findTouchPointsNoTrack(std::vector<Rect2D<double>>& rects, std::map<int, int>& closestPoint, int expand, int signalY) {
    auto layers = db_ptr_->GetTechPtr()->GetLayersRef();
    int dbuPerMicron = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();
    int M3_pitch = layers[4].GetPitchX() * dbuPerMicron;
    for(auto rect : rects) {
        //inclusive track       
        /*int left = rect.ll.x - expand;
        int right = rect.ur.x + expand;*/

        int left = rect.ll.x + expand;
        int right = rect.ur.x - expand; //actually this is shrink
        
        int midY = (rect.ll.y + rect.ur.y) / 2;

        for(int x = left; x <= right; x += M3_pitch) {
            if(closestPoint.count(x) == 0) 
                closestPoint[x] = (midY < signalY)? rect.ur.y : rect.ll.y;
            else {
                if(midY < signalY && closestPoint[x] < rect.ur.y) {
                    closestPoint[x] = rect.ur.y;   
                }
                else if(midY > signalY && closestPoint[x] > rect.ll.y) {
                    closestPoint[x] = rect.ll.y;    
                }
            }
        }
    }
}

void PWRoute::findFarthestTouchPoints(std::vector<Rect2D<double>>& rects, std::map<int, int>& farthestPoint, 
        Track track, int expand, int signalY) {
    for(auto rect : rects) {
        //inclusive track       
        int left = (rect.ll.x - expand - track.GetStart()) / track.GetStep() + 1;
        int right = (rect.ur.x + expand - track.GetStart()) / track.GetStep();
        
        int midY = (rect.ll.y + rect.ur.y) / 2;

        for(int i = left; i <= right; i++) {
            if(farthestPoint.count(i) == 0) 
                farthestPoint[i] = (midY < signalY)? rect.ll.y : rect.ur.y;
            else {
                if(midY < signalY && farthestPoint[i] > rect.ll.y) {
                    farthestPoint[i] = rect.ll.y;   
                }
                else if(midY > signalY && farthestPoint[i] < rect.ur.y) {
                    farthestPoint[i] = rect.ur.y;    
                }
            }
        }
    }
}


void PWRoute::findClosestTouchPoints(std::vector<Rect2D<double>>& rects, std::map<int, int>& closestPoint, Track track, int expand, int signalY) {
    for(auto rect : rects) {
        //inclusive track       
        int left = (rect.ll.x - expand - track.GetStart()) / track.GetStep() + 1;
        int right = (rect.ur.x + expand - track.GetStart()) / track.GetStep();
        
        int midY = (rect.ll.y + rect.ur.y) / 2;
        

        for(int i = left; i <= right; i++) {
            if(closestPoint.count(i) == 0) 
                closestPoint[i] = (midY < signalY)? rect.ur.y : rect.ll.y;
            else {
                if(midY < signalY && closestPoint[i] < rect.ur.y) {
                    closestPoint[i] = rect.ur.y;   
                }
                else if(midY > signalY && closestPoint[i] > rect.ll.y) {
                    closestPoint[i] = rect.ll.y;    
                }
            }
        }
    }
}


/*
 * returns true if point(x, y) is not blocked by its neighbors
 */
bool PWRoute::AdjMeshPointCheck(int x, int y, int track_step) {
   if(pwgnd_.use_adj_mesh_point_flag_)
      return true;
   else {
      Point2D<int> right_point(x + track_step, y);
      Point2D<int> left_point(x - track_step, y);
      if(pwgnd_.usedConnectionPoints.find(left_point) != pwgnd_.usedConnectionPoints.end()
		      || pwgnd_.usedConnectionPoints.find(right_point) != pwgnd_.usedConnectionPoints.end())
         return false;
      else
	 return true;

   }
}

bool PWRoute::M1M3DetailedRouteSNet(PWRouteComponent& component, std::string signal, int signalY, 
        std::vector<Wire>& Wires, Point2D<int>& powerPoint) {
    
    std::vector<Rect2D<double>> M2obsRect;
    std::vector<Rect2D<double>> M3obsRect;
    std::vector<Rect2D<double>> M1obsRect;
    std::vector<Rect2D<double>> V1obsRect; 
    std::vector<Rect2D<double>> V2obsRect;
    std::vector<Rect2D<double>> pinRect;

    auto tracks = db_ptr_->GetDesignPtr()->GetTracksRef();
    auto layers = db_ptr_->GetTechPtr()->GetLayersRef();
    auto vias = db_ptr_->GetTechPtr()->GetLefViasRef();
    int dbuPerMicron = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();
    double manufacturing_grid = db_ptr_->GetTechPtr()->GetManufacturingGrid();

    for(auto pin : component.pins_) {
        for(auto layerRect : pin.layer_rects_) {
            int layerIdx = db_ptr_->GetTechPtr()->GetLayerId(layerRect.layer_name_);
            if(layerIdx == 2) { //M2 
                for(auto rect : layerRect.rects_) {
                    if(pin.use_ != StrToSignalUse(signal))
                        M2obsRect.push_back(rect);
                }
            }
            else if(layerIdx == 0) { //M1
                for(auto rect : layerRect.rects_) {
                    if(pin.use_ != StrToSignalUse(signal))
                        M1obsRect.push_back(rect);
                    else
                        pinRect.push_back(rect);
                }
            }
            else if(layerIdx == 1) { //V1
                for(auto rect : layerRect.rects_) {
                    V1obsRect.push_back(rect);// can be the same signal
                }
            }
            else if (layerIdx == 3) { // V2
                for (auto rect : layerRect.rects_) {
                    V2obsRect.push_back(rect);
                }
            }
            else if (layerIdx == 4) { // M3
                if (pin.use_ != StrToSignalUse(signal)) {
                    for (auto rect : layerRect.rects_) {
                        M3obsRect.push_back(rect);
                    }
                }
            }
        }
    }
    
    for(auto layerRect : component.obs_.GetLayerRects()) {
        int layerIdx = db_ptr_->GetTechPtr()->GetLayerId(layerRect.layer_name_);
        if(layerIdx == 2) { //M2 
            for(auto rect : layerRect.rects_) {
                M2obsRect.push_back(rect);
            }
        }
        else if(layerIdx == 0) { //M1
            for(auto rect : layerRect.rects_) {
                M1obsRect.push_back(rect);
            }
        }
        else if(layerIdx == 1) { //V1
            for(auto rect : layerRect.rects_) {
                V1obsRect.push_back(rect);
            }
        }
        else if (layerIdx == 3) { // V2
                for (auto rect : layerRect.rects_) {
                    V2obsRect.push_back(rect);
                }
        }
        else if (layerIdx == 4) { // M3
            for (auto rect : layerRect.rects_) {
                M3obsRect.push_back(rect);
            }
        }
    }
  

    int width = layers[4].GetWidth() * dbuPerMicron; //M3
    std::map<int, int> trackClosestPinPoint; //map trackIdx -> y
    std::map<int, int> trackFarthestPinPoint; 

    //map<int, int> touchPinPoint; //map x -> y 
    Track track = tracks[layerid_2_trackid_[4]]; //M3
    
    findClosestTouchPoints(pinRect, trackClosestPinPoint, track, width / 2, signalY);
    if(verbose_ >= debug && component.name_ == "v_50_6" && signal == "POWER") {
        for(auto p : trackClosestPinPoint) 
            std::cout << p.first << " " << p.second << std::endl;
    }
    
    //findTouchPointsNoTrack(pinRect, touchPinPoint, track, width / 2, signalY);
    findFarthestTouchPoints(pinRect, trackFarthestPinPoint, track, width / 2, signalY);
    if(verbose_ >= debug && component.name_ == "v_50_6" && signal == "POWER") {
	for(auto p : trackFarthestPinPoint) 
	    std::cout << p.first << " " << p.second << std::endl;
    }


    std::string M1_name = layers[0].GetName();
    int M1_width = layers[0].GetWidth() * dbuPerMicron;
    int M1_pitch = layers[0].GetPitchX() * dbuPerMicron;
    
    double M1_spacing, M2_spacing;
    {
       int viaIdx = topLayerId_2_viaId_[2];//M2
       auto via = vias[viaIdx];
       auto viaRect = via.GetLayerRectsRef()[0].rects_[0]; //M1
       double h = (viaRect.ur.x - viaRect.ll.x);
       double v = (viaRect.ur.y - viaRect.ll.y);
       double viaWidth = (h > v)? v * dbuPerMicron : h * dbuPerMicron;
       double viaLength = (h > v)? h * dbuPerMicron : v * dbuPerMicron;
    
       auto M1_spacing_table = layers[0].GetSpacingTable();
       double spacing1 = M1_spacing_table->GetSpacingFor(h, v);
       double spacing2 = M1_spacing_table->GetSpacingFor(v, h);
       M1_spacing = (double) std::max(spacing1, spacing2) * (double) dbuPerMicron;
    }


    // int V1_width = layers[1].GetWidth() * dbuPerMicron;
    //go through each cut now that we have multiple ones, compute if blocked
    int V1_spacing = layers[1].GetSpacing() * dbuPerMicron;
    int V2_spacing = layers[3].GetSpacing() * dbuPerMicron;
    auto viaCutsBlocked = [dbuPerMicron](
        const LayerRect &cutLayer,
        const Point2D<double> &origin,
        const std::vector<Rect2D<double>> &obstacles,
        int spacing) {

        for (const auto &cut : cutLayer.rects_) {
            double lx = origin.x + cut.ll.x * dbuPerMicron;
            double ly = origin.y + cut.ll.y * dbuPerMicron;
            double ux = origin.x + cut.ur.x * dbuPerMicron;
            double uy = origin.y + cut.ur.y * dbuPerMicron;

            for (const auto &obs : obstacles) {
                if (lx < obs.ur.x + spacing &&
                    ux > obs.ll.x - spacing &&
                    ly < obs.ur.y + spacing &&
                    uy > obs.ll.y - spacing) {
                    return true;
                }
            }
        }
        return false;
    };
    //check via M2 obstancles
    auto generatedM2Blocked = [this](
    const Rect2D<double> &candidate, double spacing) -> bool {

        auto blockedBy = [&](const std::vector<Rect2D<double>> &obstacles) {
            for (const auto &obs : obstacles) {
                if (candidate.ll.x < obs.ur.x + spacing &&
                    candidate.ur.x > obs.ll.x - spacing &&
                    candidate.ll.y < obs.ur.y + spacing &&
                    candidate.ur.y > obs.ll.y - spacing) {
                    return true;
                }
            }
            return false;
        };
        return blockedBy(pwgnd_.powerM2Rects) || blockedBy(pwgnd_.gndM2Rects);
    };

    auto viaM2Blocked = [this, &M2obsRect, &layers, &generatedM2Blocked, dbuPerMicron](
    const LayerRect &metalLayer,
    const Point2D<double> &origin) {

    auto spacingTable = layers[2].GetSpacingTable();

    for (const auto &metal : metalLayer.rects_) {
        double h = metal.ur.x - metal.ll.x;
        double v = metal.ur.y - metal.ll.y;

        double spacing1 = spacingTable->GetSpacingFor(h, v);
        double spacing2 = spacingTable->GetSpacingFor(v, h);
        double spacing = std::max(spacing1, spacing2) * dbuPerMicron;

        double lx = origin.x + metal.ll.x * dbuPerMicron;
        double ly = origin.y + metal.ll.y * dbuPerMicron;
        double ux = origin.x + metal.ur.x * dbuPerMicron;
        double uy = origin.y + metal.ur.y * dbuPerMicron;

        //check cell obstacles AND obstacles from placed vias
        auto blockedBy = [&](const std::vector<Rect2D<double>> &obstacles) {
            for (const auto &obs : obstacles) {
                if (lx < obs.ur.x + spacing &&
                    ux > obs.ll.x - spacing &&
                    ly < obs.ur.y + spacing &&
                    uy > obs.ll.y - spacing) {
                    return true;
                }
            }
            return false;
        };
        if (blockedBy(M2obsRect)) {
            return true;
        }
        Rect2D<double> candidate(lx, ly, ux, uy);
        if (generatedM2Blocked(candidate, spacing)) {
            if (verbose_ >= debug) {
                std::cout << "Generated M2 obstruction at ("
                          << origin.x << ", " << origin.y
                          << ") DBU" << std::endl;
            }
            return true;
        }

    }
    return false;
};

    //read rects collected and figure out if they block
    auto generatedM3Blocked = [&](const Rect2D<double> &candidate, double candidateSpacing) -> bool {
        auto spacingTable = layers[4].GetSpacingTable();

        auto blockedBy = [&](const std::vector<Rect2D<double>> &obstacles, bool sameSupply) -> bool {
            for (const auto &obs : obstacles) {
                double overlapX = std::min(candidate.ur.x, obs.ur.x) - std::max(candidate.ll.x, obs.ll.x);
                double overlapY = std::min(candidate.ur.y, obs.ur.y) - std::max(candidate.ll.y, obs.ll.y);

                bool touches = (overlapX > 0 && overlapY >= 0) || (overlapY > 0 && overlapX >= 0);

                if (sameSupply && touches) {
                    continue;
                }

                double h = (obs.ur.x - obs.ll.x) / dbuPerMicron;
                double v = (obs.ur.y - obs.ll.y) / dbuPerMicron;
                double obstacleSpacing = std::max(
                    spacingTable->GetSpacingFor(h, v),
                    spacingTable->GetSpacingFor(v, h)) * dbuPerMicron;
                double spacing = std::max(candidateSpacing, obstacleSpacing);

                if (candidate.ll.x < obs.ur.x + spacing &&
                    candidate.ur.x > obs.ll.x - spacing &&
                    candidate.ll.y < obs.ur.y + spacing &&
                    candidate.ur.y > obs.ll.y - spacing) {
                    if (sameSupply) { 
                        Rect2D<double> nearRegion(
                            std::max(candidate.ll.x, obs.ll.x - spacing),
                            std::max(candidate.ll.y, obs.ll.y - spacing),
                            std::min(candidate.ur.x, obs.ur.x + spacing),
                            std::min(candidate.ur.y, obs.ur.y + spacing));

                        if (PinCoversRect(obstacles, nearRegion)) {
                            continue;
                        }
                    }
                    if (verbose_ >= debug) {
                        std::cout << "Generated M3 obstruction: supply: "
                                  << signal << " candidate: ("
                                  << candidate.ll.x << ", " << candidate.ll.y
                                  << ")(" << candidate.ur.x << ", "
                                  << candidate.ur.y << ") DBU"
                                  << " obstacle: ("
                                  << obs.ll.x << ", " << obs.ll.y
                                  << ")(" << obs.ur.x << ", " << obs.ur.y << ") DBU"
                                  << " sameSupply: " << sameSupply
                                  << " spacing: " << spacing << " DBU"
                                  << " overlapX: " << overlapX
                                  << " overlapY: " << overlapY
                                  << std::endl;
                    }
                    return true;
                }
            }
            return false;
        };
        return blockedBy(pwgnd_.powerM3Rects, signal == "POWER") ||
               blockedBy(pwgnd_.gndM3Rects, signal == "GROUND");
    };

    auto viaM3Blocked = [&M3obsRect, &layers, &generatedM3Blocked, dbuPerMicron](
        const LayerRect &metalLayer,
        const Point2D<double> &origin) {

        auto spacingTable = layers[4].GetSpacingTable();

        for (const auto &metal : metalLayer.rects_) {
            double h = metal.ur.x - metal.ll.x;
            double v = metal.ur.y - metal.ll.y;

            double spacing1 = spacingTable->GetSpacingFor(h, v);
            double spacing2 = spacingTable->GetSpacingFor(v, h);
            double spacing = std::max(spacing1, spacing2) * dbuPerMicron;

            double lx = origin.x + metal.ll.x * dbuPerMicron;
            double ly = origin.y + metal.ll.y * dbuPerMicron;
            double ux = origin.x + metal.ur.x * dbuPerMicron;
            double uy = origin.y + metal.ur.y * dbuPerMicron;

            for (const auto &obs : M3obsRect) {
                if (lx < obs.ur.x + spacing &&
                    ux > obs.ll.x - spacing &&
                    ly < obs.ur.y + spacing &&
                    uy > obs.ll.y - spacing) {
                    return true;
                }
            }
            Rect2D<double> candidate(lx, ly, ux, uy);
            if (generatedM3Blocked(candidate, spacing)) {
                return true;
            }
        }
        return false;
    };

    auto M2MetalHasMinArea = [&layers](const LayerRect &metalLayer) {
        const double minArea = layers[2].GetArea();

        for (const auto &metal : metalLayer.rects_) {
            double width = metal.ur.x - metal.ll.x;
            double height = metal.ur.y - metal.ll.y;

            if (width * height >= minArea) {
                return true;
            }
        }
        return false;
    };

 
    //use the correct frame now that vias may be centered
    auto M2LandingsOverlap = [dbuPerMicron](
        const LayerRect &a, const LayerRect &b,
        const Point2D<double> &centerOffset) {

        if (a.rects_.size() != 1 || b.rects_.size() != 1) {
            return false;
        }

        const auto &ra = a.rects_[0];
        const auto &rb = b.rects_[0];

        double dx = centerOffset.x / dbuPerMicron;
        double dy = centerOffset.y / dbuPerMicron;

        return ra.ll.x - dx < rb.ur.x &&
               ra.ur.x - dx > rb.ll.x &&
               ra.ll.y - dy < rb.ur.y &&
               ra.ur.y - dy > rb.ll.y;
    };

    int M2_minlength = layer_min_length_[2] * dbuPerMicron;
    int M2_pitch = layers[2].GetPitchX() * dbuPerMicron;
    int M2_width = layers[2].GetWidth() * dbuPerMicron;
    const int M2thinHalfLength = (int) FitGrid( M2_minlength / 2, (double) dbuPerMicron * manufacturing_grid);

    {
       int viaIdx = topLayerId_2_viaId_[detailed_route_layer];//M3
       auto via = vias[viaIdx];
       auto viaRect = via.GetLayerRectsRef()[0].rects_[0]; //M2
       double h = (viaRect.ur.x - viaRect.ll.x);
       double v = (viaRect.ur.y - viaRect.ll.y);
       double viaWidth = (h > v)? v * dbuPerMicron : h * dbuPerMicron;
       double viaLength = (h > v)? h * dbuPerMicron : v * dbuPerMicron;
    
       auto M2_spacing_table = layers[2].GetSpacingTable();
       double spacing1 = M2_spacing_table->GetSpacingFor(h, v);
       double spacing2 = M2_spacing_table->GetSpacingFor(v, h);
       M2_spacing = (double) std::max(spacing1, spacing2) * (double) dbuPerMicron;
      if (verbose_ >= debug) {
           std::cout << "M2 pad spacing: "
                     << M2_spacing / dbuPerMicron
                     << " um" << std::endl;
       }

    }
        
    int M3_width = layers[4].GetWidth() * dbuPerMicron;
    int M3_pitch = layers[4].GetPitchX() * dbuPerMicron;
    int M3_minlength = layer_min_length_[4] * dbuPerMicron;
    auto getZRouteOffset = [&](double x, double y) -> int {
        bool usesZRoute = signal == "GROUND" && x == powerPoint.x &&
                    ((y < signalY && powerPoint.y > y) ||
                     (y > signalY && powerPoint.y < y));

        if (!usesZRoute) {
            return 0;
        }

        const auto &xMesh = pwgnd_.xMesh;
        bool moveRight = false;

        for (std::size_t i = 0; i + 1 < xMesh.size(); ++i) {
            if (x > xMesh[i] && x < (xMesh[i] + xMesh[i + 1]) / 2) {
                moveRight = true;
                break;
            }
            else if (x >= (xMesh[i] + xMesh[i + 1]) / 2 && x < xMesh[i + 1]) {
                break;
            }
        }
        return moveRight ? M3_pitch : -M3_pitch;
    };
    auto makeM3Wire = [&](double x, double y) -> Wire {
        Wire wire;

        double wireX = x + getZRouteOffset(x, y);
        wire.coorX[0] = wireX;
        wire.coorX[1] = wireX;
        wire.coorY[0] = y;

        if (fabs(signalY - y) >= M3_minlength) {
            wire.coorY[1] = signalY;
        }
        else {
            wire.coorY[1] = (signalY > y)
                ? y + M3_minlength
                : y - M3_minlength;
        }
        wire.numPathPoint = 2;
        wire.layerName = layers[4].GetName();
        if (DetailedRouteCloseVia(abs(signalY - y), pwgnd_.m3_expanded_range)) {
            wire.width = std::max(pwgnd_.m3_expanded_width, M3_width);
        }
        else {
            wire.width = M3_width;
        }
        return wire;
    };
    
    auto makeM3MinAreaWire = [&](double x, double y) -> Wire {
        Wire wire;
        wire.coorX[0] = x;
        wire.coorX[1] = x;
        wire.coorY[0] = signalY;
        wire.coorY[1] = (signalY < y) ? signalY + M3_minlength : signalY - M3_minlength;
        wire.width = M3_width;
        wire.layerName = layers[4].GetName();
        wire.numPathPoint = 2;
        return wire;
    };

    auto getM3WireRect = [](const Wire &wire) -> Rect2D<double> {
        double halfWidth = wire.width / 2.0;

        return Rect2D<double>(
            wire.coorX[0] - halfWidth,
            std::min(wire.coorY[0], wire.coorY[1]) - halfWidth,
            wire.coorX[0] + halfWidth,
            std::max(wire.coorY[0], wire.coorY[1]) + halfWidth);
    };
    
    auto getM2WireRect = [](const Wire &wire) -> Rect2D<double> {
        double halfWidth = wire.width / 2.0;

        return Rect2D<double>(
            std::min(wire.coorX[0], wire.coorX[1]) - halfWidth,
            wire.coorY[0] - halfWidth,
            std::max(wire.coorX[0], wire.coorX[1]) + halfWidth,
            wire.coorY[0] + halfWidth);
    };



    
    bool foundM1M3 = false;
    double touchX, touchY;
    std::string M1M2ViaName;
    int M1M2ViaIdx = -1;
    Point2D<double> M1M2ViaOrigin;
    int M2M3ViaIdx = -1;
    Point2D<double> M2M3ViaOrigin;
    std::size_t M1M2ViaCutCount = 0;
    bool M1M2HasM2MinArea = false;
    bool needM2Pad = pwgnd_.need_m2_minarea;
    int M1M2ViaLength;
    int M2wideHalfLength = 0;
   
    bool M2thinFail = false;
    bool M2wideFail = false;
    auto makeM2PadWire = [&](double x, double y, bool wide) -> Wire {
        Wire wire;
        int halfLength = wide ? M2wideHalfLength : M2thinHalfLength;

        wire.coorX[0] = x - halfLength;
        wire.coorX[1] = x + halfLength;
        wire.coorY[0] = y;
        wire.coorY[1] = y;
        wire.width = wide ? 3 * M2_width : M2_width;
        wire.layerName = layers[2].GetName();
        wire.numPathPoint = 2;
        return wire;
    };
    auto makeM2BridgeWire = [&](double x, double y, int zOffset) -> Wire {
        Wire wire;
        wire.coorX[0] = x - M2_width / 2;
        wire.coorX[1] = x + zOffset + M2_width / 2;
        wire.coorY[0] = y;
        wire.coorY[1] = y;
        wire.numPathPoint = 2;
        wire.layerName = layers[2].GetName();

        if (DetailedRouteCloseVia(abs(signalY - y), pwgnd_.m2_expanded_range)) {
            wire.width = std::max(pwgnd_.m2_expanded_width, M2_width);
        }
        else {
            wire.width = M2_width;
        }
        return wire;
    };
    auto m2WireBlocked = [&](const Wire &wire) -> bool {
        Rect2D<double> candidate = getM2WireRect(wire);
        double h = (candidate.ur.x - candidate.ll.x) / dbuPerMicron;
        double v = (candidate.ur.y - candidate.ll.y) / dbuPerMicron;
        auto spacingTable = layers[2].GetSpacingTable();

        double spacing = std::max(
            spacingTable->GetSpacingFor(h, v),
            spacingTable->GetSpacingFor(v, h)) * dbuPerMicron;

        return generatedM2Blocked(candidate, spacing);
    };
    //helper function for finding M2-M3 via
    const int meshViaIdx = topLayerId_2_viaId_[cluster_horizontal_layer];
    auto findM2M3Via = [&](double x, double y) -> bool {
        M2M3ViaIdx = -1;
        int zOffset = getZRouteOffset(x, y);
                if (zOffset != 0 && m2WireBlocked(makeM2BridgeWire(x, y, zOffset))) {
            return false;
        }
        Point2D<double> anchor(x + zOffset, y);
        Point2D<double> meshOrigin(anchor.x, signalY);
        const auto &meshMetal = vias[meshViaIdx].GetLayerRectsRef()[0];
        if (viaM3Blocked(meshMetal, meshOrigin)) {
            return false;
        }
        const auto &lowerMetal = vias[M1M2ViaIdx].GetLayerRectsRef()[2];
        Wire m3Wire = makeM3Wire(x, y);
        Rect2D<double> m3WireRect = getM3WireRect(m3Wire);
        auto m3SpacingTable = layers[4].GetSpacingTable();
        double wireH = m3Wire.width / (double) dbuPerMicron;
        double wireV = (m3WireRect.ur.y - m3WireRect.ll.y) / dbuPerMicron;

        double wireSpacing = std::max( m3SpacingTable->GetSpacingFor(wireH, wireV),
                                       m3SpacingTable->GetSpacingFor(wireV, wireH)) * dbuPerMicron;
        if (generatedM3Blocked(m3WireRect, wireSpacing)) {
            return false;
        }
        for (const auto &obs : M3obsRect) {
            if (m3WireRect.ll.x < obs.ur.x + wireSpacing &&
                m3WireRect.ur.x > obs.ll.x - wireSpacing &&
                m3WireRect.ll.y < obs.ur.y + wireSpacing &&
                m3WireRect.ur.y > obs.ll.y - wireSpacing) {
                return false;
            }
        }
        //wire obstacle check
        if (fabs(y - signalY) < M3_minlength) {
            Wire extraWire = makeM3MinAreaWire(x, y);
            Rect2D<double> extraRect = getM3WireRect(extraWire);

            double extraH = extraWire.width / (double) dbuPerMicron;
            double extraV = (extraRect.ur.y - extraRect.ll.y) / dbuPerMicron;

            double extraSpacing = std::max(
                m3SpacingTable->GetSpacingFor(extraH, extraV),
                m3SpacingTable->GetSpacingFor(extraV, extraH)
            ) * dbuPerMicron;

            if (generatedM3Blocked(extraRect, extraSpacing)) {
                return false;
            }

            for (const auto &obs : M3obsRect) {
                if (extraRect.ll.x < obs.ur.x + extraSpacing &&
                    extraRect.ur.x > obs.ll.x - extraSpacing &&
                    extraRect.ll.y < obs.ur.y + extraSpacing &&
                    extraRect.ur.y > obs.ll.y - extraSpacing) {
                    return false;
                }
            }
        }
        for (int viaIdx : layerPair_2_viaIds_.at(std::make_pair(2, 4))) {
            const auto &shapes = vias[viaIdx].GetLayerRectsRef();
            const auto cutCount = shapes[1].rects_.size();
            if (cutCount < static_cast<std::size_t>(m2_m3_min_cuts_) || (m2_m3_max_cuts_ != 0 &&
                 cutCount > static_cast<std::size_t>(m2_m3_max_cuts_))) {
                continue;
            }
            if (shapes[0].rects_.empty() ||
                shapes[1].rects_.empty() ||
                shapes[2].rects_.empty()) {
                continue;
            }
            Point2D<double> origin = anchor;
            if (center_vias_) {
                Point2D<double> center;
                if (!GetViaCutCenterOffset(shapes[1], center)) {
                    continue;
                }
                double dx = center.x * dbuPerMicron;
                double dy = center.y * dbuPerMicron;
                double grid = manufacturing_grid * dbuPerMicron;

                bool onDBU = fabs(dx - round(dx)) < 1e-6 && fabs(dy - round(dy)) < 1e-6;
                bool onGrid = grid <= 0 || (fabs(dx / grid - round(dx / grid)) < 1e-6
                                        && fabs(dy / grid - round(dy / grid)) < 1e-6);
                if (!onDBU || !onGrid) {
                    continue;
                }
                origin.x -= round(dx);
                origin.y -= round(dy);
            }
            if (viaCutsBlocked(shapes[1], origin, V2obsRect, V2_spacing)) {
                continue;
            }
            const auto &upperMetal = shapes[0];
            if (viaM2Blocked(upperMetal, origin)) {
                continue;
            }
            if (viaM3Blocked(shapes[2], origin)) {
                continue;
            }
            //at least one M3 rectangle of this candidate should contact the wire
            bool touchesM3Wire = false;
            for (const auto &metal : shapes[2].rects_) {
                double lx = origin.x + metal.ll.x * dbuPerMicron;
                double ly = origin.y + metal.ll.y * dbuPerMicron;
                double ux = origin.x + metal.ur.x * dbuPerMicron;
                double uy = origin.y + metal.ur.y * dbuPerMicron;

                double overlapX = std::min(ux, m3WireRect.ur.x)
                                - std::max(lx, m3WireRect.ll.x);
                double overlapY = std::min(uy, m3WireRect.ur.y)
                                - std::max(ly, m3WireRect.ll.y);

                if ((overlapX > 0 && overlapY >= 0) ||
                    (overlapY > 0 && overlapX >= 0)) {
                    touchesM3Wire = true;
                    break;
                }
            }
        if (!touchesM3Wire) {
            continue;
        }
            Point2D<double> originDelta( origin.x - M1M2ViaOrigin.x, origin.y - M1M2ViaOrigin.y);
            bool connectedM2 = zOffset == 0 && M2LandingsOverlap(lowerMetal, upperMetal, originDelta);
            bool candidateNeedsPad = !M2MetalHasMinArea(upperMetal) && !(M1M2HasM2MinArea && connectedM2);
            if (candidateNeedsPad && M2thinFail && M2wideFail) {
                continue;
            }
            M2M3ViaIdx = viaIdx;
            M2M3ViaOrigin = origin;
            needM2Pad = candidateNeedsPad;
            return true;
        }
        return false;
    };
    for(auto pinPoint : trackClosestPinPoint) {
        int trackIdx = pinPoint.first;
        touchX = track.GetStart() + trackIdx * track.GetStep();
        touchY = pinPoint.second;
        
        //move touchY futher from signalY by width/2
        // if(touchY < signalY)
        //     touchY -= M3_width / 2;
        // else
        //     touchY += M3_width / 2;

        Point2D<double> touchPoint(touchX, touchY);
        bool M1cover = false;
        bool M2cover = false;
        bool V1cover = false; 
        
        // for(int viaIdx = topLayerId_2_viaId_[2]; viaIdx < topLayerId_2_viaId_[4]; viaIdx++) { //vias whose top layer is M2
        M1M2ViaName.clear();//no via has been selected
        for (int viaIdx : layerPair_2_viaIds_.at(std::make_pair(0, 2))) {

            const auto cutCount = vias[viaIdx].GetLayerRectsRef()[1].rects_.size();
            if (cutCount < static_cast<std::size_t>(m1_m2_min_cuts_) || (m1_m2_max_cuts_ != 0 &&
                 cutCount > static_cast<std::size_t>(m1_m2_max_cuts_))) {
                continue;
            }

            auto via = vias[viaIdx];
            Point2D<double> centerOffset(0, 0);

            if (center_vias_) {
                Point2D<double> centerMicrons;

                if (!GetViaCutCenterOffset(
                        via.GetLayerRectsRef()[1], centerMicrons)) {
                    continue;
                }
                double dx = centerMicrons.x * dbuPerMicron;
                double dy = centerMicrons.y * dbuPerMicron;
                double grid = manufacturing_grid * dbuPerMicron;

                bool onDBU = fabs(dx - round(dx)) < 1e-6 && fabs(dy - round(dy)) < 1e-6;

                bool onGrid = grid <= 0 ||
                    (fabs(dx / grid - round(dx / grid)) < 1e-6 &&
                     fabs(dy / grid - round(dy / grid)) < 1e-6);

                if (!onDBU || !onGrid) {
                    if (verbose_ >= debug) {
                        std::cout << "Via center cannot be represented "
                                  << "on the placement grid: "
                                  << via.GetName() << std::endl;
                    }
                    continue;
                }
                centerOffset.x = round(dx);
                centerOffset.y = round(dy);
            }
            touchPoint.x = touchX - centerOffset.x;

            bool M1pinCovered = true;
            int llx = -1, lly = -1, urx = -1, ury = -1;
            M1cover = false;
            for(auto layerRect : via.GetLayerRectsRef()) {
                if(layerRect.layer_name_ == M1_name) {
                    llx = layerRect.rects_[0].ll.x * dbuPerMicron;        
                    lly = layerRect.rects_[0].ll.y * dbuPerMicron;        
                    urx = layerRect.rects_[0].ur.x * dbuPerMicron;        
                    ury = layerRect.rects_[0].ur.y * dbuPerMicron;        
                  
                    //lowest and highest M1 offsets from origin
                    double lowerY = layerRect.rects_[0].ll.y * dbuPerMicron;
                    double upperY = layerRect.rects_[0].ur.y * dbuPerMicron;
                    for (const auto &metalRect : layerRect.rects_) {
                        lowerY = std::min(lowerY, metalRect.ll.y * dbuPerMicron);
                        upperY = std::max(upperY, metalRect.ur.y * dbuPerMicron);
                    }
                    if (pinPoint.second < signalY) {
                        touchPoint.y = pinPoint.second - upperY;
                    } else {
                        touchPoint.y = pinPoint.second - lowerY;
                    }
                    for (const auto &metalRect : layerRect.rects_) {
                        Rect2D<double> viaMetalRect(
                            touchPoint.x + metalRect.ll.x * dbuPerMicron,
                            touchPoint.y + metalRect.ll.y * dbuPerMicron,
                            touchPoint.x + metalRect.ur.x * dbuPerMicron,
                            touchPoint.y + metalRect.ur.y * dbuPerMicron);
                        bool covered = PinCoversRect(pinRect, viaMetalRect);
                        if (!covered) {
                            M1pinCovered = false;
                        }
                    }
                }
            }
            if(llx == -1 || lly == -1 || urx == -1 || ury == -1) {
                std::cout << "ERROR: unable to find M1-M2 via." << std::endl;
                exit(1);
            }
            
            for(auto rect : M1obsRect) {
                auto tmpRect = rect;
                tmpRect.ll.x -= (urx + M1_spacing); 
                tmpRect.ur.x += (- llx + M1_spacing); 
                tmpRect.ll.y -= (ury + M1_spacing);
                tmpRect.ur.y += (- lly + M1_spacing);
            
                if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
                    M1cover = true;
                    break;
                }
            }

            if (M1cover || !M1pinCovered) {
                continue;
            }
            //check cut blocks before rejecting or accepting via candidate
            V1cover = viaCutsBlocked(via.GetLayerRectsRef()[1], touchPoint, V1obsRect, V1_spacing);
            if (verbose_ >= debug && V1cover) {
                std::cout << "V1 cut obstruction: " << via.GetName()
                          << " at (" << touchPoint.x
                          << ", " << touchPoint.y << ") DBU"
                          << std::endl;
            }
            if (V1cover) {
                continue;
            }
            bool M2viaBlocked = viaM2Blocked(via.GetLayerRectsRef()[2], touchPoint);
            if (verbose_ >= debug && M2viaBlocked) {
                std::cout << "M2 via obstruction: " << via.GetName()
                          << " instance: " << component.name_
                          << " supply: " << signal
                          << " at (" << touchPoint.x
                          << ", " << touchPoint.y << ") DBU"
                          << std::endl;
            }
            if (!M1cover && !V1cover && M1pinCovered && !M2viaBlocked) {
                M1M2ViaName = via.GetName();
                M1M2ViaIdx = viaIdx;
                M1M2ViaOrigin = touchPoint;
                touchPoint.x = touchX;
                touchPoint.y += centerOffset.y;
                M1M2HasM2MinArea = M2MetalHasMinArea(via.GetLayerRectsRef()[2]);
                M1M2ViaCutCount = via.GetLayerRectsRef()[1].rects_.size();
                M1M2ViaLength=ury-lly;
                touchY = touchPoint.y;
                break;
            }
        } // find m1-m2 via that doesn't conflict with m1 obs
        if (M1M2ViaName.empty()) {
            continue;
        }
        M2wideHalfLength = std::max(M2_width / 2, M1M2ViaLength / 2);
        M2wideHalfLength = std::max(M2wideHalfLength, M2_minlength / 6);
        M2wideHalfLength = (int) FitGrid( M2wideHalfLength, (double) dbuPerMicron * manufacturing_grid);

        M2wideFail= false;
        M2thinFail = false;
        for(auto rect : M2obsRect) {
            auto tmpRect = rect;
            tmpRect.ll.x -= (M2thinHalfLength + M2_spacing);
            tmpRect.ur.x += (M2thinHalfLength + M2_spacing);
            tmpRect.ll.y -= (M2_width / 2 + M2_spacing);
            tmpRect.ur.y += (M2_width / 2 + M2_spacing);
            
            if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
                M2thinFail = true;
                break;
            }
        } // m2 side min area doesn't conflict with m2 obs
        if(touchPoint.x - M2thinHalfLength - M2_spacing / 2 < component.location_.x ||
                touchPoint.x + M2thinHalfLength + M2_spacing / 2 > component.location_.x + component.size_.x ||
                    touchPoint.y - M2_width / 2 - M2_spacing / 2 < component.location_.y || 
                    touchPoint.y + M2_width / 2 + M2_spacing / 2 > component.location_.y + component.size_.y) 
            M2thinFail = true;

        for(auto rect : M2obsRect) {
            auto tmpRect = rect;
            tmpRect.ll.x -= (M2wideHalfLength + M2_spacing); 
            tmpRect.ur.x += (M2wideHalfLength + M2_spacing); 
            tmpRect.ll.y -= (M2_width * 3 / 2 + M2_spacing);
            tmpRect.ur.y += (M2_width * 3 / 2 + M2_spacing);
            
            if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
                M2wideFail = true;
                break;
            }
        }
        if(touchPoint.x - M2wideHalfLength - M2_spacing / 2 < component.location_.x || 
                touchPoint.x + M2wideHalfLength + M2_spacing / 2 > component.location_.x + component.size_.x ||
                    touchPoint.y - M2_width * 3 / 2 - M2_spacing / 2 < component.location_.y || 
                    touchPoint.y + M2_width * 3 / 2 + M2_spacing / 2 > component.location_.y + component.size_.y) 
            M2wideFail = true;

        M2thinFail = M2thinFail || m2WireBlocked(makeM2PadWire(touchX, touchY, false));
        M2wideFail = M2wideFail || m2WireBlocked(makeM2PadWire(touchX, touchY, true));
        M2cover = M2wideFail && M2thinFail;

        
        if(verbose_ > 0 && component.name_ == "cx1") {
            std::cout << signal << ":" << std::endl;
            std::cout << "close M2cover : " << M2cover << " M2wideFail: " << M2wideFail << " M2thinFail: " << M2thinFail << std::endl;
            std::cout << "touchpoint x : " << touchPoint.x << " " << touchPoint.y;
            std::cout << " M2minlength/2: " << M2_minlength / 2 << " component.x: " << component.location_.x << std::endl; 
        }
 
        // this is now checked while we are choosing a via candidate
        // for(auto rect : V1obsRect) {
        //     auto tmpRect = rect;
        //     tmpRect.ll.x -= (V1_width / 2 + V1_spacing); 
        //     tmpRect.ur.x += (V1_width / 2 + V1_spacing); 
        //     tmpRect.ll.y -= (V1_width / 2 + V1_spacing);
        //     tmpRect.ur.y += (V1_width / 2 + V1_spacing);
        //
        //     if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
        //         V1cover = true;
        //         break;
        //     }
        // }//V1 via spacing
        if(component.name_ == "cx1")
            std::cout << V1cover << M2cover << M1cover << std::endl;
        
        
        if (!V1cover && !M1cover && AdjMeshPointCheck(touchX, signalY, track.GetStep())) {
            if (!findM2M3Via(touchX, touchY)) {
                continue;
            }
            foundM1M3 = true;
            break;
        }
    }
   
    if(!foundM1M3) //closest points don't work, find farthest point
    for(auto pinPoint : trackFarthestPinPoint) {
        int trackIdx = pinPoint.first;
        touchX = track.GetStart() + trackIdx * track.GetStep();
        //touchX = pinPoint.first;
        touchY = pinPoint.second;
        
        //move touchY towards signalY by width/2
        // if(touchY < signalY)
        //     touchY += M3_width / 2;
        // else
        //     touchY -= M3_width / 2;


        Point2D<double> touchPoint(touchX, touchY);
        bool M1cover = false;
        bool M2cover = false;
        bool V1cover = false; 
        
        // for(int viaIdx = topLayerId_2_viaId_[2]; viaIdx < topLayerId_2_viaId_[4]; viaIdx++) { //vias whose top layer is M2
        M1M2ViaName.clear();//no via has been selected
        for (int viaIdx : layerPair_2_viaIds_.at(std::make_pair(0, 2))) {

            const auto cutCount = vias[viaIdx].GetLayerRectsRef()[1].rects_.size();
            if (cutCount < static_cast<std::size_t>(m1_m2_min_cuts_) || (m1_m2_max_cuts_ != 0 &&
                 cutCount > static_cast<std::size_t>(m1_m2_max_cuts_))) {
                continue;
            }

            auto via = vias[viaIdx];
            Point2D<double> centerOffset(0, 0);

            if (center_vias_) {
                Point2D<double> centerMicrons;

                if (!GetViaCutCenterOffset(via.GetLayerRectsRef()[1], centerMicrons)) {
                    continue;
                }

                double dx = centerMicrons.x * dbuPerMicron;
                double dy = centerMicrons.y * dbuPerMicron;
                double grid = manufacturing_grid * dbuPerMicron;

                bool onDBU = fabs(dx - round(dx)) < 1e-6 && fabs(dy - round(dy)) < 1e-6;

                bool onGrid = grid <= 0 ||
                    (fabs(dx / grid - round(dx / grid)) < 1e-6 &&
                     fabs(dy / grid - round(dy / grid)) < 1e-6);

                if (!onDBU || !onGrid) {
                    if (verbose_ >= debug) {
                        std::cout << "Via center cannot be represented "
                                  << "on the placement grid: "
                                  << via.GetName() << std::endl;
                    }
                    continue;
                }
                centerOffset.x = round(dx);
                centerOffset.y = round(dy);
            }
            touchPoint.x = touchX - centerOffset.x;

            bool M1pinCovered = true;
            int llx = -1, lly = -1, urx = -1, ury = -1;
            M1cover = false;
            for(auto layerRect : via.GetLayerRectsRef()) {
                if(layerRect.layer_name_ == M1_name) {
                    llx = layerRect.rects_[0].ll.x * dbuPerMicron;        
                    lly = layerRect.rects_[0].ll.y * dbuPerMicron;        
                    urx = layerRect.rects_[0].ur.x * dbuPerMicron;        
                    ury = layerRect.rects_[0].ur.y * dbuPerMicron;        

                    //lowest and highest M1 offsets from origin
                    double lowerY = layerRect.rects_[0].ll.y * dbuPerMicron;
                    double upperY = layerRect.rects_[0].ur.y * dbuPerMicron;
                    for (const auto &metalRect : layerRect.rects_) {
                        lowerY = std::min(lowerY, metalRect.ll.y * dbuPerMicron);
                        upperY = std::max(upperY, metalRect.ur.y * dbuPerMicron);
                    }
                    if (pinPoint.second < signalY) {
                        touchPoint.y = pinPoint.second - lowerY;
                    } else {
                        touchPoint.y = pinPoint.second - upperY;
                    }
                    for (const auto &metalRect : layerRect.rects_) {
                        Rect2D<double> viaMetalRect(
                            touchPoint.x + metalRect.ll.x * dbuPerMicron,
                            touchPoint.y + metalRect.ll.y * dbuPerMicron,
                            touchPoint.x + metalRect.ur.x * dbuPerMicron,
                            touchPoint.y + metalRect.ur.y * dbuPerMicron);

                        bool covered = PinCoversRect(pinRect, viaMetalRect);
                        if (!covered) {
                            M1pinCovered = false;
                        }
                    }

                }
            }
            
            for(auto rect : M1obsRect) {
                auto tmpRect = rect;
                tmpRect.ll.x -= (urx + M1_spacing); 
                tmpRect.ur.x += (- llx + M1_spacing); 
                tmpRect.ll.y -= (ury + M1_spacing);
                tmpRect.ur.y += (- lly + M1_spacing);
            
                if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
                    M1cover = true;
                    break;
                }
            }
            if (M1cover || !M1pinCovered) {
                continue;
            }
            //check cut blocks before rejecting or accepting via candidate
            V1cover = viaCutsBlocked(via.GetLayerRectsRef()[1], touchPoint, V1obsRect, V1_spacing);
            if (verbose_ >= debug && V1cover) {
                std::cout << "V1 cut obstruction: " << via.GetName()
                          << " at (" << touchPoint.x
                          << ", " << touchPoint.y << ") DBU"
                          << std::endl;
            }
            if (V1cover) {
                continue;
            }
            bool M2viaBlocked = viaM2Blocked(via.GetLayerRectsRef()[2], touchPoint);
            if (verbose_ >= debug && M2viaBlocked) {
                std::cout << "M2 via obstruction: " << via.GetName()
                          << " instance: " << component.name_
                          << " supply: " << signal
                          << " at (" << touchPoint.x
                          << ", " << touchPoint.y << ") DBU"
                          << std::endl;
            }
            if (!M1cover && !V1cover && M1pinCovered && !M2viaBlocked) {
                M1M2ViaName = via.GetName();
                M1M2ViaIdx = viaIdx;
                M1M2ViaOrigin = touchPoint;
                touchPoint.x = touchX;
                touchPoint.y += centerOffset.y;
                M1M2HasM2MinArea = M2MetalHasMinArea(via.GetLayerRectsRef()[2]);
                M1M2ViaCutCount = via.GetLayerRectsRef()[1].rects_.size();
                M1M2ViaLength=ury-lly;
                touchY = touchPoint.y;
                break;
            }
        } // find m1-m2 via that doesn't conflict with m1 obs
        if (M1M2ViaName.empty()) {
            continue;
        }
        M2wideHalfLength = std::max(M2_width / 2, M1M2ViaLength / 2);
        M2wideHalfLength = std::max(M2wideHalfLength, M2_minlength / 6);
        M2wideHalfLength = (int) FitGrid( M2wideHalfLength, (double) dbuPerMicron * manufacturing_grid);

        M2wideFail= false;
        M2thinFail = false;
        for(auto rect : M2obsRect) {
            auto tmpRect = rect;
            tmpRect.ll.x -= (M2thinHalfLength + M2_spacing);
            tmpRect.ur.x += (M2thinHalfLength + M2_spacing);
            tmpRect.ll.y -= (M2_width / 2 + M2_spacing);
            tmpRect.ur.y += (M2_width / 2 + M2_spacing);
            
            if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
                M2thinFail = true;
                break;
            }
        } // m2 side min area doesn't conflict with m2 obs
        if(touchPoint.x - M2thinHalfLength - M2_spacing / 2 < component.location_.x ||
                    touchPoint.x + M2thinHalfLength + M2_spacing / 2 > component.location_.x + component.size_.x ||
                    touchPoint.y - M2_width / 2 - M2_spacing / 2 < component.location_.y || 
                    touchPoint.y + M2_width / 2 + M2_spacing / 2 > component.location_.y + component.size_.y) {
            M2thinFail = true;
        }
        for(auto rect : M2obsRect) {
            auto tmpRect = rect;
            tmpRect.ll.x -= (M2wideHalfLength + M2_spacing); 
            tmpRect.ur.x += (M2wideHalfLength + M2_spacing); 
            tmpRect.ll.y -= (M2_width * 3 / 2 + M2_spacing);
            tmpRect.ur.y += (M2_width * 3 / 2 + M2_spacing);
            
            if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
                M2wideFail = true;
                break;
            }
        }
        
        if(touchPoint.x - M2wideHalfLength - M2_spacing / 2 < component.location_.x || 
                    touchPoint.x + M2wideHalfLength + M2_spacing / 2 > component.location_.x + component.size_.x ||
                    touchPoint.y - M2_width * 3 / 2 - M2_spacing / 2 < component.location_.y || 
                    touchPoint.y + M2_width * 3 / 2 + M2_spacing / 2 > component.location_.y + component.size_.y) { 
            M2wideFail = true;            
        }
        M2thinFail = M2thinFail || m2WireBlocked(makeM2PadWire(touchX, touchY, false));
        M2wideFail = M2wideFail || m2WireBlocked(makeM2PadWire(touchX, touchY, true));
        M2cover = M2wideFail && M2thinFail;
        if(component.name_ == "cx1") {
            std::cout << signal << ": " << std::endl;
            std::cout << "far M2cover : " << M2cover << " M2wideFail: " << M2wideFail << " M2thinFail: " << M2thinFail << std::endl;
            std::cout << "touchpoint x : " << touchPoint.x << " " << touchPoint.y;
            std::cout << " M2minlength/2: " << M2_minlength / 2 << " component.x: " << component.location_.x << std::endl; 
        }


        //Now we check above
        // for(auto rect : V1obsRect) {
        //     auto tmpRect = rect;
        //     tmpRect.ll.x -= (V1_width / 2 + V1_spacing); 
        //     tmpRect.ur.x += (V1_width / 2 + V1_spacing); 
        //     tmpRect.ll.y -= (V1_width / 2 + V1_spacing);
        //     tmpRect.ur.y += (V1_width / 2 + V1_spacing);
        //
        //     if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
        //         V1cover = true;
        //         break;
        //     }
        // }//V1 via spacing
      
        if(component.name_ == "cx1")
            std::cout << V1cover << M2cover << M1cover << std::endl;
        
 
        if (!V1cover && !M1cover && AdjMeshPointCheck(touchX, signalY, track.GetStep())) {
            if (!findM2M3Via(touchX, touchY)) {
                continue;
            }
            foundM1M3 = true;
            break;
        }
    }


    if(foundM1M3) {
        Wire tmpWire;
        auto &placedM2Rects = (signal == "POWER") ? pwgnd_.powerM2Rects : pwgnd_.gndM2Rects;
        auto &placedM3Rects = (signal == "POWER") ? pwgnd_.powerM3Rects : pwgnd_.gndM3Rects;

        if(signal == "POWER") { 
            powerPoint.x = touchX;
            powerPoint.y = touchY;
        }
     
        tmpWire.coorX[0] = M1M2ViaOrigin.x;
        tmpWire.coorY[0] = M1M2ViaOrigin.y;
        tmpWire.numPathPoint = 1;
        tmpWire.layerName = layers[2].GetName();
        tmpWire.width = 0;
        tmpWire.viaName = M1M2ViaName;
        Wires.push_back(tmpWire);//M2 to M1 via
        RecordViaMetal(vias[M1M2ViaIdx].GetLayerRectsRef()[2], //record M2 from placed via
                      Point2D<double>(tmpWire.coorX[0], tmpWire.coorY[0]),
                      placedM2Rects);


        //report via emitted, use PhyDB warning mechanism if single cut used
        if (verbose_ >= debug) {
            std::cout << "M1-M2 emitted: " << M1M2ViaName
                      << " cuts: " << M1M2ViaCutCount
                      << " instance: " << component.name_
                      << " supply: " << signal
                      << " at (" << M1M2ViaOrigin.x << ", " << M1M2ViaOrigin.y << ") DBU"
                      << std::endl;
        }
        PhyDBWarns(
            M1M2ViaCutCount == 1,
            "PWRoute: single-cut M1-M2 fallback"
                << "; instance: " << component.name_
                << "; supply: " << signal
                << "; via: " << M1M2ViaName
                << "; location: (" << M1M2ViaOrigin.x << ", " << M1M2ViaOrigin.y << ") DBU"
                << "; DBU per micron: " << dbuPerMicron);

        int zRouteOffset = getZRouteOffset(touchX, touchY);
        if(signal == "GROUND" && touchX == powerPoint.x && ((touchY < signalY && powerPoint.y > touchY) ||
                    (touchY > signalY && powerPoint.y < touchY))) {

            tmpWire = makeM3Wire(touchX, touchY);
            Wires.push_back(tmpWire);
            placedM3Rects.push_back(getM3WireRect(tmpWire));

 
            tmpWire = makeM2BridgeWire(touchX, touchY, zRouteOffset);
            Wires.push_back(tmpWire);//M2 wire
            placedM2Rects.push_back(getM2WireRect(tmpWire));
        }
        else { //normal route without Z
            tmpWire = makeM3Wire(touchX, touchY);
            Wires.push_back(tmpWire);
            placedM3Rects.push_back(getM3WireRect(tmpWire));

        }

        int viaID = meshViaIdx; //M4
        tmpWire.coorX[0] = touchX + zRouteOffset;
        tmpWire.coorY[0] = signalY;
        tmpWire.numPathPoint = 1;
        tmpWire.layerName = layers[6].GetName();
        tmpWire.width = 0;
        tmpWire.viaName = vias[viaID].GetName();
	pwgnd_.usedConnectionPoints.insert(Point2D<int>(tmpWire.coorX[0], tmpWire.coorY[0]));
        Wires.push_back(tmpWire);//M3 to M4 via, mesh
        RecordViaMetal(vias[meshViaIdx].GetLayerRectsRef()[0], //record mesh via M3 metal
                Point2D<double>(tmpWire.coorX[0], tmpWire.coorY[0]),
                placedM3Rects);


        viaID = M2M3ViaIdx;//M3
        tmpWire.coorX[0] = M2M3ViaOrigin.x;
        tmpWire.coorY[0] = M2M3ViaOrigin.y;
        tmpWire.numPathPoint = 1;
        tmpWire.layerName = layers[4].GetName();
        tmpWire.width = 0;
        tmpWire.viaName = vias[viaID].GetName();
        Wires.push_back(tmpWire);//M3 to M2 via, touch
        RecordViaMetal(vias[M2M3ViaIdx].GetLayerRectsRef()[0], //record via M2 metal
               M2M3ViaOrigin,
               placedM2Rects);
        RecordViaMetal(vias[M2M3ViaIdx].GetLayerRectsRef()[2], //record via M3 metal
               M2M3ViaOrigin,
               placedM3Rects);
        if (verbose_ >= debug) {
            std::cout << "M2-M3 emitted: "
                      << vias[M2M3ViaIdx].GetName()
                      << " cuts: "
                      << vias[M2M3ViaIdx].GetLayerRectsRef()[1].rects_.size()
                      << " instance: " << component.name_
                      << " supply: " << signal
                      << " origin: (" << M2M3ViaOrigin.x
                      << ", " << M2M3ViaOrigin.y << ") DBU\n";
}


        
        if (needM2Pad) {
            tmpWire = makeM2PadWire(touchX, touchY, M2thinFail);
            Wires.push_back(tmpWire);//M2 min area
            placedM2Rects.push_back(getM2WireRect(tmpWire));
        }
    if (fabs(touchY - signalY) < M3_minlength) {
        tmpWire = makeM3MinAreaWire(touchX, touchY);
        Wires.push_back(tmpWire);
        placedM3Rects.push_back(getM3WireRect(tmpWire));
    }
    }
    return foundM1M3;
}  

bool PWRoute::DetailedRouteCloseVia(int dist, phydb::Range<int> range) {

    if(dist > range.begin && dist < range.end)
	return true;
    else
	return false;    
}

bool PWRoute::OutOfComponent(PWRouteComponent& component, int touchX, int touchY, int space) {
    bool out = false;

    if(touchX < component.location_.x + space || 
        touchX > component.location_.x + component.size_.x - space || 
        touchY < component.location_.y + space ||
        touchY > component.location_.y + component.size_.y - space)
        out = true;
    return out; 

}

bool PWRoute::M2DetailedRouteSNet(PWRouteComponent& component, std::string signal, int signalY, 
        std::vector<Wire>& Wires, Point2D<int>& powerPoint) {
    std::vector<Rect2D<double>> pinRect;
    std::vector<Rect2D<double>> obsRect;

    auto tracks = db_ptr_->GetDesignPtr()->GetTracksRef();
    auto layers = db_ptr_->GetTechPtr()->GetLayersRef();
    auto vias = db_ptr_->GetTechPtr()->GetLefViasRef();
    int dbuPerMicron = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();
    
    for(auto pin : component.pins_) {
        for(auto layerRect : pin.layer_rects_) {
            int layerIdx = db_ptr_->GetTechPtr()->GetLayerId(layerRect.layer_name_);
            if(layerIdx == 2) { //M2 
                for(auto rect : layerRect.rects_) {
                    if(pin.use_ == StrToSignalUse(signal))
                        pinRect.push_back(rect);
                    else
                        obsRect.push_back(rect);
                }
            }
        }
    }
    
    for(auto layerRect : component.obs_.GetLayerRects()) {
        int layerIdx = db_ptr_->GetTechPtr()->GetLayerId(layerRect.layer_name_);
        if(layerIdx == 2) { //M2
            for(auto rect : layerRect.rects_) {
                obsRect.push_back(rect);
            }
        }
    }
    
    if(pinRect.size() == 0)
        return false;

    //return true if master fits the landing
    auto viaM2PinCovered = [&](int candidateViaIdx, const Point2D<double> &origin) -> bool {
        const auto &metalLayer = vias[candidateViaIdx].GetLayerRectsRef()[0];

        if (metalLayer.rects_.empty()) {
            return false;
        }
        for (const auto &metal : metalLayer.rects_) {
            Rect2D<double> placedMetal(
                origin.x + metal.ll.x * dbuPerMicron,
                origin.y + metal.ll.y * dbuPerMicron,
                origin.x + metal.ur.x * dbuPerMicron,
                origin.y + metal.ur.y * dbuPerMicron);
            if (!PinCoversRect(pinRect, placedMetal)) {
                return false;
            }
        }
        return true;
    };


    PinEdge pin_edges;
    if(pwgnd_.same_signal_obs_reserve_) {
        //std::cout << "component: " << component.name_ << std::endl;
        ExtractPinEdge(component.name_, pinRect, pin_edges);
        /*if(component.name_ == "tst_aelem_50_6_as__10_act_acx0") {
            for(auto e : pin_edges.l_edges) 
                std::cout << "left: " << e << std::endl;
            for(auto e : pin_edges.r_edges) 
                std::cout << "right: " << e << std::endl;
        }*/
    }

    Track track = tracks[layerid_2_trackid_[4]]; //M3

    std::map<int, int> trackClosestPinPoint;
    std::map<int, int> trackFarthestPinPoint;
    
    //int M2_minLength = layer_min_length_[2] * dbuPerMicron;
    int M2_width = layers[2].GetWidth() * dbuPerMicron; 
    int M2_pitch = layers[2].GetPitchY() * dbuPerMicron; 


    int viaIdx = topLayerId_2_viaId_[detailed_route_layer];//M3
    auto via = vias[viaIdx];
    auto viaRect = via.GetLayerRectsRef()[0].rects_[0]; //M2
    double h = (viaRect.ur.x - viaRect.ll.x);
    double v = (viaRect.ur.y - viaRect.ll.y);
    double viaWidth = (h > v)? v * dbuPerMicron : h * dbuPerMicron;
    double viaLength = (h > v)? h * dbuPerMicron : v * dbuPerMicron;
    
    auto M2_spacing_table = layers[2].GetSpacingTable();
    double spacing1 = M2_spacing_table->GetSpacingFor(h, v);
    double spacing2 = M2_spacing_table->GetSpacingFor(v, h);
    double M2_spacing = (double) std::max(spacing1, spacing2) * (double) dbuPerMicron;
    int width = layers[4].GetWidth() * dbuPerMicron; //M3
    int pitch = layers[4].GetPitchX() * dbuPerMicron;
  
    h *= dbuPerMicron;
    v *= dbuPerMicron;
    int h_extend = std::max((int)h, M2_width) / 2;
    int v_extend = std::max((int)v, M2_width) / 2;  
    findClosestTouchPoints(pinRect, trackClosestPinPoint, track, 0, signalY); // no expand
    findFarthestTouchPoints(pinRect, trackFarthestPinPoint, track, 0, signalY); // no expand
    
    //findClosestTouchPoints(obsRect, trackClosestOBSPoint, track, h / 2 + M2_spacing, signalY); // 

    bool foundM2 = false;

    int selectedViaIdx = -1;
    Point2D<double> selectedViaOrigin;

    auto findDirectM2M3Via = [&](double x, double y) -> bool {
        selectedViaIdx = -1;
        bool usesZ = signal == "GROUND" && x == powerPoint.x &&
            ((y < signalY && powerPoint.y > y) ||
             (y > signalY && powerPoint.y < y));

        int zOffset = usesZ ? static_cast<int>(layers[4].GetPitchX() * dbuPerMicron) : 0;

        for (int candidateIdx : layerPair_2_viaIds_.at(std::make_pair(2, 4))) {
            const auto &shapes = vias[candidateIdx].GetLayerRectsRef();
            auto cutCount = shapes[1].rects_.size();
            if (shapes[0].rects_.empty() || shapes[1].rects_.empty() || shapes[2].rects_.empty()) {
                continue;
            }

            if (cutCount < static_cast<std::size_t>(m2_m3_min_cuts_) ||
                (m2_m3_max_cuts_ != 0 &&
                 cutCount > static_cast<std::size_t>(m2_m3_max_cuts_))) {
                continue;
            }

            Point2D<double> origin(x + zOffset, y);

            if (center_vias_) {
                Point2D<double> center;
                if (!GetViaCutCenterOffset(shapes[1], center)) {
                    continue;
                }
                double dx = center.x * dbuPerMicron;
                double dy = center.y * dbuPerMicron;
                double grid = db_ptr_->GetTechPtr()->GetManufacturingGrid() * dbuPerMicron;
                bool onDBU = fabs(dx - round(dx)) < 1e-6 &&
                             fabs(dy - round(dy)) < 1e-6;
                bool onGrid = grid <= 0 ||
                    (fabs(dx / grid - round(dx / grid)) < 1e-6 &&
                     fabs(dy / grid - round(dy / grid)) < 1e-6);
                if (!onDBU || !onGrid) {
                    continue;
                }
                origin.x -= round(dx);
                origin.y -= round(dy);
            }
            if (!viaM2PinCovered(candidateIdx, origin)) {
                continue;
            }
            selectedViaIdx = candidateIdx;
            selectedViaOrigin = origin;
            return true;
        }
        return false;
    };
    
    int touchX = -1, touchY = -1;
   

    for(auto pinPoint : trackClosestPinPoint) { // Cell library gaurantee min area on M2
        int trackIdx = pinPoint.first;
        touchX = track.GetStart() + trackIdx * track.GetStep();
        touchY = pinPoint.second;

        int farthestY = trackFarthestPinPoint[trackIdx];
        if(verbose_ > 2 && component.name_ == "tst_aelem_50_6_apush__muxi_adata__and_50_6_56_6") {
            std::cout << "close -> far : " << touchY << " " << farthestY << std::endl;
            std::cout << h_extend << " " << v_extend << " " << M2_spacing << std::endl;
        } 
	
	    while(1) {	
            if(verbose_ > 2 && component.name_ == "tst_aelem_51_6_ae25_a__84__") 
                std::cout << touchY << std::endl;            
            Point2D<double> touchPoint(touchX, touchY);
            bool M2cover = false, OutOfComp = false;

            for(auto rect : obsRect) {
                auto tmpRect = rect;
                tmpRect.ll.x -= (h_extend + M2_spacing); // just gaurantee via 
                tmpRect.ur.x += (h_extend + M2_spacing); 
                tmpRect.ll.y -= (v_extend + M2_spacing);
                tmpRect.ur.y += (v_extend + M2_spacing);
                if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
                    if(verbose_ > 2 && component.name_ == "tst_aelem_51_6_ae25_a__84__") 
                        std::cout << rect << std::endl;
                    M2cover = true;
                    break;
                }
                if(OutOfComponent(component, touchX, touchY, h/2 + M2_spacing)) {
		            OutOfComp = true;
		            break;
	            }
            } // m2 side min area doesn't conflict with m2 obs       
            if (!M2cover && !OutOfComp && AdjMeshPointCheck(touchX, signalY, track.GetStep()) &&
                SameSignalObsBlock(touchX, touchY, h_extend, v_extend, M2_spacing, pin_edges) &&
                findDirectM2M3Via(touchX, touchY)) {
                foundM2 = true;
                break;
            }
            if(touchY > signalY) {
	            touchY += M2_pitch / 2;
                if(touchY > farthestY)
                    break;
            }
            else {
                touchY -= M2_pitch / 2;
	            if(touchY < farthestY)
                    break;
            }
        }
	    if(foundM2)
	        break;
    }
    
    int M3_width = layers[4].GetWidth() * dbuPerMicron;
    int M3_pitch = layers[4].GetPitchX() * dbuPerMicron;
    int M3_minlength = layer_min_length_[4] * dbuPerMicron;

    if(foundM2) {
        Wire tmpWire;
        if(signal == "POWER") { 
            powerPoint.x = touchX;
            powerPoint.y = touchY;
        }
        /*if(touchY < signalY)
            touchY -= M2_width / 2;
        else
            touchY += M2_width / 2;
        */

        int zRouteOffset = 0;
        int viaID;
        if(signal == "GROUND" && touchX == powerPoint.x &&
                ((touchY < signalY && powerPoint.y > touchY) || (touchY > signalY && powerPoint.y < touchY))) { //Z route
           
            zRouteOffset = M3_pitch;

            tmpWire.coorX[0] = touchX + zRouteOffset;//
            tmpWire.coorX[1] = touchX + zRouteOffset;//
            tmpWire.coorY[1] = signalY;
            if(fabs(signalY - touchY) >= M3_minlength)
                tmpWire.coorY[0] = touchY;
            else {
                tmpWire.coorY[0] = (touchY > signalY)? signalY + M3_minlength : signalY - M3_minlength;
            }
            tmpWire.numPathPoint = 2;
            tmpWire.layerName = layers[detailed_route_layer].GetName(); //M3
            if(DetailedRouteCloseVia(abs(signalY - touchY), pwgnd_.m3_expanded_range))
                tmpWire.width = std::max(pwgnd_.m3_expanded_width, M3_width);
            else
                tmpWire.width = M3_width;
            Wires.push_back(tmpWire);//M3 wire
            
            tmpWire.coorX[0] = touchX - M2_width / 2;//
            tmpWire.coorY[0] = touchY;
            tmpWire.coorX[1] = touchX + zRouteOffset + M2_width / 2;//
            tmpWire.coorY[1] = touchY;
            
            tmpWire.numPathPoint = 2;
            tmpWire.layerName = layers[2].GetName(); //M2
            if(DetailedRouteCloseVia(abs(tmpWire.coorX[0] - tmpWire.coorX[1]), pwgnd_.m2_expanded_range))
	        tmpWire.width = std::max(pwgnd_.m2_expanded_width, M2_width);
	    else
	        tmpWire.width = M2_width;
            Wires.push_back(tmpWire);//M2 wire
            
            tmpWire.coorX[0] = selectedViaOrigin.x;
            tmpWire.coorY[0] = selectedViaOrigin.y;
            tmpWire.numPathPoint = 1;
            tmpWire.width = 0;
            viaID = selectedViaIdx;
            tmpWire.viaName = vias[viaID].GetName();
            tmpWire.layerName = layers[4].GetName(); //M2-M3 via
            Wires.push_back(tmpWire);//M2 via
        }
        else { //normal route without Z
            tmpWire.coorX[0] = touchX;
            tmpWire.coorX[1] = touchX;
            tmpWire.coorY[1] = signalY;
            if(fabs(signalY - touchY) >= M3_minlength)
                tmpWire.coorY[0] = touchY;
            else {
                tmpWire.coorY[0] = (touchY > signalY)? signalY + M3_minlength : signalY - M3_minlength;
            }
            tmpWire.numPathPoint = 2;
            tmpWire.layerName = layers[4].GetName(); //M3
            if(DetailedRouteCloseVia(abs(signalY - touchY), pwgnd_.m3_expanded_range))
	        tmpWire.width = std::max(pwgnd_.m3_expanded_width, M3_width);
	    else
	        tmpWire.width = M3_width;
            Wires.push_back(tmpWire);//M3 wire

            tmpWire.coorX[0] = selectedViaOrigin.x;
            tmpWire.coorY[0] = selectedViaOrigin.y;
            viaID = selectedViaIdx;
            tmpWire.numPathPoint = 1;
            tmpWire.width = 0;
            tmpWire.layerName = layers[4].GetName(); //M3
            tmpWire.viaName = vias[viaID].GetName();
            Wires.push_back(tmpWire);
        }
        if (verbose_ >= debug) {
            Point2D<double> origin(tmpWire.coorX[0], tmpWire.coorY[0]);
            bool covered = viaM2PinCovered(viaID, origin);
                        std::cout << "Direct M2 access emitted:"
                      << " instance: " << component.name_
                      << " supply: " << signal
                      << " via: " << vias[viaID].GetName()
                      << " cuts: "
                      << vias[viaID].GetLayerRectsRef()[1].rects_.size()
                      << " origin: (" << origin.x << ", " << origin.y << ") DBU"
                      << " M2-covered: " << (covered ? "yes" : "no")
                      << std::endl;

            if (!covered) {
                std::cout << "Direct M2 access protrusion:"
                          << " instance: " << component.name_
                          << " supply: " << signal
                          << " via: " << vias[viaID].GetName()
                          << " origin: (" << tmpWire.coorX[0]
                          << ", " << tmpWire.coorY[0] << ") DBU"
                          << std::endl;
            }
        }

        
        tmpWire.coorX[0] = touchX + zRouteOffset; 
        tmpWire.coorY[0] = signalY; 
        tmpWire.numPathPoint = 1;
        tmpWire.layerName = layers[6].GetName();
        tmpWire.width = 0;
        viaID = topLayerId_2_viaId_[6];
        tmpWire.viaName = vias[viaID].GetName();
	pwgnd_.usedConnectionPoints.insert(Point2D<int>(tmpWire.coorX[0], tmpWire.coorY[0]));
        Wires.push_back(tmpWire); //M3-M4 via

    }
    return foundM2;

}

void PWRoute::RecordViaMetal(const LayerRect &metalLayer, const Point2D<double> &origin, std::vector<Rect2D<double>> &rects){
    double dbuPerMicron = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();

    for (const auto &metal : metalLayer.rects_) {
        rects.emplace_back(
            origin.x + metal.ll.x * dbuPerMicron,
            origin.y + metal.ll.y * dbuPerMicron,
            origin.x + metal.ur.x * dbuPerMicron,
            origin.y + metal.ur.y * dbuPerMicron);

        if (verbose_ >= debug) {
        const auto &placed = rects.back();
            std::cout << "Via metal recorded: layer: " << metalLayer.layer_name_
                      << " origin: (" << origin.x << ", " << origin.y << ")"
                      << " rect: (" << placed.ll.x << ", " << placed.ll.y
                      << ")(" << placed.ur.x << ", " << placed.ur.y << ")"
                      << " DBU; stored: " << rects.size()
                      << std::endl;
        }
    }
}


void PWRoute::DetailedRouteSNetComp(PWRouteComponent& component) {
    int powerY, gndY;
    if(component.pins_.size() != 0) {
        auto rect = component.pins_[0].layer_rects_[0].rects_[0];
        FindRowSNet(component.name_, component.pins_[0].name_, rect, powerY, gndY);
    }
    else
        return;

    Wire tmpWire;
    Point2D<int> powerPoint(-1, -1);
    bool M1power = false, M1ground = false, M2power = false, M2ground = false, M1M3power = false, M1M3ground = false; 
    
    M1power = M1DetailedRouteSNet(component, "POWER");
    if(!M1power) {
        M2power = M2DetailedRouteSNet(component, "POWER", powerY, pwgnd_.powerWires, powerPoint);
        if(!M2power) {
            M1M3power = M1M3DetailedRouteSNet(component, "POWER", powerY, pwgnd_.powerWires, powerPoint); 
            if(!M1M3power)
                std::cout << "warning: UNABLE TO PATTERN ROUTE POWER: " << component.name_ << "; cell master: " << component.master_name_ << std::endl;
        }
    }
    M1ground = M1DetailedRouteSNet(component, "GROUND");
    if(!M1ground) {
        M2ground = M2DetailedRouteSNet(component, "GROUND", gndY, pwgnd_.gndWires, powerPoint);
        if(!M2ground) {
            M1M3ground = M1M3DetailedRouteSNet(component, "GROUND", gndY, pwgnd_.gndWires, powerPoint); 
            if(!M1M3ground)
                std::cout << "warning: UNABLE TO PATTERN ROUTE GROUND: " << component.name_ << "; cell master: " << component.master_name_ << std::endl;
        }
    }


    if(M1power || M2power || M1M3power) {
        POWER_FOUND++;
    }
    else {
        POWER_UNFOUND++;
    }

    if(M1ground || M2ground || M1M3ground) {
        GROUND_FOUND++;
    }
    else
        GROUND_UNFOUND++;
    
}


void PWRoute::M2MetalFill(std::string signal) {
    auto tracks = db_ptr_->GetDesignPtr()->GetTracksRef();
    auto layers = db_ptr_->GetTechPtr()->GetLayersRef();
    int dbuPerMicron = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();

    Track track = tracks[layerid_2_trackid_[4]]; //M3

    auto& Wires = (signal == "POWER")? pwgnd_.powerWires : pwgnd_.gndWires;
    
    int M2_minLength = layer_min_length_[2] * dbuPerMicron;
    int M2_width = layers[2].GetWidth() * dbuPerMicron;
        
    auto& usedPoints = (signal == "POWER")? pwgnd_.powerM2Points : pwgnd_.gndM2Points;
    for(auto point : usedPoints) {
        Point2D<int> leftPoint, twoLeftPoint, threeLeftPoint;
        leftPoint.x = point.x - track.GetStep();
        leftPoint.y = point.y;
        
        twoLeftPoint.x = point.x - 2 * track.GetStep();
        twoLeftPoint.y = point.y;
        
        threeLeftPoint.x = point.x - 3 * track.GetStep();
        threeLeftPoint.y = point.y;
        if(usedPoints.count(leftPoint) != 0) {
            Wire tmpWire;
            tmpWire.coorX[0] = point.x - track.GetStep() / 2 - M2_minLength / 2;
            tmpWire.coorY[0] = point.y;
            tmpWire.coorX[1] = point.x - track.GetStep() / 2 + M2_minLength / 2;
            tmpWire.coorY[1] = point.y;
            tmpWire.numPathPoint = 2;
            tmpWire.layerName = layers[2].GetName();
            tmpWire.width = M2_width;
            Wires.push_back(tmpWire); //M2 min area
        }

        else if(usedPoints.count(twoLeftPoint) != 0 || usedPoints.count(threeLeftPoint) != 0) {
            Wire tmpWire;
            if(usedPoints.count(twoLeftPoint) != 0)
                tmpWire.coorX[0] = point.x - 2 * track.GetStep();
            else
                tmpWire.coorX[0] = point.x - 3 * track.GetStep();

            tmpWire.coorY[0] = point.y;
            tmpWire.coorX[1] = point.x;
            tmpWire.coorY[1] = point.y;
            tmpWire.numPathPoint = 2;
            tmpWire.layerName = layers[2].GetName();
            tmpWire.width = M2_width;
            Wires.push_back(tmpWire); //M2 min area
        }
        else {
            Wire tmpWire;
            tmpWire.coorX[0] = point.x - M2_minLength / 2;
            tmpWire.coorY[0] = point.y;
            tmpWire.coorX[1] = point.x + M2_minLength / 2;
            tmpWire.coorY[1] = point.y;
            tmpWire.numPathPoint = 2;
            tmpWire.layerName = layers[2].GetName();
            tmpWire.width = M2_width;
            Wires.push_back(tmpWire); //M2 min area
        }
    }
}


void PWRoute::DetailedRouteSNet() {
    pwgnd_.powerM2Rects.clear();
    pwgnd_.gndM2Rects.clear();
    pwgnd_.powerM3Rects.clear();
    pwgnd_.gndM3Rects.clear();

    std::map<std::string, std::pair<int, int>> failedMasters;

    for (auto& component : components_) {
        int powerBefore = POWER_UNFOUND;
        int groundBefore = GROUND_UNFOUND;

        DetailedRouteSNetComp(component);

        if (POWER_UNFOUND > powerBefore) {
            failedMasters[component.master_name_].first++;
        }
        if (GROUND_UNFOUND > groundBefore) {
            failedMasters[component.master_name_].second++;
        }
    }


    if(verbose_ > none) {
        std::cout << "Power routing completes/fails: " << POWER_FOUND << " / " << POWER_UNFOUND << std::endl;
        std::cout << "Ground routing completes/fails: " << GROUND_FOUND << " / " << GROUND_UNFOUND << std::endl;
    }
    if (!failedMasters.empty()) {
        std::cout << "ATTENTION: SOME CELLS FAIL TO ROUTE!" << std::endl;
        std::cout << "Cells with failed power/ground pattern routing:" << std::endl;
        for (const auto& entry : failedMasters) {
            std::cout << "  " << entry.first
                      << ": POWER " << entry.second.first
                      << ", GROUND " << entry.second.second
                      << std::endl;
        }
        std::cout << "Please review the supply-pin layouts of these cells "
                     "for compatibility with the pattern-routing access."
                  << std::endl;
        std::cout << "See individual warnings for affected instances."
                  << std::endl;
    }
    
    M2MetalFill("POWER");
    M2MetalFill("GROUND");
}




}


