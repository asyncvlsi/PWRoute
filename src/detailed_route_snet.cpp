#include "pwroute.h" 
#include <math.h>
using namespace phydb;
namespace pwroute {

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


void PWRoute::findClosestTouchPoints(std::vector<Rect2D<double>>& rects, std::map<int, int>& closestPoint, 
        Track track, int expand, int signalY) {
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

bool PWRoute::M1M3DetailedRouteSNet(PWRouteComponent& component, std::string signal, int signalY, 
        std::vector<Wire>& Wires, Point2D<int>& powerPoint) {
    
    std::vector<Rect2D<double>> M2obsRect;
    std::vector<Rect2D<double>> M1obsRect;
    std::vector<Rect2D<double>> V1obsRect; 
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
        else if(layerIdx == 0) { //V1
            for(auto rect : layerRect.rects_) {
                V1obsRect.push_back(rect);
            }
        }
    }
  

    int width = layers[4].GetWidth() * dbuPerMicron; //M3
    std::map<int, int> trackClosestPinPoint; //map trackIdx -> y
    std::map<int, int> trackFarthestPinPoint; 

    //map<int, int> touchPinPoint; //map x -> y 
    Track track = tracks[layerid_2_trackid_[4]]; //M3
    
    findClosestTouchPoints(pinRect, trackClosestPinPoint, track, width / 2, signalY);
    //findTouchPointsNoTrack(pinRect, touchPinPoint, track, width / 2, signalY);
    findFarthestTouchPoints(pinRect, trackFarthestPinPoint, track, width / 2, signalY);
    
    std::string M1_name = layers[0].GetName();
    int M1_width = layers[0].GetWidth() * dbuPerMicron;
    int M1_pitch = layers[0].GetPitchX() * dbuPerMicron;
    int M1_spacing = (M1_pitch - M1_width);
    
    int V1_width = layers[1].GetWidth() * dbuPerMicron;
    int V1_spacing = layers[1].GetSpacing() * dbuPerMicron;
 
    int M2_minlength = layer_min_length_[2] * dbuPerMicron;
    int M2_pitch = layers[2].GetPitchX() * dbuPerMicron;
    int M2_width = layers[2].GetWidth() * dbuPerMicron;
    int M2_spacing = (M2_pitch - M2_width);
        
    int M3_width = layers[4].GetWidth() * dbuPerMicron;
    int M3_pitch = layers[4].GetPitchX() * dbuPerMicron;
    int M3_minlength = layer_min_length_[4] * dbuPerMicron;
    
    bool foundM1M3 = false;
    double touchX, touchY;
    std::string M1M2ViaName;
   
    bool M2thinFail = false;
    bool M2wideFail = false;
    for(auto pinPoint : trackClosestPinPoint) {
        int trackIdx = pinPoint.first;
        touchX = track.GetStart() + trackIdx * track.GetStep();
        //touchX = pinPoint.first;
        touchY = pinPoint.second;
        
        //move touchY futher from signalY by width/2
        if(touchY < signalY)
            touchY -= M3_width / 2;
        else
            touchY += M3_width / 2;

        Point2D<double> touchPoint(touchX, touchY);
        bool M1cover = false;
        bool M2cover = false;
        bool V1cover = false; 
        
        for(int viaIdx = topLayerId_2_viaId_[2]; viaIdx < topLayerId_2_viaId_[4]; viaIdx++) { //vias whose top layer is M2
            auto via = vias[viaIdx];
            int llx, lly, urx, ury;
            M1cover = false;
            for(auto layerRect : via.GetLayerRectsRef()) {
                if(layerRect.layer_name_ == M1_name) {
                    llx = layerRect.rects_[0].ll.x * dbuPerMicron;        
                    lly = layerRect.rects_[0].ll.y * dbuPerMicron;        
                    urx = layerRect.rects_[0].ur.x * dbuPerMicron;        
                    ury = layerRect.rects_[0].ur.y * dbuPerMicron;        
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
            if(!M1cover) {
                M1M2ViaName = via.GetName();
                break;
            }
        } // find m1-m2 via that doesn't conflict with m1 obs

        M2wideFail= false;
        M2thinFail = false;
        for(auto rect : M2obsRect) {
            auto tmpRect = rect;
            tmpRect.ll.x -= (M2_minlength / 2 + M2_spacing); 
            tmpRect.ur.x += (M2_minlength / 2 + M2_spacing); 
            tmpRect.ll.y -= (M2_width / 2 + M2_spacing);
            tmpRect.ur.y += (M2_width / 2 + M2_spacing);
            
            if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
                M2thinFail = true;
                break;
            }
        } // m2 side min area doesn't conflict with m2 obs
        if(touchPoint.x - M2_minlength / 2 - M2_spacing / 2 < component.location_.x || 
                touchPoint.x + M2_minlength / 2 + M2_spacing / 2 > component.location_.x + component.size_.x ||
                    touchPoint.y - M2_width / 2 - M2_spacing / 2 < component.location_.y || 
                    touchPoint.y + M2_width / 2 + M2_spacing / 2 > component.location_.y + component.size_.y) 
            M2thinFail = true;

        for(auto rect : M2obsRect) {
            auto tmpRect = rect;
            tmpRect.ll.x -= (M2_minlength / 6 + M2_spacing); 
            tmpRect.ur.x += (M2_minlength / 6 + M2_spacing); 
            tmpRect.ll.y -= (M2_width * 3 / 2 + M2_spacing);
            tmpRect.ur.y += (M2_width * 3 / 2 + M2_spacing);
            
            if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
                M2wideFail = true;
                break;
            }
        }
        if(touchPoint.x - M2_minlength / 6 - M2_spacing / 2 < component.location_.x || 
                touchPoint.x + M2_minlength / 6 + M2_spacing / 2 > component.location_.x + component.size_.x ||
                    touchPoint.y - M2_width * 3 / 2 - M2_spacing / 2 < component.location_.y || 
                    touchPoint.y + M2_width * 3 / 2 + M2_spacing / 2 > component.location_.y + component.size_.y) 
            M2wideFail = true;

        M2cover = M2wideFail && M2thinFail;  
        
        if(component.name_ == "cx1") {
            std::cout << signal << ":" << std::endl;
            std::cout << "close M2cover : " << M2cover << " M2wideFail: " << M2wideFail << " M2thinFail: " << M2thinFail << std::endl;
            std::cout << "touchpoint x : " << touchPoint.x << " " << touchPoint.y;
            std::cout << " M2minlength/2: " << M2_minlength / 2 << " component.x: " << component.location_.x << std::endl; 
        }

 
        for(auto rect : V1obsRect) {
            auto tmpRect = rect;
            tmpRect.ll.x -= (V1_width / 2 + V1_spacing); 
            tmpRect.ur.x += (V1_width / 2 + V1_spacing); 
            tmpRect.ll.y -= (V1_width / 2 + V1_spacing);
            tmpRect.ur.y += (V1_width / 2 + V1_spacing);
            
            if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
                V1cover = true;
                break;
            }
        }//V1 via spacing
        if(component.name_ == "cx1")
            std::cout << V1cover << M2cover << M1cover << std::endl;
        
        
        if(V1cover == false && M2cover == false && M1cover == false) {       
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
        if(touchY < signalY)
            touchY += M3_width / 2;
        else
            touchY -= M3_width / 2;


        Point2D<double> touchPoint(touchX, touchY);
        bool M1cover = false;
        bool M2cover = false;
        bool V1cover = false; 
        
        for(int viaIdx = topLayerId_2_viaId_[2]; viaIdx < topLayerId_2_viaId_[4]; viaIdx++) { //vias whose top layer is M2
            auto via = vias[viaIdx];
            int llx = -1, lly = -1, urx = -1, ury = -1;
            M1cover = false;
            for(auto layerRect : via.GetLayerRectsRef()) {
                if(layerRect.layer_name_ == M1_name) {
                    llx = layerRect.rects_[0].ll.x * dbuPerMicron;        
                    lly = layerRect.rects_[0].ll.y * dbuPerMicron;        
                    urx = layerRect.rects_[0].ur.x * dbuPerMicron;        
                    ury = layerRect.rects_[0].ur.y * dbuPerMicron;        
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
            if(!M1cover) {
                M1M2ViaName = via.GetName();
                break;
            }
        } // find m1-m2 via that doesn't conflict with m1 obs

        M2wideFail= false;
        M2thinFail = false;
        for(auto rect : M2obsRect) {
            auto tmpRect = rect;
            tmpRect.ll.x -= (M2_minlength / 2 + M2_spacing); 
            tmpRect.ur.x += (M2_minlength / 2 + M2_spacing); 
            tmpRect.ll.y -= (M2_width / 2 + M2_spacing);
            tmpRect.ur.y += (M2_width / 2 + M2_spacing);
            
            if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
                M2thinFail = true;
                break;
            }
        } // m2 side min area doesn't conflict with m2 obs
        if(touchPoint.x - M2_minlength / 2 - M2_spacing / 2 < component.location_.x || 
                    touchPoint.x + M2_minlength / 2 + M2_spacing / 2 > component.location_.x + component.size_.x ||
                    touchPoint.y - M2_width / 2 - M2_spacing / 2 < component.location_.y || 
                    touchPoint.y + M2_width / 2 + M2_spacing / 2 > component.location_.y + component.size_.y) {
            M2thinFail = true;
        }
        for(auto rect : M2obsRect) {
            auto tmpRect = rect;
            tmpRect.ll.x -= (M2_minlength / 6 + M2_spacing); 
            tmpRect.ur.x += (M2_minlength / 6 + M2_spacing); 
            tmpRect.ll.y -= (M2_width * 3 / 2 + M2_spacing);
            tmpRect.ur.y += (M2_width * 3 / 2 + M2_spacing);
            
            if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
                M2wideFail = true;
                break;
            }
        }
        
        if(touchPoint.x - M2_minlength / 6 - M2_spacing / 2 < component.location_.x || 
                    touchPoint.x + M2_minlength / 6 + M2_spacing / 2 > component.location_.x + component.size_.x ||
                    touchPoint.y - M2_width * 3 / 2 - M2_spacing / 2 < component.location_.y || 
                    touchPoint.y + M2_width * 3 / 2 + M2_spacing / 2 > component.location_.y + component.size_.y) { 
            M2wideFail = true;            
        }
        M2cover = M2wideFail && M2thinFail;  
        if(component.name_ == "cx1") {
            std::cout << signal << ": " << std::endl;
            std::cout << "far M2cover : " << M2cover << " M2wideFail: " << M2wideFail << " M2thinFail: " << M2thinFail << std::endl;
            std::cout << "touchpoint x : " << touchPoint.x << " " << touchPoint.y;
            std::cout << " M2minlength/2: " << M2_minlength / 2 << " component.x: " << component.location_.x << std::endl; 
        }


        for(auto rect : V1obsRect) {
            auto tmpRect = rect;
            tmpRect.ll.x -= (V1_width / 2 + V1_spacing); 
            tmpRect.ur.x += (V1_width / 2 + V1_spacing); 
            tmpRect.ll.y -= (V1_width / 2 + V1_spacing);
            tmpRect.ur.y += (V1_width / 2 + V1_spacing);
            
            if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
                V1cover = true;
                break;
            }
        }//V1 via spacing
      
        if(component.name_ == "cx1")
            std::cout << V1cover << M2cover << M1cover << std::endl;
        
 
       if(V1cover == false && M2cover == false && M1cover == false) {       
            foundM1M3 = true;
            break;
        }
    }


    if(foundM1M3) {
        Wire tmpWire;
        
        if(signal == "POWER") { 
            powerPoint.x = touchX;
            powerPoint.y = touchY;
        }
     
        auto& xMesh = pwgnd_.xMesh;
        bool move_right = false;
        for(int i = 0; i < xMesh.size() - 1; i++) {
            if(touchX > xMesh[i] && touchX < (xMesh[i] + xMesh[i + 1]) / 2) {
                move_right = true; 
                break;
            }
            else if(touchX >= (xMesh[i] + xMesh[i + 1]) / 2 && touchX < xMesh[i + 1]) {
                move_right = false;
                break;
            }
        }
   
        tmpWire.coorX[0] = touchX;
        tmpWire.coorY[0] = touchY;
        tmpWire.numPathPoint = 1;
        tmpWire.layerName = layers[2].GetName();
        tmpWire.width = 0;
        tmpWire.viaName = M1M2ViaName;
        Wires.push_back(tmpWire);//M2 to M1 via

        int zRouteOffset = 0;
        if(signal == "GROUND" && touchX == powerPoint.x &&
                ((touchY < signalY && powerPoint.y > touchY) || (touchY > signalY && powerPoint.y < touchY))) { //Z route, if power and gnd pins have the same X
           
            if(move_right)
                zRouteOffset = M3_pitch;
            else
                zRouteOffset = - M3_pitch;

            tmpWire.coorX[0] = touchX + zRouteOffset;//
            tmpWire.coorY[0] = touchY;
            tmpWire.coorX[1] = touchX + zRouteOffset;//
            if(fabs(signalY - touchY) >= M3_minlength)
                tmpWire.coorY[1] = signalY;
            else {
                tmpWire.coorY[1] = (signalY > touchY)? touchY + M3_minlength : touchY - M3_minlength;
            }
            tmpWire.numPathPoint = 2;
            tmpWire.layerName = layers[4].GetName(); //M3
            tmpWire.width = M3_width;
            Wires.push_back(tmpWire);//M3 wire
            
            tmpWire.coorX[0] = touchX - M2_width / 2;//
            tmpWire.coorY[0] = touchY;
            tmpWire.coorX[1] = touchX + zRouteOffset + M2_width / 2;//
            tmpWire.coorY[1] = touchY;
            
            tmpWire.numPathPoint = 2;
            tmpWire.layerName = layers[2].GetName(); //M2
            tmpWire.width = M2_width;
            Wires.push_back(tmpWire);//M2 wire

        }
        else { //normal route without Z
            tmpWire.coorX[0] = touchX;
            tmpWire.coorY[0] = touchY;
            tmpWire.coorX[1] = touchX;
            if(fabs(signalY - touchY) >= M3_minlength)
                tmpWire.coorY[1] = signalY;
            else {
                tmpWire.coorY[1] = (signalY > touchY)? touchY + M3_minlength : touchY - M3_minlength;
            }
            tmpWire.numPathPoint = 2;
            tmpWire.layerName = layers[4].GetName(); //M3
            tmpWire.width = M3_width;
            Wires.push_back(tmpWire);//M3 wire

        }

        int viaID = topLayerId_2_viaId_[6];
        tmpWire.coorX[0] = touchX + zRouteOffset;
        tmpWire.coorY[0] = signalY;
        tmpWire.numPathPoint = 1;
        tmpWire.layerName = layers[6].GetName();
        tmpWire.width = 0;
        tmpWire.viaName = vias[viaID].GetName();
        Wires.push_back(tmpWire);//M3 to M4 via, mesh


        viaID = topLayerId_2_viaId_[4];
        tmpWire.coorX[0] = touchX + zRouteOffset;//
        tmpWire.coorY[0] = touchY;
        tmpWire.numPathPoint = 1;
        tmpWire.layerName = layers[4].GetName();
        tmpWire.width = 0;
        tmpWire.viaName = vias[viaID].GetName();
        Wires.push_back(tmpWire);//M3 to M2 via, touch
        
        if(!M2thinFail) {
            tmpWire.coorX[0] = touchX - (int) FitGrid(M2_minlength / 2, (double) dbuPerMicron * manufacturing_grid);
            tmpWire.coorX[1] = touchX + (int) FitGrid(M2_minlength / 2, (double) dbuPerMicron * manufacturing_grid);
            tmpWire.width = M2_width;
        }
        else {
            int length = std::max(M2_width / 2, (int) FitGrid(M2_minlength / 6, (double) dbuPerMicron * manufacturing_grid)); //sometimes M2_minlength / 3 < M2_width, too thin 
            tmpWire.coorX[0] = touchX - length;
            tmpWire.coorX[1] = touchX + length;
            tmpWire.width = 3 * M2_width;
        }

        tmpWire.coorY[0] = touchY;
        tmpWire.coorY[1] = touchY;
        tmpWire.numPathPoint = 2;
        tmpWire.layerName = layers[2].GetName(); //M2
        Wires.push_back(tmpWire);//M2 min area 
        
        
        int M3_minLength = layer_min_length_[4] * dbuPerMicron;
        if(fabs(touchY - signalY) < M3_minLength) {
            tmpWire.coorX[0] = touchX; 
            tmpWire.coorX[1] = touchX; 
            tmpWire.coorY[0] = signalY;
            tmpWire.coorY[1] = (signalY < touchY)? signalY + M3_minLength : signalY - M3_minLength;
            
            tmpWire.width = M3_width;
            tmpWire.layerName = layers[4].GetName();
            tmpWire.numPathPoint = 2;
            Wires.push_back(tmpWire); //M3 min area
        }

    }
    return foundM1M3;
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

    Track track = tracks[layerid_2_trackid_[4]]; //M3

    std::map<int, int> trackClosestPinPoint;
    std::map<int, int> trackClosestOBSPoint;
    
    /*map<int, int> closestPinPoint;
    map<int, int> closestOBSPoint;
    */
    int M2_minLength = layer_min_length_[2] * dbuPerMicron;
    int M2_pitch = layers[2].GetPitchX() * dbuPerMicron;
    int M2_width = layers[2].GetWidth() * dbuPerMicron;
    int M2_spacing = (M2_pitch - M2_width);
     
    int viaIdx = topLayerId_2_viaId_[2];
    auto via = vias[viaIdx];
    auto viaRect = via.GetLayerRectsRef()[0].rects_[0];
    double h = (viaRect.ur.x - viaRect.ll.x) * dbuPerMicron;
    double v = (viaRect.ur.y - viaRect.ll.y) * dbuPerMicron;
    double viaWidth = (h > v)? v : h;
    double viaLength = (h > v)? h : v;
    
    int width = layers[4].GetWidth() * dbuPerMicron; //M3
    int pitch = layers[4].GetPitchX() * dbuPerMicron;
    findClosestTouchPoints(pinRect, trackClosestPinPoint, track, 0, signalY); // no expand
    findClosestTouchPoints(obsRect, trackClosestOBSPoint, track, pitch - width / 2, signalY); // this is width/2 + spacing

    bool foundM2 = false;
    
    int touchX, touchY;
   

    for(auto pinPoint : trackClosestPinPoint) { // Cell library gaurantee min area on M2
        int trackIdx = pinPoint.first;
        //touchX = pinPoint.first;
        touchX = track.GetStart() + trackIdx * track.GetStep();
        touchY = pinPoint.second;
 
        Point2D<double> touchPoint(touchX, touchY);
        bool M2cover = false;
        for(auto rect : obsRect) {
            auto tmpRect = rect;
            tmpRect.ll.x -= (viaLength / 2 + M2_spacing); // just gaurantee via 
            tmpRect.ur.x += (viaLength / 2 + M2_spacing); 
            tmpRect.ll.y -= M2_spacing;
            tmpRect.ur.y += M2_spacing;
            
            if(tmpRect.BoundaryExclusiveCover(touchPoint)) {
                if(component.name_ == "p_ae__131_acpx0" && signal == "GROUND") {
                    std::cout << "M2 " << component.name_ << ": " << touchX << " " << touchY << std::endl;
                    std::cout << rect << std::endl;
                }
                
                M2cover = true;
                break;
            }
        } // m2 side min area doesn't conflict with m2 obs       
        if(!M2cover) {
            foundM2 = true;
            break;
        }
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
        
        if(touchY < signalY)
            touchY -= M2_width / 2;
        else
            touchY += M2_width / 2;

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
            tmpWire.layerName = layers[4].GetName(); //M3
            tmpWire.width = M3_width;
            Wires.push_back(tmpWire);//M3 wire
            
            tmpWire.coorX[0] = touchX - M2_width / 2;//
            tmpWire.coorY[0] = touchY;
            tmpWire.coorX[1] = touchX + zRouteOffset + M2_width / 2;//
            tmpWire.coorY[1] = touchY;
            
            tmpWire.numPathPoint = 2;
            tmpWire.layerName = layers[2].GetName(); //M2
            tmpWire.width = M2_width;
            Wires.push_back(tmpWire);//M2 wire
            
            tmpWire.coorX[0] = touchX + zRouteOffset;
            tmpWire.coorY[0] = touchY;
            tmpWire.numPathPoint = 1;
            tmpWire.width = 0;
            viaID = topLayerId_2_viaId_[4]; //M2-M3 via, touch side
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
            tmpWire.width = M3_width;
            Wires.push_back(tmpWire);//M3 wire

            tmpWire.coorX[0] = touchX;
            tmpWire.coorY[0] = touchY;
            viaID = topLayerId_2_viaId_[4]; //M2-M3 via, touch side
            tmpWire.numPathPoint = 1;
            tmpWire.width = 0;
            tmpWire.layerName = layers[4].GetName(); //M3
            tmpWire.viaName = vias[viaID].GetName();
            Wires.push_back(tmpWire);
        }

        int M3_minLength = layer_min_length_[4] * dbuPerMicron;
        
        int M2_width = layers[2].GetWidth() * dbuPerMicron;
        int M3_width = layers[4].GetWidth() * dbuPerMicron;
        
        tmpWire.coorX[0] = touchX + zRouteOffset; 
        tmpWire.coorY[0] = signalY; 
        tmpWire.numPathPoint = 1;
        tmpWire.layerName = layers[6].GetName();
        tmpWire.width = 0;
        viaID = topLayerId_2_viaId_[6];
        tmpWire.viaName = vias[viaID].GetName();
        Wires.push_back(tmpWire); //M3-M4 via

    }
    return foundM2;

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
                std::cout << "warning: UNABLE TO PATTERN ROUTE POWER: " << component.name_ << std::endl;
        }
    }
    M1ground = M1DetailedRouteSNet(component, "GROUND");
    if(!M1ground) {
        M2ground = M2DetailedRouteSNet(component, "GROUND", gndY, pwgnd_.gndWires, powerPoint);
        if(!M2ground) {
            M1M3ground = M1M3DetailedRouteSNet(component, "GROUND", gndY, pwgnd_.gndWires, powerPoint); 
            if(!M1M3ground)
                std::cout << "warning: UNABLE TO PATTERN ROUTE GROUND: " << component.name_ << std::endl;
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
    
    //if(component.name_ == "p_atv__56_57_6_acx1")
    //    cout << "component: " << component.name_ << " " << M1power << M2power << M1M3power << " " << M1ground << M2ground << M1M3ground << endl;
    
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
    for(auto& component : components_) {
        DetailedRouteSNetComp(component);
    }

    if(verbose_ > none) {
        std::cout << "Power routing completes/fails: " << POWER_FOUND << " / " << POWER_UNFOUND << std::endl;
        std::cout << "Ground routing completes/fails: " << GROUND_FOUND << " / " << GROUND_UNFOUND << std::endl;
    }
    
    M2MetalFill("POWER");
    M2MetalFill("GROUND");
}




}


