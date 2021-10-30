
#include <algorithm>
#include <math.h>
#include "pwroute.h"

using namespace phydb;
namespace pwroute {

void PWRoute::LinkTrackToLayer() {
    auto tracks_ref = db_ptr_->GetTracksRef();

    for(auto layer : db_ptr_->GetLayersRef()) {
        if(layer.GetType() == ROUTING) {    //only on routing layer
            if(layer.GetDirection() == HORIZONTAL) { // HORIZONTAL corresponds to Y location of a track
                for(int i = 0; i < tracks_ref.size(); i++) {
                    auto track = tracks_ref[i];
                    for(auto layerName : track.GetLayerNames()) {
                        if(layerName == layer.GetName() && track.GetDirection() == Y) {
                            layerid_2_trackid_.insert( std::pair<int, int> (layer.GetID(), i));
                        }
                    }
                }
            }
            else if(layer.GetDirection() == VERTICAL) { // VERTICAL corresponds to X location of a track
                for(int i = 0; i < tracks_ref.size(); i++) {
                    auto track = tracks_ref[i];
                    for(auto layerName : track.GetLayerNames()){
                        if(layerName == layer.GetName() && track.GetDirection() == X) {
                            layerid_2_trackid_.insert( std::pair<int, int> (layer.GetID(), i));
                        }
                    }
                }
            }
            else {
                std::cout << "error: unknown direction of lefDB layers" << std::endl;
                exit(1);
            }
        }
    }
}

void PWRoute::SetDefaultVia() {
    bool debug = false;
    auto layer_v = db_ptr_->GetTechPtr()->GetLayersRef();
    auto vias = db_ptr_->GetTechPtr()->GetLefViasRef();
    for(int layerid = 0; layerid < layer_v.size(); layerid++) {
        if(layer_v[layerid].GetType() != ROUTING)
            continue;
        std::string layer_name = layer_v[layerid].GetName();

        int found_lefvia_id = -1;
        for(int lefvia_id = 0; lefvia_id < vias.size(); lefvia_id++) {
            auto lefvia = vias[lefvia_id];
            auto layer_rects = lefvia.GetLayerRectsRef();
            if(layer_rects[0].layer_name_ == layer_name || 
               layer_rects[1].layer_name_ == layer_name || 
               layer_rects[2].layer_name_ == layer_name) {
                   found_lefvia_id = lefvia_id;
                   break;
               }
        }
        assert(found_lefvia_id != -1);
        topLayerId_2_viaId_[layerid] = found_lefvia_id;
        if(debug)
            std::cout << "layerid: " << layerid << " " << found_lefvia_id << std::endl;
    }
}

void PWRoute::InitCluster() {
    bool debug = false;
    int lx, ux, ly, uy;
    int cnt_y = 0, cnt_x = 0;
    
    int pre_ux = 0, pre_uy;
    int xoffset = pwgnd_.vmeshWidth + pwgnd_.vmeshSpacing / 2;
    for(auto& column : db_ptr_->GetDesignPtr()->GetClusterColsRef()) {
        
        std::set<int> yPW;
        std::set<int> yGND;
        lx = column.GetLX();
        ux = column.GetUX();
        std::vector<int> ly_v = column.GetLY();
        std::vector<int> uy_v = column.GetUY();
        
        //start 
        pre_uy = 0;
        if(column.GetBotSignal() == "GND")     //GND
            cnt_y = 0;
        else                                   //VDD
            cnt_y = 1;

        if(pre_ux == 0) {
            int x1 = (lx + db_ptr_->GetDesignPtr()->GetDieArea().LLX()) / 2;
            int x2 = db_ptr_->GetDesignPtr()->GetDieArea().LLX() + xoffset;
            pwgnd_.xMesh.push_back(std::max(x1, x2)); //first column, the left boundary   
        }   
        else 
            pwgnd_.xMesh.push_back((lx + pre_ux)/2); 
        
        pre_ux = ux;

        //iterate 
        for(int i = 0; i < ly_v.size(); i++) {
            ly = ly_v[i];
            if(pre_uy != 0 && ly != pre_uy) //if two rows has a gap, use the previous uy
                ly = pre_uy;
            uy = uy_v[i];
            pre_uy = uy;
            if(cnt_y % 2 == 0) {
                yGND.insert(ly);
                yPW.insert(uy);
            }
            else {
                yPW.insert(ly);
                yGND.insert(uy);
            }
            cnt_y++;
        }

        // end
        
        std::vector<int> tmpPW;
        std::vector<int> tmpGND;
        for(auto y : yGND)
            tmpGND.push_back(y);
        sort(tmpGND.begin(), tmpGND.end());

        for(auto y : yPW)
            tmpPW.push_back(y);
        sort(tmpPW.begin(), tmpPW.end()); //ascending order
            
        pwgnd_.poweryMesh.push_back(tmpPW);
        pwgnd_.gndyMesh.push_back(tmpGND);
    }
    
    int x1 = (ux + db_ptr_->GetDesignPtr()->GetDieArea().URX()) / 2;
    int x2 = db_ptr_->GetDesignPtr()->GetDieArea().URX() - xoffset;
    
    pwgnd_.xMesh.push_back(std::min(x1, x2));  //The right boundary  
    if(debug) {
        std::cout << "xMesh: " << std::endl;
        for(int i = 0; i < pwgnd_.xMesh.size(); i++)
            std::cout << pwgnd_.xMesh[i] << " ";
        std::cout << std::endl;
    }
}


/* round num to be a multiple of 2*manufacturingGrid
The wirewidth should be multiple of 2*manufacturingGrid */ 
double PWRoute::FitGrid(double num, double manufacturingGrid) {  
    
    if(manufacturingGrid == 0)
        return num;
    else {
        int multiple = (num / manufacturingGrid) / 2;
        if(num == ((double) multiple )* manufacturingGrid * 2)   //if it is a multiple of 2*manufacturing grid
            return num;
        else {
            return (multiple + 1) * manufacturingGrid * 2;
        }

    }
}

void PWRoute::SNetConfig() {
    
    int dbuPerMicron = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();
    double manufacturingGrid = db_ptr_->GetTechPtr()->GetManufacturingGrid();
    pwgnd_.signalNetWidth = db_ptr_->GetLayersRef().at(2).GetWidth() * dbuPerMicron; //use m2 width as standard

    /*set hmeshWidth for horizontal mesh M4*/ 
    int viaIdx = topLayerId_2_viaId_[cluster_horizontal_layer]; //check the via of M3-M4
    auto via = db_ptr_->GetTechPtr()->GetLefViasRef()[viaIdx];
    auto rect = via.GetLayerRectsRef()[2].rects_[0];
    double h = rect.URX() - rect.LLX();
    double v = rect.URY() - rect.LLY();
    int viaLength = (h > v)? h * dbuPerMicron : v * dbuPerMicron;
    pwgnd_.hmeshWidth = std::max(cluster_mesh_multiple_width * pwgnd_.signalNetWidth, viaLength); 
    pwgnd_.hmeshWidth = FitGrid(pwgnd_.hmeshWidth, manufacturingGrid * dbuPerMicron);
    if(verbose_ > 1)
    	std::cout << "hmeshWidth: " << pwgnd_.hmeshWidth << std::endl;

    /*set hmeshExt for horizontal mesh M4*/ 
    viaIdx = topLayerId_2_viaId_[cluster_vertical_layer]; //check the via of M4-M5
    via = db_ptr_->GetTechPtr()->GetLefViasRef()[viaIdx];
    rect = via.GetLayerRectsRef()[0].rects_[0]; //get M4 segment
    h = rect.URX() - rect.LLX();
    v = rect.URY() - rect.LLY();
    viaLength = (h > v)? h * dbuPerMicron : v * dbuPerMicron;
    pwgnd_.hmeshExt = std::max(pwgnd_.hmeshWidth, viaLength) / 2; 
    pwgnd_.hmeshExt = FitGrid(pwgnd_.hmeshExt, manufacturingGrid * dbuPerMicron / 2); 
    if(verbose_ > 1)
    	std::cout << "hmeshExt: " << pwgnd_.hmeshExt << std::endl;

    /*set vmeshWidth for vertical mesh M5*/ 
    viaIdx = topLayerId_2_viaId_[cluster_vertical_layer]; //check the via of M4-M5
    via = db_ptr_->GetTechPtr()->GetLefViasRef()[viaIdx];
    rect = via.GetLayerRectsRef()[2].rects_[0];
    h = rect.URX() - rect.LLX();
    v = rect.URY() - rect.LLY();
    viaLength = (h > v)? h * dbuPerMicron : v * dbuPerMicron;
    pwgnd_.vmeshWidth = std::max(cluster_mesh_multiple_width * pwgnd_.signalNetWidth, viaLength); 
    pwgnd_.vmeshWidth = FitGrid(pwgnd_.vmeshWidth, manufacturingGrid * dbuPerMicron);

    if(verbose_ > 1)
    	std::cout << "vmeshWidth: " << pwgnd_.hmeshWidth << std::endl;

    /*set vmeshExt for vertical mesh M5*/ 
    viaIdx = topLayerId_2_viaId_[high_mesh_layer]; //check the via of M5-M6
    via = db_ptr_->GetTechPtr()->GetLefViasRef()[viaIdx];
    rect = via.GetLayerRectsRef()[0].rects_[0]; //get M5 segment
    h = rect.URX() - rect.LLX();
    v = rect.URY() - rect.LLY();
    viaLength = (h > v)? h * dbuPerMicron : v * dbuPerMicron;
    pwgnd_.vmeshExt = std::max(pwgnd_.vmeshWidth, viaLength) / 2; 
    pwgnd_.vmeshExt = FitGrid(pwgnd_.vmeshExt, manufacturingGrid * dbuPerMicron / 2);
    
        auto layers = db_ptr_->GetLayersRef();
    if(layers[0].GetDirection() == HORIZONTAL) {
	    pwgnd_.vMeshLayerName = db_ptr_->GetLayersRef().at(cluster_horizontal_layer).GetName(); //M4
	    pwgnd_.hMeshLayerName = db_ptr_->GetLayersRef().at(cluster_vertical_layer).GetName(); //M5
    }
    else { //usually this branch because m2/m4 = horizontal
        pwgnd_.vMeshLayerName = db_ptr_->GetLayersRef().at(cluster_vertical_layer).GetName(); //M5
	    pwgnd_.hMeshLayerName = db_ptr_->GetLayersRef().at(cluster_horizontal_layer).GetName(); //M4
         
    }
    if(verbose_ > none) {
        std::cout << "vertical mesh layer: " << pwgnd_.vMeshLayerName << std::endl;
        std::cout << "horizontal mesh layer: " << pwgnd_.hMeshLayerName << std::endl;
    }
    
    /*set vmeshSpacing for vertical mesh M5*/
    auto* spacing_table = layers[cluster_vertical_layer].GetSpacingTable();
    double m5_spacing = spacing_table->GetSpacingForWidth(pwgnd_.vmeshWidth);
    pwgnd_.vmeshSpacing = pwgnd_.vmeshWidth + m5_spacing * dbuPerMicron;
    if(verbose_ > 1)
    	std::cout << "vmesh width: " << pwgnd_.vmeshWidth << " m5_spacing: " << m5_spacing << " vmesh spacing: " << pwgnd_.vmeshSpacing << std::endl; 

    pwgnd_.lastHLayerID = high_mesh_layer;
    pwgnd_.lastHLayerName = layers[high_mesh_layer].GetName();
    if(verbose_ > none) {
        std::cout << "High H layer: " << pwgnd_.lastHLayerName << std::endl;
    }
}

void PWRoute::PreprocessComponents()
{
    for(auto& component : db_ptr_->GetDesignPtr()->GetComponentsRef()) { 
        this->components_.emplace_back(db_ptr_, component);  
    }

}

void PWRoute::SetDBPtr(phydb::PhyDB* p) {
    db_ptr_ = p;
}

int PWRoute::SafeBoundaryDistance() {
    auto vias = db_ptr_->GetTechPtr()->GetLefViasRef();
    double dbuPerMicron = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();
    auto V1 = vias[topLayerId_2_viaId_[2]]; //the via of M1-M2 
    auto V2 = vias[topLayerId_2_viaId_[4]]; //the via of M2-M3

    double maxDistance = 0;

    for(int i = 0; i < 3; i++) { // via has only 3 layerRects, auto layerRect : V1.GetLayerRects()) 
        auto layer_rect = V1.GetLayerRectsRef()[i];
        for(auto rect : layer_rect.rects_) {
            maxDistance = (maxDistance > fabs(rect.ll.x))? maxDistance : fabs(rect.ll.x);
            maxDistance = (maxDistance > fabs(rect.ll.y))? maxDistance : fabs(rect.ll.y);
            maxDistance = (maxDistance > fabs(rect.ur.x))? maxDistance : fabs(rect.ur.x);
            maxDistance = (maxDistance > fabs(rect.ur.y))? maxDistance : fabs(rect.ur.y);
        }
    }
    for(int i = 0; i < 3; i++) { // via has only 3 layerRects, auto layerRect : V1.GetLayerRects()) 
        auto layer_rect = V2.GetLayerRectsRef()[i];
        for(auto rect : layer_rect.rects_) {
            maxDistance = (maxDistance > fabs(rect.ll.x))? maxDistance : fabs(rect.ll.x);
            maxDistance = (maxDistance > fabs(rect.ll.y))? maxDistance : fabs(rect.ll.y);
            maxDistance = (maxDistance > fabs(rect.ur.x))? maxDistance : fabs(rect.ur.x);
            maxDistance = (maxDistance > fabs(rect.ur.y))? maxDistance : fabs(rect.ur.y);
        }
    }
    //cout << "maxBoundaryDistance: " << maxDistance << std::endl;
    return (int)(maxDistance * dbuPerMicron); 
}

void PWRoute::PlaceHighLayerVias(std::vector<Wire>& wires, int X, int Y, 
        std::string topLayerName, int viaIdx, int length, int viaDistance, int layerWidth) {
    
    auto& vias = db_ptr_->GetTechPtr()->GetLefViasRef();
    double dbuPerMicron = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();

    auto via = vias[viaIdx];
    std::string viaName = via.GetName();
    auto rect = via.GetLayerRectsRef()[0].rects_[0];
    double h = rect.ur.x - rect.ll.x;
    double v = rect.ur.y - rect.ll.y;
    double viaLength = (h > v)? h : v;
    viaLength *= dbuPerMicron;

    Wire tmpWire;
    
    tmpWire.layerName = topLayerName;
    tmpWire.width = 0;
    tmpWire.coorX[0] = X;
    tmpWire.numPathPoint = 1;
    tmpWire.viaName = viaName;
     
    int numVias = length / viaDistance;
    int ystart = Y - viaDistance * (numVias - 1) / 2;
    int yend = Y + viaDistance * (numVias - 1) / 2;
    int ypos = ystart;

    for(int i = 0; i < numVias; i++) {
        tmpWire.coorY[0] = ypos;
        ypos += viaDistance;
        wires.push_back(tmpWire);
    }

    tmpWire.width = viaLength;
    tmpWire.numPathPoint = 2;
    tmpWire.coorX[0] = X;
    tmpWire.coorY[0] = ystart - viaDistance / 2;
    tmpWire.coorX[1] = X;
    tmpWire.coorY[1] = yend + viaDistance / 2;
    wires.push_back(tmpWire);
    
}

void PWRoute::RouteHighLayerReinforcement(std::string signal) {
    Wire tmpWire;

    Rect2D<int> dieArea = db_ptr_->GetDesignPtr()->GetDieArea();
    auto& layers = db_ptr_->GetLayersRef();
    auto& vias = db_ptr_->GetTechPtr()->GetLefViasRef();
    double dbuPerMicron = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();

    std::string lastHLayerName = pwgnd_.lastHLayerName;
    int lastHLayerID = pwgnd_.lastHLayerID;
    
    int width = layers[lastHLayerID].GetWidth() * dbuPerMicron * high_mesh_multiple_width;
    double layer_pitchy = layers[lastHLayerID].GetPitchY();
    int pitch = layer_pitchy * dbuPerMicron * high_mesh_multiple_width;  // Because the high mesh is always horizontal, we use pitchy as the pitch

    int step =  pitch * high_mesh_multiple_step;
    int safeDistance = SafeBoundaryDistance();
    double manufacturing_grid = db_ptr_->GetTechPtr()->GetManufacturingGrid();
    width = FitGrid(width, manufacturing_grid * dbuPerMicron);
    pitch = FitGrid(pitch, manufacturing_grid * dbuPerMicron);
    int nReinforcement = (dieArea.ur.y - dieArea.ll.y - 2 * (pitch + safeDistance + width / 2)) / step + 1;
    
    int offset;
    
    phydb::Range<int> yrange; 
    int viaDistance = (layers[lastHLayerID - 1].GetWidth() + layers[lastHLayerID - 1].GetSpacing()) * dbuPerMicron; 
    
    if(signal == "POWER") {
        offset = pitch;
    }
    else {
        offset = - pitch;
    }
    auto& xMesh = pwgnd_.xMesh;
    auto& wires = (signal == "POWER")? pwgnd_.powerWires : pwgnd_.gndWires;
    auto& yranges = (signal == "POWER")? pwgnd_.powerHighLayerY : pwgnd_.gndHighLayerY;

    std::string vMeshLayerName = pwgnd_.vMeshLayerName;
    int vlayerID = db_ptr_->GetTechPtr()->GetLayerId(vMeshLayerName);
    if((nReinforcement == 1 && dieArea.ur.y - dieArea.ll.y > 3 * pitch + 2 * safeDistance) ||
            nReinforcement > 1) {
        
        for(int i = 0; i < nReinforcement; i++) {
            tmpWire.layerName = lastHLayerName;
            tmpWire.width = width;
            tmpWire.coorX[0] = dieArea.ll.x;
            tmpWire.coorY[0] = dieArea.ll.y + offset + i * step + pitch + safeDistance + width / 2;
            tmpWire.coorX[1] = dieArea.ur.x;
            tmpWire.coorY[1] = dieArea.ll.y + offset + i * step + pitch + safeDistance + width / 2;
            tmpWire.numPathPoint = 2;
            tmpWire.viaName = "";
            wires.push_back(tmpWire);

            yrange.begin = tmpWire.coorY[0] - width / 2; 
            yrange.end = tmpWire.coorY[0] + width / 2; 
            yranges.push_back(yrange);
    
            for(int i = vlayerID + 2; i <= lastHLayerID; i += 2) { //M5-M6
                int viaIdx = topLayerId_2_viaId_[i];
                std::string topLayerName = layers[i].GetName();
                int layerWidth = layers[i].GetWidth() * dbuPerMicron;
                std::string viaName = vias[viaIdx].GetName();
                for(int j = 0; j < xMesh.size(); j ++) {
                    int xpos = (signal == "POWER")? xMesh[j] + pwgnd_.vmeshSpacing : xMesh[j] - pwgnd_.vmeshSpacing;
                    PlaceHighLayerVias(wires, xpos, tmpWire.coorY[0], topLayerName, viaIdx, width, viaDistance, layerWidth);
                }
            }
        }
        
        int lasty = dieArea.ll.y + pitch + (nReinforcement - 1) * step + pitch + safeDistance + width / 2; 
   

        if(dieArea.ur.y - lasty > 2 * pitch + safeDistance + width / 2) {
            //cout << "enter extra" << endl;
            tmpWire.layerName = lastHLayerName;
            tmpWire.width = width;
            tmpWire.coorX[0] = dieArea.ll.x;
            tmpWire.coorY[0] = dieArea.ur.y + offset - pitch - safeDistance - width / 2;
            tmpWire.coorX[1] = dieArea.ur.x;
            tmpWire.coorY[1] = dieArea.ur.y + offset - pitch - safeDistance - width / 2;
            tmpWire.numPathPoint = 2;
            tmpWire.viaName = "";
            wires.push_back(tmpWire);

            yrange.begin = tmpWire.coorY[0] - width / 2; 
            yrange.end = tmpWire.coorY[0] + width / 2; 
            yranges.push_back(yrange);
        
            for(int i = vlayerID + 2; i <= lastHLayerID; i += 2) { //M5-M6
                int viaIdx = topLayerId_2_viaId_[i];
                std::string topLayerName = layers[i].GetName();
                int layerWidth = layers[i].GetWidth() * dbuPerMicron;
                std::string viaName = vias[viaIdx].GetName();
                for(int j = 0; j < xMesh.size(); j ++) {
                    int xpos = (signal == "POWER")? xMesh[j] + pwgnd_.vmeshSpacing : xMesh[j] - pwgnd_.vmeshSpacing;
                    PlaceHighLayerVias(wires, xpos, tmpWire.coorY[0], topLayerName, viaIdx, width, viaDistance, layerWidth);
                }
            }
        }
    }
    else {  //If there is only one reinforcement 
            tmpWire.layerName = lastHLayerName;
            tmpWire.width = width;
            tmpWire.coorX[0] = dieArea.ll.x;
            tmpWire.coorY[0] = (dieArea.ll.y + dieArea.ur.y) / 2 + offset + width / 2;
            tmpWire.coorX[1] = dieArea.ur.x;
            tmpWire.coorY[1] = (dieArea.ll.y + dieArea.ur.y) / 2 + offset + width / 2;
            tmpWire.numPathPoint = 2;
            tmpWire.viaName = "";
            wires.push_back(tmpWire);

            yrange.begin = tmpWire.coorY[0] - width / 2; 
            yrange.end = tmpWire.coorY[0] + width / 2; 
            yranges.push_back(yrange);
    
            for(int i = vlayerID + 2; i <= lastHLayerID; i += 2) { //M5-M6
                int viaIdx = topLayerId_2_viaId_[i];
                std::string topLayerName = layers[i].GetName();
                int layerWidth = layers[i].GetWidth() * dbuPerMicron;
                std::string viaName = vias[viaIdx].GetName();
                for(int j = 0; j < xMesh.size(); j ++) {
                    int xpos = (signal == "POWER")? xMesh[j] + pwgnd_.vmeshSpacing : xMesh[j] - pwgnd_.vmeshSpacing;
                    PlaceHighLayerVias(wires, xpos, tmpWire.coorY[0], topLayerName, viaIdx, width, viaDistance, layerWidth);
                }
            }


    }

}

bool PWRoute::InHighLayerViaRange(std::string signal, int ypos) {

    auto& yranges = (signal == "POWER")? pwgnd_.powerHighLayerY : pwgnd_.gndHighLayerY;
    bool in = false;
    for(auto yrange : yranges) {
        if(yrange.begin < ypos && yrange.end > ypos) {
            in = true;
            break;
        }
    }
    return in;
}

bool PWRoute::NearHighLayerViaRange(std::string signal, int ypos, int& midRange) {
    double dbuPerMicron = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();
    auto& layers = db_ptr_->GetLayersRef();
    
    double layer_pitchy = layers[2].GetPitchY();

    auto& yranges = (signal == "POWER")? pwgnd_.powerHighLayerY : pwgnd_.gndHighLayerY;
    bool in = false;
    int M2_spacing = (layer_pitchy - layers[2].GetWidth()) * dbuPerMicron;
    int width = pwgnd_.hmeshWidth;

    for(auto yrange : yranges) {
        if(yrange.begin - M2_spacing < ypos + width / 2 && yrange.end + M2_spacing > ypos - width / 2) {
            in = true;
            midRange = (yrange.begin + yrange.end) / 2;
            break;
        }
    }
    return in;
}

void PWRoute::RouteLowLayerMesh(std::string signal) {
    Wire tmpWire;

    Rect2D<int> dieArea = db_ptr_->GetDesignPtr()->GetDieArea();
    auto& layers = db_ptr_->GetLayersRef();
    auto& vias = db_ptr_->GetTechPtr()->GetLefViasRef();
    double dbuPerMicron = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();
    
    std::string vMeshLayerName = pwgnd_.vMeshLayerName;
    std::string hMeshLayerName = pwgnd_.hMeshLayerName;
    int vLayerID = db_ptr_->GetTechPtr()->GetLayerId(vMeshLayerName);
    int hLayerID = db_ptr_->GetTechPtr()->GetLayerId(hMeshLayerName);
    
    int xoffset = (signal == "POWER")? pwgnd_.vmeshSpacing / 2 : (-1) * pwgnd_.vmeshSpacing / 2; 
    
    auto& xMesh = pwgnd_.xMesh;
    auto& yMesh = (signal == "POWER")? pwgnd_.poweryMesh : pwgnd_.gndyMesh;
    auto& opyMesh = (signal == "POWER")? pwgnd_.gndyMesh : pwgnd_.poweryMesh;
    auto& wires = (signal == "POWER")? pwgnd_.powerWires : pwgnd_.gndWires;

    int safeDistance = SafeBoundaryDistance();

    for(int i = 0; i < xMesh.size(); i++) {
        tmpWire.layerName = vMeshLayerName;
        tmpWire.width = pwgnd_.vmeshWidth;
        
        tmpWire.coorX[0] = xMesh[i] + xoffset;
        tmpWire.coorY[0] = dieArea.ll.y;
        tmpWire.coorX[1] = xMesh[i] + xoffset;
        tmpWire.coorY[1] = dieArea.ur.y;
        tmpWire.numPathPoint = 2;
        wires.push_back(tmpWire);
    }

    int left_offset, right_offset;
    
    int width = pwgnd_.hmeshWidth; 
    for(int i = 0; i < yMesh.size(); i++) {
        for(int j = 0; j < yMesh[i].size(); j++) {
            if(yMesh[i][j] + safeDistance >= dieArea.ur.y) {
                yMesh[i][j] = dieArea.ur.y - safeDistance;
            }
            else if(yMesh[i][j] - safeDistance <= dieArea.ll.y) {
                yMesh[i][j] = dieArea.ll.y + safeDistance;
            }     
            bool hasLeftConflict = false;
            if(signal == "GROUND" && i != 0) {
                for(int posy : opyMesh[i - 1]) {
                    if(abs(posy - yMesh[i][j]) <= width) {
                        hasLeftConflict = true;
                        break;
                    }
                }
            }
            
            if(hasLeftConflict) {
                tmpWire.coorX[0] = xMesh[i] + 2 * width;
                tmpWire.coorX[1] = xMesh[i + 1] + xoffset + width / 2;
                
                tmpWire.coorY[0] = yMesh[i][j];
                tmpWire.coorY[1] = yMesh[i][j];
                tmpWire.layerName = hMeshLayerName;
                tmpWire.width = width;
                tmpWire.numPathPoint = 2;
                tmpWire.viaName = "";
                wires.push_back(tmpWire);
                
                int moveup;
                if(yMesh[i][j] + 3 * width < dieArea.ur.y)  
                    moveup = 1;
                else 
                    moveup = -1;

                tmpWire.coorX[0] = xMesh[i] + 2.5 * width;
                tmpWire.coorX[1] = xMesh[i] + 2.5 * width;
                tmpWire.coorY[0] = yMesh[i][j] + moveup * width / 2;
                tmpWire.coorY[1] = yMesh[i][j] + moveup * 3 * width;
                wires.push_back(tmpWire);

                tmpWire.coorX[0] = xMesh[i] - 1.5 * width;
                tmpWire.coorX[1] = xMesh[i] + 3 * width;
                tmpWire.coorY[0] = yMesh[i][j] + moveup * 3.5 * width;
                tmpWire.coorY[1] = yMesh[i][j] + moveup * 3.5 * width;
                wires.push_back(tmpWire);


                tmpWire.coorX[0] = xMesh[i] - width;
                tmpWire.coorY[0] = yMesh[i][j] + moveup * 3.5 * width;
                tmpWire.numPathPoint = 1;
                tmpWire.layerName = vMeshLayerName;
                int viaID = topLayerId_2_viaId_[vLayerID]; //M4-M5
                tmpWire.viaName = vias[viaID].GetName();
                tmpWire.width = 0;
                wires.push_back(tmpWire);
            }
            else {
                tmpWire.coorX[0] = xMesh[i] + xoffset;
                tmpWire.coorX[1] = xMesh[i + 1] + xoffset;
                if(yMesh[i][j] + safeDistance >= dieArea.ur.y) {
                    yMesh[i][j] = dieArea.ur.y - safeDistance;
                }
                else if(yMesh[i][j] - safeDistance <= dieArea.ll.y) {
                    yMesh[i][j] = dieArea.ll.y + safeDistance;
                }
            
                tmpWire.coorY[0] = yMesh[i][j];
                tmpWire.coorY[1] = yMesh[i][j];
                tmpWire.layerName = hMeshLayerName;
                tmpWire.width = width;
                tmpWire.numPathPoint = 2;
                tmpWire.viaName = "";
		tmpWire.ext[0] = std::max(pwgnd_.hmeshWidth / 2, pwgnd_.hmeshExt);
		tmpWire.ext[1] = std::max(pwgnd_.hmeshWidth / 2, pwgnd_.hmeshExt);
                wires.push_back(tmpWire);
        	tmpWire.ext[0] = -1;
		tmpWire.ext[1] = -1;
		   
                tmpWire.numPathPoint = 1;
                tmpWire.layerName = vMeshLayerName;
                int viaID = topLayerId_2_viaId_[vLayerID]; //M4-M5
                tmpWire.viaName = vias[viaID].GetName();
            
                tmpWire.coorX[0] = xMesh[i] + xoffset;
                tmpWire.coorY[0] = yMesh[i][j];
                tmpWire.width = 0;
                wires.push_back(tmpWire); 
            
            }
  
            tmpWire.coorX[0] = xMesh[i + 1] + xoffset;
            tmpWire.coorY[0] = yMesh[i][j];
            tmpWire.width = 0;
            wires.push_back(tmpWire); // M4-M5 will always not be covered
            
            int midRange = 0;
            if(NearHighLayerViaRange(signal, tmpWire.coorY[0], midRange)) {// if near, need to add metal to connect 
                int M2_spacing = (layers[2].GetPitchY() - layers[2].GetWidth()) * dbuPerMicron;
                if(yMesh[i][j] < midRange) {
                    tmpWire.coorY[0] = yMesh[i][j];
                    tmpWire.coorY[1] = yMesh[i][j] + width / 2 + M2_spacing;
                }
                else {
                    tmpWire.coorY[0] = yMesh[i][j] - width / 2 - M2_spacing;
                    tmpWire.coorY[1] = yMesh[i][j];
                }
                
                tmpWire.coorX[1] = tmpWire.coorX[0];

                tmpWire.layerName = vMeshLayerName; 
                tmpWire.width = width;
                tmpWire.numPathPoint = 2;
                tmpWire.viaName = "";
                wires.push_back(tmpWire);
                
            }
        }
    }
}

void PWRoute::FindRowSNet(std::string componentName, std::string pinName, Rect2D<double> rect, int& powerY, int& gndY) {
    Point2D<int> midPoint;
    midPoint.x = (rect.ll.x + rect.ur.x) / 2;
    midPoint.y = (rect.ll.y + rect.ur.y) / 2;

    int column = -1;
    auto& xMesh = pwgnd_.xMesh;
    for(int i = 0; i < xMesh.size() - 1; i++) {
        if(midPoint.x > xMesh[i] && midPoint.x < xMesh[i + 1]) {
            column = i;
            break;
        }
    }
    if(column == -1) {
        std::cout << "ERROR: Pin's X is not in any column!" << std::endl;
        std::cout << componentName << "/" << pinName << std::endl;
        std::cout << "Pin: " << rect << std::endl;
        std::cout << "xMesh : " << std::endl;
        for(int i = 0; i < xMesh.size() - 1; i++) 
            std::cout << xMesh[i] << " ";
        std::cout << std::endl;
        exit(1);
    }
    
    auto& poweryMesh = pwgnd_.poweryMesh[column];
    auto& gndyMesh = pwgnd_.gndyMesh[column];

    auto powerit = std::upper_bound (poweryMesh.begin(), poweryMesh.end(), midPoint.y);  
    auto gndit = std::upper_bound (gndyMesh.begin(), gndyMesh.end(), midPoint.y);

    if(powerit == poweryMesh.begin() && gndit == gndyMesh.begin()) {
        std::cout << "ERROR: Pin's Y is not in any row!" << std::endl;
        std::cout << componentName << "/" << pinName << std::endl;
        std::cout << "Pin: " << rect << std::endl;
        exit(1);
    }

    if(powerit == poweryMesh.end() || gndit == gndyMesh.end()) {
        powerY = poweryMesh[poweryMesh.size() - 1];
        gndY = gndyMesh[gndyMesh.size() - 1];
    }
    else {
        if(*powerit < *gndit) {
            gndit--;
            powerY = *powerit;
            gndY = *gndit;
        }
        else {
            powerit--;
            powerY = *powerit;
            gndY = *gndit;
        }
    }
}

void PWRoute::MarkUnusablePointComp(PWRouteComponent& component) {
    
    Rect2D<int> dieArea = db_ptr_->GetDesignPtr()->GetDieArea();
    auto& layers = db_ptr_->GetLayersRef();
    auto& vias = db_ptr_->GetTechPtr()->GetLefViasRef();
    double dbuPerMicron = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();

    int powerY, gndY;
    if(component.pins_.size() != 0) {
        auto rect = component.pins_[0].layer_rects_[0].rects_[0];
        FindRowSNet(component.name_, component.pins_[0].name_, rect, powerY, gndY);
    }
    else
        return;

    std::vector<Rect2D<double>> pinRect;
    
    for(auto pin : component.pins_) {
        for(auto layerRect : pin.layer_rects_) {
            int layerIdx = db_ptr_->GetTechPtr()->GetLayerId(layerRect.layer_name_);
            if(layerIdx == 0 || layerIdx == 2) { //M1 or M2 
                for(auto rect : layerRect.rects_) {
                    pinRect.push_back(rect);
                }
            }
        }
    }
    
    for(auto layerRect : component.obs_.GetLayerRects()) {
        int layerIdx = db_ptr_->GetTechPtr()->GetLayerId(layerRect.layer_name_);
        if(layerIdx == 0 || layerIdx == 2) { //M1 or M2
            for(auto rect : layerRect.rects_) {
                pinRect.push_back(rect);
            }
        }
    }

    int viaIdx = topLayerId_2_viaId_[2];
    auto via = vias[viaIdx];
    auto viaRect = via.GetLayerRectsRef()[0].rects_[0];
    double h = (viaRect.ur.x - viaRect.ll.x) * dbuPerMicron;
    double v = (viaRect.ur.y - viaRect.ll.y) * dbuPerMicron;
    double viaWidth = (h > v)? v : h;
    double viaLength = (h > v)? h : v;
    int M1_spacing = (layers[0].GetPitchY() - layers[0].GetWidth()) * dbuPerMicron;

    auto tracks = db_ptr_->GetDesignPtr()->GetTracksRef();
    phydb::Track track = tracks[layerid_2_trackid_[4]]; //M3
    
    int M2_minLength = layer_min_length_[2] * dbuPerMicron;
    int M2_spacing = (layers[2].GetPitchY() - layers[2].GetWidth()) * dbuPerMicron;

    for(auto rect : pinRect) {
        if(fabs(rect.ll.y - powerY) < M1_spacing + viaWidth / 2 ||  
            fabs(rect.ur.y - powerY) < M1_spacing + viaWidth / 2 ) {

            int start = (rect.ll.x - M2_spacing - viaLength - M2_minLength / 2 - track.GetStart()) / track.GetStep() + 1;
            int end = (rect.ur.x + M2_spacing + viaLength + M2_minLength / 2 - track.GetStart()) / track.GetStep();

            for(int i = start; i <= end; i++) {
                int xpos = track.GetStart() + track.GetStep() * i; 
                Point2D<int> point;
                point.x = xpos; 
                point.y = powerY;
                pwgnd_.unusablePoints.insert(point);
            }
        }
        if( fabs(rect.ll.y - gndY) < M1_spacing + viaWidth / 2 ||
            fabs(rect.ur.y - gndY) < M1_spacing + viaWidth / 2 ) {

            int start = (rect.ll.x - M2_spacing - viaLength - M2_minLength / 2 - track.GetStart()) / track.GetStep() + 1;
            int end = (rect.ur.x + M2_spacing + viaLength + M2_minLength / 2 - track.GetStart()) / track.GetStep();

            for(int i = start; i <= end; i++) {
                int xpos = track.GetStart() + track.GetStep() * i; 
                Point2D<int> point;
                point.x = xpos; 
                point.y = gndY;
                pwgnd_.unusablePoints.insert(point);
            }
        }
    }
}


void PWRoute::MarkUnusablePoint() {
    for(auto& component : components_) {
        MarkUnusablePointComp(component);
    }
}

void PWRoute::RouteSNet() {
    auto layers = db_ptr_->GetTechPtr()->GetLayersRef();
    if((layers[0].GetDirection() == VERTICAL && layers.size() < high_mesh_layer) 
    	|| (layers[0].GetDirection() == HORIZONTAL && layers.size() < high_mesh_layer + 2)) {
	std::cout << "Warning: Not enough metal layers for POWER/GROUND reinforcement!" << std::endl;
	std::cout << "if m1 is V, requires m6; if m1 is H, requires m7." << std::endl; 
    }
    else {
	if(use_reinforcement_)
    	    RouteHighLayerReinforcement("POWER");
        else if(verbose_ > 0) {
	    std::cout << "Warning: reinforcement connection on high metal layer is disabled." << std::endl; 
	}	
    }
    
    RouteLowLayerMesh("POWER");

    if((layers[0].GetDirection() == VERTICAL && layers.size() < high_mesh_layer) 
    	|| (layers[0].GetDirection() == HORIZONTAL && layers.size() < high_mesh_layer + 2)) {
	std::cout << "Warning: Not enough metal layers for POWER/GROUND reinforcement!" << std::endl;
	std::cout << "if m1 is V, requires m6; if m1 is H, requires m7." << std::endl; 
    }
    else {
    	if(use_reinforcement_)
    	    RouteHighLayerReinforcement("GROUND");
        else if(verbose_ > 0) {
	    std::cout << "Warning: reinforcement connection on high metal layer is disabled." << std::endl; 
	} 
    }
    
    RouteLowLayerMesh("GROUND");

    //MarkUnusablePoint();
    AdjMeshPointFlag();
    DetailedRouteSNet();
    if(POWER_UNFOUND != 0 || GROUND_UNFOUND != 0)
        std::cout << "ATTENTION: SOME CELLS FAIL TO ROUTE!" << std::endl;
    
    //M3CloseViasPad();

    std::cout << "power/ground routing done!" << std::endl;
}

void PWRoute::AdjMeshPointFlag() {
   int dbuPerMicron = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();
   auto layers = db_ptr_->GetLayersRef();
   int M3_track_step = layers[detailed_route_layer].GetPitchX() * (double) dbuPerMicron;
   auto* spacing_table = layers[detailed_route_layer].GetSpacingTable(); //M3 spacing table
   
   int viaIdx = topLayerId_2_viaId_[cluster_horizontal_layer]; //check the via of M3-M4
   auto via = db_ptr_->GetTechPtr()->GetLefViasRef()[viaIdx];
   auto rect = via.GetLayerRectsRef()[0].rects_[0];
   double M3M4_via_h = rect.URX() - rect.LLX();
   //double M3M4_via_v = rect.URY() - rect.LLY();
   int M3_spacing = spacing_table->GetSpacingForWidth(M3M4_via_h) * dbuPerMicron;
   
   if(verbose_ > 1) {
      std::cout << "M3 track step: " << M3_track_step << " M3_spacing: " << M3_spacing;
      std::cout << " M3M4 via h: " << M3M4_via_h * dbuPerMicron << std::endl;
   }
   if(M3_track_step >= M3_spacing + M3M4_via_h * dbuPerMicron) {
      pwgnd_.use_adj_mesh_point_flag_ = true;
   }
   else {
      pwgnd_.use_adj_mesh_point_flag_ = false;
   }
}

/*
if there is a spacing violation for vias in adjacent grid points, add a pad to connect them
*/
void PWRoute::M3CloseViasPad() {
    
   int dbuPerMicron = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();
   auto layers = db_ptr_->GetLayersRef();
   int M3_track_step = layers[detailed_route_layer].GetPitchX() * (double) dbuPerMicron;
   auto* spacing_table = layers[detailed_route_layer].GetSpacingTable(); //M3 spacing table
   
   int viaIdx = topLayerId_2_viaId_[cluster_horizontal_layer]; //check the via of M3-M4
   auto via = db_ptr_->GetTechPtr()->GetLefViasRef()[viaIdx];
   auto rect = via.GetLayerRectsRef()[0].rects_[0];
   double M3M4_via_h = rect.URX() - rect.LLX();
   //double M3M4_via_v = rect.URY() - rect.LLY();
   int M3_spacing = spacing_table->GetSpacingForWidth(M3M4_via_h) * dbuPerMicron;
   
   for(auto p : pwgnd_.usedConnectionPoints) {
      int x = p.x;
      int y = p.y;
      if(pwgnd_.usedConnectionPoints.find(Point2D<int>(x + M3_track_step, y)) 
		      != pwgnd_.usedConnectionPoints.end()) {
         Wire tmpWire;
	 tmpWire.layerName = layers[detailed_route_layer].GetName();
    	 tmpWire.width = M3_track_step + M3M4_via_h * dbuPerMicron;
    	 tmpWire.coorX[0] = x + M3_track_step / 2;
    	 tmpWire.coorY[0] = y;
	 tmpWire.ext[0] = pwgnd_.hmeshWidth / 2;
    	 tmpWire.coorX[1] = x + M3_track_step / 2;
    	 tmpWire.coorY[1] = y;
	 tmpWire.ext[1] = pwgnd_.hmeshWidth / 2;
    	 tmpWire.numPathPoint = 2;
    	 tmpWire.viaName = "";
	
	 //identify it is POWER or GROUND net
	 int column = -1;
         auto& xMesh = pwgnd_.xMesh;
         for(int i = 0; i < xMesh.size() - 1; i++) {
            if(x > xMesh[i] && x < xMesh[i + 1]) {
               column = i;
               break;
            }
         }
    	 if(column == -1) {
            std::cout << "ERROR: used point is not in any column!" << std::endl;
            exit(1);
    	 }
    
         auto& poweryMesh = pwgnd_.poweryMesh[column];
         auto& gndyMesh = pwgnd_.gndyMesh[column];
         bool found = false;
	 std::string signal;
	 for(auto coorY : poweryMesh) {
	    if(y == coorY) {
	       found = true;
	       signal = "POWER";
	       break;
	    }
	 }
	 if(!found) {
	    for(auto coorY : gndyMesh) {
	       if(y == coorY) {
	          found = true;
	          signal = "GROUND";
		  break;
	       }
	    }
	 }

	 if(signal == "POWER") {
	    pwgnd_.powerWires.push_back(tmpWire);
	 }
	 else if(signal == "GROUND") {
            pwgnd_.gndWires.push_back(tmpWire);
	 }
	 else {
	    std::cout << "ERROR: unable to find signal for point " << x << ", " << y << std::endl;
	    exit(1);
	 }
      }
   }
    
}

void PWRoute::ComputeMinLength() {
    auto& layers = db_ptr_->GetLayersRef();
    double manufacturingGrid = db_ptr_->GetTechPtr()->GetManufacturingGrid();
    double min_length;
    for(int i = 0; i < layers.size(); i++) {
        if(layers[i].GetType() == ROUTING)
            min_length = FitGrid(layers[i].GetArea() / layers[i].GetWidth(), manufacturingGrid);
        else
            min_length = -1;
        layer_min_length_.push_back(min_length);
    }

}

void PWRoute::SetMeshWidthStep(int high_width, int high_step, int mesh_width) {
    high_mesh_multiple_width = high_width; 
    high_mesh_multiple_step = high_step; 
    cluster_mesh_multiple_width = mesh_width;
}

void PWRoute::SetReinforcement(bool use_reinforcement) {
    use_reinforcement_ = use_reinforcement;
    if(verbose_ > 0) 
       std::cout << "use_reinforcement_ set to: " << use_reinforcement_ << std::endl;
}

void PWRoute::RunPWRoute() {
    if(verbose_ > none) {
        std::cout << "high_mesh_multiple_width: "  << high_mesh_multiple_width << std::endl;
        std::cout << "high_mesh_multiple_step: "  << high_mesh_multiple_step << std::endl;
        std::cout << "cluster_mesh_multiple_width: "  << cluster_mesh_multiple_width << std::endl;
    }
    if(db_ptr_ == NULL) {
        std::cout << "no phydb in pwroute" << std::endl;
        exit(1);
    }
    auto layers = db_ptr_->GetTechPtr()->GetLayersRef();
    if((layers[0].GetDirection() == HORIZONTAL && layers.size() < cluster_horizontal_layer) 
    	|| (layers[0].GetDirection() == VERTICAL && layers.size() < cluster_horizontal_layer + 2)) {
	std::cout << "ERROR: Not enough metal layers for POWER/GROUND mesh!" << std::endl;
	std::cout << " ---- Requires at least two vertical metal layers above metal2." << std::endl; 
	std::cout << " ---- i.e. if m1 is H, vertical stripe requires m6; if m1 is V, it requires m5." << std::endl; 
	exit(1);
    }
    SetDefaultVia();
    if(verbose_ > none) 
	std::cout << "set default via done" << std::endl;
    ComputeMinLength();
    if(verbose_ > none) 
	std::cout << "compute min length done" << std::endl;
    LinkTrackToLayer();
    if(verbose_ > none) 
	std::cout << "link track to layer done" << std::endl;
    PreprocessComponents();
    if(verbose_ > none) 
	std::cout << "preprocess components done" << std::endl;
    SNetConfig();
    if(verbose_ > none) 
	std::cout << "snet config done" << std::endl;
    InitCluster();
    if(verbose_ > none) 
	std::cout << "initialize cluster done" << std::endl;
    RouteSNet();
    if(verbose_ > none) 
	std::cout << "route snet done" << std::endl;
}

phydb::SNet* PWRoute::FindSNet(SignalUse signal) {
    bool debug = false;
    auto snet_ref_v = db_ptr_->GetSNetRef();
    bool found = false;
    phydb::SNet* snet_ptr = nullptr;
    for(int i = 0; i < snet_ref_v.size(); i++) {
        snet_ptr = &snet_ref_v[i];
        if(snet_ptr->GetUse() == signal) {
            found = true;
            break;
        }
    }
    if(found == false) {
        if(signal == POWER) {
            std::string Vdd = std::string("Vdd");
            snet_ptr = db_ptr_->AddSNet(Vdd, signal);
        }
        else {
            std::string GND = std::string("GND");
            snet_ptr = db_ptr_->AddSNet(GND, signal);
        }
    }
    if(debug)
        std::cout << "added net ptr: " << snet_ptr << std::endl;

    assert(snet_ptr != nullptr);
    return snet_ptr;
}

void PWRoute::ExportToPhyDB() {
    bool debug = false;
    
    //export POWER
    phydb::SNet* power_ptr = FindSNet(POWER);
    
    for(auto wire : pwgnd_.powerWires) {
        std::string stripe = std::string("STRIPE");
        phydb::Path* path_ptr = power_ptr->AddPath(wire.layerName, stripe, wire.width);

        if(wire.numPathPoint == 1) {
	    path_ptr->AddRoutingPoint(wire.coorX[0], wire.coorY[0]);
	    if(wire.viaName != "") {
		path_ptr->SetViaName(wire.viaName);
	    }
	    else {
		if(verbose_ > 0)
		    std::cout << "Warning: dangling routing point " << wire.coorX[0] << " " << wire.coorY[0] << std::endl;
	    }
	}
	else {
	    path_ptr->AddRoutingPoint(wire.coorX[0], wire.coorY[0], wire.ext[0]);
            path_ptr->AddRoutingPoint(wire.coorX[1], wire.coorY[1], wire.ext[1]);
        } 
    }

    if(debug) {
        std::cout << "phydb snet power paths size: " << power_ptr->GetPathsRef().size() << std::endl;
        std::cout << "phydb power ptr: " << power_ptr << std::endl;
        std::string snet_name = "Vdd";
        std::cout << "phydb power ptr2: " << db_ptr_->GetSNet(snet_name) << std::endl;
        std::cout << "power path ptr: " << &(power_ptr->GetPathsRef()) << std::endl;
    }

    phydb::SNet* ground_ptr = FindSNet(GROUND);
    //export GROUND
    for(auto wire : pwgnd_.gndWires) {
        phydb::Path* path_ptr = ground_ptr->AddPath(wire.layerName, "STRIPE", wire.width);

        if(wire.numPathPoint == 1) {
	    path_ptr->AddRoutingPoint(wire.coorX[0], wire.coorY[0]);
	    if(wire.viaName != "") {
		path_ptr->SetViaName(wire.viaName);
	    }
	    else {
		if(verbose_ > 0)
		    std::cout << "Warning: dangling routing point " << wire.coorX[0] << " " << wire.coorY[0] << std::endl;
	    }
	}
	else {
	    path_ptr->AddRoutingPoint(wire.coorX[0], wire.coorY[0], wire.ext[0]);
            path_ptr->AddRoutingPoint(wire.coorX[1], wire.coorY[1], wire.ext[1]);
        }
    }

    if(debug) {
        std::cout << "phydb snet ground paths size: " << ground_ptr->GetPathsRef().size() << std::endl;
        std::cout << "phydb ground ptr: " << ground_ptr << std::endl;
        std::string snet_name = "GND";
        std::cout << "phydb ground ptr2: " << db_ptr_->GetSNet(snet_name) << std::endl;
        std::cout << "ground path ptr: " << &(ground_ptr->GetPathsRef()) << std::endl;
        std::cout << std::endl;
        std::cout << " total number of snets " << db_ptr_->GetSNetRef().size() << std::endl;
    }

}

}

