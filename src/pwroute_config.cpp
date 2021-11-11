#include <algorithm>
#include <math.h>
#include "pwroute.h"

using namespace phydb;
namespace pwroute {



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
    pwgnd_.hmeshWidth = std::max((int) cluster_mesh_multiple_width * pwgnd_.signalNetWidth, viaLength); 
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
    pwgnd_.vmeshWidth = std::max((int) cluster_mesh_multiple_width * pwgnd_.signalNetWidth, viaLength); 
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
    if(layers[0].GetDirection() == phydb::MetalDirection::HORIZONTAL) {
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
   
    /*set hmeshPitch for vertical mesh M4*/
    auto* m4_spacing_table = layers[cluster_horizontal_layer].GetSpacingTable();
    double m4_spacing = m4_spacing_table->GetSpacingForWidth((double) pwgnd_.hmeshWidth / (double)dbuPerMicron);
    pwgnd_.hmeshPitch = pwgnd_.hmeshWidth + m4_spacing * dbuPerMicron;
    if(verbose_ > 1)
    	std::cout << "hmesh width: " << pwgnd_.hmeshWidth << " m4_spacing: " << m4_spacing << " hmesh pitch: " << pwgnd_.hmeshPitch << std::endl; 

 
    /*set vmeshPitch for vertical mesh M5*/
    auto* m5_spacing_table = layers[cluster_vertical_layer].GetSpacingTable();
    double m5_spacing = m5_spacing_table->GetSpacingForWidth((double) pwgnd_.vmeshWidth / (double)dbuPerMicron);
    pwgnd_.vmeshPitch = pwgnd_.vmeshWidth + m5_spacing * dbuPerMicron;
    if(verbose_ > 1)
    	std::cout << "vmesh width: " << pwgnd_.vmeshWidth << " m5_spacing: " << m5_spacing << " vmesh pitch: " << pwgnd_.vmeshPitch << std::endl; 

    /*set m2_expanded_width and m2_expanded_length*/
    viaIdx = topLayerId_2_viaId_[2]; //check the via of M1-M2
    via = db_ptr_->GetTechPtr()->GetLefViasRef()[viaIdx];
    rect = via.GetLayerRectsRef()[2].rects_[0]; //top layer of the via
    v = rect.URY() - rect.LLY();
    h = rect.URX() - rect.LLX();
    double width1 = v * dbuPerMicron;
    double length1 = h * dbuPerMicron;

    viaIdx = topLayerId_2_viaId_[detailed_route_layer]; //check the via of M2-M3
    via = db_ptr_->GetTechPtr()->GetLefViasRef()[viaIdx];
    rect = via.GetLayerRectsRef()[0].rects_[0]; //bot layer of the via
    v = rect.URY() - rect.LLY();
    h = rect.URX() - rect.LLX();
    pwgnd_.need_m2_minarea = (v * h < layers[2].GetArea());
    double width2 = v * dbuPerMicron;
    double length2 = h * dbuPerMicron;
    pwgnd_.m2_expanded_width = std::min(width1, width2);

    double max_length = std::max(length1, length2);
    auto* m2_spacing_table = layers[2].GetSpacingTable();
    double spacing = m2_spacing_table->GetSpacingFor( max_length / (double)dbuPerMicron, std::min(width1, width2) / (double) dbuPerMicron);
    pwgnd_.m2_expanded_range.begin = (length1 + length2) / 2;
    pwgnd_.m2_expanded_range.end = spacing * dbuPerMicron + (length1 + length2) / 2;
    
    if(verbose_ > 1) {
	std::cout << "need m2 minarea " << pwgnd_.need_m2_minarea << std::endl;
        std::cout << "m2 expand: " << pwgnd_.m2_expanded_width << " ";
	std::cout << pwgnd_.m2_expanded_range.begin << " ";
	std::cout << pwgnd_.m2_expanded_range.end << " " << std::endl;
    }

    /*set m3_expanded_width*/
    viaIdx = topLayerId_2_viaId_[detailed_route_layer]; //check the via of M2-M3
    via = db_ptr_->GetTechPtr()->GetLefViasRef()[viaIdx];
    rect = via.GetLayerRectsRef()[2].rects_[0]; //top layer of the via
    v = rect.URY() - rect.LLY();
    h = rect.URX() - rect.LLX();
    width1 = h * dbuPerMicron;
    length1 = v * dbuPerMicron;

    viaIdx = topLayerId_2_viaId_[cluster_horizontal_layer]; //check the via of M3-M4
    via = db_ptr_->GetTechPtr()->GetLefViasRef()[viaIdx];
    rect = via.GetLayerRectsRef()[0].rects_[0]; //bot layer of the via
    v = rect.URY() - rect.LLY();
    h = rect.URX() - rect.LLX();
    width1 = h * dbuPerMicron;
    length1 = v * dbuPerMicron;
    pwgnd_.m3_expanded_width = std::min(width1, width2);
    
    max_length = std::max(length1, length2);
    auto* m3_spacing_table = layers[detailed_route_layer].GetSpacingTable();
    spacing = m3_spacing_table->GetSpacingFor( max_length / (double)dbuPerMicron, std::min(width1, width2) / (double) dbuPerMicron);
    pwgnd_.m3_expanded_range.begin = (length1 + length2) / 2;
    pwgnd_.m3_expanded_range.end = spacing * dbuPerMicron + (length1 + length2) / 2;
    if(verbose_ > 1) {
        std::cout << "m3 expand: " << pwgnd_.m3_expanded_width << " ";
	std::cout << pwgnd_.m3_expanded_range.begin << " ";
	std::cout << pwgnd_.m3_expanded_range.end << " " << std::endl;
    }    
    
    pwgnd_.lastHLayerID = high_mesh_layer;
    pwgnd_.lastHLayerName = layers[high_mesh_layer].GetName();
    if(verbose_ > none) {
        std::cout << "High H layer: " << pwgnd_.lastHLayerName << std::endl;
    }
}


} //namespace PWRoute
