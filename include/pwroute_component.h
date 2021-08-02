#ifndef PWROUTE_COMPONENT_H
#define PWROUTE_COMPONENT_H

#include <phydb/phydb.h>
using namespace phydb;
namespace pwroute {

class PWRoutePin {
public:
    std::string name_;
    SignalUse use_;
    vector<LayerRect> layer_rects_;

    PWRoutePin(phydb::Pin& pin, int dbuPerMicron) {
        name_ = pin.GetName();
        use_ = pin.GetUse();
        layer_rects_ = pin.GetLayerRectCpy();

        for(auto& layerRect : layer_rects_)  {
            for(auto& rect : layerRect.rects_)  {
                rect.ll.x *= dbuPerMicron;
                rect.ll.y *= dbuPerMicron;
                rect.ur.x *= dbuPerMicron;
                rect.ur.y *= dbuPerMicron;
            }
        }
    }
};


class PWRouteComponent {
public:
    std::string name_;
    phydb::Point2D<int> location_;
    phydb::Point2D<int> size_;
    vector<PWRoutePin> pins_;  
    phydb::OBS obs_;

    void ComputeLocation(Rect2D<double>& rect, CompOrient orient, Point2D<int> location, Point2D<int> orig, Point2D<int> size);

    PWRouteComponent(PhyDB* db_ptr_, phydb::Component& component) {
        int dbuPerMicron = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();

        Macro* macro_ptr = component.GetMacro();

        Point2D<int> orig;
        orig.x = macro_ptr->GetOriginX() * dbuPerMicron;
        orig.y = macro_ptr->GetOriginY() * dbuPerMicron;
 
        size_.x = macro_ptr->GetWidth() * dbuPerMicron;
        size_.y = macro_ptr->GetHeight() * dbuPerMicron;

        name_ = component.GetName();
        location_ = component.GetLocation();
        //enlarge by dbuPermicron
        for(auto& pin : macro_ptr->GetPinsRef()) {
            pins_.emplace_back(pin, dbuPerMicron);
        }
        phydb::OBS obs;
        auto obs_ptr = macro_ptr->GetObs();

        obs_ = *obs_ptr; // make a copy of obs

        for(auto& layerRect : obs_.GetLayerRectsRef())  {
            for(auto& rect : layerRect.rects_)  {
                rect.ll.x *= dbuPerMicron;
                rect.ll.y *= dbuPerMicron;
                rect.ur.x *= dbuPerMicron;
                rect.ur.y *= dbuPerMicron;
            }
        }

        CompOrient orient = component.GetOrientation();
        //compute final coordinate 
        for(auto& p : pins_) {
            for(auto& layer_rect : p.layer_rects_) {
                for(auto& rect : layer_rect.rects_) {
                    ComputeLocation(rect, orient, location_, orig, size_);
                }
            }
        }

        for(auto& layer_rect : obs_.GetLayerRectsRef()) {
            for(auto& rect : layer_rect.rects_) {
                ComputeLocation(rect, orient, location_, orig, size_);
            }
        }

    }
};

}


#endif
