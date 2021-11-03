#include "pwroute_component.h"
#include <math.h>

namespace pwroute {

void PWRouteComponent::ComputeLocation(Rect2D<double>& rect, CompOrient orient, Point2D<int> location, Point2D<int> origin, Point2D<int> size) {
    int llx, lly, urx, ury;
    switch(orient) {
        case phydb::CompOrient::N: { // "N"
            llx = location.x + origin.x + rect.ll.x;
            lly = location.y + origin.y + rect.ll.y;
            urx = location.x + origin.x + rect.ur.x;
            ury = location.y + origin.y + rect.ur.y;
            rect.Set(llx, lly, urx, ury);
            break;
        }
        case phydb::CompOrient::S: { // "S"
            llx = location.x + origin.x + size.x - rect.ur.x;
            lly = location.y + origin.y + size.y - rect.ur.y;
            urx = location.x + origin.x + size.x - rect.ll.x;
            ury = location.y + origin.y + size.y - rect.ll.y;
            rect.Set(llx, lly, urx, ury);
            break;
        }
        case phydb::CompOrient::W: { // "W"
            float centerx = (rect.ll.x + rect.ur.x)/2.0;
            float centery = (rect.ll.y + rect.ur.y)/2.0;
            int rectwidth = rect.ur.x - rect.ll.x;
            int rectheight = rect.ur.y - rect.ll.y;
            float centerxprime = -centery;
            // assumption origin.x and origin.y is always 0
            // cos(90deg)centerx - sin(90deg)centery
            float centeryprime =  centerx;
            // sin(90deg)centerx + cos(90deg)centery
            int tmplength;
            tmplength = rectwidth;
            // 90deg rotation, width and height swap
            rectwidth = rectheight;
            rectheight = tmplength;
            // macro width and height should also swap, but we cannot do it here, because we are at pin level now
            // but remember, when we need macro width, we should call its height, and vice versa
            llx = std::round(centerxprime - rectwidth/2.0);
            lly = std::round(centeryprime - rectheight/2.0);
            urx = std::round(centerxprime + rectwidth/2.0);
            ury = std::round(centeryprime + rectheight/2.0);
            // shift rightward for a macro width, now we should use macro.size.y, after shift move to (comp.placeLoc.x, comp.placeLoc.y)
            llx += size.y + location.x;
            lly += location.y;
            urx += size.y + location.x;
            ury += location.y;


            llx += origin.x;
            lly += origin.y;
            urx += origin.x;
            ury += origin.y;
            rect.Set(llx, lly, urx, ury);
            break;
        }

        case phydb::CompOrient::E : {   //"E"
            float centerx = (rect.ll.x + rect.ur.x)/2.0;
            float centery = (rect.ll.y + rect.ur.y)/2.0;
            int rectwidth = rect.ur.x - rect.ll.x;
            int rectheight = rect.ur.y - rect.ll.y;
            float centerxprime =  centery;
            // assumption origin.x and origin.y is always 0
            // cos(270deg)centerx - sin(270deg)centery
            float centeryprime = -centerx;
            // sin(270deg)centerx + cos(270deg)centery
            int tmplength;
            tmplength = rectwidth;
            // 270deg rotation, width and height swap
            rectwidth = rectheight;
            rectheight = tmplength;
            // macro width and height should also swap, but we cannot do it here, because we are at pin level now
            // but remember, when we need macro width, we should call its height, and vice versa
            llx = centerxprime - rectwidth/2.0;
            lly = centeryprime - rectheight/2.0;
            urx = centerxprime + rectwidth/2.0;
            ury = centeryprime + rectheight/2.0;
            // shift upward for a macro height, now we should use macro.size.x, after shift move to (comp.placeLoc.x, comp.placeLoc.y)
            llx += location.x;
            lly += size.x + location.y;
            urx += location.x;
            ury += size.x + location.y;

            llx += origin.x;
            lly += origin.y;
            urx += origin.x;
            ury += origin.y;
            rect.Set(llx, lly, urx, ury);
            break;
        }

        case phydb::CompOrient::FN: { // "FN"
            llx = location.x + size.x - rect.ur.x;
            lly = location.y + rect.ll.y;
            urx = location.x + size.x - rect.ll.x;
            ury = location.y + rect.ur.y;

            llx += origin.x;
            lly += origin.y;
            urx += origin.x;
            ury += origin.y;
            rect.Set(llx, lly, urx, ury);
            break;
        }
        case phydb::CompOrient::FS: { //"FS"
            llx = location.x + rect.ll.x;
            lly = location.y + size.y - rect.ur.y;
            urx = location.x + rect.ur.x;
            ury = location.y + size.y - rect.ll.y;

            llx += origin.x;
            lly += origin.y;
            urx += origin.x;
            ury += origin.y;
            rect.Set(llx, lly, urx, ury);
            break;
        }
        case phydb::CompOrient::FW: { //"FW"
            float centerx = (rect.ll.x + rect.ur.x)/2.0;
            float centery = (rect.ll.y + rect.ur.y)/2.0;
            int rectwidth = rect.ur.x - rect.ll.x;
            int rectheight = rect.ur.y - rect.ll.y;
            float centerxprime = -centery;
            // assumption origin.x and origin.y is always 0
            // cos(90deg)centerx - sin(90deg)centery
            float centeryprime =  centerx;
            // sin(90deg)centerx + cos(90deg)centery
            int tmplength;
            tmplength = rectwidth;
            // 90deg rotation, width and height swap
            rectwidth = rectheight;
            rectheight = tmplength;
            // macro width and height should also swap, but we cannot do it here, because we are at pin level now
            // but remember, when we need macro width, we should call its height, and vice versa
            llx = std::round(centerxprime - rectwidth/2.0);
            lly = std::round(centeryprime - rectheight/2.0);
            urx = std::round(centerxprime + rectwidth/2.0);
            ury = std::round(centeryprime + rectheight/2.0);
            // shift rightward for a macro width, now we should use macro.size.y
            llx += size.y;
            urx += size.y;
            // flip, this is done using the right boundary location, which is macro.size.y
            int axis = size.y;
            centerx = (llx + urx)/2.0;
            centery = (lly + ury)/2.0;
            rectwidth = urx - llx;
            rectheight = ury - lly;

            centerx = axis - centerx;

            llx = std::round(centerx - rectwidth/2.0) + location.x;
            lly = std::round(centery - rectheight/2.0) + location.y;
            urx = std::round(centerx + rectwidth/2.0) + location.x;
            ury = std::round(centery + rectheight/2.0) + location.y;

            llx += origin.x;
            lly += origin.y;
            urx += origin.x;
            ury += origin.y;

            rect.Set(llx, lly, urx, ury);
            break;
        }
        case phydb::CompOrient::FE: { //" FE"
            float centerx = (rect.ll.x + rect.ur.x)/2.0;
            float centery = (rect.ll.y + rect.ur.y)/2.0;
            int rectwidth = rect.ur.x - rect.ll.x;
            int rectheight = rect.ur.y - rect.ll.y;
            float centerxprime =  centery;
            // assumption origin.x and origin.y is always 0
            // cos(270deg)centerx - sin(270deg)centery
            float centeryprime = -centerx;
            // sin(270deg)centerx + cos(270deg)centery
            int tmplength;
            tmplength = rectwidth;
            // 270deg rotation, width and height swap
            rectwidth = rectheight;
            rectheight = tmplength;
            // macro width and height should also swap, but we cannot do it here, because we are at pin level now
            // but remember, when we need macro width, we should call its height, and vice versa
            llx = centerxprime - rectwidth/2.0;
            lly = centeryprime - rectheight/2.0;
            urx = centerxprime + rectwidth/2.0;
            ury = centeryprime + rectheight/2.0;
            // shift upward for a macro height, now we should use macro.size.x
            lly += size.x;
            ury += size.x;

            int axis = size.y;
            centerx = (llx + urx)/2.0;
            centery = (lly + ury)/2.0;
            rectwidth = urx - llx;
            rectheight = ury - lly;

            centerx = axis - centerx;

            llx = std::round(centerx - rectwidth/2.0) + location.x;
            lly = std::round(centery - rectheight/2.0) + location.y;
            urx = std::round(centerx + rectwidth/2.0) + location.x;
            ury = std::round(centery + rectheight/2.0) + location.y;

            llx += origin.x;
            lly += origin.y;
            urx += origin.x;
            ury += origin.y;

            rect.Set(llx, lly, urx, ury);
            break;
        }
        default: {
            std::cout << "unknown orientation for component: " << phydb::CompOrientStr(orient) << std::endl;
            break;
        }
    }
}



}
