#ifndef PWGND_ROUTE_H
#define PWGND_ROUTE_H

#include "header.h"
#include "defDataBase.h"
#include "global.h"
extern parser::lefDataBase lefDB;
extern parser::defDataBase defDB;


#define HIGHMESH_MULTIPLE 8
#define HIGHMESH_STEP 8
#define LOWMESH_MULTIPLE 2
int POWER_FOUND = 0;
int POWER_UNFOUND = 0;
int GROUND_FOUND = 0;
int GROUND_UNFOUND = 0;

#define HORIZONTAL_M4 1

#define HighHLayer 10 //M6


/*void simplePWGNDMesh() {
	cout << "route pwgnd" << endl;
	defDB.pwgnd.layerWidth = lefDB.layers.at(6).width * defDB.dbuPerMicro;

	defDB.pwgnd.pitch = 10 * defDB.pwgnd.layerWidth;
	defDB.pwgnd.width = 2 * defDB.pwgnd.layerWidth;

	string layerName = lefDB.layers.at(6).name;
	defDB.pwgnd.meshLayerName = layerName;
	defDB.pwgnd.routeLayerName = lefDB.layers.at(4).name;
	
	string direction = lefDB.layers.at(6).direction;
	//string direction = "HORIZONTAL";

	defDB.pwgnd.direction = direction;

	cout << "PW GND direction: " << direction << " " << layerName << endl;

	int range = (direction == "VERTICAL")? 
		defDB.dieArea.upperRight.x - defDB.dieArea.lowerLeft.x : defDB.dieArea.upperRight.y - defDB.dieArea.lowerLeft.y ;
	
	int cnt = (range - defDB.pwgnd.width / 2)/ defDB.pwgnd.pitch;
	cout << "range: " << range << " pitch: " << defDB.pwgnd.pitch << endl;
	cout << "pitch cnt: " << cnt << endl;
	string lastNetPower = (cnt % 2 == 0)? true : false;
    int netSize = (lastNetPower)? defDB.powerNets.size() : defDB.gndNets.size();
        cnt = cnt % netSize;
        if(lastNetPower)
            defDB.pwgnd.upperBoundNet = defDB.powerNets[cnt];
        else
            defDB.pwgnd.upperBoundNet = defDB.gndNets[cnt];
}*/

void readCluster(string clusterFileName) {

    ifstream infile(clusterFileName);
    string tmp1, tmp2, tmp3;
    int lx, ux, ly, uy;
    int cnt_y = 0, cnt_x = 0;
    set<int> yPW;
    set<int> yGND;
    int pre_ux = 0, pre_uy;
    int width = 0;
    while(!infile.eof()) {
        infile >> tmp1 >> tmp2;
        if(tmp1 == "STRIP") {
            infile >> lx >> ux >> tmp3;
            defDB.pwgnd.startSNet.push_back(tmp3);
            pre_uy = 0;
            if(tmp3 == "GND")   
                cnt_y = 0;
            else
                cnt_y = 1;

            if(pre_ux == 0)
                defDB.pwgnd.xMesh.push_back((lx + defDB.dieArea.lowerLeft.x) / 2);//first gnd       
            else {
                defDB.pwgnd.xMesh.push_back((lx + pre_ux)/2); 
                width = lx - pre_ux;
            }
            pre_ux = ux;

            yPW.clear();
            yGND.clear();
        }
        else if(tmp1 == "END") {
            vector<int> tmpPW;
            vector<int> tmpGND;
            for(auto y : yGND)
                tmpGND.push_back(y);
            sort(tmpGND.begin(), tmpGND.end());

            for(auto y : yPW)
                tmpPW.push_back(y);
            sort(tmpPW.begin(), tmpPW.end()); //ascending order
            
            defDB.pwgnd.poweryMesh.push_back(tmpPW);
            defDB.pwgnd.gndyMesh.push_back(tmpGND);
            cnt_x++;
            //cout << " cnt_x: " << cnt_x << endl;
            tmp1.clear();
        }
        else {
            ly = atoi(tmp1.c_str());
            if(pre_uy != 0 && ly != pre_uy)
                ly = pre_uy;
            uy = atoi(tmp2.c_str());
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
    }
    
    defDB.pwgnd.xMesh.push_back((ux + defDB.dieArea.upperRight.x) / 2);       

    /*cout << "x: ";
    for(int i = 0; i < defDB.pwgnd.xMesh.size(); i++) {
        cout << defDB.pwgnd.xMesh[i] << " ";
    }
    cout << endl;

    /*cout << "powerx: ";
    for(int i = 1; i < defDB.pwgnd.xMesh.size(); i += 2) {
        cout << defDB.pwgnd.xMesh[i] << " ";
    }
    cout << endl;

    cout << "gndy: " << endl;
    for(int i = 0; i < defDB.pwgnd.gndyMesh.size(); i++) {
        for(int j = 0; j < defDB.pwgnd.gndyMesh[i].size(); j++) {
            cout << defDB.pwgnd.gndyMesh[i][j] << " ";
        }
        cout << endl;
    }
    cout << endl;

    cout << "powery: " << endl;
    for(int i = 0; i < defDB.pwgnd.poweryMesh.size(); i++) {
        for(int j = 0; j < defDB.pwgnd.poweryMesh[i].size(); j++) {
            cout << defDB.pwgnd.poweryMesh[i][j] << " ";
        }
        cout << endl;
    }
    cout << endl;
    */

}

float fitGrid(float num) {
    float manufacturingGrid = lefDB.manufacturingGrid;
    float grid = lefDB.manufacturingGrid;  
    if(manufacturingGrid == 0)
        return num;
    else {
        int multiple = (num / grid) / 2;
        if(num == ((float) multiple )* grid * 2)
            return num;
        else {
            return (multiple + 1) * grid * 2;
        }

    }
}

void SNetConfig(string clusterFileName) {
    readCluster(clusterFileName);

    defDB.pwgnd.layerWidth = lefDB.layers.at(2).width * defDB.dbuPerMicro; //use m2 width as standard

	//defDB.pwgnd.pitch = 10 * defDB.pwgnd.layerWidth;
    
    int viaIdx = lefDB.topLayerIdx2ViaIdx[2];
    auto via = lefDB.vias[viaIdx];
    auto rect = via.layerRects[0].rects[0];
    int h = rect.upperRight.x - rect.lowerLeft.x;
    int v = rect.upperRight.y - rect.lowerLeft.y;
    int viaLength = (h > v)? h : v;
    viaLength *= defDB.dbuPerMicro;
    defDB.pwgnd.width = max(LOWMESH_MULTIPLE * defDB.pwgnd.layerWidth, viaLength);

    defDB.pwgnd.width = fitGrid(defDB.pwgnd.width);

#ifdef HORIZONTAL_M4
    if(lefDB.layers.at(0).direction == "HORIZONTAL") {
	    defDB.pwgnd.vLayerName = lefDB.layers.at(6).name;
	    defDB.pwgnd.hLayerName = lefDB.layers.at(8).name;
    }
    else { //usually this branch because m2 = horizontal
        defDB.pwgnd.vLayerName = lefDB.layers.at(8).name; //M5
	    defDB.pwgnd.hLayerName = lefDB.layers.at(6).name; //M4
    }
#else
    if(lefDB.layers.at(0).direction == "HORIZONTAL") {
	    defDB.pwgnd.vLayerName = lefDB.layers.at(2).name;
	    defDB.pwgnd.hLayerName = lefDB.layers.at(0).name;
    }
    else { //usually this branch because m2 = horizontal
        defDB.pwgnd.vLayerName = lefDB.layers.at(0).name; //M1
	    defDB.pwgnd.hLayerName = lefDB.layers.at(2).name; //M2
    }
#endif
    cout << "vertical: " << defDB.pwgnd.vLayerName << endl;
    cout << "horizontal: " << defDB.pwgnd.hLayerName << endl;

    /*for(int i = lefDB.layers.size() - 1; i >= 0; i--) {
        if(lefDB.layers[i].direction == "HORIZONTAL") {
            defDB.pwgnd.lastHLayerID = i;
            defDB.pwgnd.lastHLayerName = lefDB.layers[i].name;
            break;
        }
    }*/
    defDB.pwgnd.lastHLayerID = HighHLayer;
    defDB.pwgnd.lastHLayerName = lefDB.layers[HighHLayer].name;
    cout << "High H layer: " << defDB.pwgnd.lastHLayerName << endl;
}


/*parser::Point2D<int> findClosestVDD(parser::Point2D<int> point, string netName) {
	parser::Point2D<int> touch;
	int pitch = defDB.pwgnd.pitch;

	if(defDB.pwgnd.direction == "VERTICAL") {
		int cnt = (point.x - defDB.dieArea.lowerLeft.x - defDB.pwgnd.width / 2) / pitch;
		cnt = (cnt % 2 == 0)? cnt : cnt + 1;
		touch.x = defDB.dieArea.lowerLeft.x + defDB.pwgnd.width / 2 + cnt * pitch;

		touch.y = point.y;

		if(touch.x > defDB.dieArea.upperRight.x) {
			if(defDB.pwgnd.upperBoundNet == "VDD") {
				touch.x = defDB.dieArea.upperRight.x - defDB.pwgnd.width / 2;
			}
			else {
				touch.x -= 2 * pitch;
			}
		}
	}
	else {
		touch.x = point.x;
		
		int cnt = (point.y - defDB.dieArea.lowerLeft.y - defDB.pwgnd.width / 2) / pitch;
		cnt = (cnt % 2 == 0)? cnt : cnt + 1;
		touch.y = defDB.dieArea.lowerLeft.y + defDB.pwgnd.width / 2 + cnt * pitch;

		if(touch.y > defDB.dieArea.upperRight.y) {
			if(defDB.pwgnd.upperBoundNet == "VDD") {
				touch.y = defDB.dieArea.upperRight.y - defDB.pwgnd.width / 2;
			}
			else {
				touch.y -= 2 * pitch;
			}
		}

	}
	return touch;
}

parser::Point2D<int> findClosestGND(parser::Point2D<int> point, string netName) {
	parser::Point2D<int> touch;
	int pitch = defDB.pwgnd.pitch;

	if(defDB.pwgnd.direction == "VERTICAL") {
		int cnt = (point.x - defDB.dieArea.lowerLeft.x - defDB.pwgnd.width / 2) / pitch;
		cnt = (cnt % 2 == 1)? cnt : cnt + 1;
		touch.x = defDB.dieArea.lowerLeft.x + defDB.pwgnd.width / 2 + cnt * pitch;

		touch.y = point.y;

		if(touch.x > defDB.dieArea.upperRight.x) {
			if(defDB.pwgnd.upperBoundNet == "GND") {
				touch.x = defDB.dieArea.upperRight.x - defDB.pwgnd.width / 2;
			}
			else {
				touch.x -= 2 * pitch;
			}
		}
	}
	else {
		touch.x = point.x;
		
		int cnt = (point.y - defDB.dieArea.lowerLeft.y - defDB.pwgnd.width / 2) / pitch;
		cnt = (cnt % 2 == 1)? cnt : cnt + 1;
		touch.y = defDB.dieArea.lowerLeft.y + defDB.pwgnd.width / 2 + cnt * pitch;

		if(touch.y > defDB.dieArea.upperRight.y) {
			if(defDB.pwgnd.upperBoundNet == "GND") {
				touch.y = defDB.dieArea.upperRight.y - defDB.pwgnd.width / 2;
			}
			else {
				touch.y -= 2 * pitch;
			}
		}
	}
	return touch;
}

parser::Point2D<int> findVDDClusterTouch(parser::Point2D<int> point) {

    parser::PWGND pwgnd = defDB.pwgnd;
    int column = 0, row = 0;
    for(int i = 0; i < pwgnd.powerxMesh.size() + pwgnd.gndxMesh.size() - 1; i++) {
        int right, left;
        if(i % 2 == 0) {
            left = pwgnd.gndxMesh[i / 2];
            right = pwgnd.powerxMesh[i / 2]; 
        }
        else {
            left = pwgnd.powerxMesh[i / 2];
            right = pwgnd.gndxMesh[i / 2 + 1];
        }

        if(point.x > left && point.x <= right) {
            column = i ;
            break;
        }
    }

    vector<int> yVDD = pwgnd.poweryMesh[column];
    vector<int> yGND = pwgnd.gndyMesh[column];

    int bot, top;
    for(int i = 0; i < yVDD.size() + yGND.size() - 1; i++) {
        if(i % 2 == 0) {
            bot = yGND[i / 2];
            top = yVDD[i / 2]; 
        }
        else {
            bot = yVDD[i / 2];
            top = yGND[i / 2 + 1];
        }

        if(point.y > bot && point.y <= top) {
            row = i ;
            break;
        }
    }
    parser::Point2D<int> touch;
    touch.x = point.x;
    if(row % 2 == 0)
        touch.y = top;
    else
        touch.y = bot;

    return touch;
}

parser::Point2D<int> findGNDClusterTouch(parser::Point2D<int> point) {

    parser::PWGND pwgnd = defDB.pwgnd;
    int column = 0, row = 0;
    for(int i = 0; i < pwgnd.powerxMesh.size() + pwgnd.gndxMesh.size() - 1; i++) {
        int right, left;
        if(i % 2 == 0) {
            left = pwgnd.gndxMesh[i / 2];
            right = pwgnd.powerxMesh[i / 2]; 
        }
        else {
            left = pwgnd.powerxMesh[i / 2];
            right = pwgnd.gndxMesh[i / 2 + 1];
        }

        if(point.x > left && point.x <= right) {
            column = i ;
            break;
        }
    }

    vector<int> yVDD = pwgnd.poweryMesh[column];
    vector<int> yGND = pwgnd.gndyMesh[column];

    int bot, top;
    for(int i = 0; i < yVDD.size() + yGND.size() - 1; i++) {
        if(i % 2 == 0) {
            bot = yGND[i / 2];
            top = yVDD[i / 2]; 
        }
        else {
            bot = yVDD[i / 2];
            top = yGND[i / 2 + 1];
        }

        if(point.y > bot && point.y <= top) {
            row = i ;
            break;
        }
    }
    parser::Point2D<int> touch;
    touch.x = point.x;
    if(row % 2 == 0)
        touch.y = bot;
    else
        touch.y = top;

    return touch;
}*/


void placeHighLayerVias(vector<parser::Wire>& wires, int X, int Y, 
        string topLayerName, int viaIdx, int length, int viaDistance, int layerWidth) {
    
    auto via = lefDB.vias[viaIdx];
    string viaName = via.name;
    auto rect = via.layerRects[0].rects[0];
    float h = rect.upperRight.x - rect.lowerLeft.x;
    float v = rect.upperRight.y - rect.lowerLeft.y;
    float viaLength = (h > v)? h : v;
    viaLength *= defDB.dbuPerMicro;

    parser::Wire tmpWire;
    
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

int safeBoundaryDistance() {
    parser::lefVia V1 = lefDB.vias[lefDB.topLayerIdx2ViaIdx[2]]; //M1-M2 
    parser::lefVia V2 = lefDB.vias[lefDB.topLayerIdx2ViaIdx[4]]; //M2-M3

    float maxDistance = 0;

    for(auto layerRect : V1.layerRects) {
       for(auto rect : layerRect.rects) {
            maxDistance = (maxDistance > fabs(rect.lowerLeft.x))? maxDistance : fabs(rect.lowerLeft.x);
            maxDistance = (maxDistance > fabs(rect.lowerLeft.y))? maxDistance : fabs(rect.lowerLeft.y);
            maxDistance = (maxDistance > fabs(rect.upperRight.x))? maxDistance : fabs(rect.upperRight.x);
            maxDistance = (maxDistance > fabs(rect.upperRight.y))? maxDistance : fabs(rect.upperRight.y);
       }
    }
    for(auto layerRect : V2.layerRects) {
       for(auto rect : layerRect.rects) {
            maxDistance = (maxDistance > fabs(rect.lowerLeft.x))? maxDistance : fabs(rect.lowerLeft.x);
            maxDistance = (maxDistance > fabs(rect.lowerLeft.y))? maxDistance : fabs(rect.lowerLeft.y);
            maxDistance = (maxDistance > fabs(rect.upperRight.x))? maxDistance : fabs(rect.upperRight.x);
            maxDistance = (maxDistance > fabs(rect.upperRight.y))? maxDistance : fabs(rect.upperRight.y);
       }
    }
    //cout << "maxBoundaryDistance: " << maxDistance << endl;
    return maxDistance * defDB.dbuPerMicro; 
}


void routeHighLayerSNet(string signal) {
    parser::Wire tmpWire;

    parser::PWGND& pwgnd = defDB.pwgnd;
    parser::Rect2D<int> dieArea = defDB.dieArea;
    string lastHLayerName = defDB.pwgnd.lastHLayerName;
    int lastHLayerID = defDB.pwgnd.lastHLayerID;
    int width = lefDB.layers[lastHLayerID].width * defDB.dbuPerMicro * HIGHMESH_MULTIPLE;
    int pitch = lefDB.layers[lastHLayerID].pitchy * defDB.dbuPerMicro * HIGHMESH_MULTIPLE;
    int step = (2 * pitch) * HIGHMESH_STEP;
    int safeDistance = safeBoundaryDistance();
    int nReinforcement = (defDB.dieArea.upperRight.y - defDB.dieArea.lowerLeft.y - 2 * (pitch + safeDistance)) / step + 1;
    width = fitGrid(width);
    pitch = fitGrid(pitch);
    
    int offset;
    int midoffset;
    
    parser::Range<int> yrange; 
    int viaDistance = (lefDB.layers[lastHLayerID - 1].width + lefDB.layers[lastHLayerID - 1].spacing) * defDB.dbuPerMicro; 
    
    if(signal == "POWER") {
        offset = pitch;
    }
    else {
        offset = - pitch;
    }
    auto& xMesh = pwgnd.xMesh;
    auto& wires = (signal == "POWER")? pwgnd.powerWires : pwgnd.gndWires;
    auto& yranges = (signal == "POWER")? pwgnd.powerHighLayerY : pwgnd.gndHighLayerY;

    string vLayerName = pwgnd.vLayerName;
    int vlayerID = lefDB.layer2idx[vLayerName];

    if((nReinforcement == 1 && defDB.dieArea.upperRight.y - defDB.dieArea.lowerLeft.y > 3 * pitch + 2 * safeDistance) ||
            nReinforcement > 1) {
        
        for(int i = 0; i < nReinforcement; i++) {
            tmpWire.layerName = lastHLayerName;
            tmpWire.width = width;
            tmpWire.coorX[0] = dieArea.lowerLeft.x;
            tmpWire.coorY[0] = dieArea.lowerLeft.y + offset + i * step + pitch + safeDistance + width / 2;
            tmpWire.coorX[1] = dieArea.upperRight.x;
            tmpWire.coorY[1] = dieArea.lowerLeft.y + offset + i * step + pitch + safeDistance + width / 2;
            tmpWire.numPathPoint = 2;
            wires.push_back(tmpWire);

            yrange.start = tmpWire.coorY[0] - width / 2; 
            yrange.end = tmpWire.coorY[0] + width / 2; 
            yranges.push_back(yrange);
    
            for(int i = vlayerID + 2; i <= lastHLayerID; i += 2) { //M5-M6
                int viaIdx = lefDB.topLayerIdx2ViaIdx[i];
                string topLayerName = lefDB.layers[i].name;
                int layerWidth = lefDB.layers[i].width * defDB.dbuPerMicro;
                string viaName = lefDB.vias[viaIdx].name;
                for(int j = 0; j < xMesh.size(); j ++) {
                    int xpos = (signal == "POWER")? xMesh[j] + defDB.pwgnd.width : xMesh[j] - defDB.pwgnd.width;
                    placeHighLayerVias(wires, xpos, tmpWire.coorY[0], topLayerName, viaIdx, width, viaDistance, layerWidth);
                }
            }
        }
        
        int lasty = dieArea.lowerLeft.y + pitch + (nReinforcement - 1) * step + pitch + safeDistance + width / 2; 
   

    if(dieArea.upperRight.y - lasty > 2 * pitch + safeDistance + width / 2) {
        //cout << "enter extra" << endl;
        tmpWire.layerName = lastHLayerName;
        tmpWire.width = width;
        tmpWire.coorX[0] = dieArea.lowerLeft.x;
        tmpWire.coorY[0] = dieArea.upperRight.y + offset - pitch - safeDistance - width / 2;
        tmpWire.coorX[1] = dieArea.upperRight.x;
        tmpWire.coorY[1] = dieArea.upperRight.y + offset - pitch - safeDistance - width / 2;
        tmpWire.numPathPoint = 2;
        wires.push_back(tmpWire);

        yrange.start = tmpWire.coorY[0] - width / 2; 
        yrange.end = tmpWire.coorY[0] + width / 2; 
        yranges.push_back(yrange);
    
        for(int i = vlayerID + 2; i <= lastHLayerID; i += 2) { //M5-M6
            int viaIdx = lefDB.topLayerIdx2ViaIdx[i];
            string topLayerName = lefDB.layers[i].name;
            int layerWidth = lefDB.layers[i].width * defDB.dbuPerMicro;
            string viaName = lefDB.vias[viaIdx].name;
            for(int j = 0; j < xMesh.size(); j ++) {
                int xpos = (signal == "POWER")? xMesh[j] + defDB.pwgnd.width : xMesh[j] - defDB.pwgnd.width;
                placeHighLayerVias(wires, xpos, tmpWire.coorY[0], topLayerName, viaIdx, width, viaDistance, layerWidth);
            }
        }
        }
    }
    else {
        tmpWire.layerName = lastHLayerName;
            tmpWire.width = width;
            tmpWire.coorX[0] = dieArea.lowerLeft.x;
            tmpWire.coorY[0] = (dieArea.lowerLeft.y + dieArea.upperRight.y) / 2 + offset + width / 2;
            tmpWire.coorX[1] = dieArea.upperRight.x;
            tmpWire.coorY[1] = (dieArea.lowerLeft.y + dieArea.upperRight.y) / 2 + offset + width / 2;
            tmpWire.numPathPoint = 2;
            wires.push_back(tmpWire);

            yrange.start = tmpWire.coorY[0] - width / 2; 
            yrange.end = tmpWire.coorY[0] + width / 2; 
            yranges.push_back(yrange);
    
            for(int i = vlayerID + 2; i <= lastHLayerID; i += 2) { //M5-M6
                int viaIdx = lefDB.topLayerIdx2ViaIdx[i];
                string topLayerName = lefDB.layers[i].name;
                int layerWidth = lefDB.layers[i].width * defDB.dbuPerMicro;
                string viaName = lefDB.vias[viaIdx].name;
                for(int j = 0; j < xMesh.size(); j ++) {
                    int xpos = (signal == "POWER")? xMesh[j] + defDB.pwgnd.width : xMesh[j] - defDB.pwgnd.width;
                    placeHighLayerVias(wires, xpos, tmpWire.coorY[0], topLayerName, viaIdx, width, viaDistance, layerWidth);
                }
            }


    }

}

bool nearHighLayerViaRange(string signal, int ypos, int& midRange) {

    parser::PWGND& pwgnd = defDB.pwgnd;
    auto& yranges = (signal == "POWER")? pwgnd.powerHighLayerY : pwgnd.gndHighLayerY;
    bool in = false;
    int M2_spacing = (lefDB.layers[2].pitchy - lefDB.layers[2].width) * defDB.dbuPerMicro;
    int width = pwgnd.width;

    for(auto yrange : yranges) {
        if(yrange.start - M2_spacing < ypos + width / 2 && yrange.end + M2_spacing > ypos - width / 2) {
            in = true;
            midRange = (yrange.start + yrange.end) / 2;
            break;
        }
    }
    return in;
}

bool inHighLayerViaRange(string signal, int ypos) {

    parser::PWGND& pwgnd = defDB.pwgnd;
    auto& yranges = (signal == "POWER")? pwgnd.powerHighLayerY : pwgnd.gndHighLayerY;
    bool in = false;
    for(auto yrange : yranges) {
        if(yrange.start < ypos && yrange.end > ypos) {
            in = true;
            break;
        }
    }
    return in;
}

void routeLowLayerMesh(string signal) {
    parser::Wire tmpWire;

    parser::PWGND& pwgnd = defDB.pwgnd;
    parser::Rect2D<int> dieArea = defDB.dieArea;
    
    string vLayerName = pwgnd.vLayerName;
    string hLayerName = pwgnd.hLayerName;
    int vLayerID = lefDB.layer2idx[vLayerName];
    int hLayerID = lefDB.layer2idx[hLayerName];
    
    int width = pwgnd.width;
    int xoffset = (signal == "POWER")? width : (-1) * width; 
    
    auto& xMesh = pwgnd.xMesh;
    auto& yMesh = (signal == "POWER")? pwgnd.poweryMesh : pwgnd.gndyMesh;
    auto& opyMesh = (signal == "POWER")? pwgnd.gndyMesh : pwgnd.poweryMesh;
    auto& wires = (signal == "POWER")? pwgnd.powerWires : pwgnd.gndWires;

    int safeDistance = safeBoundaryDistance();
    /*xMesh[0] = dieArea.lowerLeft.x + safeDistance;
    xMesh[xMesh.size() - 1] = dieArea.upperRight.x - safeDistance;
    */
    for(int i = 0; i < xMesh.size(); i++) {
        tmpWire.layerName = vLayerName;
        tmpWire.width = width;
        
        tmpWire.coorX[0] = xMesh[i] + xoffset;
        tmpWire.coorY[0] = dieArea.lowerLeft.y;
        tmpWire.coorX[1] = xMesh[i] + xoffset;
        tmpWire.coorY[1] = dieArea.upperRight.y;
        tmpWire.numPathPoint = 2;
        wires.push_back(tmpWire);
    }

    int left_offset, right_offset;
       
    for(int i = 0; i < yMesh.size(); i++) {
        
                  
        for(int j = 0; j < yMesh[i].size(); j++) {
            if(yMesh[i][j] + safeDistance >= dieArea.upperRight.y) {
                yMesh[i][j] = dieArea.upperRight.y - safeDistance;
            }
            else if(yMesh[i][j] - safeDistance <= dieArea.lowerLeft.y) {
                yMesh[i][j] = dieArea.lowerLeft.y + safeDistance;
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
                tmpWire.layerName = hLayerName;
                tmpWire.width = width;
                tmpWire.numPathPoint = 2;
                wires.push_back(tmpWire);
                
                int moveup;
                if(yMesh[i][j] + 3 * width < defDB.dieArea.upperRight.y)  
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
                tmpWire.layerName = vLayerName;
                int viaID = lefDB.topLayerIdx2ViaIdx[vLayerID]; //M4-M5
                tmpWire.viaName = lefDB.vias[viaID].name;
                tmpWire.width = 0;
                wires.push_back(tmpWire);
                

            }
            else {

                tmpWire.coorX[0] = xMesh[i] + xoffset - width / 2;
                tmpWire.coorX[1] = xMesh[i + 1] + xoffset + width / 2;
                if(yMesh[i][j] + safeDistance >= dieArea.upperRight.y) {
                    yMesh[i][j] = dieArea.upperRight.y - safeDistance;
                }
                else if(yMesh[i][j] - safeDistance <= dieArea.lowerLeft.y) {
                    yMesh[i][j] = dieArea.lowerLeft.y + safeDistance;
                }
            
                tmpWire.coorY[0] = yMesh[i][j];
                tmpWire.coorY[1] = yMesh[i][j];
                tmpWire.layerName = hLayerName;
                tmpWire.width = width;
                tmpWire.numPathPoint = 2;
                wires.push_back(tmpWire);
            
                
                tmpWire.numPathPoint = 1;
                tmpWire.layerName = vLayerName;
                int viaID = lefDB.topLayerIdx2ViaIdx[vLayerID]; //M4-M5
                tmpWire.viaName = lefDB.vias[viaID].name;
            
                tmpWire.coorX[0] = xMesh[i] + xoffset;
                tmpWire.coorY[0] = yMesh[i][j];
                tmpWire.width = 0;
                wires.push_back(tmpWire); 
            
            }

                       
#ifdef HORIZONTAL_M4
            tmpWire.coorX[0] = xMesh[i + 1] + xoffset;
            tmpWire.coorY[0] = yMesh[i][j];
            tmpWire.width = 0;
            wires.push_back(tmpWire); // M4-M5 will always not be covered
#else   
            
            if(!inHighLayerViaRange(signal, tmpWire.coorY[0])) // if covered by high via layers, no need to add via
                wires.push_back(tmpWire);
#endif

            int midRange = 0;
            if(nearHighLayerViaRange(signal, tmpWire.coorY[0], midRange)) {// if near, need to add metal to connect 
                int M2_spacing = (lefDB.layers[2].pitchy - lefDB.layers[2].width) * defDB.dbuPerMicro;
                if(yMesh[i][j] < midRange) {
                    tmpWire.coorY[0] = yMesh[i][j];
                    tmpWire.coorY[1] = yMesh[i][j] + width / 2 + M2_spacing;
                }
                else {
                    tmpWire.coorY[0] = yMesh[i][j] - width / 2 - M2_spacing;
                    tmpWire.coorY[1] = yMesh[i][j];
                }
                
                tmpWire.coorX[1] = tmpWire.coorX[0];

                tmpWire.layerName = vLayerName; 
                tmpWire.width = width;
                tmpWire.numPathPoint = 2;
                wires.push_back(tmpWire);
                
            }
        }
    }
}

void findRowSNet(string componentName, string pinName, parser::Rect2D<float> rect, int& powerY, int& gndY) {
    Point2D<int> midPoint;
    midPoint.x = (rect.lowerLeft.x + rect.upperRight.x) / 2;
    midPoint.y = (rect.lowerLeft.y + rect.upperRight.y) / 2;

    int column = -1;
    auto& xMesh = defDB.pwgnd.xMesh;
    for(int i = 0; i < xMesh.size() - 1; i++) {
        if(midPoint.x > xMesh[i] && midPoint.x < xMesh[i + 1]) {
            column = i;
            break;
        }
    }
    if(column == -1) {
        cout << "ERROR: Pin's X is not in any column!" << endl;
        cout << componentName << "/" << pinName << endl;
        cout << "Pin: " << rect << endl;
        exit(1);
    }
    
    auto& poweryMesh = defDB.pwgnd.poweryMesh[column];
    auto& gndyMesh = defDB.pwgnd.gndyMesh[column];

    auto powerit = std::upper_bound (poweryMesh.begin(), poweryMesh.end(), midPoint.y);  
    auto gndit = std::upper_bound (gndyMesh.begin(), gndyMesh.end(), midPoint.y);

    if(powerit == poweryMesh.begin() && gndit == gndyMesh.begin()) {
        cout << "ERROR: Pin's Y is not in any row!" << endl;
        cout << componentName << "/" << pinName << endl;
        cout << "Pin: " << rect << endl;
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

void findTouchPointsOBSNoTrack(vector<parser::Rect2D<float>>& rects, const map<int, int>& closestPinPoint, 
        map<int, int>& closestOBSPoint, int expand, int signalY) {
    for(auto rect : rects) {
        //inclusive track       
        int left = rect.lowerLeft.x - expand;
        int right = rect.upperRight.x + expand;
        
        int midY = (rect.lowerLeft.y + rect.upperRight.y) / 2;
        
        for(auto touchPoint : closestPinPoint) {
            int x = touchPoint.first;
            if(left <= x && right >= x) {
                if(closestOBSPoint.count(x) == 0) 
                    closestOBSPoint[x] = (midY < signalY)? rect.upperRight.y : rect.lowerLeft.y;
                else {
                    if(midY < signalY && closestOBSPoint[x] < rect.upperRight.y) {
                        closestOBSPoint[x] = rect.upperRight.y;   
                    }
                    else if(midY > signalY && closestOBSPoint[x] > rect.lowerLeft.y) {
                        closestOBSPoint[x] = rect.lowerLeft.y;    
                    }
                }
            }
        }
    }
}
void findTouchPointsNoTrack(vector<parser::Rect2D<float>>& rects, map<int, int>& closestPoint, int expand, int signalY) {
    int M3_pitch = lefDB.layers[4].pitchx * defDB.dbuPerMicro;
    for(auto rect : rects) {
        //inclusive track       
        /*int left = rect.lowerLeft.x - expand;
        int right = rect.upperRight.x + expand;*/

        int left = rect.lowerLeft.x + expand;
        int right = rect.upperRight.x - expand; //actually this is shrink
        
        int midX = (rect.lowerLeft.x + rect.upperRight.x) / 2;
        int midY = (rect.lowerLeft.y + rect.upperRight.y) / 2;
        
        

        for(int x = left; x <= right; x += M3_pitch) {
            if(closestPoint.count(x) == 0) 
                closestPoint[x] = (midY < signalY)? rect.upperRight.y : rect.lowerLeft.y;
            else {
                if(midY < signalY && closestPoint[x] < rect.upperRight.y) {
                    closestPoint[x] = rect.upperRight.y;   
                }
                else if(midY > signalY && closestPoint[x] > rect.lowerLeft.y) {
                    closestPoint[x] = rect.lowerLeft.y;    
                }
            }
        }
    }
}

void findFarthestTouchPoints(vector<parser::Rect2D<float>>& rects, map<int, int>& farthestPoint, 
        parser::Track track, int expand, int signalY) {
    for(auto rect : rects) {
        //inclusive track       
        int left = (rect.lowerLeft.x - expand - track.start) / track.step + 1;
        int right = (rect.upperRight.x + expand - track.start) / track.step;
        
        int midY = (rect.lowerLeft.y + rect.upperRight.y) / 2;

        for(int i = left; i <= right; i++) {
            if(farthestPoint.count(i) == 0) 
                farthestPoint[i] = (midY < signalY)? rect.lowerLeft.y : rect.upperRight.y;
            else {
                if(midY < signalY && farthestPoint[i] > rect.lowerLeft.y) {
                    farthestPoint[i] = rect.lowerLeft.y;   
                }
                else if(midY > signalY && farthestPoint[i] < rect.upperRight.y) {
                    farthestPoint[i] = rect.upperRight.y;    
                }
            }
        }
    }
}


void findClosestTouchPoints(vector<parser::Rect2D<float>>& rects, map<int, int>& closestPoint, 
        parser::Track track, int expand, int signalY) {
    for(auto rect : rects) {
        //inclusive track       
        int left = (rect.lowerLeft.x - expand - track.start) / track.step + 1;
        int right = (rect.upperRight.x + expand - track.start) / track.step;
        
        int midY = (rect.lowerLeft.y + rect.upperRight.y) / 2;
        

        for(int i = left; i <= right; i++) {
            if(closestPoint.count(i) == 0) 
                closestPoint[i] = (midY < signalY)? rect.upperRight.y : rect.lowerLeft.y;
            else {
                if(midY < signalY && closestPoint[i] < rect.upperRight.y) {
                    closestPoint[i] = rect.upperRight.y;   
                }
                else if(midY > signalY && closestPoint[i] > rect.lowerLeft.y) {
                    closestPoint[i] = rect.lowerLeft.y;    
                }
            }
        }
    }
}

bool M1M3DetailedRouteSNet(parser::Component& component, string signal, int signalY, 
        vector<parser::Wire>& Wires, Point2D<int>& powerPoint) {
    
    auto& pwgnd = defDB.pwgnd;
    
    vector<parser::Rect2D<float>> M2obsRect;
    vector<parser::Rect2D<float>> M1obsRect;
    vector<parser::Rect2D<float>> V1obsRect; 
    vector<parser::Rect2D<float>> pinRect;

    for(auto pin : component.macro.pins) {
        for(auto layerRect : pin.layerRects) {
            int layerIdx = lefDB.layer2idx[layerRect.layerName];
            if(layerIdx == 2) { //M2 
                for(auto rect : layerRect.rects) {
                    if(pin.use != signal)
                        M2obsRect.push_back(rect);
                }
            }
            else if(layerIdx == 0) { //M1
                for(auto rect : layerRect.rects) {
                    if(pin.use != signal)
                        M1obsRect.push_back(rect);
                    else
                        pinRect.push_back(rect);
                }
            }
            else if(layerIdx == 1) { //V1
                for(auto rect : layerRect.rects) {
                    V1obsRect.push_back(rect);// can be the same signal
                }
            }
        }
    }
    
    for(auto layerRect : component.macro.obs.layerRects) {
        int layerIdx = lefDB.layer2idx[layerRect.layerName];
        if(layerIdx == 2) { //M2 
            for(auto rect : layerRect.rects) {
                M2obsRect.push_back(rect);
            }
        }
        else if(layerIdx == 0) { //M1
            for(auto rect : layerRect.rects) {
                M1obsRect.push_back(rect);
            }
        }
        else if(layerIdx == 0) { //V1
            for(auto rect : layerRect.rects) {
                V1obsRect.push_back(rect);
            }
        }
    }
  

    int width = lefDB.layers[4].width * defDB.dbuPerMicro; //M3
    map<int, int> trackClosestPinPoint; //map trackIdx -> y
    map<int, int> trackFarthestPinPoint; 

    //map<int, int> touchPinPoint; //map x -> y 
    parser::Track track = defDB.tracks[defDB.layeridx2trackidx[4]]; //M3
    
    findClosestTouchPoints(pinRect, trackClosestPinPoint, track, width / 2, signalY);
    //findTouchPointsNoTrack(pinRect, touchPinPoint, track, width / 2, signalY);
    findFarthestTouchPoints(pinRect, trackFarthestPinPoint, track, width / 2, signalY);
    
    string M1_name = lefDB.layers[0].name;
    int M1_width = lefDB.layers[0].width * defDB.dbuPerMicro;
    int M1_pitch = lefDB.layers[0].pitchx * defDB.dbuPerMicro;
    int M1_minlength = lefDB.layers[0].minLength * defDB.dbuPerMicro;
    int M1_spacing = (M1_pitch - M1_width);
    
    int V1_width = lefDB.layers[1].width * defDB.dbuPerMicro;
    int V1_spacing = lefDB.layers[1].spacing * defDB.dbuPerMicro;
 
    int M2_minlength = lefDB.layers[2].minLength * defDB.dbuPerMicro;
    int M2_pitch = lefDB.layers[2].pitchx * defDB.dbuPerMicro;
    int M2_width = lefDB.layers[2].width * defDB.dbuPerMicro;
    int M2_spacing = (M2_pitch - M2_width);
        
    int M3_width = lefDB.layers[4].width * defDB.dbuPerMicro;
    int M3_pitch = lefDB.layers[4].pitchx * defDB.dbuPerMicro;
    int M3_minlength = lefDB.layers[4].minLength * defDB.dbuPerMicro;
    
    bool foundM1M3 = false;
    float touchX, touchY;
    string M1M2ViaName;
   
    bool M2thinFail = false;
    bool M2wideFail = false;
    for(auto pinPoint : trackClosestPinPoint) {
        int trackIdx = pinPoint.first;
        touchX = track.start + trackIdx * track.step;
        //touchX = pinPoint.first;
        touchY = pinPoint.second;
        
        //move touchY futher from signalY by width/2
        if(touchY < signalY)
            touchY -= M3_width / 2;
        else
            touchY += M3_width / 2;


        Point2D<float> touchPoint(touchX, touchY);
        bool M1cover = false;
        bool M2cover = false;
        bool V1cover = false; 
        
        for(int viaIdx = lefDB.topLayerIdx2ViaIdx[2]; viaIdx < lefDB.topLayerIdx2ViaIdx[4]; viaIdx++) { //vias whose top layer is M2
            parser::lefVia via = lefDB.vias[viaIdx];
            int llx, lly, urx, ury;
            M1cover = false;
            for(auto layerRect : via.layerRects) {
                if(layerRect.layerName == M1_name) {
                    llx = layerRect.rects[0].lowerLeft.x * defDB.dbuPerMicro;        
                    lly = layerRect.rects[0].lowerLeft.y * defDB.dbuPerMicro;        
                    urx = layerRect.rects[0].upperRight.x * defDB.dbuPerMicro;        
                    ury = layerRect.rects[0].upperRight.y * defDB.dbuPerMicro;        
                }
            }
            
            for(auto rect : M1obsRect) {
                auto tmpRect = rect;
                tmpRect.lowerLeft.x -= (urx + M1_spacing); 
                tmpRect.upperRight.x += (- llx + M1_spacing); 
                tmpRect.lowerLeft.y -= (ury + M1_spacing);
                tmpRect.upperRight.y += (- lly + M1_spacing);
            
                if(tmpRect.boundaryExclusiveCover(touchPoint)) {
                    M1cover = true;
                    break;
                }
            }
            if(!M1cover) {
                M1M2ViaName = via.name;
                break;
            }
        } // find m1-m2 via that doesn't conflict with m1 obs

        M2wideFail= false;
        M2thinFail = false;
        for(auto rect : M2obsRect) {
            auto tmpRect = rect;
            tmpRect.lowerLeft.x -= (M2_minlength / 2 + M2_spacing); 
            tmpRect.upperRight.x += (M2_minlength / 2 + M2_spacing); 
            tmpRect.lowerLeft.y -= (M2_width / 2 + M2_spacing);
            tmpRect.upperRight.y += (M2_width / 2 + M2_spacing);
            
            if(tmpRect.boundaryExclusiveCover(touchPoint)) {
                M2thinFail = true;
                break;
            }
        } // m2 side min area doesn't conflict with m2 obs
        if(touchPoint.x - M2_minlength / 2 - M2_spacing / 2 < component.location.x || 
                touchPoint.x + M2_minlength / 2 + M2_spacing / 2 > component.location.x + component.macro.size.x ||
                    touchPoint.y - M2_width / 2 - M2_spacing / 2 < component.location.y || 
                    touchPoint.y + M2_width / 2 + M2_spacing / 2 > component.location.y + component.macro.size.y) 
            M2thinFail = true;

        for(auto rect : M2obsRect) {
            auto tmpRect = rect;
            tmpRect.lowerLeft.x -= (M2_minlength / 6 + M2_spacing); 
            tmpRect.upperRight.x += (M2_minlength / 6 + M2_spacing); 
            tmpRect.lowerLeft.y -= (M2_width * 3 / 2 + M2_spacing);
            tmpRect.upperRight.y += (M2_width * 3 / 2 + M2_spacing);
            
            if(tmpRect.boundaryExclusiveCover(touchPoint)) {
                M2wideFail = true;
                break;
            }
        }
        if(touchPoint.x - M2_minlength / 6 - M2_spacing / 2 < component.location.x || 
                touchPoint.x + M2_minlength / 6 + M2_spacing / 2 > component.location.x + component.macro.size.x ||
                    touchPoint.y - M2_width * 3 / 2 - M2_spacing / 2 < component.location.y || 
                    touchPoint.y + M2_width * 3 / 2 + M2_spacing / 2 > component.location.y + component.macro.size.y) 
            M2wideFail = true;

        M2cover = M2wideFail && M2thinFail;  
        
        if(component.name == "cx1") {
            cout << signal << ":" << endl;
            cout << "close M2cover : " << M2cover << " M2wideFail: " << M2wideFail << " M2thinFail: " << M2thinFail << endl;
            cout << "touchpoint x : " << touchPoint.x << " " << touchPoint.y;
            cout << " M2minlength/2: " << M2_minlength / 2 << " component.x: " << component.location.x << endl; 
        }

 
        for(auto rect : V1obsRect) {
            auto tmpRect = rect;
            tmpRect.lowerLeft.x -= (V1_width / 2 + V1_spacing); 
            tmpRect.upperRight.x += (V1_width / 2 + V1_spacing); 
            tmpRect.lowerLeft.y -= (V1_width / 2 + V1_spacing);
            tmpRect.upperRight.y += (V1_width / 2 + V1_spacing);
            
            if(tmpRect.boundaryExclusiveCover(touchPoint)) {
                V1cover = true;
                break;
            }
        }//V1 via spacing
        if(component.name == "cx1")
            cout << V1cover << M2cover << M1cover << endl;
        
        
        if(V1cover == false && M2cover == false && M1cover == false) {       
            foundM1M3 = true;
            break;
        }
    }
   
    if(!foundM1M3)
    for(auto pinPoint : trackFarthestPinPoint) {
        int trackIdx = pinPoint.first;
        touchX = track.start + trackIdx * track.step;
        //touchX = pinPoint.first;
        touchY = pinPoint.second;
        
        //move touchY towards signalY by width/2
        if(touchY < signalY)
            touchY += M3_width / 2;
        else
            touchY -= M3_width / 2;


        Point2D<float> touchPoint(touchX, touchY);
        bool M1cover = false;
        bool M2cover = false;
        bool V1cover = false; 
        
        for(int viaIdx = lefDB.topLayerIdx2ViaIdx[2]; viaIdx < lefDB.topLayerIdx2ViaIdx[4]; viaIdx++) { //vias whose top layer is M2
            parser::lefVia via = lefDB.vias[viaIdx];
            int llx, lly, urx, ury;
            M1cover = false;
            for(auto layerRect : via.layerRects) {
                if(layerRect.layerName == M1_name) {
                    llx = layerRect.rects[0].lowerLeft.x * defDB.dbuPerMicro;        
                    lly = layerRect.rects[0].lowerLeft.y * defDB.dbuPerMicro;        
                    urx = layerRect.rects[0].upperRight.x * defDB.dbuPerMicro;        
                    ury = layerRect.rects[0].upperRight.y * defDB.dbuPerMicro;        
                }
            }
            
            for(auto rect : M1obsRect) {
                auto tmpRect = rect;
                tmpRect.lowerLeft.x -= (urx + M1_spacing); 
                tmpRect.upperRight.x += (- llx + M1_spacing); 
                tmpRect.lowerLeft.y -= (ury + M1_spacing);
                tmpRect.upperRight.y += (- lly + M1_spacing);
            
                if(tmpRect.boundaryExclusiveCover(touchPoint)) {
                    M1cover = true;
                    break;
                }
            }
            if(!M1cover) {
                M1M2ViaName = via.name;
                break;
            }
        } // find m1-m2 via that doesn't conflict with m1 obs

        M2wideFail= false;
        M2thinFail = false;
        for(auto rect : M2obsRect) {
            auto tmpRect = rect;
            tmpRect.lowerLeft.x -= (M2_minlength / 2 + M2_spacing); 
            tmpRect.upperRight.x += (M2_minlength / 2 + M2_spacing); 
            tmpRect.lowerLeft.y -= (M2_width / 2 + M2_spacing);
            tmpRect.upperRight.y += (M2_width / 2 + M2_spacing);
            
            if(tmpRect.boundaryExclusiveCover(touchPoint)) {
                M2thinFail = true;
                break;
            }
        } // m2 side min area doesn't conflict with m2 obs
        if(touchPoint.x - M2_minlength / 2 - M2_spacing / 2 < component.location.x || 
                    touchPoint.x + M2_minlength / 2 + M2_spacing / 2 > component.location.x + component.macro.size.x ||
                    touchPoint.y - M2_width / 2 - M2_spacing / 2 < component.location.y || 
                    touchPoint.y + M2_width / 2 + M2_spacing / 2 > component.location.y + component.macro.size.y) {
            M2thinFail = true;
        }
        for(auto rect : M2obsRect) {
            auto tmpRect = rect;
            tmpRect.lowerLeft.x -= (M2_minlength / 6 + M2_spacing); 
            tmpRect.upperRight.x += (M2_minlength / 6 + M2_spacing); 
            tmpRect.lowerLeft.y -= (M2_width * 3 / 2 + M2_spacing);
            tmpRect.upperRight.y += (M2_width * 3 / 2 + M2_spacing);
            
            if(tmpRect.boundaryExclusiveCover(touchPoint)) {
                M2wideFail = true;
                break;
            }
        }
        
        if(touchPoint.x - M2_minlength / 6 - M2_spacing / 2 < component.location.x || 
                    touchPoint.x + M2_minlength / 6 + M2_spacing / 2 > component.location.x + component.macro.size.x ||
                    touchPoint.y - M2_width * 3 / 2 - M2_spacing / 2 < component.location.y || 
                    touchPoint.y + M2_width * 3 / 2 + M2_spacing / 2 > component.location.y + component.macro.size.y) { 
            M2wideFail = true;            
        }
        M2cover = M2wideFail && M2thinFail;  
        if(component.name == "cx1") {
            cout << signal << ": " << endl;
            cout << "far M2cover : " << M2cover << " M2wideFail: " << M2wideFail << " M2thinFail: " << M2thinFail << endl;
            cout << "touchpoint x : " << touchPoint.x << " " << touchPoint.y;
            cout << " M2minlength/2: " << M2_minlength / 2 << " component.x: " << component.location.x << endl; 
        }


        for(auto rect : V1obsRect) {
            auto tmpRect = rect;
            tmpRect.lowerLeft.x -= (V1_width / 2 + V1_spacing); 
            tmpRect.upperRight.x += (V1_width / 2 + V1_spacing); 
            tmpRect.lowerLeft.y -= (V1_width / 2 + V1_spacing);
            tmpRect.upperRight.y += (V1_width / 2 + V1_spacing);
            
            if(tmpRect.boundaryExclusiveCover(touchPoint)) {
                V1cover = true;
                break;
            }
        }//V1 via spacing
      
        if(component.name == "cx1")
            cout << V1cover << M2cover << M1cover << endl;
        
 
       if(V1cover == false && M2cover == false && M1cover == false) {       
            foundM1M3 = true;
            break;
        }
    }


    if(foundM1M3) {
        parser::Wire tmpWire;
        
        if(signal == "POWER") { 
            powerPoint.x = touchX;
            powerPoint.y = touchY;
        }
     
        auto& xMesh = defDB.pwgnd.xMesh;
        bool move_right;
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
        tmpWire.layerName = lefDB.layers[2].name;
        tmpWire.width = 0;
        tmpWire.viaName = M1M2ViaName;
        Wires.push_back(tmpWire);//M2 to M1 via

        int zRouteOffset = 0;
        if(signal == "GROUND" && touchX == powerPoint.x &&
                ((touchY < signalY && powerPoint.y > touchY) || (touchY > signalY && powerPoint.y < touchY))) { //Z route
           
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
            tmpWire.layerName = lefDB.layers[4].name; //M3
            tmpWire.width = M3_width;
            Wires.push_back(tmpWire);//M3 wire
            
            tmpWire.coorX[0] = touchX - M2_width / 2;//
            tmpWire.coorY[0] = touchY;
            tmpWire.coorX[1] = touchX + zRouteOffset + M2_width / 2;//
            tmpWire.coorY[1] = touchY;
            
            tmpWire.numPathPoint = 2;
            tmpWire.layerName = lefDB.layers[2].name; //M2
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
            tmpWire.layerName = lefDB.layers[4].name; //M3
            tmpWire.width = M3_width;
            Wires.push_back(tmpWire);//M3 wire

        }

#ifdef HORIZONTAL_M4
        int viaID = lefDB.topLayerIdx2ViaIdx[6];
        tmpWire.coorX[0] = touchX + zRouteOffset;
        tmpWire.coorY[0] = signalY;
        tmpWire.numPathPoint = 1;
        tmpWire.layerName = lefDB.layers[6].name;
        tmpWire.width = 0;
        tmpWire.viaName = lefDB.vias[viaID].name;
        Wires.push_back(tmpWire);//M3 to M4 via, mesh
#else
        int viaID = lefDB.topLayerIdx2ViaIdx[4];
        tmpWire.coorX[0] = touchX + zRouteOffset;
        tmpWire.coorY[0] = signalY;
        tmpWire.numPathPoint = 1;
        tmpWire.layerName = lefDB.layers[4].name;
        tmpWire.width = 0;
        tmpWire.viaName = lefDB.vias[viaID].name;
        Wires.push_back(tmpWire);//M3 to M2 via, mesh
#endif

        viaID = lefDB.topLayerIdx2ViaIdx[4];
        tmpWire.coorX[0] = touchX + zRouteOffset;//
        tmpWire.coorY[0] = touchY;
        tmpWire.numPathPoint = 1;
        tmpWire.layerName = lefDB.layers[4].name;
        tmpWire.width = 0;
        tmpWire.viaName = lefDB.vias[viaID].name;
        Wires.push_back(tmpWire);//M3 to M2 via, touch
        
        if(!M2thinFail) {
            tmpWire.coorX[0] = touchX - M2_minlength / 2;
            tmpWire.coorX[1] = touchX + M2_minlength / 2;
            tmpWire.width = M2_width;
        }
        else {
            tmpWire.coorX[0] = touchX - M2_minlength / 6;
            tmpWire.coorX[1] = touchX + M2_minlength / 6;
            tmpWire.width = 3 * M2_width;
        }

        tmpWire.coorY[0] = touchY;
        tmpWire.coorY[1] = touchY;
        tmpWire.numPathPoint = 2;
        tmpWire.layerName = lefDB.layers[2].name; //M2
        Wires.push_back(tmpWire);//M2 min area 
        
        
        int M3_minLength = lefDB.layers[4].minLength * defDB.dbuPerMicro;
        if(fabs(touchY - signalY) < M3_minLength) {
            tmpWire.coorX[0] = touchX; 
            tmpWire.coorX[1] = touchX; 
            tmpWire.coorY[0] = signalY;
            tmpWire.coorY[1] = (signalY < touchY)? signalY + M3_minLength : signalY - M3_minLength;
            
            tmpWire.width = M3_width;
            tmpWire.layerName = lefDB.layers[4].name;
            tmpWire.numPathPoint = 2;
            Wires.push_back(tmpWire); //M3 min area
        }

    }
    return foundM1M3;
}   


bool M2DetailedRouteSNet(parser::Component& component, string signal, int signalY, 
        vector<parser::Wire>& Wires, Point2D<int>& powerPoint) {
    vector<parser::Rect2D<float>> pinRect;
    vector<parser::Rect2D<float>> obsRect;
    
    for(auto pin : component.macro.pins) {
        for(auto layerRect : pin.layerRects) {
            int layerIdx = lefDB.layer2idx[layerRect.layerName];
            if(layerIdx == 2) { //M2 
                for(auto rect : layerRect.rects) {
                    if(pin.use == signal)
                        pinRect.push_back(rect);
                    else
                        obsRect.push_back(rect);
                }
            }
        }
    }
    
    for(auto layerRect : component.macro.obs.layerRects) {
        int layerIdx = lefDB.layer2idx[layerRect.layerName];
        if(layerIdx == 2) { //M2
            for(auto rect : layerRect.rects) {
                obsRect.push_back(rect);
            }
        }
    }
    
    if(pinRect.size() == 0)
        return false;

    parser::Track track = defDB.tracks[defDB.layeridx2trackidx[4]]; //M3

    map<int, int> trackClosestPinPoint;
    map<int, int> trackClosestOBSPoint;
    
    /*map<int, int> closestPinPoint;
    map<int, int> closestOBSPoint;
    */
    int M2_minLength = lefDB.layers[2].minLength * defDB.dbuPerMicro;
    int M2_pitch = lefDB.layers[2].pitchx * defDB.dbuPerMicro;
    int M2_width = lefDB.layers[2].width * defDB.dbuPerMicro;
    int M2_spacing = (M2_pitch - M2_width);
     
    int viaIdx = lefDB.topLayerIdx2ViaIdx[2];
    auto via = lefDB.vias[viaIdx];
    auto viaRect = via.layerRects[0].rects[0];
    float h = (viaRect.upperRight.x - viaRect.lowerLeft.x) * defDB.dbuPerMicro;
    float v = (viaRect.upperRight.y - viaRect.lowerLeft.y) * defDB.dbuPerMicro;
    float viaWidth = (h > v)? v : h;
    float viaLength = (h > v)? h : v;
    
    int width = lefDB.layers[4].width * defDB.dbuPerMicro; //M3
    int pitch = lefDB.layers[4].pitchx * defDB.dbuPerMicro;
    findClosestTouchPoints(pinRect, trackClosestPinPoint, track, 0, signalY); // no expand
    findClosestTouchPoints(obsRect, trackClosestOBSPoint, track, pitch - width / 2, signalY); // this is width/2 + spacing

    bool foundM2 = false;
    
    int touchX, touchY;
   

    for(auto pinPoint : trackClosestPinPoint) { // Cell library gaurantee min area on M2
        int trackIdx = pinPoint.first;
        //touchX = pinPoint.first;
        touchX = track.start + trackIdx * track.step;
        touchY = pinPoint.second;
        float obsY = trackClosestOBSPoint[trackIdx];
 
        Point2D<float> touchPoint(touchX, touchY);
        bool M2cover = false;
        for(auto rect : obsRect) {
            auto tmpRect = rect;
            tmpRect.lowerLeft.x -= (viaLength / 2 + M2_spacing); // just gaurantee via 
            tmpRect.upperRight.x += (viaLength / 2 + M2_spacing); 
            tmpRect.lowerLeft.y -= M2_spacing;
            tmpRect.upperRight.y += M2_spacing;
            
            if(tmpRect.boundaryExclusiveCover(touchPoint)) {
                if(component.name == "p_ae__131_acpx0" && signal == "GROUND") {
                    cout << "M2 " << component.name << ": " << touchX << " " << touchY << endl;
                    cout << rect << endl;
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
    
    
    int M3_width = lefDB.layers[4].width * defDB.dbuPerMicro;
    int M3_pitch = lefDB.layers[4].pitchx * defDB.dbuPerMicro;
    int M3_minlength = lefDB.layers[4].minLength * defDB.dbuPerMicro;
    

    if(foundM2) {
        parser::Wire tmpWire;
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
            tmpWire.layerName = lefDB.layers[4].name; //M3
            tmpWire.width = M3_width;
            Wires.push_back(tmpWire);//M3 wire
            
            tmpWire.coorX[0] = touchX - M2_width / 2;//
            tmpWire.coorY[0] = touchY;
            tmpWire.coorX[1] = touchX + zRouteOffset + M2_width / 2;//
            tmpWire.coorY[1] = touchY;
            
            tmpWire.numPathPoint = 2;
            tmpWire.layerName = lefDB.layers[2].name; //M2
            tmpWire.width = M2_width;
            Wires.push_back(tmpWire);//M2 wire
            
            tmpWire.coorX[0] = touchX + zRouteOffset;
            tmpWire.coorY[0] = touchY;
            tmpWire.numPathPoint = 1;
            tmpWire.width = 0;
            viaID = lefDB.topLayerIdx2ViaIdx[4]; //M2-M3 via, touch side
            tmpWire.viaName = lefDB.vias[viaID].name;
            tmpWire.layerName = lefDB.layers[4].name; //M2-M3 via
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
            tmpWire.layerName = lefDB.layers[4].name; //M3
            tmpWire.width = M3_width;
            Wires.push_back(tmpWire);//M3 wire

            tmpWire.coorX[0] = touchX;
            tmpWire.coorY[0] = touchY;
            viaID = lefDB.topLayerIdx2ViaIdx[4]; //M2-M3 via, touch side
            tmpWire.numPathPoint = 1;
            tmpWire.width = 0;
            tmpWire.layerName = lefDB.layers[4].name; //M3
            tmpWire.viaName = lefDB.vias[viaID].name;
            Wires.push_back(tmpWire);
        }

#ifdef HORIZONTAL_M4
        int M3_minLength = lefDB.layers[4].minLength * defDB.dbuPerMicro;
        
        int M2_width = lefDB.layers[2].width * defDB.dbuPerMicro;
        int M3_width = lefDB.layers[4].width * defDB.dbuPerMicro;
        
        tmpWire.coorX[0] = touchX + zRouteOffset; 
        tmpWire.coorY[0] = signalY; 
        tmpWire.numPathPoint = 1;
        tmpWire.layerName = lefDB.layers[6].name;
        tmpWire.width = 0;
        viaID = lefDB.topLayerIdx2ViaIdx[6];
        tmpWire.viaName = lefDB.vias[viaID].name;
        Wires.push_back(tmpWire); //M3-M4 via

        /*if(fabs(touchY - signalY) < M3_minLength) {
            tmpWire.coorX[0] = touchX + zRouteOffset; 
            tmpWire.coorX[1] = touchX + zRouteOffset; 
            tmpWire.coorY[0] = signalY;
            tmpWire.coorY[1] = (touchY > signalY)? signalY + M3_minLength : signalY - M3_minLength;
            
            tmpWire.width = M3_width;
            tmpWire.layerName = lefDB.layers[4].name;
            tmpWire.numPathPoint = 2;
            Wires.push_back(tmpWire); //M3 min area
        }*/
#else
        viaID = lefDB.topLayerIdx2ViaIdx[4]; //M2-M3 via, mesh side
        tmpWire.coorY[0] = signalY;
        Wires.push_back(tmpWire);
#endif
    }
    return foundM2;

}

bool M1DetailedRouteSNet(parser::Component& component, string signal, int signalY, 
        vector<parser::Wire>& Wires, Point2D<int>& powerPoint) {
    vector<parser::Rect2D<float>> pinRect;
    vector<parser::Rect2D<float>> M2pinRect;
    vector<parser::Rect2D<float>> obsRect;
    
    for(auto pin : component.macro.pins) {
        for(auto layerRect : pin.layerRects) {
            int layerIdx = lefDB.layer2idx[layerRect.layerName];
            if(layerIdx == 0) { //M1 
                for(auto rect : layerRect.rects) {
                    if(pin.use == signal)
                        pinRect.push_back(rect);
                    else
                        obsRect.push_back(rect);
                }
            }
            else if(layerIdx == 2) { //M2
                for(auto rect : layerRect.rects) {
                    if(pin.use == signal)
                        M2pinRect.push_back(rect);
                }
            }
        }
    }
    
    for(auto layerRect : component.macro.obs.layerRects) {
        int layerIdx = lefDB.layer2idx[layerRect.layerName];
        if(layerIdx == 0) { //M1
            for(auto rect : layerRect.rects) {
                obsRect.push_back(rect);
            }
        }
    }

    if(pinRect.size() == 0 && M2pinRect.size() == 0) // no pin
        return true;
    else if(pinRect.size() == 0) // no pin on m1
        return false;
   
#ifdef HORIZONTAL_M4
    return false;
#endif

    parser::Track track = defDB.tracks[defDB.layeridx2trackidx[4]]; //M3

    map<int, int> trackClosestPinPoint;
    map<int, int> trackClosestOBSPoint;
    
    /*map<int, int> closestPinPoint;
    map<int, int> closestOBSPoint;
    */
 
    int M3_width = lefDB.layers[4].width * defDB.dbuPerMicro;
    int M3_pitch = lefDB.layers[4].pitchx * defDB.dbuPerMicro;
    
    findClosestTouchPoints(pinRect, trackClosestPinPoint, track, M3_width / 2, signalY);
    findClosestTouchPoints(obsRect, trackClosestOBSPoint, track, M3_pitch - M3_width / 2, signalY); // this is width/2 + spacing

    /*findTouchPointsNoTrack(pinRect, closestPinPoint, width / 2, signalY);
    findTouchPointsOBSNoTrack(obsRect, closestPinPoint, closestOBSPoint, pitch - width / 2, signalY);
    */
    
    int M1_width = lefDB.layers[0].width * defDB.dbuPerMicro;
    int M1_pitch = lefDB.layers[0].pitchx * defDB.dbuPerMicro;
    int M1_spacing = (M1_pitch - M1_width);
    
        bool foundM1 = false;
    int touchX, touchY;
    for(auto pinPoint : trackClosestPinPoint) {
        int trackIdx = pinPoint.first;
        //touchX = pinPoint.first;
        touchX = track.start + trackIdx * track.step;
        touchY = pinPoint.second;
        float obsY = trackClosestOBSPoint[trackIdx];
        
        bool M1cover = false;
        Point2D<float> touchPoint(touchX, touchY);
        
        for(auto rect : obsRect) {
            auto tmpRect = rect;
            tmpRect.lowerLeft.x -= (M3_width / 2 + M1_spacing); 
            tmpRect.upperRight.x += (M3_width / 2 + M1_spacing); 
            tmpRect.lowerLeft.y -= (M3_width + M1_spacing);
            tmpRect.upperRight.y += (M3_width + M1_spacing);
            
            if(tmpRect.boundaryExclusiveCover(touchPoint)) {
                M1cover = true;
                break;
            }
        } // m2 side min area doesn't conflict with m2 obs       
        if(M1cover)
     continue;
        
        
        if(trackClosestOBSPoint.count(trackIdx) != 0) {
            if((touchY < signalY && obsY > touchY) || 
                    (touchY > signalY && obsY < touchY)) {
                continue; // there is obs
            }
            else {
                foundM1 = true;
                touchY = (touchY < signalY)? touchY - M3_width : touchY + M3_width;
                break;
            }
        }
        else {
            foundM1 = true;
            touchY = (touchY < signalY)? touchY - M3_width : touchY + M3_width;
            break;
        }
    }
    
    Point2D<int> point;
    point.x = touchX;
    point.y = signalY;
    if(foundM1 && defDB.pwgnd.unusablePoints.count(point) != 0)
        foundM1 = false;

    if(foundM1) {
        auto& usedPoints = (signal == "POWER")? defDB.pwgnd.powerM2Points : defDB.pwgnd.gndM2Points;
        usedPoints.insert(point);
        
        parser::Wire tmpWire;
        tmpWire.coorX[0] = touchX;
        tmpWire.coorY[0] = touchY;
        tmpWire.coorX[1] = touchX;
        tmpWire.coorY[1] = signalY;
        tmpWire.numPathPoint = 2;
        tmpWire.layerName = lefDB.layers[0].name;
        tmpWire.width = M3_width;
        Wires.push_back(tmpWire); //M1 wire

        int viaID = lefDB.topLayerIdx2ViaIdx[2];
        for(int i = lefDB.topLayerIdx2ViaIdx[2]; i < lefDB.topLayerIdx2ViaIdx[4]; i++) {
            auto via = lefDB.vias[i];
            if(via.layerRects[0].rects[0].lowerLeft.x < via.layerRects[0].rects[0].lowerLeft.y) {// bot layer overhang is horizontal
                viaID = i;
                break;
            }
        }

        tmpWire.coorY[0] = signalY;
        tmpWire.numPathPoint = 1;
        tmpWire.layerName = lefDB.layers[2].name;
        tmpWire.width = 0;
        tmpWire.viaName = lefDB.vias[viaID].name;
        Wires.push_back(tmpWire); //M1-M2 via
        


#if HORIZONTAL_M4
        int M2_minLength = lefDB.layers[2].minLength * defDB.dbuPerMicro;
        int M3_minLength = lefDB.layers[4].minLength * defDB.dbuPerMicro;
        
        int M2_width = lefDB.layers[2].width * defDB.dbuPerMicro;
        int M3_width = lefDB.layers[4].width * defDB.dbuPerMicro;
        
        tmpWire.layerName = lefDB.layers[4].name;
        viaID = lefDB.topLayerIdx2ViaIdx[4];
        tmpWire.viaName = lefDB.vias[viaID].name;
        Wires.push_back(tmpWire); //M2-M3 via

        tmpWire.layerName = lefDB.layers[6].name;
        viaID = lefDB.topLayerIdx2ViaIdx[6];
        tmpWire.viaName = lefDB.vias[viaID].name;
        Wires.push_back(tmpWire); //M3-M4 via
        
        tmpWire.coorX[0] = touchX; 
        tmpWire.coorX[1] = touchX;

        if(signalY - M3_minLength / 2 < defDB.dieArea.lowerLeft.y) {
            tmpWire.coorY[0] = defDB.dieArea.lowerLeft.y;
            tmpWire.coorY[1] = defDB.dieArea.lowerLeft.y + M3_minLength;
        }
        else if(signalY + M3_minLength / 2 > defDB.dieArea.upperRight.y) {
            tmpWire.coorY[0] = defDB.dieArea.upperRight.y;
            tmpWire.coorY[1] = defDB.dieArea.upperRight.y - M3_minLength;
        }
        else {
            tmpWire.coorY[0] = signalY;
            tmpWire.coorY[1] = (touchY < signalY)? signalY - M3_minLength : signalY + M3_minLength;
        }
        tmpWire.width = M3_width;
        tmpWire.numPathPoint = 2;
        tmpWire.layerName = lefDB.layers[4].name;
        Wires.push_back(tmpWire); //M3 min area

#endif
    }
    return foundM1;
     
    /*bool foundM1M3 = M1M3DetailedRouteSNet(component, trackClosestPinPoint, 
            signal, signalY, Wires, powerPoint);
    
    if(foundM1M3)
        return foundM1M3;
    else {
        map<int, int> trackFarthestPinPoint;
        
        findFarthestTouchPoints(pinRect, trackFarthestPinPoint, track, width / 2, signalY);
        
        bool foundM1M3AllPin = M1M3DetailedRouteSNet(component, trackFarthestPinPoint, 
            signal, signalY, Wires, powerPoint);
        return foundM1M3AllPin;
    }*/


}


void detailedRouteSNetComp(parser::Component& component) {
    int powerY, gndY;
    if(component.macro.pins.size() != 0)
        findRowSNet(component.name, component.macro.pins[0].name, component.macro.pins[0].layerRects[0].rects[0], powerY, gndY);
    else
        return;

    parser::Wire tmpWire;
    Point2D<int> powerPoint(-1, -1);
    bool M1power = false, M1ground = false, M2power = false, M2ground = false, M1M3power = false, M1M3ground = false; 
    
    M1power = M1DetailedRouteSNet(component, "POWER", powerY, defDB.pwgnd.powerWires, powerPoint);
    if(!M1power) {
        M2power = M2DetailedRouteSNet(component, "POWER", powerY, defDB.pwgnd.powerWires, powerPoint);
        if(!M2power) {
            M1M3power = M1M3DetailedRouteSNet(component, "POWER", powerY, defDB.pwgnd.powerWires, powerPoint); 
            if(!M1M3power)
                cout << "warning: UNABLE TO PATTERN ROUTE POWER: " << component.name << endl;
        }
    }
    M1ground = M1DetailedRouteSNet(component, "GROUND", gndY, defDB.pwgnd.gndWires, powerPoint);
    if(!M1ground) {
        M2ground = M2DetailedRouteSNet(component, "GROUND", gndY, defDB.pwgnd.gndWires, powerPoint);
        if(!M2ground) {
            M1M3ground = M1M3DetailedRouteSNet(component, "GROUND", gndY, defDB.pwgnd.gndWires, powerPoint); 
            if(!M1M3ground)
                cout << "warning: UNABLE TO PATTERN ROUTE GROUND: " << component.name << endl;
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
    
    //if(component.name == "p_atv__56_57_6_acx1")
    //    cout << "component: " << component.name << " " << M1power << M2power << M1M3power << " " << M1ground << M2ground << M1M3ground << endl;
    
}

void markUnusablePointComp(parser::Component& component) {
    int powerY, gndY;
    if(component.macro.pins.size() != 0)
        findRowSNet(component.name, component.macro.pins[0].name, component.macro.pins[0].layerRects[0].rects[0], powerY, gndY);
    else
        return;

    vector<parser::Rect2D<float>> pinRect;
    
    for(auto pin : component.macro.pins) {
        for(auto layerRect : pin.layerRects) {
            int layerIdx = lefDB.layer2idx[layerRect.layerName];
            if(layerIdx == 0 || layerIdx == 2) { //M1 or M2 
                for(auto rect : layerRect.rects) {
                    pinRect.push_back(rect);
                }
            }
        }
    }
    
    for(auto layerRect : component.macro.obs.layerRects) {
        int layerIdx = lefDB.layer2idx[layerRect.layerName];
        if(layerIdx == 0 || layerIdx == 2) { //M1 or M2
            for(auto rect : layerRect.rects) {
                pinRect.push_back(rect);
            }
        }
    }

    int viaIdx = lefDB.topLayerIdx2ViaIdx[2];
    auto via = lefDB.vias[viaIdx];
    auto viaRect = via.layerRects[0].rects[0];
    float h = (viaRect.upperRight.x - viaRect.lowerLeft.x) * defDB.dbuPerMicro;
    float v = (viaRect.upperRight.y - viaRect.lowerLeft.y) * defDB.dbuPerMicro;
    float viaWidth = (h > v)? v : h;
    float viaLength = (h > v)? h : v;
    int M1_spacing = (lefDB.layers[0].pitchy - lefDB.layers[0].width) * defDB.dbuPerMicro;

    parser::Track track = defDB.tracks[defDB.layeridx2trackidx[4]]; //M3
    
    int M2_minLength = lefDB.layers[2].minLength * defDB.dbuPerMicro;
    int M2_spacing = (lefDB.layers[2].pitchy - lefDB.layers[2].width) * defDB.dbuPerMicro;

    for(auto rect : pinRect) {
        if(fabs(rect.lowerLeft.y - powerY) < M1_spacing + viaWidth / 2 ||  
            fabs(rect.upperRight.y - powerY) < M1_spacing + viaWidth / 2 ) {
#ifdef HORIZONTAL_M4
           int start = (rect.lowerLeft.x - M2_spacing - viaLength - M2_minLength / 2 - track.start) / track.step + 1;
           int end = (rect.upperRight.x + M2_spacing + viaLength + M2_minLength / 2 - track.start) / track.step;
#else
           int start = (rect.lowerLeft.x - M2_spacing - viaLength - track.start) / track.step + 1;
           int end = (rect.upperRight.x + M2_spacing + viaLength - track.start) / track.step;
#endif
           for(int i = start; i <= end; i++) {
               int xpos = track.start + track.step * i; 
               Point2D<int> point;
               point.x = xpos; 
               point.y = powerY;
               defDB.pwgnd.unusablePoints.insert(point);
           }
        }
        if( fabs(rect.lowerLeft.y - gndY) < M1_spacing + viaWidth / 2 ||
            fabs(rect.upperRight.y - gndY) < M1_spacing + viaWidth / 2 ) {
#ifdef HORIZONTAL_M4
           int start = (rect.lowerLeft.x - M2_spacing - viaLength - M2_minLength / 2 - track.start) / track.step + 1;
           int end = (rect.upperRight.x + M2_spacing + viaLength + M2_minLength / 2 - track.start) / track.step;
#else
           int start = (rect.lowerLeft.x - M2_spacing - viaLength - track.start) / track.step + 1;
           int end = (rect.upperRight.x + M2_spacing + viaLength - track.start) / track.step;
#endif
           for(int i = start; i <= end; i++) {
               int xpos = track.start + track.step * i; 
               Point2D<int> point;
               point.x = xpos; 
               point.y = gndY;
               defDB.pwgnd.unusablePoints.insert(point);
           }
        }
    }
}

void M2MetalFill(string signal) {
    
    parser::Track track = defDB.tracks[defDB.layeridx2trackidx[4]]; //M3
    auto& Wires = (signal == "POWER")? defDB.pwgnd.powerWires : defDB.pwgnd.gndWires;
    
    int M2_minLength = lefDB.layers[2].minLength * defDB.dbuPerMicro;
    int M2_width = lefDB.layers[2].width * defDB.dbuPerMicro;
        
    auto& usedPoints = (signal == "POWER")? defDB.pwgnd.powerM2Points : defDB.pwgnd.gndM2Points;
    for(auto point : usedPoints) {
        Point2D<int> leftPoint, twoLeftPoint, threeLeftPoint;
        leftPoint.x = point.x - track.step;
        leftPoint.y = point.y;
        
        twoLeftPoint.x = point.x - 2 * track.step;
        twoLeftPoint.y = point.y;
        
        threeLeftPoint.x = point.x - 3 * track.step;
        threeLeftPoint.y = point.y;
        if(usedPoints.count(leftPoint) != 0) {
            parser::Wire tmpWire;
            tmpWire.coorX[0] = point.x - track.step / 2 - M2_minLength / 2;
            tmpWire.coorY[0] = point.y;
            tmpWire.coorX[1] = point.x - track.step / 2 + M2_minLength / 2;
            tmpWire.coorY[1] = point.y;
            tmpWire.numPathPoint = 2;
            tmpWire.layerName = lefDB.layers[2].name;
            tmpWire.width = M2_width;
            Wires.push_back(tmpWire); //M2 min area
        }

        else if(usedPoints.count(twoLeftPoint) != 0 || usedPoints.count(threeLeftPoint) != 0) {
            parser::Wire tmpWire;
            if(usedPoints.count(twoLeftPoint) != 0)
                tmpWire.coorX[0] = point.x - 2 * track.step;
            else
                tmpWire.coorX[0] = point.x - 3 * track.step;

            tmpWire.coorY[0] = point.y;
            tmpWire.coorX[1] = point.x;
            tmpWire.coorY[1] = point.y;
            tmpWire.numPathPoint = 2;
            tmpWire.layerName = lefDB.layers[2].name;
            tmpWire.width = M2_width;
            Wires.push_back(tmpWire); //M2 min area
        }
        else {
            parser::Wire tmpWire;
            tmpWire.coorX[0] = point.x - M2_minLength / 2;
            tmpWire.coorY[0] = point.y;
            tmpWire.coorX[1] = point.x + M2_minLength / 2;
            tmpWire.coorY[1] = point.y;
            tmpWire.numPathPoint = 2;
            tmpWire.layerName = lefDB.layers[2].name;
            tmpWire.width = M2_width;
            Wires.push_back(tmpWire); //M2 min area
        }
    }
}

void detailedRouteSNet() {
    for(auto& component : defDB.components) {
        detailedRouteSNetComp(component);
    }

    cout << "Power routing completes/fails: " << POWER_FOUND << " / " << POWER_UNFOUND << endl;
    cout << "Ground routing completes/fails: " << GROUND_FOUND << " / " << GROUND_UNFOUND << endl;
    
    M2MetalFill("POWER");
    M2MetalFill("GROUND");
}

void markUnusablePoint() {
    for(auto& component : defDB.components) {
        markUnusablePointComp(component);
    }
    //cout << " total unusable points: " << defDB.pwgnd.unusablePoints.size() << endl; 

    //for(auto point : defDB.pwgnd.unusablePoints)
    //    cout << point << endl;
}

void routeSNet() {

    double coorX[3], coorY[3];
  
    int width = defDB.pwgnd.width;
    parser::PWGND& pwgnd = defDB.pwgnd;
    parser::Rect2D<int> dieArea = defDB.dieArea;
    string vLayerName = pwgnd.vLayerName;
    string hLayerName = pwgnd.hLayerName;

    int vLayerID = lefDB.layer2idx[vLayerName];
    int vLayerWidth = lefDB.layers[vLayerID].width * defDB.dbuPerMicro;
    
   
    routeHighLayerSNet("POWER"); 
    
    routeLowLayerMesh("POWER");
    cout << "#vddpin:" << " " << pwgnd.VDDpins[0].size() << endl; 
   
    cout << "#gndpin:" << " " << pwgnd.GNDpins[0].size() << endl; 
    routeHighLayerSNet("GROUND"); 
    
    routeLowLayerMesh("GROUND");

    markUnusablePoint();
    detailedRouteSNet();
    cout << endl;
    if(POWER_UNFOUND != 0 || GROUND_UNFOUND != 0)
        cout << "ATTENTION: SOME CELLS FAIL TO ROUTE!" << endl;
    cout << "power/ground routing done!" << endl;
}

#endif
