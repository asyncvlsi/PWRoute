#ifndef PWGND_ROUTE_H
#define PWGND_ROUTE_H

#include "header.h"
#include "defDataBase.h"
#include "global.h"
extern parser::lefDataBase lefDB;
extern parser::defDataBase defDB;


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
    vector<int> yPW;
    vector<int> yGND;
    while(!infile.eof()) {
        infile >> tmp1 >> tmp2;
        if(tmp1 == "STRIP") {
            infile >> lx >> ux >> tmp3;
            defDB.pwgnd.startSNet.push_back(tmp3);
            if(tmp3 == "GND")   
                cnt_y = 0;
            else
                cnt_y = 1;


            if(cnt_x % 2 == 0) {
                defDB.pwgnd.gndxMesh.push_back(lx);//first gnd       
            }
            else {
                defDB.pwgnd.powerxMesh.push_back(lx);
            }

            yPW.clear();
            yGND.clear();
        }
        else if(tmp1 == "END") {
            if(cnt_y % 2 == 0) {
                yGND.push_back(uy);       
            }
            else {
                yPW.push_back(uy);
            }
            defDB.pwgnd.poweryMesh.push_back(yPW);
            defDB.pwgnd.gndyMesh.push_back(yGND);
            cnt_x++;
            //cout << " cnt_x: " << cnt_x << endl;
            tmp1.clear();
        }
        else {
            ly = atoi(tmp1.c_str());
            uy = atoi(tmp2.c_str());
            if(cnt_y % 2 == 0) {
                yGND.push_back(ly);       
            }
            else {
                yPW.push_back(ly);
            }
            cnt_y++;
        }
    }
    if(cnt_x % 2 == 0) {
        defDB.pwgnd.gndxMesh.push_back(ux);       
    }
    else {
        defDB.pwgnd.powerxMesh.push_back(ux);
    }


    cout << "gndx: ";
    for(int i = 0; i < defDB.pwgnd.gndxMesh.size(); i++) {
        cout << defDB.pwgnd.gndxMesh[i] << " ";
    }
    cout << endl;

    cout << "powerx: ";
    for(int i = 0; i < defDB.pwgnd.powerxMesh.size(); i++) {
        cout << defDB.pwgnd.powerxMesh[i] << " ";
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


}


void clusterPWGNDMesh(string clusterFileName) {
    cout << "cluster pwgnd mesh" << endl;
    readCluster(clusterFileName);

    defDB.pwgnd.layerWidth = lefDB.layers.at(4).width * defDB.dbuPerMicro;

	//defDB.pwgnd.pitch = 10 * defDB.pwgnd.layerWidth;
	defDB.pwgnd.width = 2 * defDB.pwgnd.layerWidth;

    if(lefDB.layers.at(4).direction == "HORIZONTAL") {
	    defDB.pwgnd.verticalLayerName = lefDB.layers.at(2).name;
	    defDB.pwgnd.horizontalLayerName = lefDB.layers.at(4).name;
    }
    else {
        defDB.pwgnd.verticalLayerName = lefDB.layers.at(4).name;
	    defDB.pwgnd.horizontalLayerName = lefDB.layers.at(2).name;
    }

    cout << "vertical: " << defDB.pwgnd.verticalLayerName << endl;
    cout << "horizontal: " << defDB.pwgnd.horizontalLayerName << endl;
	
}


parser::Point2D<int> findClosestVDD(parser::Point2D<int> point, string netName) {
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
}


#endif
