
#include "pwroute.h"
#include <phydb/phydb.h>

using namespace std;
using namespace phydb;

int main(int argc, char** argv)
{
    string defFileName, lefFileName, outFileName, cellFileName, clusterFileName;
    int verb = 0;
    int reinforcement = 1;
    for(int i = 1; i < argc; i++)
    {
        string tmp(argv[i]);
        if(tmp == "-lef")
        {   
            lefFileName = string(argv[i+1]);
        }
        else if(tmp == "-def")
        {
            defFileName = string(argv[i+1]);
        }
        else if(tmp == "-output")
        {   
            outFileName = string(argv[i+1]);
        }
        else if(tmp == "-cluster") {
            clusterFileName = string(argv[i+1]);
        }
	else if(tmp == "-set_reinforcement") {
	    reinforcement = atoi(argv[i + 1]);
	}
	else if(tmp == "-verbose") {
            verb = atoi(argv[i + 1]);
        }
    }
    if(lefFileName.size() == 0 || defFileName.size() == 0 || outFileName.size() == 0 || clusterFileName.size() == 0)
    {
        cout << "usage: ./PWRoute -lef [LefFile] -def [DefFile] -cluster [clusterFile] -output [Output file]" << endl;
        exit(1);
    }

    phydb::PhyDB db;

    //lefFileName = "final.lef";
    db.ReadLef(lefFileName);
    cout << "lef reading done" << endl;

    //defFileName = "final.def";
    db.ReadDef(defFileName);
    cout << "def reading done" << endl;

    //string clusterFileName = "circuit_router.cluster";
    db.ReadCluster(clusterFileName);

    pwroute::PWRoute router(&db, verb);
    router.SetReinforcement(reinforcement);
    router.RunPWRoute();
    router.ExportToPhyDB();

    db.WriteDef(outFileName);

    return 0;
}
