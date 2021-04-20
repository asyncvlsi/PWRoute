
#include "pwroute.h"
#include <phydb/phydb.h>

using namespace std;
using namespace phydb;

int main(int argc, char** argv)
{
    string defFileName, lefFileName, outFileName, cellFileName;
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
            cellFileName = string(argv[i+1]);
        }
    }
    if(defFileName.size() == 0 || defFileName.size() == 0 || outFileName.size() == 0 || cellFileName.size() == 0)
    {
        cout << "usage: ./PWRoute -lef [LefFile] -def [DefFile] -cluster [clusterFile] -output [Output file]" << endl;
        //exit(1);
    }

    phydb::PhyDB db;

    lefFileName = "final.lef";
    db.ReadLef(lefFileName);
    cout << "lef reading done" << endl;

    defFileName = "final.def";
    db.ReadDef(defFileName);
    cout << "def reading done" << endl;

    cellFileName = "processor.cell";
    db.ReadCell(cellFileName);

    string clusterFileName = "circuit_router.cluster";
    db.ReadCluster(clusterFileName);

    pwroute::PWRoute router(&db);
    router.RunPWRoute();
    router.ExportToPhyDB();

    return 1;
}
