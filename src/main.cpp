
#include <iostream>
#include "datareading.h"
#include "frame.h"
#include "feature_track.h"
#include "vioodometry.h"
#include "opencv2/core/core.hpp"
#include "parameters.h"
#include "cameramodel.h"
using namespace std;
using namespace cv;

int main()
{

    //read data from local file
    DataReading datareading("../sfm-project/data/cam0/data","../sfm-project/data/cam0/times.txt");
    vector<string> strimgs;
    vector<double> strtimestamps;
    datareading.readImage(strimgs,strtimestamps);
    vector<Vector3d> accs;
    vector<Vector3d> omigas;
    vector<double> imutimestamps;
    DataReading::readImudata("../sfm-project/data/imu0/data.csv",accs,omigas,imutimestamps);
    readParameters("../sfm-project/config/config.yaml");
    //create vio system
    VioOdometry vioOdometry;
    vioOdometry.Initialparameters();

    for(size_t i=0;i<strimgs.size();i++){
        vector<Vector3d> slic_accs;
        vector<Vector3d> slic_omigas;;
        vector<double> slic_imustamps;
        if(i>0){
            for(size_t k=(i-1)*10;k<=(i-1)*10+10;k++){
                slic_accs.push_back(accs[k]);
                slic_omigas.push_back(omigas[k]);
                slic_imustamps.push_back(imutimestamps[k]);
            }
        }
        vioOdometry.ImuPreintergration(slic_accs,slic_omigas,slic_imustamps,int(i));
        Mat img=imread(strimgs[i],IMREAD_GRAYSCALE);

        if(EQUALIZE){
            Ptr<CLAHE> clahe=createCLAHE(3.0,Size(8,8));
            clahe->apply(img,img);
        }
        vioOdometry.Track(img,strtimestamps[i],int(i));

    }
    cout<<vioOdometry.featurePoints.size()<<endl;
    cout << "Hello World!" << endl;
    return 0;
}
