
#include <iostream>
#include "datareading.h"
#include "frame.h"
#include "feature_track.h"
#include "vioodometry.h"
#include "opencv2/core/core.hpp"
#include "parameters.h"
#include "cameramodel.h"
#include"parameters.h"
using namespace std;
using namespace cv;

int main()
{

    //read data from local file
    DataReading datareading("../sfm-project/data/cam0_1/data","../sfm-project/data/cam0_1/times.txt");
    vector<string> strimgs;
    vector<double> strtimestamps;
    datareading.readImage(strimgs,strtimestamps);
    vector<Vector3d> accs;
    vector<Vector3d> omigas;
    vector<double> imutimestamps;
    DataReading::readImudata("../sfm-project/data/imu0_1/data.csv",accs,omigas,imutimestamps);
    readParameters("../sfm-project/config/config.yaml");
    //create vio system
    VioOdometry vioOdometry;
    vioOdometry.Initialparameters();
    ulong k=0;
    vector<Vector3d> slic_accs;
    vector<Vector3d> slic_omigas;;
    vector<double> slic_imustamps;
    for(size_t i=0;i<strimgs.size();i++){

        //add coorespond data to frame
        if(i>0){
            while(1){
                if(imutimestamps[k]>=strtimestamps[i-1]&&imutimestamps[k]<=strtimestamps[i]){
                    slic_accs.push_back(accs[k]);
                    slic_omigas.push_back(omigas[k]);
                    slic_imustamps.push_back(imutimestamps[k]);
                    k++;
                }else if(imutimestamps[k]<strtimestamps[i-1]){
                    k++;
                }else if(imutimestamps[k]>strtimestamps[i]){
                    if(k==0)
                        break;
                    k--;
                    break;
                }
            }
        }
        Mat img=imread(strimgs[i],IMREAD_GRAYSCALE);

        if(EQUALIZE){
            Ptr<CLAHE> clahe=createCLAHE(3.0,Size(8,8));
            clahe->apply(img,img);
        }
        vioOdometry.Track(img,strtimestamps[i],slic_accs,slic_omigas,slic_imustamps,int(i));
        slic_accs.clear();
        slic_imustamps.clear();
        slic_omigas.clear();

    }
    cout<<vioOdometry.featurePoints.size()<<endl;
    cout << "Hello World!" << endl;
    return 0;
}
