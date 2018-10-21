
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
    readParameters("../sfm-project/config/config.yaml");
    //create vio system
    VioOdometry vioOdometry;

    for(size_t i=0;i<strimgs.size();i++){
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
