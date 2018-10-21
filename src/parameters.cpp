#include "parameters.h"
std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
double MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;
int NUMOFFRAMESTOINIT;
int STARTFRAME;
bool SHOWIMG;
bool IFSHOWPOINTS;

void readParameters(string configdir){
    FileStorage fsettings(configdir,FileStorage::READ);
    if(!fsettings.isOpened()){
        cerr<<"error open config file"<<endl;
    }
    MAX_CNT = fsettings["max_cnt"];
    MIN_DIST = fsettings["min_dist"];
    ROW = fsettings["image_height"];
    COL = fsettings["image_width"];
    F_THRESHOLD = fsettings["F_threshold"];
    NUMOFFRAMESTOINIT=fsettings["numofframestoinit"];
    EQUALIZE=fsettings["equalize"];
    STARTFRAME=fsettings["startframe"];
    cout<<STARTFRAME<<endl;
    fsettings["showimg"]>>SHOWIMG;
    cout<<SHOWIMG<<endl;
    fsettings["ifshowpoints"]>>IFSHOWPOINTS;

    Cameramodel::imagewidth=fsettings["image_width"];
    Cameramodel::imageheight=fsettings["image_height"];
    FileNode n=fsettings["distortion_parameters"];
    Cameramodel::k1 = static_cast<float>(n["k1"]);
    Cameramodel::k2 = static_cast<float>(n["k2"]);
    Cameramodel::p1 = static_cast<float>(n["p1"]);
    Cameramodel::p2 = static_cast<float>(n["p2"]);

    n = fsettings["projection_parameters"];
    Cameramodel::fx = static_cast<float>(n["fx"]);
    Cameramodel::fy = static_cast<float>(n["fy"]);
    Cameramodel::cx = static_cast<float>(n["cx"]);
    Cameramodel::cy = static_cast<float>(n["cy"]);


}
