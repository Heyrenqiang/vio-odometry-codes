#include "datareading.h"

DataReading::DataReading(const string imgdir,const string timestampdir)
    :imgdir(imgdir),timestampdir(timestampdir){

}
void DataReading::readImage(vector<string> &strimage,vector<double> &timestamps){
    ifstream timefile;
    timefile.open(timestampdir.c_str());
    strimage.reserve(5000);
    timestamps.reserve(5000);
    while(!timefile.eof()){
        string s;
        getline(timefile,s);
        if(!s.empty()){
            stringstream ss;
            ss<<s;
            strimage.push_back(imgdir+"/"+ss.str());
            s.erase(s.end()-4,s.end());
            ss<<s;
            double t;
            ss>>t;
            timestamps.push_back(t/1e9);
        }
    }
}
void DataReading::readCameraParameters(){

}

void DataReading::showTrajectory(vector<string> filepaths, int startframe, int endframe)
{
    vector<VecSE3> poses_s;

    for(size_t k=0;k<filepaths.size();k++){
        VecSE3 poses;
        ifstream fin(filepaths[k]);
        string linestr;
        while(getline(fin,linestr)){
            stringstream ss(linestr);
            double data[8];
            int j=0;
            while(1){
                if(ss.fail()){
                    break;
                }
                ss>>data[j];
                j++;
            }
            if(k==2){
                poses.push_back(Sophus::SE3<double,0>(
                                    Quaterniond(data[1],data[2],data[3],data[4]),
                        100*Vector3d(data[5]-4.68,data[6]+1.78,data[7]-0.8)));
            }else{
                poses.push_back(Sophus::SE3<double,0>(
                                    Quaterniond(data[1],data[2],data[3],data[4]),
                        10*Vector3d(data[5],data[6],data[7])));
            }
            //            poses.push_back(Sophus::SE3<double,0>(
            //                                Quaterniond(data[4],data[5],data[6],data[7]),
            //                    Vector3d(data[1],data[2],data[3])));
            if(!fin.good())
                break;
        }
        fin.close();

        poses_s.push_back(poses);
    }

    Draw(poses_s);

}
void DataReading::Draw(const vector<VecSE3> &poses_s){

    pangolin::CreateWindowAndBind("Trajectory Viewer",1024,768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_CONSTANT_ALPHA);
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
                pangolin::ModelViewLookAt(0,-0.1,-1.8,0,0,0,0.0,-1,0));
    pangolin::View &d_cam=pangolin::CreateDisplay().SetBounds(0.0,1,pangolin::Attach::Pix(175),1,-1024.0/768.0)
            .SetHandler(new pangolin::Handler3D(s_cam));
    while(pangolin::ShouldQuit()==false){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);//消除颜色缓冲
        d_cam.Activate(s_cam);

        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        pangolin::glDrawAxis(3);

        // draw poses
        for(size_t k=0;k<poses_s.size();k++){
            if(poses_s[k].empty()){
                continue;
            }
            for (auto &Tcw: poses_s[k])//从poses中取位姿
            {
                glPushMatrix();
                Sophus::Matrix4f m = Tcw.matrix().cast<float>();
                glMultMatrixf(static_cast<GLfloat *>(m.data()));

                const float w = 0.25;
                const float h = w*static_cast<float>(0.75);
                const float z = w*static_cast<float>(0.6);
                glColor3f(1, 0, 0);
                glLineWidth(2);
                glBegin(GL_LINES);

                glVertex3f(0, 0, 0);
                glVertex3f(w,h,z);
                glVertex3f(0, 0, 0);
                glVertex3f(w,-h,z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w,-h,z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);
                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);
                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);
                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);

                glEnd();
                glPopMatrix();
            }
            glLineWidth(2);
            for (size_t i = 0; i < poses_s[k].size() - 1; i++)
            {
                glColor3f(1 - static_cast<float>(i) / poses_s[k].size(), 0.0f, static_cast<float>(i) / poses_s[k].size());
                glBegin(GL_LINES);
                auto p1 = poses_s[k][i], p2 = poses_s[k][i + 1];
                glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
                glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
                //                glBegin(GL_POINT);
                //                glVertex3d(i*10,i*10,i*10);
            }
            glEnd();

        }
        pangolin::FinishFrame();
    }
    pangolin::DestroyWindow("Trajectory Viewer");
    return;
}

void DataReading::showTrajectoryandpoints(vector<Quaterniond> poses_R,vector<Vector3d> poses_t,vector<Vector3d> points){
    VecSE3 pose_s;
    for(size_t i=0;i<poses_R.size();i++){
        pose_s.push_back(Sophus::SE3<double,0>(poses_R[i],10*poses_t[i]));
    }
    pangolin::CreateWindowAndBind("Trajectory and points Viewer",1024,768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_CONSTANT_ALPHA);
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
                pangolin::ModelViewLookAt(0,-0.1,-1.8,0,0,0,0.0,-1,0));
    pangolin::View &d_cam=pangolin::CreateDisplay().SetBounds(0.0,1,pangolin::Attach::Pix(175),1,-1024.0/768.0)
            .SetHandler(new pangolin::Handler3D(s_cam));
    while(pangolin::ShouldQuit()==false){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);//消除颜色缓冲
        d_cam.Activate(s_cam);

        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        pangolin::glDrawAxis(3);

        // draw poses
        if(pose_s.empty()){
            return;
        }
        for (auto &Tcw: pose_s)//从poses中取位姿
        {
            glPushMatrix();
            Sophus::Matrix4f m = Tcw.matrix().cast<float>();
            glMultMatrixf(static_cast<GLfloat *>(m.data()));

            const float w = 0.25;
            const float h = w*static_cast<float>(0.75);
            const float z = w*static_cast<float>(0.6);
            glColor3f(1, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(w,h,z);
            glVertex3f(0, 0, 0);
            glVertex3f(w,-h,z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w,-h,z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);
            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);
            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);
            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);

            glEnd();
            glPopMatrix();
        }
        glLineWidth(2);
        for (size_t i = 0; i < pose_s.size() - 1; i++)
        {
            glColor3f(1 - static_cast<float>(i) / pose_s.size(), 0.0f, static_cast<float>(i) / pose_s.size());
            glBegin(GL_LINES);
            auto p1 = pose_s[i], p2 = pose_s[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
        }
        glEnd();
        if(IFSHOWPOINTS){
            glPointSize(3.0f);
            glBegin(GL_POINTS);
            for(size_t k=0;k<points.size();k++){
                glVertex3d(10*points[k].x(),10*points[k].y(),10*points[k].z());
            }
            //点的创建
            glEnd();
        }
        //cv::waitKey(12220);
        pangolin::FinishFrame();
    }
    pangolin::DestroyWindow("Trajectory and points Viewer");
    return;

}
