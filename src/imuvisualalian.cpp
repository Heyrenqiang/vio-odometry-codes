#include "imuvisualalian.h"

bool VioOdometry::VisualImuAlian()
{
    int k=0;
    for(int i=0;i<=curframeid;i++){
        if(frames[ulong(i)].posesolved){
            k=i;
            break;
        }
    }
    Matrix3d R=Matrix3d::Zero();
    Vector3d A=Vector3d::Zero();
    Vector3d delta_bg;
    for(int i=curframeid-NUMOFFRAMESTOINIT+1;i<curframeid;i++){
        int j=i+1;
        Quaterniond q_i_j_visual(frames[ulong(i)].M3d_pose_R.inverse()*frames[ulong(j)].M3d_pose_R);
        Quaterniond q_i_j_imu=frames[ulong(j)].pre_integration_pose_gama;
        Vector3d temp_A=2*(q_i_j_visual.inverse()*q_i_j_imu).vec();
        Matrix3d temp_R=frames[ulong(j)].jacobian.block<3,3>(6,12);
        A=A+temp_R.transpose()*temp_A;
        R=R+temp_R.transpose()*temp_R;
    }
    Vector3d delta_gyr_bias=R.ldlt().solve(A);
    for(int i=0;i<=curframeid;i++){
        frames[ulong(i)].delta_gyr_bias=delta_gyr_bias;
        frames[ulong(i)].gyr_bias=gyr_bias+delta_gyr_bias;
    }
    gyr_bias=gyr_bias+delta_gyr_bias;

    Reintegration();
    return true;
}

void VioOdometry::Showvisualimuerr()
{
    for(size_t i=0;i<ulong(curframeid);i++){
        if(frames[i].posesolved&&frames[i+1].posesolved){
            Matrix3d visualr=frames[i].M3d_pose_R.inverse()*frames[i+1].M3d_pose_R;
            Quaterniond visualrq(visualr);
            cout<<"visual:"<<visualrq.w()<<visualrq.x()<<visualrq.y()<<visualrq.z()<<endl;
            //cout<<"visual:"<<visualr<<endl;
            cout<<"imu:"<<frames[i+1].pre_integration_pose_gama.w()
               <<frames[i+1].pre_integration_pose_gama.x()
              <<frames[i+1].pre_integration_pose_gama.y()
             <<frames[i+1].pre_integration_pose_gama.z()<<endl;
        }
    }
}
