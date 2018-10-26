#include"imuintergration.h"
bool hellsd(int a)
{
    cout<<a<<endl;
    return true;
}







void VioOdometry::ImuPreintegration(int id)
{
    if(id==0||!frames[ulong(id)].hasenoughimudata){
        return;
    }
    else{
        //preintergration of between two frames

        Quaterniond q[11];
        Vector3d v[11];
        Vector3d s[11];
        q[0].w()=1;
        q[0].x()=0;
        q[0].y()=0;
        q[0].z()=0;
        v->x()=0;
        v->y()=0;
        v->z()=0;
        s->x()=0;
        s->y()=0;
        s->z()=0;

        double deltat=0.005,deltatsquare=deltat*deltat;

        MatrixXd jacobian=MatrixXd::Identity(15,15);
        MatrixXd covariance=MatrixXd::Zero(15,15);

        for(size_t k=1;k<frames[ulong(id)].slic_accs.size();k++){
            Matrix3d R_q(q[k-1]);
            Vector3d un_omiga=frames[ulong(id)].slic_omigas[k-1]-gyr_bias;
            Quaterniond deltaq(1,0.5*un_omiga.x()*deltat,0.5*un_omiga.y()*deltat,0.5*un_omiga.z()*deltat);
            q[k]=q[k-1]*deltaq;

            v[k]=v[k-1]+R_q*frames[ulong(id)].slic_accs[k-1]*deltat;
            s[k]=s[k-1]+v[k-1]*deltat+0.5*R_q*frames[ulong(id)].slic_accs[k-1]*deltatsquare;

            Vector3d b_a_t(0,0,0);
            Vector3d acc_min_b_a_t=frames[ulong(id)].slic_accs[k-1]-b_a_t;
            Matrix3d acc_min_b_a_t_skew;
            acc_min_b_a_t_skew<<0,-acc_min_b_a_t(2),acc_min_b_a_t(1),
                    acc_min_b_a_t(2),0,-acc_min_b_a_t(0),
                    -acc_min_b_a_t(1),acc_min_b_a_t(0),0;
            Vector3d b_w_t(0,0,0);
            Vector3d omiga_min_b_w_t=frames[ulong(id)].slic_omigas[k-1]-b_w_t;
            Matrix3d omiga_min_b_w_t_skew;
            omiga_min_b_w_t_skew<<0,-omiga_min_b_w_t(2),omiga_min_b_w_t(1),
                    omiga_min_b_w_t(2),0,-omiga_min_b_w_t(0),
                    -omiga_min_b_w_t(1),omiga_min_b_w_t(0),0;

            MatrixXd F=MatrixXd::Zero(15,15);
            MatrixXd V=MatrixXd::Zero(15,12);
            F.block<3,3>(0,0)=Matrix3d::Identity();
            F.block<3,3>(3,3)=Matrix3d::Identity();
            F.block<3,3>(6,6)=Matrix3d::Identity()-omiga_min_b_w_t_skew*deltat;
            F.block<3,3>(9,9)=Matrix3d::Identity();
            F.block<3,3>(12,12)=Matrix3d::Identity();
            F.block<3,3>(0,3)=Matrix3d::Identity()*deltat;
            F.block<3,3>(3,6)=-R_q*acc_min_b_a_t_skew*deltat;
            F.block<3,3>(3,9)=-R_q*deltat;
            F.block<3,3>(6,12)=-Matrix3d::Identity()*deltat;

            V.block<3,3>(3,0)=-R_q*deltat;
            V.block<3,3>(6,3)=-Matrix3d::Identity()*deltat;
            V.block<3,3>(9,6)=Matrix3d::Identity()*deltat;
            V.block<3,3>(12,9)=Matrix3d::Identity()*deltat;

            jacobian=F*jacobian;

            covariance=F*covariance*F.transpose()+V*noise*V.transpose();

        }
        frames[ulong(id)].pre_integration_pose_alpha=s[10];
        frames[ulong(id)].pre_integration_pose_beta=v[10];
        frames[ulong(id)].pre_integration_pose_gama=q[10];
        frames[ulong(id)].jacobian=jacobian;
        frames[ulong(id)].covariance=covariance;

    }


}

void VioOdometry::ImuMidPreintegration(int id)
{
    if(id==0||!frames[ulong(id)].hasenoughimudata){
        return;
    }
    else{
        //preintergration of between two frames

        Quaterniond q[11];
        Vector3d v[11];
        Vector3d s[11];
        q[0].w()=1;
        q[0].x()=0;
        q[0].y()=0;
        q[0].z()=0;
        v->x()=0;
        v->y()=0;
        v->z()=0;
        s->x()=0;
        s->y()=0;
        s->z()=0;

        double dt=0.005;

        MatrixXd jacobian=MatrixXd::Identity(15,15);
        MatrixXd covariance=MatrixXd::Zero(15,15);

        for(size_t k=1;k<frames[ulong(id)].slic_accs.size();k++){
            //ROS_INFO("midpoint integration");
            Quaterniond delta_q=q[k-1];
            Vector3d _acc_0=frames[ulong(id)].slic_accs[k-1];
            Vector3d _gyr_0=frames[ulong(id)].slic_omigas[k-1];
            Vector3d _gyr_1=frames[ulong(id)].slic_omigas[k];
            Vector3d _acc_1=frames[ulong(id)].slic_accs[k];

            Vector3d un_acc_0 = delta_q * (_acc_0 - acc_bias);
            Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - gyr_bias;
            Quaterniond result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * dt / 2, un_gyr(1) * dt / 2, un_gyr(2) * dt / 2);
            q[k]=result_delta_q;

            Vector3d un_acc_1 = q[k] * (_acc_1 - acc_bias);
            Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
            Vector3d delta_p=s[k-1];
            Vector3d delta_v=v[k-1];
            Vector3d result_delta_p = delta_p + delta_v * dt + 0.5 * un_acc * dt * dt;
            s[k]=result_delta_p;

            Vector3d result_delta_v = delta_v + un_acc * dt;
            v[k]=result_delta_v;


            Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - gyr_bias;
            Vector3d a_0_x = _acc_0 - acc_bias;
            Vector3d a_1_x = _acc_1 - acc_bias;
            Matrix3d R_w_x, R_a_0_x, R_a_1_x;

            R_w_x<<0, -w_x(2), w_x(1),
                    w_x(2), 0, -w_x(0),
                    -w_x(1), w_x(0), 0;
            R_a_0_x<<0, -a_0_x(2), a_0_x(1),
                    a_0_x(2), 0, -a_0_x(0),
                    -a_0_x(1), a_0_x(0), 0;
            R_a_1_x<<0, -a_1_x(2), a_1_x(1),
                    a_1_x(2), 0, -a_1_x(0),
                    -a_1_x(1), a_1_x(0), 0;

            MatrixXd F = MatrixXd::Zero(15, 15);
            F.block<3, 3>(0, 0) = Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * dt * dt +
                    -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * dt) * dt * dt;
            F.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * dt;
            F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * dt * dt;
            F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * dt * dt * -dt;
            F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * dt;
            F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3,3) * dt;
            F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * dt +
                    -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * dt) * dt;
            F.block<3, 3>(6, 6) = Matrix3d::Identity();
            F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * dt;
            F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * dt * -dt;
            F.block<3, 3>(9, 9) = Matrix3d::Identity();
            F.block<3, 3>(12, 12) = Matrix3d::Identity();
            //cout<<"A"<<endl<<A<<endl;

            MatrixXd V = MatrixXd::Zero(15,18);
            V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * dt * dt;
            V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * dt * dt * 0.5 * dt;
            V.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() * dt * dt;
            V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
            V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * dt;
            V.block<3, 3>(3, 9) =  0.5 * MatrixXd::Identity(3,3) * dt;
            V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * dt;
            V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * dt * 0.5 * dt;
            V.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() * dt;
            V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);
            V.block<3, 3>(9, 12) = MatrixXd::Identity(3,3) * dt;
            V.block<3, 3>(12, 15) = MatrixXd::Identity(3,3) * dt;

            //step_jacobian = F;
            //step_V = V;
            jacobian = F * jacobian;
            covariance = F * covariance * F.transpose() + V * noise_18_18 * V.transpose();


        }
        frames[ulong(id)].pre_integration_pose_alpha=s[10];
        frames[ulong(id)].pre_integration_pose_beta=v[10];
        frames[ulong(id)].pre_integration_pose_gama=q[10];
        frames[ulong(id)].jacobian=jacobian;
        frames[ulong(id)].covariance=covariance;

    }

}

void VioOdometry::Reintegration(){
    if(REINTEGRATIONBYJACOBIAN){
        for(int i=0;i<=curframeid;i++){
            //update by 1 level jacobian
            Vector3d delta_R=frames[ulong(i)].jacobian.block<3,3>(6,12)*frames[ulong(i)].delta_gyr_bias;
            Quaterniond delta_q(1,0.5*delta_R.x(),0.5*delta_R.y(),0.5*delta_R.z());
            frames[ulong(i)].pre_integration_pose_gama=frames[ulong(i)].pre_integration_pose_gama*delta_q;
        }
    }else{
        //update by repreintegration
        for(int i=0;i<=curframeid;i++){
            //update by 1 level jacobian
            if(MIDINTEGRATION)
                ImuMidPreintegration(i);
            else
                ImuPreintegration(i);
        }
    }
}
