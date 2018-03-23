#include <stdio.h>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/proximity/CollisionDetector.hpp>

#include <rw/invkin/JacobianIKSolver.hpp>

#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Rotation3DVector.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/loaders.hpp>


#include <opencv2/opencv.hpp>

#include <fstream>

#include <iostream>


using namespace std;
using namespace cv;

Mat inverse_T(Mat T)
{
    Mat T_inverse= Mat::eye(4, 4, CV_32F);
    Mat translation=Mat::zeros(3, 1, CV_32F);
    Mat R= Mat::zeros(3, 3, CV_32F);
    T(Rect(0,0,3,3)).copyTo(R);
//    cout << "##########################################\n" << R << endl;

    Mat R_trans;
    transpose(R,R_trans);
//    cout << R_trans << endl;
    R_trans.copyTo(T_inverse(Rect(0,0,R_trans.cols,R_trans.rows)));
//    cout << T_inverse << endl;
    Mat P=T(Rect(3,0,1,3));
    Mat P_inv=-(R_trans*P);
    P_inv.copyTo(T_inverse(Rect(3,0,P_inv.cols,P_inv.rows)));
//    cout << T_inverse << endl;
    return T_inverse;
}

Eigen::MatrixXd make_skew(rw::math::Transform3D<> data)
{
    Eigen::MatrixXd result(3,3);
    result <<   0,          -data.P()(2),   data.P()(1),
                data.P()(2),0,              -data.P()(0),
                -data.P()(1),data.P()(0),   0;

    return result;

}
//
//Mat make_skew_mat(rw::math::Transform3D<> data)
//{
//    Mat m = (Mat_<float>(3,3) <<    0,          -data.P()(2),   data.P()(1),
//                                    data.P()(2),0,              -data.P()(0),
//                                    -data.P()(1),data.P()(0),   0);
//    return m;
//}
Mat make_skew_mat(Mat data)
{
//    Mat m = (Mat_<float>(3,3) <<    0,          -data.at<float>(2,3),   data.at<float>(1,3),
//                                    data.at<float>(2,3),0,              -data.at<float>(0,3),
//                                    -data.at<float>(1,3),data.at<float>(0,3),   0);
    Mat m = (Mat_<float>(3,3) <<    1,          -data.at<float>(2,3),   data.at<float>(1,3),
                                    data.at<float>(2,3),1,              -data.at<float>(0,3),
                                    -data.at<float>(1,3),data.at<float>(0,3),   1);
    return m;
}

Mat Transform3D_to_Matfloat(rw::math::Transform3D<> matrix)
{

    Mat m = (Mat_<float>(4,4) <<    matrix.R()(0,0), matrix.R()(0,1), matrix.R()(0,2),matrix.P()(0),
                                    matrix.R()(1,0), matrix.R()(1,1), matrix.R()(1,2),matrix.P()(1),
                                    matrix.R()(2,0), matrix.R()(2,1), matrix.R()(2,2),matrix.P()(2),
                                    0,              0,              0,                1);
    return m;
}

rw::math::Transform3D<> Matfloat_to_Transform3D(Mat matrix)
{
    rw::math::Vector3D<> P(matrix.at<float>(0,3),matrix.at<float>(1,3),matrix.at<float>(2,3));
    rw::math::Rotation3D<> R(matrix.at<float>(0,0),matrix.at<float>(0,1),matrix.at<float>(0,2),
                             matrix.at<float>(1,0),matrix.at<float>(1,1),matrix.at<float>(1,2),
                             matrix.at<float>(2,0),matrix.at<float>(2,1),matrix.at<float>(2,2));
    return rw::math::Transform3D<>(P,R);
}

Mat calc_pose_error(Mat current, Mat desired) // transformation matrices
{
    Mat error=Mat::zeros(6,1,CV_32F);
    error(Rect(0,0,1,3))=desired(Rect(3,0,1,3))-current(Rect(3,0,1,3));
    Mat commonRcurrent=current(Rect(0,0,3,3)),currentRcommon;
    Mat commonRdesired=desired(Rect(0,0,3,3));
    transpose(commonRcurrent,currentRcommon);
    Mat currentRdesired=currentRcommon*commonRdesired,currentRdesired_EAA;
//    cout << currentRdesired << endl;
    Rodrigues(currentRdesired,currentRdesired_EAA);
//    cout << currentRdesired_EAA << endl;
//    error(Rect(0,3,1,3))=currentRdesired_EAA;
    error.at<float>(3,0)=currentRdesired_EAA.at<float>(0,0);
    error.at<float>(4,0)=currentRdesired_EAA.at<float>(1,0);
    error.at<float>(5,0)=currentRdesired_EAA.at<float>(2,0);
//    cout << error(Rect(0,3,1,3)) << endl;
    return error;
}
void insert(Mat &src, Mat data, int x, int y)
{
    for(int i=0 ; i<data.rows ; i++)
    {
        for(int j=0; j<data.cols; j++)
        {
            src.at<float>(i+y,j+x)=data.at<float>(i,j);
        }
    }
}
double sinc(const double x)
{
    if (x==0)
        return 1;
    return sin(x)/x;
}

Mat calc_L_EAA(Mat EAA_vec)
{
    double sigma=norm(EAA_vec);
    Mat unit_vector=EAA_vec/sigma;
    Mat result=Mat::eye(3,3,CV_32F);
    result=result-sigma/2*make_skew_mat(unit_vector)+(1-(sinc(sigma)/pow(sinc(sigma/2),2)))*(make_skew_mat(unit_vector)*make_skew_mat(unit_vector));

    return result;
}

int main(int argc, char** argv )
{
    rw::models::WorkCell::Ptr workcell_ = rw::loaders::XMLRWLoader::load("../Mini-Picker/Mini-Picker_v2_DHJoints.wc.xml");

    auto currentState = workcell_->getDefaultState();
    auto device_ = workcell_->findDevice("UR5");
    
    
    rw::math::Q current_configuration = device_->getQ(currentState);
    auto joint_limits=device_->getBounds();

    device_->setQ(current_configuration,currentState);



    auto wTb=Transform3D_to_Matfloat((device_->worldTbase(currentState)));
    auto bTw = inverse_T(wTb);

    auto marker2=workcell_->findFrame("marker2_f");
    auto cam=workcell_->findFrame("cam");

    auto base=workcell_->findFrame("UR5.Base");

    auto bTc = Transform3D_to_Matfloat(base->fTf(cam,currentState));
    double correction=0.05;
    int number_of_iteration=100;
//    Current error - first error
//    [-0.055490062;
//    0.76351929;
//    0.068633437;
//    0.51828706;
//    -0.030617062;
//    0.34916669]

    ofstream myfile;
    myfile.open ("example.txt");
    for(int i=0; i<number_of_iteration ; i++)
    {
        cout << "################### ITERATION " << i << "#######################" << endl;

        auto marker1=workcell_->findFrame("marker_f");
        rw::invkin::JacobianIKSolver findQ(device_,marker1,currentState);

        auto cTm1=Transform3D_to_Matfloat(cam->fTf(marker1,currentState));

        // data logging for Matlab
        // needs to be in format x,y,z, length in x,length in y,length in z
        Mat x=(Mat_<float>(4,1) << 0.01,0,0,1);
        Mat y=(Mat_<float>(4,1) << 0.0,0.01,0,1);
        Mat z=(Mat_<float>(4,1) << 0.0,0,0.01,1);
        Mat zero=(Mat_<float>(4,1) << 0.0,0,0,1);

        Mat bx=bTc*cTm1*x;
        Mat by=bTc*cTm1*y;
        Mat bz=bTc*cTm1*z;
        Mat bzero=bTc*cTm1*zero;
        bx=bx-bzero;
        by=by-bzero;
        bz=bz-bzero;
        myfile << bzero.at<float>(0,0) << " , " << bzero.at<float>(1,0) << " , " << bzero.at<float>(2,0) << " , " << bx.at<float>(0,0) << " , " << bx.at<float>(1,0) << " , " << bx.at<float>(2,0)  << endl;
        myfile << bzero.at<float>(0,0) << " , " << bzero.at<float>(1,0) << " , " << bzero.at<float>(2,0) << " , " << by.at<float>(0,0) << " , " << by.at<float>(1,0) << " , " << by.at<float>(2,0)  << endl;
        myfile << bzero.at<float>(0,0) << " , " << bzero.at<float>(1,0) << " , " << bzero.at<float>(2,0) << " , " << bz.at<float>(0,0) << " , " << bz.at<float>(1,0) << " , " << bz.at<float>(2,0)  << endl;
        // end of logging


        auto cTm2=Transform3D_to_Matfloat(cam->fTf(marker2,currentState));
        Mat error=calc_pose_error(cTm1,cTm2);
        cout << "Current error \n" << error << endl;


        cout << "cTm1 before correction is: \n" << cTm1 << endl;
        Mat Le=Mat::eye(6,6,CV_32F);
//        Le(Rect(3,0,3,3))=make_skew_mat(error(Rect(0,0,1,3)));
        insert(Le,make_skew_mat(error(Rect(0,0,1,3))),3,0);
//        insert(Le,calc_L_EAA(error(Rect(0,3,1,3))),3,3);
        cout << Le << endl;
//        Mat Le2=Mat::eye(6,6,CV_32F);
//        Mat error_correction=Le*error;
//        Mat error_correction2=Le2*error;
        Mat error_correction=correction*Le*error;
//        Mat error_correction2=correction*Le2*error;
        cout << error_correction << endl;
//        cout << error_correction2 << endl;
        cTm1(Rect(3,0,1,3))=cTm1(Rect(3,0,1,3))+error_correction(Rect(0,0,1,3));
        Mat _1R2;
        Rodrigues(error_correction(Rect(0,3,1,3)),_1R2);
        cTm1(Rect(0,0,3,3))=cTm1(Rect(0,0,3,3))*_1R2;
        cout << "cTm1 after correction is: \n" << cTm1 << endl;

        Mat desired_transform=bTc*cTm1;

        auto desired_transform_robwork=Matfloat_to_Transform3D(desired_transform);




        cout << "Desired transform: " << desired_transform_robwork << endl;
        cout << "current: " << base->fTf(marker1,currentState) << endl;

        auto end_configuration = findQ.solve(desired_transform_robwork,currentState);
        if(end_configuration.size() > 0)
        {
            cout << "Old: " << device_->getQ(currentState) << endl;
            cout << "New: " << end_configuration[0] << endl;
            device_->setQ(end_configuration[0],currentState);
        }
        else
        {
            cout <<"No solution to JacobianIKSolver" << endl;
            i=number_of_iteration;
        }

    }
    myfile.close();

//    cout << "################### ITERATION " << i << "#######################" << endl;
//    auto marker1=workcell_->findFrame("marker_f");
//    rw::invkin::JacobianIKSolver findQ(device_,marker1,currentState);
//
//    auto m1Tc=Transform3D_to_Matfloat(marker1->fTf(cam,currentState));
//    auto m2Tc=Transform3D_to_Matfloat(marker2->fTf(cam,currentState));
//    cout << "Current progress: m2tm1: \n" <<inverse_T(m2Tc)(Rect(3,0,1,3))-inverse_T(m1Tc)(Rect(3,0,1,3)) << endl;
//    Mat desired_position=m2Tc(Rect(3,0,1,3));
//
//    Mat current_position=m1Tc(Rect(3,0,1,3));
//
//    Mat cR1;
//    transpose(m1Tc(Rect(0,0,3,3)),cR1);
//    Mat _2R1=m2Tc(Rect(0,0,3,3))*cR1;
//    Mat _2R1_EAA;
//    Rodrigues(_2R1,_2R1_EAA);
//    Mat _1R2;
//    transpose(_2R1,_1R2);
////        auto Vc=-correction*(_1R2)*(desired_position-current_position);
////        auto Vc=-correction*(_1R2)*(current_position-desired_position);
//    auto Vc=-correction*(desired_position-current_position)+make_skew_mat(m1Tc)*_2R1_EAA;
//    auto Rc=-correction*_2R1_EAA;
//
//    cout << "Vc is: \n" << Vc << endl;
//    cout << "Rc is: \n" << Rc << endl;
//
//    // calc desired transform. Its given in base frame
//    cout << "m1Tc before correction is: \n" << m1Tc << endl;
//    Mat Rotation_correction;
//    Rodrigues(Rc,Rotation_correction);
//    m1Tc(Rect(0,0,3,3))=m1Tc(Rect(0,0,3,3))*Rotation_correction;
//    m1Tc(Rect(3,0,1,3))=m1Tc(Rect(3,0,1,3))+Vc*0.1;
//    cout << "m1Tc after correction is: \n" << m1Tc << endl;
//
//    Mat desired_transform=bTc*inverse_T(m1Tc);
//
//    auto desired_transform_robwork=Matfloat_to_Transform3D(desired_transform);
//
//
//
//
//    cout << "Desired transform: " << desired_transform_robwork << endl;
//    cout << "current: " << base->fTf(marker1,currentState) << endl;
//
//    auto end_configuration = findQ.solve(desired_transform_robwork,currentState);
//    if(end_configuration.size() > 0)
//    {
//        cout << "Old: " << device_->getQ(currentState) << endl;
//        cout << "New: " << end_configuration[0] << endl;
//        device_->setQ(end_configuration[0],currentState);
//    }
//    else
//    {
//        cout <<"No solution to JacobianIKSolver" << endl;
//        i=number_of_iteration;
//    }




//
//    rw::math::Vector3D<double> desired_position_world(req.pose[0],req.pose[1],req.pose[2]);
//    auto desired_position_base=bTw*desired_position_world;
//
////             auto current_rotation=bTtool.R();
//
//    auto desired_rotation_world_vec=rw::math::EAA<double>(req.pose[3],req.pose[4],req.pose[5]);
//    auto desired_rotation_world=desired_rotation_world_vec.toRotation3D();
//    rw::math::RPY<double> test(desired_rotation_world);
////         cout << "Desired rotation in world: " <<test << endl;
//    auto desired_rotation_base=bTw.R()*desired_rotation_world;
//
//    rw::math::Transform3D<double> desired_transform(desired_position_base,desired_rotation_base);
//
//    auto end_configuration = findQ.solve(desired_transform,currentState);




//    for(int i=0; i<1 ; i++)
//    {
//        cout << "################### ITERATION " << i << "#######################" << endl;
//        auto marker1=workcell_->findFrame("marker_f");
//        rw::invkin::JacobianIKSolver findQ(device_,marker1,currentState);
//        auto bTtool=(device_->baseTframe(marker1,currentState));
//        auto m1Tc=marker1->fTf(cam,currentState);
//        auto m2Tc=marker2->fTf(cam,currentState);
//        auto desired_transform=bTc*inverse(m1Tc);
//        cout << "Desired transform: " << desired_transform << endl;
//        cout << "Current progress: m2tm1: \n" <<(m2Tc*inverse(m1Tc)).P() << endl;
//
//        cout << m1Tc << endl;
//        Eigen::VectorXd desired=m2Tc.P().e();
//        cout << "Desired transform: " << m1Tc << endl;
//        Eigen::VectorXd current=m1Tc.P().e();
//        cout << "Desired transform: " << m1Tc << endl;
//        rw::math::EAA<> R(m2Tc.R()*m1Tc.R().inverse());
//        cout << "Desired transform: " << m1Tc << endl;
//        Eigen::VectorXd R_vec(3);
//        cout << "Desired transform: " << m1Tc << endl;
//        R_vec << R(0), R(1), R(2);
//        cout << "Desired transform: " << m1Tc << endl;
////        auto Vc=-correction*(desired-current)+make_skew(m_t_c)*R_vec;
//        auto Vc=-correction*((m1Tc.R()*m2Tc.R().inverse()).e())*(desired-current);
//        cout << "Desired transform: " << m1Tc << endl;
//        auto Rc=-correction*R_vec;
//        cout << "Desired transform: " << m1Tc << endl;
//
////        cout << "Vc is: \n" << Vc << endl;
////        cout << "Rc is: \n" << Rc << endl;
//
//        rw::math::Vector3D<> m1tc(m1Tc.P()(0)+Vc(0), m1Tc.P()(1)+Vc(1), m1Tc.P()(2)+Vc(2));
//        cout << "Desired transform: " << m1Tc << endl;
//        rw::math::Transform3D<> m1Tc_changed(m1tc,m1Tc.R());
//        cout << "Desired transform: " << m1Tc << endl;
//
//
//
//        desired_transform=bTc*inverse(m1Tc);;
//        cout << "Desired transform: " << desired_transform << endl;
//
//        cout << "current: " << base->fTf(marker1,currentState) << endl;
//
//        auto end_configuration = findQ.solve(desired_transform,currentState);
//        if(end_configuration.size() > 0)
//        {
//            cout << "Old: " << device_->getQ(currentState) << endl;
//            cout << "New: " << end_configuration[0] << endl;
//            device_->setQ(end_configuration[0],currentState);
//        }
//        else
//        {
//            cout <<"No solution to JacobianIKSolver" << endl;
//
//        }
//        cout << Transform3D_to_Matfloat(desired_transform) << endl;
//        cout << Matfloat_to_Transform3D(Transform3D_to_Matfloat(desired_transform)) << endl;
//    }
}


