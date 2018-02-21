#ifndef FOO_H_   /* Include guard */
#define FOO_H_


#include <string>
#include <sstream>

#include <tf/transform_broadcaster.h>


#include <ros/ros.h>
#include <ros/time.h>
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose.h"
// #include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <message_package/matched_points.h>
#include <message_package/currentToolPosition.h>
#include "mp_mini_picker/moveToPoint.h"
#include "mp_mini_picker/moveToPose.h"
#include "mp_mini_picker/moveToQ.h"
#include "mp_mini_picker/currentQ.h"

#include "match_points/stopMatching.h"
#include "match_points/getNextMatchingPoint.h"
#include "match_points/setDistBetwChosenPoints.h"
#include <random>


using namespace std;
using namespace cv;


void sendTransformTf_PoseStamped(geometry_msgs::PoseStamped pose, string to_frame, string from_frame)
{
    Mat Rvec=(Mat_<float>(3,1) << pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z);
    Mat R;
    Rodrigues(Rvec,R);


    tf::Matrix3x3 rot;
    rot.setValue(R.at<float>(0,0),R.at<float>(0,1),R.at<float>(0,2),
                 R.at<float>(1,0),R.at<float>(1,1),R.at<float>(1,2),
                 R.at<float>(2,0),R.at<float>(2,1),R.at<float>(2,2));
    tf::Vector3 t;
    t.setValue(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
    tf::Transform wTc(rot,t);

    tf::StampedTransform msg(wTc,ros::Time::now(),to_frame,from_frame);
    static tf::TransformBroadcaster br;
    br.sendTransform(msg);
}

void sendTransformTf(Mat translation, Mat rotation, string to_frame, string from_frame) {
    tf::Matrix3x3 rot;
    rot.setValue(rotation.at<float>(0, 0), rotation.at<float>(0, 1), rotation.at<float>(0, 2),
                 rotation.at<float>(1, 0), rotation.at<float>(1, 1), rotation.at<float>(1, 2),
                 rotation.at<float>(2, 0), rotation.at<float>(2, 1), rotation.at<float>(2, 2));
    tf::Vector3 t;
    t.setValue(translation.at<float>(0, 0), translation.at<float>(1, 0), translation.at<float>(2, 0));
    tf::Transform wTc(rot, t);

    tf::StampedTransform msg(wTc, ros::Time::now(), to_frame, from_frame);
    static tf::TransformBroadcaster br;
    br.sendTransform(msg);
}

Mat convert_geomsg_to_mat(geometry_msgs::PoseStamped msg)
{
    Mat point=Mat::zeros(3,1,CV_32F);
    point.at<float>(0,0)=(float)msg.pose.position.x;
    point.at<float>(1,0)=(float)msg.pose.position.y;
    point.at<float>(2,0)=(float)msg.pose.position.z;

    return point;
}

geometry_msgs::PoseStamped convert_mat_to_geomsg(Mat point)
{
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x=point.at<float>(0,0);
    msg.pose.position.y=point.at<float>(1,0);
    msg.pose.position.z=point.at<float>(2,0);

    msg.pose.orientation.x=99;
    msg.pose.orientation.y=99;
    msg.pose.orientation.z=99;
    msg.pose.orientation.w=99;

    msg.header.frame_id=99;
    msg.header.seq=99;
    msg.header.stamp=ros::Time::now();
}

Mat convert_geomsg_to_hommat(geometry_msgs::PoseStamped msg)
{
    Mat point=Mat::zeros(4,1,CV_32F);
    point.at<float>(0,0)=(float)msg.pose.position.x;
    point.at<float>(1,0)=(float)msg.pose.position.y;
    point.at<float>(2,0)=(float)msg.pose.position.z;
    point.at<float>(3,0)=1;

    return point;
}

Mat find_transformation(vector<message_package::matched_points> &dataMatchedPoints)
{
    // finding centroids
    // frame 1 is camera
    Mat frame1_sum=Mat::zeros(3,1,CV_32F);
    Mat frame2_sum=Mat::zeros(3,1,CV_32F);
    for(int i=0 ; i<dataMatchedPoints.size(); i++)
    {
        Mat cam_point=convert_geomsg_to_mat(dataMatchedPoints[i].camera);
        Mat rob_point=convert_geomsg_to_mat(dataMatchedPoints[i].robot);

        frame1_sum=frame1_sum+cam_point;
        frame2_sum=frame2_sum+rob_point;
    }

    frame1_sum=frame1_sum/((float)dataMatchedPoints.size());
    frame2_sum=frame2_sum/((float)dataMatchedPoints.size());

    // Using SVD - create H matrix
    Mat H = Mat::zeros(3,3,CV_32F);
    for(int i=0 ; i<dataMatchedPoints.size(); i++)
    {
        Mat transposed;
        transpose(convert_geomsg_to_mat(dataMatchedPoints[i].robot)-frame2_sum,transposed);
        H=H+(convert_geomsg_to_mat(dataMatchedPoints[i].camera)-frame1_sum)*transposed;
    }

    Mat w,u,vt,v,ut;
    SVD::compute(H,w,u,vt);
    transpose(vt,v);
    transpose(u,ut);

    Mat R_computed=v*ut;

    if(cv::determinant(R_computed) < 0)
    {
//             cout << "----------------------------"  << " Reflection special case " << " ----------------------------" << endl;
//             cout << "Before "  << R_computed << endl;
        for(int i=0; i<R_computed.rows ; i++)
        {
            R_computed.at<float>(i,2)=R_computed.at<float>(i,2)*(-1);
        }
//             cout << "After "  << R_computed << endl;
//
//             cout << "----------------------------"  << " END: Reflection special case " << " ----------------------------" << endl;
    }

    // getting the translation

    Mat t_computed= -R_computed*frame1_sum+frame2_sum;

    cout <<R_computed << endl;
    cout << t_computed << endl;

    Mat T = Mat::zeros(4,4,CV_32F);
    T.at<float>(3,3)=1;

    R_computed.copyTo(T(Rect(0,0,R_computed.cols,R_computed.rows)));

    T.at<float>(0,3)=t_computed.at<float>(0,0);
    T.at<float>(1,3)=t_computed.at<float>(1,0);
    T.at<float>(2,3)=t_computed.at<float>(2,0);
    cout << T << endl;
    return T;
}


Mat find_transformation(vector<Mat> &dataMatchedPoints, Mat tcp_robot)
{
    // finding centroids
    // frame 1 is camera
    Mat frame1_sum=Mat::zeros(3,1,CV_32F);
    Mat frame2_sum=Mat::zeros(3,1,CV_32F);
    for(int i=0 ; i<dataMatchedPoints.size(); i++)
    {
        Mat cam_point=dataMatchedPoints[i];
        Mat rob_point=tcp_robot;

        frame1_sum=frame1_sum+cam_point;
        frame2_sum=frame2_sum+rob_point;
    }

    frame1_sum=frame1_sum/((float)dataMatchedPoints.size());
    frame2_sum=frame2_sum/((float)dataMatchedPoints.size());

    // Using SVD - create H matrix
    Mat H = Mat::zeros(3,3,CV_32F);
    for(int i=0 ; i<dataMatchedPoints.size(); i++)
    {
        Mat transposed;
        transpose(tcp_robot-frame2_sum,transposed);
        H=H+(dataMatchedPoints[i]-frame1_sum)*transposed;
    }

    Mat w,u,vt,v,ut;
    SVD::compute(H,w,u,vt);
    transpose(vt,v);
    transpose(u,ut);

    Mat R_computed=v*ut;

    if(cv::determinant(R_computed) < 0)
    {
//             cout << "----------------------------"  << " Reflection special case " << " ----------------------------" << endl;
//             cout << "Before "  << R_computed << endl;
        for(int i=0; i<R_computed.rows ; i++)
        {
            R_computed.at<float>(i,2)=R_computed.at<float>(i,2)*(-1);
        }
//             cout << "After "  << R_computed << endl;
//
//             cout << "----------------------------"  << " END: Reflection special case " << " ----------------------------" << endl;
    }

    // getting the translation

    Mat t_computed= -R_computed*frame1_sum+frame2_sum;

    cout <<R_computed << endl;
    cout << t_computed << endl;

    Mat T = Mat::zeros(4,4,CV_32F);
    T.at<float>(3,3)=1;

    R_computed.copyTo(T(Rect(0,0,R_computed.cols,R_computed.rows)));

    T.at<float>(0,3)=t_computed.at<float>(0,0);
    T.at<float>(1,3)=t_computed.at<float>(1,0);
    T.at<float>(2,3)=t_computed.at<float>(2,0);
    cout << T << endl;
    return T;
}


bool compare_3D_points(geometry_msgs::Pose new_position, geometry_msgs::Pose old_position, double expected_distance)
{
    double old_dist=sqrt(pow(old_position.position.x,2)+pow(old_position.position.y,2)+pow(old_position.position.z,2));
    double new_dist=sqrt(pow(new_position.position.x,2)+pow(new_position.position.y,2)+pow(new_position.position.z,2));

    if(abs(new_dist-old_dist)>=expected_distance)
        return true;
    else
        return false;
}
bool compare_3D_poses(geometry_msgs::Pose new_position, geometry_msgs::Pose old_position, double accepted_distance, double accepted_rotational_error)
{
    // translation error
    double old_dist=sqrt(pow(old_position.position.x,2)+pow(old_position.position.y,2)+pow(old_position.position.z,2));
    double new_dist=sqrt(pow(new_position.position.x,2)+pow(new_position.position.y,2)+pow(new_position.position.z,2));
//        desired_robot_position.pose,current_robot_position.pose
    // rotational error
    Mat wRt_vec= (Mat_<float>(3,1) <<    old_position.orientation.x, old_position.orientation.y, old_position.orientation.z);
    Mat wRd_vec= (Mat_<float>(3,1) <<    new_position.orientation.x, new_position.orientation.y, new_position.orientation.z);
    Mat wRt, wRd,tRw;
    Rodrigues(wRd_vec,wRd);
    Rodrigues(wRt_vec,wRt);
    transpose(wRt,tRw);
    Mat tRd=tRw*wRd;
    Mat tRd_vec;
    Rodrigues(tRd,tRd_vec);
    if(abs(new_dist-old_dist)<accepted_distance && norm(tRd_vec)<accepted_rotational_error) {
//        cout << "#################################################" << endl;
//        cout << "Accepted distance error: "<< abs(new_dist-old_dist) << endl;
//        cout << "Accepted rotation error: " << norm(tRd_vec) << endl;
//        cout << "#################################################" << endl;

        return true;
    }
    else
        return false;
}

double dot_product(Mat vec1, Mat vec2)
{
    if(vec1.rows==vec2.rows && vec1.cols == vec2.cols && vec1.cols==1)
    {
        double result=0;
        for(int i=0; i<vec1.rows ; i++)
        {
            result=result+vec1.at<double>(i,0)*vec2.at<double>(i,0);
        }
        return result;
    }
    else
    {
        cout << "Weird dimensions of your vectors" << endl;
    }
}

double dist_between_points(geometry_msgs::Pose new_position, geometry_msgs::Pose old_position)
{
    double old_dist=sqrt(pow(old_position.position.x,2)+pow(old_position.position.y,2)+pow(old_position.position.z,2));
    double new_dist=sqrt(pow(new_position.position.x,2)+pow(new_position.position.y,2)+pow(new_position.position.z,2));
    return abs(new_dist-old_dist);
}

Mat convert_geomsg_to_trans(geometry_msgs::PoseStamped T)
{
    Mat transformation;
    transformation = Mat::zeros(4,4,CV_32F);
    transformation.at<float>(3,3)=1;
    transformation.at<float>(0,3)=T.pose.position.x;
    transformation.at<float>(1,3)=T.pose.position.y;
    transformation.at<float>(2,3)=T.pose.position.z;
    Mat R_vec;
    R_vec= Mat::zeros(3,1,CV_32F);
    R_vec.at<float>(0,0)=T.pose.orientation.x;
    R_vec.at<float>(1,0)=T.pose.orientation.y;
    R_vec.at<float>(2,0)=T.pose.orientation.z;
    Mat R;
    Rodrigues(R_vec,R);

    R.copyTo(transformation(Rect(0,0,R.cols,R.rows)));

    return transformation;
}

Mat convert_geomsg_to_R(geometry_msgs::PoseStamped R)
{
    Mat R_vec;
    R_vec= Mat::zeros(3,1,CV_32F);
    R_vec.at<float>(0,0)=R.pose.orientation.x;
    R_vec.at<float>(1,0)=R.pose.orientation.y;
    R_vec.at<float>(2,0)=R.pose.orientation.z;
    Mat R_result;
    Rodrigues(R_vec,R_result);
    return R_result;
}

Mat convert_geomsg_to_Rvec(geometry_msgs::PoseStamped R)
{
    Mat R_vec;
    R_vec= Mat::zeros(3,1,CV_32F);
    R_vec.at<float>(0,0)=R.pose.orientation.x;
    R_vec.at<float>(1,0)=R.pose.orientation.y;
    R_vec.at<float>(2,0)=R.pose.orientation.z;
    return R_vec;
}

Mat calc_transformation_of_point(geometry_msgs::PoseStamped T, geometry_msgs::PoseStamped point)
{
    Mat Point_transformed=convert_geomsg_to_trans(T)*convert_geomsg_to_hommat(point);
    return Point_transformed;
}

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

Mat generate_random_vector(double length,double mean=0.0, double std_dev=10.0)
{
//    std::default_random_engine generator;
    std::random_device generator;
    std::normal_distribution<double> distribution(mean,std_dev); // mean, std. dev

    double x=distribution(generator);
    double y=distribution(generator);
    double z=distribution(generator);
    double factor=sqrt(pow(x,2)+pow(y,2)+pow(z,2));
    Mat result=Mat::zeros(3,1,CV_64F);
    result.at<double>(0,0)=x/factor*length;
    result.at<double>(1,0)=y/factor*length;
    result.at<double>(2,0)=z/factor*length;

    return result;
}

double get_sum(vector<Mat> data, string variable1="0", string variable2="0", string variable3="0") {
    double sum = 0;
    for (Mat data_point : data) {
        if (variable1 == "x" && variable2 == "0" && variable3 == "0") {
            sum = sum + data_point.at<double>(0, 0);
        } else if (variable1 == "y" && variable2 == "0" && variable3 == "0") {
            sum = sum + data_point.at<double>(1, 0);
        } else if (variable1 == "z" && variable2 == "0" && variable3 == "0") {
            sum = sum + data_point.at<double>(2, 0);
        } else if (variable1 == "x" && variable2 == "x" && variable3 == "0") {
            sum = sum + data_point.at<double>(0, 0) * data_point.at<double>(0, 0);
        } else if (variable1 == "y" && variable2 == "y" && variable3 == "0") {
            sum = sum + data_point.at<double>(1, 0) * data_point.at<double>(1, 0);
        } else if (variable1 == "z" && variable2 == "z" && variable3 == "0") {
            sum = sum + data_point.at<double>(2, 0) * data_point.at<double>(2, 0);
        } else if (variable1 == "x" && variable2 == "y" && variable3 == "0") {
            sum = sum + data_point.at<double>(0, 0) * data_point.at<double>(1, 0);
        } else if (variable1 == "x" && variable2 == "z" && variable3 == "0") {
            sum = sum + data_point.at<double>(0, 0) * data_point.at<double>(2, 0);
        } else if (variable1 == "y" && variable2 == "z" && variable3 == "0") {
            sum = sum + data_point.at<double>(1, 0) * data_point.at<double>(2, 0);
        } else if (variable1 == "x" && variable2 == "x" && variable3 == "x") {
            sum = sum + data_point.at<double>(0, 0) * data_point.at<double>(0, 0) * data_point.at<double>(0, 0);
        } else if (variable1 == "y" && variable2 == "y" && variable3 == "y") {
            sum = sum + data_point.at<double>(1, 0) * data_point.at<double>(1, 0) * data_point.at<double>(1, 0);
        } else if (variable1 == "z" && variable2 == "z" && variable3 == "z") {
            sum = sum + data_point.at<double>(2, 0) * data_point.at<double>(2, 0) * data_point.at<double>(2, 0);
        } else if (variable1 == "x" && variable2 == "y" && variable3 == "y") {
            sum = sum + data_point.at<double>(0, 0) * data_point.at<double>(1, 0) * data_point.at<double>(1, 0);
        } else if (variable1 == "x" && variable2 == "z" && variable3 == "z") {
            sum = sum + data_point.at<double>(0, 0) * data_point.at<double>(2, 0) * data_point.at<double>(2, 0);
        } else if (variable1 == "x" && variable2 == "x" && variable3 == "y") {
            sum = sum + data_point.at<double>(0, 0) * data_point.at<double>(0, 0) * data_point.at<double>(1, 0);
        } else if (variable1 == "x" && variable2 == "x" && variable3 == "z") {
            sum = sum + data_point.at<double>(0, 0) * data_point.at<double>(0, 0) * data_point.at<double>(2, 0);
        } else if (variable1 == "y" && variable2 == "y" && variable3 == "z") {
            sum = sum + data_point.at<double>(1, 0) * data_point.at<double>(1, 0) * data_point.at<double>(2, 0);
        } else if (variable1 == "y" && variable2 == "z" && variable3 == "z") {
            sum = sum + data_point.at<double>(1, 0) * data_point.at<double>(2, 0) * data_point.at<double>(2, 0);
        }
    }
}

Mat fit_sphere(vector<Mat> points)
{
        double N=points.size();
        double Sx=get_sum(points,"x");
        double Sy=get_sum(points,"y");
        double Sz=get_sum(points,"z");
        cout << Sx << " , " << Sy << " , " << Sz << endl;

        double Sxx=get_sum(points,"x","x");
        double Syy=get_sum(points,"y","y");
        double Szz=get_sum(points,"z","z");
        double Sxy=get_sum(points,"x","y");
        double Sxz=get_sum(points,"x","z");
        double Syz=get_sum(points,"y","z");
        cout << endl;
        cout << Sxx << " , " << Syy << " , " << Szz << endl;
        cout << Sxy << " , " << Sxz << " , " << Syz << endl;

        double Sxxx=get_sum(points,"x","x","x");
        double Syyy=get_sum(points,"y","y","y");
        double Szzz=get_sum(points,"z","z","z");
        double Sxyy=get_sum(points,"x","y","y");
        double Sxzz=get_sum(points,"x","z","z");
        double Sxxy=get_sum(points,"x","x","y");
        double Sxxz=get_sum(points,"x","x","z");
        double Syyz=get_sum(points,"y","y","z");
        double Syzz=get_sum(points,"y","z","z");

        cout << endl;
        cout << Sxxx << " , " << Syyy << " , " << Szzz << endl;
        cout << Sxyy << " , " << Syzz << " , " << Sxxy << endl;
        cout << Sxxz << " , " << Syyz << " , " << Syzz << endl;

        double A1 = Sxx +Syy +Szz;

        double a=2*Sx*Sx-2*N*Sxx;
        double b=2*Sx*Sy-2*N*Sxy;
        double c=2*Sx*Sz-2*N*Sxz;
        double d=-N*(Sxxx +Sxyy +Sxzz)+A1*Sx;

        double e=2*Sx*Sy-2*N*Sxy;
        double f=2*Sy*Sy-2*N*Syy;
        double g=2*Sy*Sz-2*N*Syz;
        double h=-N*(Sxxy +Syyy +Syzz)+A1*Sy;

        double j=2*Sx*Sz-2*N*Sxz;
        double k=2*Sy*Sz-2*N*Syz;
        double l=2*Sz*Sz-2*N*Szz;
        double m=-N*(Sxxz +Syyz + Szzz)+A1*Sz;
        double delta = a*(f*l - g*k)-e*(b*l-c*k) + j*(b*g-c*f);

        double xc = (d*(f*l-g*k) -h*(b*l-c*k) +m*(b*g-c*f))/delta;
        double yc = (a*(h*l-m*g) -e*(d*l-m*c) +j*(d*g-h*c))/delta;
        double zc = (a*(f*m-h*k) -e*(b*m-d*k) +j*(b*h-d*f))/delta;
        double R = sqrt(pow(xc,2)+pow(yc,2)+pow(zc,2)+(A1-2*(xc*Sx+yc*Sy+zc*Sz))/N);

        Mat result=Mat::zeros(4,1,CV_64F);
        result.at<double>(0,0)=xc;
        result.at<double>(1,0)=yc;
        result.at<double>(2,0)=zc;
        result.at<double>(3,0)=R;
        return result;
}

Mat roll(float angle)
{
    float  ca = cos(angle);
    float  sa = sin(angle);
    Mat rotation=(Mat_<float>(3, 3) << ca,-sa,0,
            sa,ca,0,
            0,0,1);
    return rotation;
}

Mat pitch(float angle)
{
    float  ca = cos(angle);
    float  sa = sin(angle);
    Mat rotation=(Mat_<float>(3, 3) << ca,0,sa,
            0,1,0,
            -sa,0,ca);
    return rotation;
}

Mat yaw(float angle)
{
    float  ca = cos(angle);
    float  sa = sin(angle);
    Mat rotation=(Mat_<float>(3, 3) << 1,0,0,
            0,ca,-sa,
            0,sa,ca);
    return rotation;
}

Mat RPY_to_rotation(float r, float p, float y)
{
    return (roll(r)*pitch(p)*yaw(y));
}

//void move_iteratively_to_marker2(double error_correction=0.1)
//{
//    // find desired pose and dR
//    Mat cR2=convert_geomsg_to_R(marker2_position);
//    Mat cRdesired=cR2*_2R1;
//    Mat cR1=convert_geomsg_to_R(marker1_position);
//    Mat _1Rc;
//    transpose(cR1,_1Rc);
//    Mat _1Rdesired=_1Rc*cRdesired;
//
//    // Reduce rotation error with error correction variable
//    Mat _1Rdesired_vec;
//    Rodrigues(_1Rdesired,_1Rdesired_vec);
//    _1Rdesired_vec=_1Rdesired_vec*error_correction;
////        Rodrigues(_1Rdesired_vec,_1Rdesired);
//
//    // Now correct current rotation with our rotation error
//    Mat tcpRmarker=tcpTmarker(Rect(0,0,3,3));
//    Mat wRtcp=convert_geomsg_to_R(current_robot_position.tcp);
//    Mat wRdesired=wRtcp*tcpRmarker*_1Rdesired,wRdesired_vec;
//    Rodrigues(wRdesired,wRdesired_vec);
//
//    //correct marker 1 to be the tip of tool - measured observation
//    Mat cTmarker1=convert_geomsg_to_trans(marker1_position); // given as a relative transformation
//    cout << "CTmarker1: \n" << convert_geomsg_to_hommat(marker1_position) <<  endl;
//    Mat tip_correction = (Mat_<float>(4,1) << 0.170, -0.014, -0.015, 1.0); // correcting for tip seen from marker 1 frame
//    Mat wPtip=wTc*cTmarker1*tip_correction;
//    cout << "CPmarker1: \n" << convert_geomsg_to_hommat(marker1_position) <<  endl;
//    Mat wPmarker1=wTc*convert_geomsg_to_hommat(marker1_position);
//
//
//    // correct marker 2
//    Mat cPmarker2=convert_geomsg_to_hommat(marker2_position);
//    Mat wPmarker2=wTc*cPmarker2;
//    cout << "wPmarker true value is: \n" << wPmarker2 << endl;
//    Mat correction = (Mat_<float>(4,1) << 0, 0, 0.2, 0.0); // we do not want to fuck up, so 20 cm over marker
//    wPmarker2=wPmarker2+correction;
//    cout << "wPmarker corrected value is: \n" << wPmarker2 << endl;
//    // Find positional error and the desired correction of that error
////        Mat positional_error=(wPmarker2-wPmarker1)*error_correction;
//    Mat positional_error=(wPmarker2-wPtip);
//
//    Mat marker1Porigin = (Mat_<float>(4,1) << 0.0, 0.0, 0.0, 1.0);
//    Mat wTmarker1=convert_geomsg_to_trans(current_robot_position.wTtcp)*tcpTmarker;
//    Mat wPmarker1_origin=wTmarker1*marker1Porigin;
//    Mat wPmarker1_origin_corrected=wPmarker1_origin+positional_error;
//
//
//
//    mp_mini_picker::moveToPoseMarker serv;
//    // Setting pose
//    serv.request.pose[0]=wPmarker1_origin_corrected.at<float>(0,0);
//    serv.request.pose[1]=wPmarker1_origin_corrected.at<float>(1,0);
//    serv.request.pose[2]=wPmarker1_origin_corrected.at<float>(2,0);
//    serv.request.pose[3]=wRdesired_vec.at<float>(0,0);
//    serv.request.pose[4]=wRdesired_vec.at<float>(1,0);
//    serv.request.pose[5]=wRdesired_vec.at<float>(2,0);
//
//    // Setting R vector for transformation
//    Mat tcpRvecmarker;
//    Rodrigues(tcpRmarker,tcpRvecmarker);
//    serv.request.tcpRmarker[0]=tcpRvecmarker.at<float>(0,0);
//    serv.request.tcpRmarker[1]=tcpRvecmarker.at<float>(1,0);
//    serv.request.tcpRmarker[2]=tcpRvecmarker.at<float>(2,0);
//    // setting P for transformation
//    serv.request.tcpPmarker[0]=tcpTmarker.at<float>(0,3);
//    serv.request.tcpPmarker[1]=tcpTmarker.at<float>(1,3);
//    serv.request.tcpPmarker[2]=tcpTmarker.at<float>(2,3);
//
//    cout << "################################################### \nNew move for visual servoing" << endl;
//    cout << "wTc is\n" << wTc << endl;
//    cout << "tcpTmarker is\n" << tcpTmarker << endl;
//    cout << "wRdesired is: \n" << wRdesired << endl;
//    cout << "Positional error is: " << positional_error << endl;
//    cout << "Tip location: : \n" << wPtip << endl;
//    cout << "Marker 1 location world: \n" << wPmarker1 << endl;
//
////        while(true)
////        {
//////            Mat wowoesa = (Mat_<float>(3,1) << 0.6, 0.8, 0.6);
//////            wowoesa.at<float>(0, 0)=wTc.at<float>(0, 3);
//////            wowoesa.at<float>(1, 0)=wTc.at<float>(1, 3);
//////            wowoesa.at<float>(2, 0)=wTc.at<float>(2, 3);
//////
//////            sendTransformTf(wowoesa,wTc(Rect(0,0,3,3))/determinant(wTc(Rect(0,0,3,3))), "Camera", "map");
////////            sendTransformTf_PoseStamped(current_robot_marker_position.tcp,"Marker1","map");
//////            wPmarker2=wTc*cPmarker2;
//////            Mat wRmarker2=wTc(Rect(0,0,3,3))*convert_geomsg_to_R(marker2_position);
//////            sendTransformTf(wPmarker2,wRmarker2/determinant(wRmarker2),"Marker2", "map");
//////            sendTransformTf(wPmarker1_origin_corrected,_1Rdesired/determinant(_1Rdesired), "Desired pose", "map");
////            ros::spinOnce();
////        }
//
//    bool done= false;
//    serv_move_robot_pose_marker_.call(serv);
//    if (serv.response.ok == 1)
//    {
//        done = true;
//    } else if(serv.response.ok == 2)
//    {
//        cout << "In collision" << endl;
//        while(true)
//        {
//            ros::spinOnce();
//        }
//    }else if(serv.response.ok == 3)
//    {
//        cout << "No solution to jacobian solver" << endl;
//        while(true)
//        {
//            ros::spinOnce();
//        }
//    } else
//    {
//        cout << "No error code" << endl;
//        while(true)
//        {
//            ros::spinOnce();
//        }
//    }
////        }
//
//    // change this to TCP view
//
//    desired_robot_position.pose.position.x=serv.request.pose[0];
//    desired_robot_position.pose.position.y=serv.request.pose[1];
//    desired_robot_position.pose.position.z=serv.request.pose[2];
//    desired_robot_position.pose.orientation.x=serv.request.pose[3];
//    desired_robot_position.pose.orientation.y=serv.request.pose[4];
//    desired_robot_position.pose.orientation.z=serv.request.pose[5];
//
//    while(true)
//    {
//        ros::spinOnce();
//    }
//}


//if(ic.got_marker1_positions() && found_wRc==true && asked_for_points==true && found_minMax_angles==true)
//{
//sphere_coordinates.push_back(ic.calc_marker1_positions_mean());
//asked_for_points=false;
//}
//if(found_wRc==true && asked_for_points==false && found_minMax_angles==true)
//{
//if(ic.robot_at_asked_pose(0.0001,M_PI/(2*90)))
//{
//if(changed_pose==true)
//{
//cout << "Asking for cam coordinates: " << endl;
//ic.set_get_marker_positions(true);
//last_publish=ros::Time::now();
//asked_for_points=true;
//changed_pose=false;
//}
//else if(changed_pose==false)
//{
//if(sphere_coordinates.size()!=total_number_of_tcp_rotations){
//cout << "#################################\n" <<"New tcp pose number: " << sphere_coordinates.size() << endl;
//ic.change_tcp_pose();
//
//changed_pose=true;
//} else if(sphere_coordinates.size()==total_number_of_tcp_rotations && got_sphere==false)
//{
//for(int i=0 ; i< sphere_coordinates.size() ; i++)
//{
//cout << "####################### iteration " << i << " #################"<< endl;
//cout << sphere_coordinates[i] << endl;
//}
//Mat sphere=fit_sphere(sphere_coordinates);
//cout << "Our sphere is: \n" << sphere << endl;
//got_sphere=true;
//cout << "Our translation is: \n" << ic.calculate_wtc(sphere) << endl;
//Mat result=ic.estimate_tcpTmarker();
//cout << "tcpTmarker: \n"<< result << endl;
//ic.move_to_marker2();
//}
//}
//}
//}
//if(ros::Time::now()-last_publish>ros::Duration(10) && found_minMax_angles==true )
//{
//if(found_wRc==true && asked_for_points==true && got_sphere==false) {
//
//cout << "ALERT !! Not enough coordinates within 5 seconds - going back to old config" << endl;
//changed_pose = false;
//asked_for_points = false;
//ic.set_get_marker_positions(false);
//ic.go_to_old_position();
//last_publish=ros::Time::now();
//cout << "Should have sent it back to old pose" << endl;
//}
//}



#endif // FOO_H_