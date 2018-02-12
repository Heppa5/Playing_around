


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
#include <message_package/currentToolPosition2.h>
#include "mp_mini_picker/moveToPointTcp.h"
#include "mp_mini_picker/moveToPoseTcp.h"
#include "mp_mini_picker/moveToPointMarker.h"
#include "mp_mini_picker/moveToPoseMarker.h"
#include "mp_mini_picker/moveToQ.h"
#include "mp_mini_picker/currentQ.h"

#include "match_points/stopMatching.h"
#include "match_points/getNextMatchingPoint.h"
#include "match_points/setDistBetwChosenPoints.h"

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <random>

#include <helping_functions.h>


using namespace std;
using namespace cv;



class TestClass
{
    ros::NodeHandle nh_;
    ros::Subscriber matched_points_sub_;
    ros::Subscriber transformation_sub_;

    ros::Subscriber marker2_sub_;
    ros::Subscriber marker1_sub_;

    ros::Subscriber robot_position_sub_;

    ros::Publisher difference_pub_;

    ros::Publisher marker_dist_pub_;

    ros::ServiceClient serv_move_robot_point_tcp_;

    ros::ServiceClient serv_move_robot_pose_tcp_;

    ros::ServiceClient serv_move_robot_point_marker_;

    ros::ServiceClient serv_move_robot_pose_marker_;
    
    ros::ServiceClient serv_move_robotQ_;

    ros::ServiceClient serv_get_robotQ_;

    ros::ServiceClient serv_stop_matching_;

    ros::ServiceClient serv_set_distance_betw_matched_points_;

    ros::ServiceClient serv_get_matched_point_;

    bool debug=false;
    bool use_incremental_correction;
public:
    TestClass(bool incremental_correction)
    {
        use_incremental_correction=incremental_correction;
        // Subscribe to different topics
        matched_points_sub_ = nh_.subscribe("/matched_points/chosenMatchedPoints", 1, &TestClass::new_chosen_point, this);
        robot_position_sub_ = nh_.subscribe("/robot/moved", 1, &TestClass::update_robot_position, this);
        marker2_sub_ = nh_.subscribe("/kalman_filter/marker2",1,&TestClass::update_marker2_position, this);
        marker1_sub_ = nh_.subscribe("/kalman_filter/marker1",1,&TestClass::update_marker1_position, this);
        
        
        
        // Publishers
        difference_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/matched_points/difference", 1);
        marker_dist_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/test/euclidian_distance_markers", 1);
        
        // services
        serv_move_robot_point_marker_ = nh_.serviceClient<mp_mini_picker::moveToPointMarker>("/robot/MoveToPointMarker");
        serv_move_robot_pose_marker_ = nh_.serviceClient<mp_mini_picker::moveToPoseMarker>("/robot/MoveToPoseMarker");
        serv_move_robot_point_tcp_ = nh_.serviceClient<mp_mini_picker::moveToPointTcp>("/robot/MoveToPointTcp");
        serv_move_robot_pose_tcp_ = nh_.serviceClient<mp_mini_picker::moveToPoseTcp>("/robot/MoveToPoseTcp");
        serv_move_robotQ_ = nh_.serviceClient<mp_mini_picker::moveToQ>("/robot/MoveToQ");
        serv_get_robotQ_ = nh_.serviceClient<mp_mini_picker::currentQ>("/robot/GetCurrentQ");
        serv_stop_matching_ = nh_.serviceClient<match_points::stopMatching>("/match_points/stopMatching");
        serv_get_matched_point_ = nh_.serviceClient<match_points::getNextMatchingPoint>("/match_points/getNextMatchedPoint");
        serv_set_distance_betw_matched_points_ = nh_.serviceClient<match_points::setDistBetwChosenPoints>("/match_points/distanceBetweenAcceptedPoints");
        
        // create our transformation matrix as homogenous
        wTc_estimate = Mat::zeros(4,4,CV_32F);
        wTc_estimate.at<float>(3,3)=1;
        tcpTmarker_estimate = Mat::zeros(4,4,CV_32F);
        tcpTmarker_estimate.at<float>(3,3)=1;

        // Set our seed 
        srand(time(NULL));

        
//         //New home position
        srv_home.request.Q[0]=-0.5745800177203577;
        srv_home.request.Q[1]=-1.7719739119159144;
        srv_home.request.Q[2]=-2.2175000349627894;
        srv_home.request.Q[3]=-0.793243710194723;
        srv_home.request.Q[4]=1.5708764791488647;
        srv_home.request.Q[5]=-0.9832060972796839;
//        srv_home.request.Q[0]=0.24905504286289215;
//        srv_home.request.Q[1]=-1.2299278418170374;
//        srv_home.request.Q[2]=-1.6500452200519007;
//        srv_home.request.Q[3]=-1.8819416205035608;
//        srv_home.request.Q[4]=1.6246130466461182;
//        srv_home.request.Q[5]=-0.15994674364198858;


//        [-0.5745800177203577, -1.7719739119159144, -2.2175000349627894, -0.793243710194723, 1.5708764791488647, -0.9832060972796839]
        
        ros::Time last_publish=ros::Time::now();
        ros::Duration time_between_publishing(5); // Start up time
        ros::Duration time_difference;
        bool command_send=false;
        bool continue_program=false;
        ROS_INFO("Test node moving robot in 5 seconds");
        while(continue_program == false)
        {
            ros::Time current_time=ros::Time::now();
            time_difference=current_time-last_publish;
            if(time_difference>=time_between_publishing)
            {
                last_publish=ros::Time::now();
                if(command_send==false)
                {
                    serv_move_robotQ_.call(srv_home);
                    if (srv_home.response.ok==1)
                    {
                        ROS_INFO("Moving to start position");
                        command_send=true;
                    }
                    else
                    {
                        ROS_INFO("Did not succeed in moving robot to start position");
                    }
                    ROS_INFO("Waiting another 5 seconds");
                }
                else
                {
                    continue_program=true;
                }
            }
            ros::spinOnce();
        }
        wTb = Mat::zeros(4,4,CV_32F);
         wTb = (Mat_<float>(4,4) <<    3.06162e-16, -1.0, 0.0, -0.326,
                                                  1.0, 3.06162e-16, 0.0,  -0.303,
                                                  0.0, 0.0, 1.0, 0.642,
                                                    0.0, 0.0, 0.0, 1.0);
//        cout << wTb << endl;
    }

    ~TestClass()
    {
        //     cv::destroyWindow(OPENCV_WINDOW);
    }

    void calculate_wTc()
    {
        wTc_estimate=find_transformation(wTc_ChosenMatchedPoints);
        cout << "wTc_estimate \n" << wTc_estimate << endl;
    }
    void print_point_pair(message_package::matched_points current)
    {
        cout << current.camera.pose.position.x << " , ";
        cout << current.camera.pose.position.y << " , ";
        cout << current.camera.pose.position.z << endl;
        cout << current.robot.pose.position.x << " , ";
        cout << current.robot.pose.position.y << " , ";
        cout << current.robot.pose.position.z << endl;
    }

    void print_desired_robot_position()
    {
        cout << "Desired robot position is: " << endl;
        cout << desired_robot_position.pose.position.x << " , ";
        cout << desired_robot_position.pose.position.y << " , ";
        cout << desired_robot_position.pose.position.z << endl;

    }



    void update_robot_position(const message_package::currentToolPosition2::ConstPtr msg)
    {
        current_robot_position=*msg;
//        sendTransformTf_PoseStamped(current_robot_position.tcp,"world","marker1_robot");

    }

    void new_chosen_point(const message_package::matched_points::ConstPtr msg)
    {
//        if(wTc_ChosenMatchedPoints.size()<dataset_size)
//        {
            wTc_ChosenMatchedPoints.push_back(*msg);
//            cout << "We have " << wTc_ChosenMatchedPoints.size() << " matched points" << endl;
//        }
        if(getNextMatchingPoint)
        {

//            tcpTmarker_ChosenMatchedPoints.push_back(*msg);
//            cout << "We have reached " << tcpTmarker_ChosenMatchedPoints.size() << " out of "<<wTc_ChosenMatchedPoints.size()<< " matched points" << endl;
            getNextMatchingPoint=false;
            gotMatchedPoint=true;
        }

    }
    int count=0;


    
    bool got_matching_point()
    {
        return gotMatchedPoint;
    }

    void get_a_matching_point()
    {
        getNextMatchingPoint=true;
        match_points::getNextMatchingPoint serv;
        serv.request.getMatchedPoint=true;
        bool response= false;
        while(response==false)
        {
            serv_get_matched_point_.call(serv);
            if(serv.response.done==true)
            {
                response=true;
            }
        }
        cout << "Asked for Matching Point" << endl;

    }

    void start_matching_points()
    {
        match_points::stopMatching srv;
        srv.request.stop=false;
        bool got_response=false;
        while(got_response==false)
        {
            serv_stop_matching_.call(srv);
            if(srv.response.done==true)
                got_response=true;
        }
    }
    void stop_matching_points()
    {
        match_points::stopMatching srv;
        srv.request.stop=true;
        bool got_response=false;
        while(got_response==false)
        {
            serv_stop_matching_.call(srv);
            if(srv.response.done==true)
                got_response=true;
        }
    }

    
    bool robot_at_home_position()
    {
        mp_mini_picker::currentQ srv;
        srv.request.getQ=1;
        if(serv_get_robotQ_.call(srv))
        {
            bool robot_at_position=true;
            for(int i=0; i<6 ; i++)
            {
                if(srv.response.Q[i] >= (srv_home.request.Q[i]-0.009) && srv.response.Q[i] <= (srv_home.request.Q[i]+0.009))
                {
                    // nothing
                }
                else
                {
                    robot_at_position=false;
                }
            }
            if(robot_at_position==true)
            {
                robot_initialized=true;
                return true;
            }
            else
                return false;
        }
        else 
            return false;
    }
    

    
    bool robot_at_asked_position(double allowed_distance=0.005)
    {
        // compare_3D_points checks if the distance is greater than expected_distance. If yes then return true. Since we want these two variables to be close to eachother, then we invert the result. 
        if(compare_3D_points(desired_robot_position.pose,current_robot_position.tcp.pose,allowed_distance) == true)
            return false;
        else {
//             cout << "Accepted point: " << current_robot_position.tcp.pose.position.x << " , " <<  current_robot_position.tcp.pose.position.y << " , " << current_robot_position.tcp.pose.position.z << endl;
            return true;
        }
    }
    void print_current_robot_position()
    {
        cout << "###############################################" << endl;
        cout << "current robot position: " << current_robot_position.tcp.pose.position.x << " , " <<  current_robot_position.tcp.pose.position.y << " , " << current_robot_position.tcp.pose.position.z << endl;
        cout << "desired robot position: " << desired_robot_position.pose.position.x << " , " <<  desired_robot_position.pose.position.y << " , " << desired_robot_position.pose.position.z << endl;
        cout << "###############################################" << endl;
    }
    bool robot_at_asked_pose(double ok_trans_error, double ok_rot_error)
    {
        // poses is inverted compared point-call above.
        if(compare_3D_poses(desired_robot_position.pose,current_robot_position.tcp.pose,ok_trans_error,ok_rot_error))
            return true;
        else
            return false;
    }

    long int counter=0;

    
    bool program_finished()
    {
        return !(move_robot_randomly);
    }
    
    bool wtc_dataset_ready()
    {
        return (wTc_ChosenMatchedPoints.size()>=dataset_size);
    }

    vector<Mat> latest_marker1_position;
    bool get_marker_positions=false;
    int number_marker1_measurements=10;

    void update_marker1_position(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
        marker1_position=*msg;
        Mat cam_marker1 = convert_geomsg_to_mat(marker1_position);
        if(get_marker_positions)
        {
            cout << "Was asked for point" << endl;
            if(latest_marker1_position.size()==number_marker1_measurements-1)
            {
                get_marker_positions=false;
            }
            latest_marker1_position.push_back(cam_marker1);
        }
    }

    void update_marker2_position(const geometry_msgs::PoseStamped::ConstPtr msg) {
        marker2_position = *msg;
    }
    void set_get_marker_positions(bool desired=true)
    {

        get_marker_positions=desired;
        if(desired==false)
        {
            latest_marker1_position.clear();
        }
    }

    bool got_marker1_positions()
    {
        return (number_marker1_measurements<=latest_marker1_position.size());
    }

    Mat calc_marker1_positions_mean()
    {
        Mat average=Mat::zeros(3,1,CV_64F);
        for(Mat measurement:latest_marker1_position)
        {
            Mat current=Mat::zeros(3,1,CV_64F);
            current.at<double>(0,0)=measurement.at<float>(0,0);
            current.at<double>(1,0)=measurement.at<float>(1,0);
            current.at<double>(2,0)=measurement.at<float>(2,0);
            average=average+current;
        }
        int size=latest_marker1_position.size();
        latest_marker1_position.clear();
        return (average/size);
    }


    void move_tcp(Mat direction, double length)
    {
        cout << "Moving tcp with direction: \n" << direction << endl;
        mp_mini_picker::moveToPointTcp serv;
        serv.request.point[0] = direction.at<float>(0, 0)*length+current_robot_position.tcp.pose.position.x;
        serv.request.point[1] = direction.at<float>(1, 0)*length+current_robot_position.tcp.pose.position.y;
        serv.request.point[2] = direction.at<float>(2, 0)*length+current_robot_position.tcp.pose.position.z;
        bool done = false;
        while (done == false) {
            serv_move_robot_point_tcp_.call(serv);
            if (serv.response.ok == true) {
                done = true;
            }
            ros::spinOnce();
        }
        desired_robot_position.pose.position.x = serv.request.point[0];
        desired_robot_position.pose.position.y = serv.request.point[1];
        desired_robot_position.pose.position.z = serv.request.point[2];
        cout << "Has send move tcp command" << endl;
    }

    void move_tcp_absolute(Mat point)
    {
        cout << "Moving tcp to point: \n" << point << endl;
        mp_mini_picker::moveToPointTcp serv;
        serv.request.point[0] = point.at<double>(0, 0);
        serv.request.point[1] = point.at<double>(1, 0);
        serv.request.point[2] = point.at<double>(2, 0);
        bool done = false;
        while (done == false) {
            serv_move_robot_point_tcp_.call(serv);
            if (serv.response.ok == true) {
                done = true;
            }
            ros::spinOnce();
        }
        desired_robot_position.pose.position.x = serv.request.point[0];
        desired_robot_position.pose.position.y = serv.request.point[1];
        desired_robot_position.pose.position.z = serv.request.point[2];
        cout << "Has send move tcp command" << endl;
    }

    Mat calc_wRc(Mat Xtool, Mat Ytool, Mat Ztool, Mat tool_start)
    {
        cout << "#########################################\nCalculating R and its components" << endl;
        Mat Xcam = (Mat_<double>(3, 1) <<  1, 0, 0);
        Mat Ycam = (Mat_<double>(3, 1) <<  0, 1, 0);
        Mat Zcam = (Mat_<double>(3, 1) <<  0, 0, 1);

        Mat Xtool_updated=Xtool-tool_start;
        Mat Xtool_updated_unit=Xtool_updated/norm(Xtool_updated);
        cout << "Xtool_updated: \n" <<Xtool_updated << "\n Length is: " << norm(Xtool_updated) << endl;
//        Mat Ytool_updated=Ytool-tool_start;
        Mat Ytool_updated=Ytool-Xtool;
        Mat Ytool_updated_unit=Ytool_updated/norm(Ytool_updated);
        cout << "Ytool_updated: \n" <<Ytool_updated << "\n Length is: " << norm(Ytool_updated) << endl;
        Mat Ztool_updated=Ztool-Ytool;
//        Mat Ztool_updated=Ztool-tool_start;
        Mat Ztool_updated_unit=Ztool_updated/norm(Ztool_updated);
        cout << "Ztool_updated: \n" <<Ztool_updated << "\n Length is: "  << norm(Ztool_updated) << endl;

        cout << "X_unit * Y_unit: " << dot_product(Xtool_updated_unit,Ytool_updated_unit) << "\nX_unit * Z_unit: " << dot_product(Xtool_updated_unit,Ztool_updated_unit)  << "\nY_unit * Z_unit: " << dot_product(Ytool_updated_unit,Ztool_updated_unit) << endl;

        Mat R = (Mat_<double>(3, 3) <<  dot_product(Xtool_updated_unit,Xcam), dot_product(Xtool_updated_unit,Ycam), dot_product(Xtool_updated_unit,Zcam),
                dot_product(Ytool_updated_unit,Xcam), dot_product(Ytool_updated_unit,Ycam), dot_product(Ytool_updated_unit,Zcam),
                dot_product(Ztool_updated_unit,Xcam), dot_product(Ztool_updated_unit,Ycam), dot_product(Ztool_updated_unit,Zcam));
        cout << "Estimated R: \n" << R << endl;
        cout << "Determinant of rotation matrix is: " << determinant(R) << endl;

        R=R/determinant(R);
        cout << "Normalized estimated R: \n" << R << endl;
        wRc=R;

        return R;
    }

    void change_tcp_pose(double x=0.0,double y=0.0, double z=0.0,double rx=0.0,double ry=0.0, double rz=0.0)
    {
        old_robot_position=current_robot_position;
        mp_mini_picker::moveToPoseTcp serv;
        // Case of constant tcp position and then generating a random rotation
        if(x==0 && y==0 && z==0 && rx==0 && ry==0 && rz==0)
        {

            serv.request.pose[0]=current_robot_position.tcp.pose.position.x;
            serv.request.pose[1]=current_robot_position.tcp.pose.position.y;
            serv.request.pose[2]=current_robot_position.tcp.pose.position.z;

            bool done=false;
            while(done==false)
            {
                Mat EAA_vector=generate_random_vector(M_PI/(2*9));
                Mat EAA_R;
                Rodrigues(EAA_vector,EAA_R);
                EAA_vector.at<double>(0,0)=current_robot_position.tcp.pose.orientation.x;
                EAA_vector.at<double>(1,0)=current_robot_position.tcp.pose.orientation.y;
                EAA_vector.at<double>(2,0)=current_robot_position.tcp.pose.orientation.z;
                Mat current_R;
                Rodrigues(EAA_vector,current_R);
                Mat R=current_R*EAA_R;
                Rodrigues(R,EAA_vector);

                serv.request.pose[3]=EAA_vector.at<double>(0,0);
                serv.request.pose[4]=EAA_vector.at<double>(1,0);
                serv.request.pose[5]=EAA_vector.at<double>(2,0);
                serv_move_robot_pose_tcp_.call(serv);
                if(serv.response.ok==true)
                {
                    done=true;
                }
            }
        }
        else if(x==0 && y==0 && z==0)
        {
            serv.request.pose[0]=current_robot_position.tcp.pose.position.x;
            serv.request.pose[1]=current_robot_position.tcp.pose.position.y;
            serv.request.pose[2]=current_robot_position.tcp.pose.position.z;

            bool done=false;
            while(done==false)
            {
                Mat EAA_vector=Mat::zeros(3,1,CV_64F);
                EAA_vector.at<double>(0,0)=rx;
                EAA_vector.at<double>(1,0)=ry;
                EAA_vector.at<double>(2,0)=rz;
                Mat EAA_R;
                Rodrigues(EAA_vector,EAA_R);
                EAA_vector.at<double>(0,0)=current_robot_position.tcp.pose.orientation.x;
                EAA_vector.at<double>(1,0)=current_robot_position.tcp.pose.orientation.y;
                EAA_vector.at<double>(2,0)=current_robot_position.tcp.pose.orientation.z;
                Mat current_R;
                Rodrigues(EAA_vector,current_R);
                Mat R=current_R*EAA_R;
                Rodrigues(R,EAA_vector);

                serv.request.pose[3]=EAA_vector.at<double>(0,0);
                serv.request.pose[4]=EAA_vector.at<double>(1,0);
                serv.request.pose[5]=EAA_vector.at<double>(2,0);
                serv_move_robot_pose_tcp_.call(serv);
                if(serv.response.ok==true)
                {
                    done=true;
                }
            }

        }
        else
        {
            // case where x y z is absolute position, while rx, ry, rz is dR
        }
        desired_robot_position.pose.position.x=serv.request.pose[0];
        desired_robot_position.pose.position.y=serv.request.pose[1];
        desired_robot_position.pose.position.z=serv.request.pose[2];
        desired_robot_position.pose.orientation.x=serv.request.pose[3];
        desired_robot_position.pose.orientation.y=serv.request.pose[4];
        desired_robot_position.pose.orientation.z=serv.request.pose[5];

    }

    void go_to_old_position()
    {
        mp_mini_picker::moveToPoseTcp serv;
        serv.request.pose[0]=old_robot_position.tcp.pose.position.x;
        serv.request.pose[1]=old_robot_position.tcp.pose.position.y;
        serv.request.pose[2]=old_robot_position.tcp.pose.position.z;
        serv.request.pose[3]=old_robot_position.tcp.pose.orientation.x;
        serv.request.pose[4]=old_robot_position.tcp.pose.orientation.y;
        serv.request.pose[5]=old_robot_position.tcp.pose.orientation.z;

        bool done=false;
        while(done==false)
        {
            serv_move_robot_pose_tcp_.call(serv);
            if(serv.response.ok==true)
            {
                done=true;
            }
        }

        desired_robot_position.pose.position.x=serv.request.pose[0];
        desired_robot_position.pose.position.y=serv.request.pose[1];
        desired_robot_position.pose.position.z=serv.request.pose[2];
        desired_robot_position.pose.orientation.x=serv.request.pose[3];
        desired_robot_position.pose.orientation.y=serv.request.pose[4];
        desired_robot_position.pose.orientation.z=serv.request.pose[5];
    }

    Mat calculate_wtc(Mat sphere)
    {
        Mat wPtcp=Mat::zeros(3,1,CV_64F), camPtcp=Mat::zeros(3,1,CV_64F);
        wPtcp.at<double>(0,0)=current_robot_position.tcp.pose.position.x;
        wPtcp.at<double>(1,0)=current_robot_position.tcp.pose.position.y;
        wPtcp.at<double>(2,0)=current_robot_position.tcp.pose.position.z;

        camPtcp.at<double>(0,0)=sphere.at<double>(0,0);
        camPtcp.at<double>(1,0)=sphere.at<double>(1,0);
        camPtcp.at<double>(2,0)=sphere.at<double>(2,0);

        wtc=-wRc*camPtcp+wPtcp;
        return wtc;
    }

    Mat estimate_tcpTmarker()
    {

        cout << "############################################\n Estimaing tcpTMarker" <<endl;
//        Mat wPtcp=Mat::zeros(3,1,CV_32F);
//        wPtcp.at<float>(0,0)=current_robot_position.tcp.pose.position.x;
//        wPtcp.at<float>(1,0)=current_robot_position.tcp.pose.position.y;
//        wPtcp.at<float>(2,0)=current_robot_position.tcp.pose.position.z;
//        find_transformation()
        // Make wTc:
        wTc=Mat::eye(4,4,CV_32F);
        wRc.copyTo(wTc(Rect(0,0,wRc.cols,wRc.rows)));
        wTc.at<float>(0,3)=(float)wtc.at<double>(0,0);
        wTc.at<float>(1,3)=(float)wtc.at<double>(1,0);
        wTc.at<float>(2,3)=(float)wtc.at<double>(2,0);

        cout << "wTc is: \n" << wTc << endl;

        Mat wTtcp=convert_geomsg_to_trans(current_robot_position.wTtcp);
        cout << "wTtcp is: \n" << wTtcp << endl;
        Mat tcpTw=inverse_T(wTtcp);
        cout << "tcpTw is: \n" << tcpTw << endl;
        Mat bTw=inverse_T(wTb);
        cout << "bTw: \n" << bTw << endl;


//         Mat wPtcp=convert_geomsg_to_hommat(current_robot_position.tcp);
//         cout << "wPtcp is: \n" << wPtcp << endl;
//         Mat wRtcp=convert_geomsg_to_R(current_robot_position.tcp);
//         cout << "wRtcp is: \n" << wRtcp << endl;
        Mat camPmarker=convert_geomsg_to_hommat(marker1_position);
        cout << "camPmarker is: \n" << camPmarker << endl;
        Mat camRmarker=convert_geomsg_to_R(marker1_position);
        cout << "camRmarker is: \n" << camRmarker << endl;

        // convert camera coordinate first:
        Mat tcpPmarker=tcpTw*wTc*camPmarker;
        Mat tcpRmarker=tcpTw(Rect(0,0,3,3))*wTc(Rect(0,0,3,3))*camRmarker;

        cout << "Translation is: \n" << tcpPmarker << endl;
        cout << "Rotation is: \n" << tcpRmarker << endl;
        cout << "Determinant of rotation is: " << determinant(tcpRmarker);

        tcpTmarker=Mat::eye(4,4,CV_32F);
        tcpRmarker.copyTo(tcpTmarker(Rect(0,0,3,3)));
        tcpTmarker.at<float>(0,3)=tcpPmarker.at<float>(0,0);
        tcpTmarker.at<float>(1,3)=tcpPmarker.at<float>(1,0);
        tcpTmarker.at<float>(2,3)=tcpPmarker.at<float>(2,0);

        return tcpTmarker;


    }

    void move_to_marker2()
    {
        Mat cam_marker2=convert_geomsg_to_hommat(marker2_position);
        cout << "Marker 2 position seen from cam: \n" << cam_marker2 << endl;
        Mat wPmarker2=wTc*cam_marker2;
        cout << "Marker 2 position seen from world: \n" << cam_marker2 << endl;
        Mat displacement=(Mat_<float>(4,1) <<  0.2, 0.2, 0.0, 0.0 );
//         Mat displacement=(Mat_<float>(4,1) <<  0.0, 0.0, 0.25, 0.0 );
        wPmarker2=wPmarker2+displacement;
        cout << "Desired location in world: \n" << wPmarker2 << endl;


        Mat cRmarker1=convert_geomsg_to_R(marker1_position);
        Mat cRmarker2=convert_geomsg_to_R(marker2_position);
        Mat marker1Rc;
        transpose(cRmarker1,marker1Rc);
        Mat marker1Rmarker2=marker1Rc*cRmarker2;
        cout << "1R2 is: \n" << marker1Rmarker2 << endl;

        Mat tcpRmarker=tcpTmarker(Rect(0,0,3,3)),markerRtcp,tcpRvecmarker;
        Rodrigues(tcpRmarker,tcpRvecmarker);
        transpose(tcpRmarker,markerRtcp);
        Mat wRtcp=convert_geomsg_to_R(current_robot_position.tcp);
        Mat wRmarker=wRtcp*tcpRmarker*marker1Rmarker2,wRvecmarker;
        Rodrigues(wRmarker,wRvecmarker);

        mp_mini_picker::moveToPoseMarker serv;
        // Setting pose
        serv.request.pose[0]=wPmarker2.at<float>(0,0);
        serv.request.pose[1]=wPmarker2.at<float>(1,0);
        serv.request.pose[2]=wPmarker2.at<float>(2,0);
        serv.request.pose[3]=wRvecmarker.at<float>(0,0);
        serv.request.pose[4]=wRvecmarker.at<float>(1,0);
        serv.request.pose[5]=wRvecmarker.at<float>(2,0);
        cout << "Pose: \n" << serv.request.pose[0] << " , "<< serv.request.pose[1] << " , "<< serv.request.pose[2] << " , "<< serv.request.pose[3]<< " , " << serv.request.pose[4] << " , " << serv.request.pose[5] << endl;
        // Setting R vector for transformation
        serv.request.tcpRmarker[0]=tcpRvecmarker.at<float>(0,0);
        serv.request.tcpRmarker[1]=tcpRvecmarker.at<float>(1,0);
        serv.request.tcpRmarker[2]=tcpRvecmarker.at<float>(2,0);
        cout << "Rotation: \n " << serv.request.tcpRmarker[0] << " , " << serv.request.tcpRmarker[1] << " , " << serv.request.tcpRmarker[2] << endl;
        // setting P for transformation
        serv.request.tcpPmarker[0]=tcpTmarker.at<float>(0,3);
        serv.request.tcpPmarker[1]=tcpTmarker.at<float>(1,3);
        serv.request.tcpPmarker[2]=tcpTmarker.at<float>(2,3);
        cout << "translation: \n " << serv.request.tcpPmarker[0] << " , " << serv.request.tcpPmarker[1] << " , " << serv.request.tcpPmarker[2] << endl;

        bool done= false;
//        while (done ==false)
//        {
            serv_move_robot_pose_marker_.call(serv);
            if(serv.response.ok==true)
            {
                done=true;
            } else
            {
                cout << "This is not working for some reason" << endl;
            }
//        }
        cout << "ASKED FOR MOVE" << endl;
    }

private:

    bool got_trans_mat=false,robot_initialized=false;
    
    bool gotMatchedPoint=false;
    
    bool move_robot_randomly=true,getNextMatchingPoint=false;
    
    geometry_msgs::PoseStamped desired_robot_position,marker2_position,marker1_position;
    message_package::currentToolPosition2 current_robot_position,old_robot_position;

    vector<message_package::matched_points> wTc_ChosenMatchedPoints,tcpTmarker_ChosenMatchedPoints;

//    message_package::matched_points latest_desired_matched_point;

    mp_mini_picker::moveToQ srv_home;

    Mat wTc_estimate,tcpTmarker_estimate;

    int dataset_size = 30;

    Mat wTb,wRc,wtc,tcpTmarker,wTc;

};

int main(int argc, char** argv)
{
    bool use_incremental_correction=true;
    ros::init(argc, argv, "test matched points");
    TestClass ic(use_incremental_correction);
    ROS_INFO("Starting test node up");
    
    vector<Mat> sphere_coordinates;


    Mat x_world,y_world,z_world,tool_start, wRc;
    bool first_iteration=true, found_wRc=false, asked_for_points=false;
    int find_wRc_round=0;
    double length=0.1;
    int total_number_of_tcp_rotations=15;

    bool wTc_dataset_collected=false,tcpTmarker_dataset_collected=false, wTc_calculated=false, changed_pose=false, got_sphere=false;

    ros::Time last_publish=ros::Time::now();
    ros::Duration start_sequence(3);
    ros::Duration debug_sequence(10);
    ros::Duration time_inverval_after_transformation_is_settled(0.5);
    
    ros::Duration time_difference;

    ros::Duration debug_time_difference;
    ros::Time debug_last_publish;
    while(true)
    {
        if(first_iteration) {
            while (!ic.robot_at_home_position()) {
                // wait
                usleep(100000);
                ros::spinOnce();
            }

            Mat point=Mat::zeros(3,1,CV_64F);
            point.at<double>(2,0)=0.765;
            ic.move_tcp_absolute(point);
            while (!ic.robot_at_asked_position()) {
                // wait
                usleep(100000);
                ros::spinOnce();
            }

            ic.set_get_marker_positions();
            asked_for_points=true;
            usleep(1000000); // half a seconds
            first_iteration = false;
            cout << "out of first itereation" << endl;
        }
        if(ic.got_marker1_positions() && asked_for_points==true && found_wRc==false)
        {
            cout << "Round: " <<find_wRc_round << " In calculating mean part" << endl;
            if (find_wRc_round == 0) {
                cout << "####################################\n " << "Calculating tool start" << endl;
                tool_start = ic.calc_marker1_positions_mean();
                cout << "Tool start is: \n" << tool_start << endl;
                ic.move_tcp(tool_start,0.0); // dummy to desired robot location updated
                ic.print_desired_robot_position();
            } else if (find_wRc_round == 2) {
                cout << "####################################\n " << "Calculating x" << endl;
                x_world = ic.calc_marker1_positions_mean();
                cout << "x_world is: \n" << x_world << endl;
            } else if (find_wRc_round == 4) {
                cout << "####################################\n " << "Calculating y" << endl;
                y_world = ic.calc_marker1_positions_mean();
                cout << "y_world is: \n" << y_world << endl;
            } else if (find_wRc_round == 6) {
                cout << "####################################\n " << "Calculating z" << endl;
                z_world = ic.calc_marker1_positions_mean();
                cout << "z_world is: \n" << y_world << endl;
                wRc = ic.calc_wRc(x_world, y_world, z_world, tool_start);
                found_wRc = true;
                ic.change_tcp_pose();
                changed_pose=true;
//                last_publish=ros::Time::now();
            }
            asked_for_points = false;
            cout << "Following case: " << found_wRc << asked_for_points << endl;
        }
        if(found_wRc==false && asked_for_points==false)
        {
            if(ic.robot_at_asked_position(0.0001))
            {
                cout << "sleeping" << endl;
                usleep(500000); // half a seconds
                cout << "At asked position .. round is: " << find_wRc_round <<endl;
                Mat direction;
                switch(find_wRc_round) {
                    case 0:
                        cout << "########################################\n Case X direction" << endl;
                        direction=(Mat_<float>(3, 1) << 1.0, 0.0, 0.0);
                        ic.move_tcp(direction,length);
                        ic.print_desired_robot_position();
                        break;
                    case 1:
                        cout << "########################################\n Case X I'm there" << endl;
                        ic.set_get_marker_positions();
                        asked_for_points=true;
                        break;
                    case 2:
                        cout << "########################################\n Case y direction" << endl;
                        direction=(Mat_<float>(3, 1) << 0.0, 1.0, 0.0);
                        ic.move_tcp(direction,length);
                        ic.print_desired_robot_position();
                        break;
                    case 3:
                        ic.set_get_marker_positions();
                        asked_for_points=true;
                        break;
                    case 4:
                        direction=(Mat_<float>(3, 1) << 0.0, 0.0, 1.0);
                        ic.move_tcp(direction,length);
                        ic.print_desired_robot_position();
                        break;
                    case 5:
                        ic.set_get_marker_positions();
                        asked_for_points=true;
                        break;
                }

                find_wRc_round++;
            }
        }
        if(ic.got_marker1_positions() && found_wRc==true && asked_for_points==true)
        {
            sphere_coordinates.push_back(ic.calc_marker1_positions_mean());
            asked_for_points=false;
        }
        if(found_wRc==true && asked_for_points==false)
        {
            if(ic.robot_at_asked_pose(0.0001,M_PI/(2*90)))
            {
                if(changed_pose==true)
                {
                    cout << "Asking for cam coordinates: " << endl;
                    ic.set_get_marker_positions(true);
                    last_publish=ros::Time::now();
                    asked_for_points=true;
                    changed_pose=false;
                }
                else if(changed_pose==false)
                {
                    if(sphere_coordinates.size()!=total_number_of_tcp_rotations){
                        cout << "#################################\n" <<"New tcp pose number: " << sphere_coordinates.size() << endl;
                        ic.change_tcp_pose();

                        changed_pose=true;
                    } else if(sphere_coordinates.size()==total_number_of_tcp_rotations && got_sphere==false)
                    {
                        for(int i=0 ; i< sphere_coordinates.size() ; i++)
                        {
                            cout << "####################### iteration " << i << " #################"<< endl;
                            cout << sphere_coordinates[i] << endl;
                        }
                        Mat sphere=fit_sphere(sphere_coordinates);
                        cout << "Our sphere is: \n" << sphere << endl;
                        got_sphere=true;
                        cout << "Our translation is: \n" << ic.calculate_wtc(sphere) << endl;
                        Mat result=ic.estimate_tcpTmarker();
                        cout << "tcpTmarker: \n"<< result << endl;
                        ic.move_to_marker2();
                    }
                }
            }
        }
        if(ros::Time::now()-last_publish>ros::Duration(10) )
        {
            if(found_wRc==true && asked_for_points==true && got_sphere==false) {

                cout << "ALERT !! Not enough coordinates within 5 seconds - going back to old config" << endl;
                changed_pose = false;
                asked_for_points = false;
                ic.set_get_marker_positions(false);
                ic.go_to_old_position();
                last_publish=ros::Time::now();
                cout << "Should have sent it back to old pose" << endl;
            }
        }
        ros::spinOnce();
        
    }

    return 0;
}

