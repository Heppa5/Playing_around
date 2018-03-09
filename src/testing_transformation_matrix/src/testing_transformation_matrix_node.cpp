


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
#include "mp_mini_picker/changeTcpTMarker.h"

#include "match_points/stopMatching.h"
#include "match_points/getNextMatchingPoint.h"
#include "match_points/setDistBetwChosenPoints.h"

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <random>

#include <helping_functions.h>
#include <find_gradient.h>

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

    ros::Subscriber robot_marker_position_sub_;

    ros::Publisher difference_pub_;

    ros::Publisher marker_dist_pub_;

    ros::Publisher markerW_1_pub_;
    ros::Publisher markerW_2_pub_;



    ros::ServiceClient serv_move_robot_point_tcp_;

    ros::ServiceClient serv_move_robot_pose_tcp_;

    ros::ServiceClient serv_move_robot_point_marker_;

    ros::ServiceClient serv_move_robot_pose_marker_;

    ros::ServiceClient serv_change_tcpTmarker_;

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
        robot_marker_position_sub_= nh_.subscribe("/robot/moved_marker", 1, &TestClass::update_robot_marker_position, this);
        marker2_sub_ = nh_.subscribe("/kalman_filter/marker2",1,&TestClass::update_marker2_position, this);
        marker1_sub_ = nh_.subscribe("/kalman_filter/marker1",1,&TestClass::update_marker1_position, this);
        
        
        
        // Publishers
        difference_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/matched_points/difference", 1);
        markerW_1_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/wTmarker1", 1);
        markerW_2_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/wTmarker2", 1);

        
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
        serv_change_tcpTmarker_ = nh_.serviceClient<mp_mini_picker::changeTcpTMarker>("/robot/tcpTmarker");



        // Set our seed 
        srand(time(NULL));

        
//         //New home position
//        srv_home.request.Q[0]=-0.5745800177203577;
//        srv_home.request.Q[1]=-1.7719739119159144;
//        srv_home.request.Q[2]=-2.2175000349627894;
//        srv_home.request.Q[3]=-0.793243710194723;
//        srv_home.request.Q[4]=1.5708764791488647;
//        srv_home.request.Q[5]=-0.9832060972796839;
        
        srv_home.request.Q[0]=-0.5358255545245569;
        srv_home.request.Q[1]=-2.1341140905963343;
        srv_home.request.Q[2]=-1.8766534964190882;
        srv_home.request.Q[3]=-0.8037985006915491;
        srv_home.request.Q[4]=1.554266333580017;
        srv_home.request.Q[5]=-0.9420283476458948;
//         srv_home.request.Q[0]=-32.0/180.0*M_PI;
//         srv_home.request.Q[1]=-130.0/180.0*M_PI;
//         srv_home.request.Q[2]=-96.0/180.0*M_PI;
//         srv_home.request.Q[3]=-40.0/180.0*M_PI;
//         srv_home.request.Q[4]=59.0/180.0*M_PI;
//         srv_home.request.Q[5]=-50.0/180.0*M_PI;

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
//        wTb = Mat::zeros(4,4,CV_32F);
//         wTb = (Mat_<float>(4,4) <<    3.06162e-16, -1.0, 0.0, -0.326,
//                                                  1.0, 3.06162e-16, 0.0,  -0.303,
//                                                  0.0, 0.0, 1.0, 0.642,
//                                                    0.0, 0.0, 0.0, 1.0);
////        _2R1 = (Mat_<float>(3,3) << -0.071901843, 0.99607193, 0.05167985,
////                                    0.030538231, 0.053988129, -0.99807453,
////                                    -0.99694413, -0.070185184, -0.034300119);
//        _2R1 = (Mat_<float>(3,3) << -0.10985642, 0.98899734, 0.0990749,
//                                    -0.02960752, 0.096377879, -0.9949044,
//                                    -0.99350643, -0.11223002, 0.018694013);


    }

    ~TestClass()
    {
        //     cv::destroyWindow(OPENCV_WINDOW);
    }

    void get_all_matching_points(bool boolean=true)
    {
        if(boolean==true)
        {
            match_points::setDistBetwChosenPoints serv;
            serv.request.dist=0.0;
            serv_set_distance_betw_matched_points_.call(serv);
            match_points::stopMatching serv2;
            serv2.request.stop=false;
            serv_stop_matching_.call(serv2);
        } else{
            // back to normal - 2 cm distance and no matching
            match_points::setDistBetwChosenPoints serv;
            serv.request.dist=0.02;
            serv_set_distance_betw_matched_points_.call(serv);
            match_points::stopMatching serv2;
            serv2.request.stop=true;
            serv_stop_matching_.call(serv2);
        }
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
    

//    bool have_matching_point=false;
    void update_robot_position(const message_package::currentToolPosition2::ConstPtr msg)
    {
        current_robot_position=*msg;
        
        Mat wTtcp=convert_geomsg_to_trans(current_robot_position.tcp);
        Mat tcpTmarker_correct = (Mat_<float>(4, 4) <<  0, 0, 1, 0.0325,
                                                        0, -1, 0, -0.097,
                                                        1, 0, 0, 0.064,
                                                        0, 0, 0, 1);
        Mat result=wTtcp*tcpTmarker_correct;
        sendTransformTf(result(Rect(3,0,1,3)),result(Rect(0,0,3,3)),"world","marker1_robot");
//         cout << "#####################################" << endl;
//         cout << "wTtcp \n" << wTtcp << endl;
//         cout << "wTmarker\n" << result << endl;
    
//        sendTransformTf_PoseStamped(current_robot_position.tcp,"world","marker1_robot");
//        test_jesper.wTtcp=current_robot_position.tcp;
//        if(test_jesper.camera.pose.position.x!=99999)
//        {
//
////            cout << test_jesper.camera.pose.position.x << " , " << test_jesper.camera.pose.position.y << " , " << test_jesper.camera.pose.position.z << " , ";
////            cout << test_jesper.camera.pose.orientation.x << " , " << test_jesper.camera.pose.orientation.y << " , " << test_jesper.camera.pose.orientation.z << endl;
////            cout << test_jesper.wTtcp.pose.position.x << " , " << test_jesper.wTtcp.pose.position.y << " , " << test_jesper.wTtcp.pose.position.z << " , ";
////            cout << test_jesper.wTtcp.pose.orientation.x << " , " << test_jesper.wTtcp.pose.orientation.y << " , " << test_jesper.wTtcp.pose.orientation.z << endl;
////            test_jesper.camera.pose.position.x=99999;
//        }

    }

    void update_robot_marker_position(const message_package::currentToolPosition2::ConstPtr msg)
    {
        current_robot_marker_position=*msg;
        sendTransformTf_PoseStamped(current_robot_position.tcp,"world","marker1_robot_est");

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
//        cout << "Asked for Matching Point" << endl;

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
    bool robot_at_asked_pose(double ok_trans_error, double ok_rot_error, bool tcp=true)
    {
        if(tcp==true) {
            // poses is inverted compared point-call above.
            if (compare_3D_poses(desired_robot_position.pose, current_robot_position.tcp.pose, ok_trans_error,
                                 ok_rot_error))
                return true;
            else
                return false;
        } else{
            // poses is inverted compared point-call above.
            if (compare_3D_poses(desired_robot_position.pose, current_robot_marker_position.tcp.pose, ok_trans_error,
                                 ok_rot_error))
                return true;
            else
                return false;
        }
    }

    long int counter=0;

    


    vector<Mat> latest_marker1_position;
    bool get_marker_positions=false;
    int number_marker1_measurements=10;

    bool got_marker1=false;


    void update_marker1_position(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
        got_marker1=true;
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
        if(dataset_size==31) {
            geometry_msgs::PoseStamped wow;
            Mat cPmarker1 = convert_geomsg_to_hommat(marker1_position);
            Mat cRmarker1 = convert_geomsg_to_R(marker1_position);
            Mat wPmarker1=wTc*cPmarker1;
            Mat wRmarker1=wTc(Rect(0,0,3,3))*cRmarker1, wRmarker1_vec;
            Rodrigues(wRmarker1,wRmarker1_vec);
            wow.pose.position.x=wPmarker1.at<float>(0,0);
            wow.pose.position.y=wPmarker1.at<float>(1,0);
            wow.pose.position.z=wPmarker1.at<float>(2,0);
            wow.pose.orientation.x=wRmarker1_vec.at<float>(0,0);
            wow.pose.orientation.y=wRmarker1_vec.at<float>(1,0);
            wow.pose.orientation.z=wRmarker1_vec.at<float>(2,0);
            wow.header.stamp=ros::Time::now();
            markerW_1_pub_.publish(wow);

            //sendTransformTf_PoseStamped(marker1_position,"cam", "Marker1");
            Mat cTmarker=convert_geomsg_to_trans(marker1_position);
            /*cout << "¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤" << endl;
            cout << "wTmarker from cam\n " << wTc*cTmarker << endl*/;
            sendTransformTf(cTmarker(Rect(3,0,1,3)),cTmarker(Rect(0,0,3,3)),"cam","Marker1_cam");
            sendTransformTf(wTc(Rect(3,0,1,3)),wTc(Rect(0,0,3,3)),"world","cam");

        }

//        markerW_2_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/wTmarker2", 1);


        saw_marker1=true;
    }

    void update_marker2_position(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
        marker2_position = *msg;
        saw_marker2=true;


        if(dataset_size==31) {
            geometry_msgs::PoseStamped wow;
            Mat cPmarker2 = convert_geomsg_to_hommat(marker2_position);
            Mat cRmarker2 = convert_geomsg_to_R(marker2_position);
            Mat wPmarker2=wTc*cPmarker2;
            Mat wRmarker2=wTc(Rect(0,0,3,3))*cRmarker2, wRmarker2_vec;
            Rodrigues(wRmarker2,wRmarker2_vec);
            wow.pose.position.x=wPmarker2.at<float>(0,0);
            wow.pose.position.y=wPmarker2.at<float>(1,0);
            wow.pose.position.z=wPmarker2.at<float>(2,0);
            wow.pose.orientation.x=wRmarker2_vec.at<float>(0,0);
            wow.pose.orientation.y=wRmarker2_vec.at<float>(1,0);
            wow.pose.orientation.z=wRmarker2_vec.at<float>(2,0);
            wow.header.stamp=ros::Time::now();
            markerW_2_pub_.publish(wow);
            sendTransformTf_PoseStamped(marker2_position,"cam", "Marker2");
            sendTransformTf(wTc(Rect(3,0,1,3)),wTc(Rect(0,0,3,3)),"world","cam");

        }

//        if(got_marker1==true)
//        {
//            Mat R2=convert_geomsg_to_R(marker2_position);
//            Mat R1=convert_geomsg_to_R(marker1_position);
//            Mat _R1,_2R;
////            transpose(R1,_R1);
//            transpose(R2,_2R);
//            Mat _2R1=_2R*R1;
//            cout << _2R1 << endl;
//        }
        
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

    vector<Mat> return_marker1_list()
    {
        vector<Mat> copy;
        for(Mat measurement:latest_marker1_position)
        {
            Mat wow=Mat::zeros(3,1,CV_64F);
            wow.at<double>(0,0)=measurement.at<float>(0,0);
            wow.at<double>(1,0)=measurement.at<float>(1,0);
            wow.at<double>(2,0)=measurement.at<float>(2,0);

            copy.push_back(wow);
        }
        latest_marker1_position.clear();
        return copy;
    }


    bool move_tcp(Mat direction, double length)
    {
//        cout << "Moving tcp with direction: \n" << direction << endl;
        mp_mini_picker::moveToPointTcp serv;
        serv.request.point[0] = direction.at<float>(0, 0)*length+current_robot_position.tcp.pose.position.x;
        serv.request.point[1] = direction.at<float>(1, 0)*length+current_robot_position.tcp.pose.position.y;
        serv.request.point[2] = direction.at<float>(2, 0)*length+current_robot_position.tcp.pose.position.z;
        bool done = false;
//        while (done == false) {
            serv_move_robot_point_tcp_.call(serv);
//            if (serv.response.ok == true) {
//                done = true;
//            }
//            ros::spinOnce();
//        }
        desired_robot_position.pose.position.x = serv.request.point[0];
        desired_robot_position.pose.position.y = serv.request.point[1];
        desired_robot_position.pose.position.z = serv.request.point[2];
//        cout << "Has send move tcp command" << endl;
        return serv.response.ok;
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

    Mat estimate_tcpTmarker() {

        cout << "############################################\n Estimaing tcpTMarker" << endl;
//        Mat wPtcp=Mat::zeros(3,1,CV_32F);
//        wPtcp.at<float>(0,0)=current_robot_position.tcp.pose.position.x;
//        wPtcp.at<float>(1,0)=current_robot_position.tcp.pose.position.y;
//        wPtcp.at<float>(2,0)=current_robot_position.tcp.pose.position.z;
//        find_transformation()
        // Make wTc:
        wTc = Mat::eye(4, 4, CV_32F);
        wRc=wRc/determinant(wRc);
        wRc.copyTo(wTc(Rect(0, 0, wRc.cols, wRc.rows)));
        wTc.at<float>(0, 3) = (float) wtc.at<double>(0, 0);
        wTc.at<float>(1, 3) = (float) wtc.at<double>(1, 0);
        wTc.at<float>(2, 3) = (float) wtc.at<double>(2, 0);



        cout << "wTc is: \n" << wTc << endl;

        Mat wTtcp = convert_geomsg_to_trans(current_robot_position.wTtcp);
        cout << "wTtcp is: \n" << wTtcp << endl;
        Mat tcpTw = inverse_T(wTtcp);
        cout << "tcpTw is: \n" << tcpTw << endl;
        Mat bTw = inverse_T(wTb);
        cout << "bTw: \n" << bTw << endl;


//         Mat wPtcp=convert_geomsg_to_hommat(current_robot_position.tcp);
//         cout << "wPtcp is: \n" << wPtcp << endl;
//         Mat wRtcp=convert_geomsg_to_R(current_robot_position.tcp);
//         cout << "wRtcp is: \n" << wRtcp << endl;
        Mat camPmarker = convert_geomsg_to_hommat(marker1_position);
        cout << "camPmarker is: \n" << camPmarker << endl;
        Mat camRmarker = convert_geomsg_to_R(marker1_position);
        cout << "camRmarker is: \n" << camRmarker << endl;

        // convert camera coordinate first:
        Mat tcpPmarker = tcpTw * wTc * camPmarker;
        Mat tcpRmarker = tcpTw(Rect(0, 0, 3, 3)) * wTc(Rect(0, 0, 3, 3)) * camRmarker;

        cout << "Translation is: \n" << tcpPmarker << endl;
        cout << "Rotation is: \n" << tcpRmarker << endl;
        cout << "Determinant of rotation is: " << determinant(tcpRmarker);

        tcpTmarker = Mat::eye(4, 4, CV_32F);
        tcpRmarker.copyTo(tcpTmarker(Rect(0, 0, 3, 3)));
        tcpTmarker.at<float>(0, 3) = tcpPmarker.at<float>(0, 0);
        tcpTmarker.at<float>(1, 3) = tcpPmarker.at<float>(1, 0);
        tcpTmarker.at<float>(2, 3) = tcpPmarker.at<float>(2, 0);



        // Update transformation in mp_mini_picker
        mp_mini_picker::changeTcpTMarker serv;
        Mat tcpRvecmarker;
        Rodrigues(tcpRmarker, tcpRvecmarker);
        // Setting R vector for transformation
        serv.request.tcpRmarker[0] = tcpRvecmarker.at<float>(0, 0);
        serv.request.tcpRmarker[1] = tcpRvecmarker.at<float>(1, 0);
        serv.request.tcpRmarker[2] = tcpRvecmarker.at<float>(2, 0);
        // setting P for transformation
        serv.request.tcpPmarker[0] = tcpTmarker.at<float>(0, 3);
        serv.request.tcpPmarker[1] = tcpTmarker.at<float>(1, 3);
        serv.request.tcpPmarker[2] = tcpTmarker.at<float>(2, 3);

        bool done = false;
//        while (done ==false)
//        {
            serv_change_tcpTmarker_.call(serv);
            if (serv.response.ok == true) {
                done = true;
            } else {
                cout << "We could not change the transformation" << endl;
            }
//        }



        dataset_size=31; // Signal that we can send commands now. # bad
        return tcpTmarker;


    }

    void move_to_marker2()
    {
        Mat cam_marker2=convert_geomsg_to_hommat(marker2_position);
        cout << "Marker 2 position seen from cam: \n" << cam_marker2 << endl;
        Mat wPmarker2=wTc*cam_marker2;
        cout << "Marker 2 position seen from world: \n" << wPmarker2 << endl;
        Mat displacement=(Mat_<float>(4,1) <<  0, 0, 0.25, 0.0 );
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

        cout << "wTc is: \n" << wTc << endl;
        cout << "tcpRmarker is: \n" << tcpTmarker << endl;


    }

    Mat find_rotation_error_between_markers(double error_correction=0.1)
    {
        // find desired pose and dR
        Mat cR2=convert_geomsg_to_R(marker2_position);
        _2R1=roll(-M_PI/2); // Hard coded offset in current setup
        Mat cRdesired=cR2*_2R1;
//        Mat cRdesired=cR2;
        Mat cR1=convert_geomsg_to_R(marker1_position);
        Mat _1Rc;
        transpose(cR1,_1Rc);
        Mat _1Rdesired=_1Rc*cRdesired;

        // Reduce rotation error with error correction variable
        Mat _1Rdesired_vec;
        Rodrigues(_1Rdesired,_1Rdesired_vec);
//        cout << "_1Rdesired_vec is: \n" << _1Rdesired_vec << endl;
//        cout << "_1Rdesired_vec*error_correction is: \n" << _1Rdesired_vec*error_correction << endl;
        _1Rdesired_vec=_1Rdesired_vec*error_correction;
        Rodrigues(_1Rdesired_vec,_1Rdesired);
        return _1Rdesired;
    }

    Mat calc_wPtool_tip()
    {
        Mat cTmarker1=convert_geomsg_to_trans(marker1_position); // given as a relative transformation
        cout << "CTmarker1: \n" << convert_geomsg_to_trans(marker1_position) <<  endl;
        Mat tip_correction = (Mat_<float>(4,1) << 0.169, -0.021, -0.028, 1.0); // correcting for tip seen from marker 1 frame
//        Mat tip_correction = (Mat_<float>(4,1) << 0.168, -0.025, -0.027, 1.0); // correcting for tip seen from marker 1 frame
//        Mat tip_correction = (Mat_<float>(4,1) << 0, 0, 0, 1.0); // correcting for tip seen from marker 1 frame
        Mat wPtip=wTc*cTmarker1*tip_correction;
        return wPtip;
    }

    Mat calc_positional_iterative_error()
    {
        Mat wPtip=calc_wPtool_tip();
        // correct marker 2

        Mat cTmarker2=convert_geomsg_to_trans(marker2_position);
        Mat corner_in_marker2F = (Mat_<float>(4,1) << -0.006, 0.008, 0, 1);
        Mat correction = (Mat_<float>(4,1) << 0, 0, 0.01, 0.0);
        Mat wPcorner=wTc*cTmarker2*corner_in_marker2F+correction;

        // Find positional error and the desired correction of that error
        Mat positional_error=(wPcorner-wPtip);
        return positional_error;
    }

    void move_iteratively_to_marker2(double error_correction=0.1)
    {
        cout << "################################################### \nNew move for visual servoing" << endl;
        cout << "Rotation error: " << find_rotation_error_between_markers(error_correction) << endl;
        Mat _1Rdesired=find_rotation_error_between_markers(error_correction);
        // Now correct current rotation with our rotation error
        Mat tcpRmarker=tcpTmarker(Rect(0,0,3,3));
        Mat wRtcp=convert_geomsg_to_R(current_robot_position.tcp);
        Mat wRdesired=wRtcp*tcpRmarker*_1Rdesired,wRdesired_vec;
        Rodrigues(wRdesired,wRdesired_vec);
        cout << "wRdesired_vec is: \n" << wRdesired_vec << endl;


        Mat positional_error=calc_positional_iterative_error()*error_correction;

//        Mat positional_error=(wPmarker2-wPtip)*error_correction;
//        Mat positional_error=(wPmarker2-wPtip);


        // why not use the camera coordinate of marker 1 and my transformations?
        Mat marker1Porigin = (Mat_<float>(4,1) << 0.0, 0.0, 0.0, 1.0);
        Mat wTmarker1=convert_geomsg_to_trans(current_robot_position.wTtcp)*tcpTmarker;
        Mat wPmarker1_origin=wTmarker1*marker1Porigin;
        Mat wPmarker1_origin_corrected=wPmarker1_origin+positional_error;
        cout << "wPmarker1_origin: \n" << wPmarker1_origin <<  endl;
        cout << "wPmarker1_origin_corrected - our desired location: \n" << wPmarker1_origin_corrected  <<  endl;


        mp_mini_picker::moveToPoseMarker serv;
        // Setting pose
        serv.request.pose[0]=wPmarker1_origin_corrected.at<float>(0,0);
        serv.request.pose[1]=wPmarker1_origin_corrected.at<float>(1,0);
        serv.request.pose[2]=wPmarker1_origin_corrected.at<float>(2,0);
        serv.request.pose[3]=wRdesired_vec.at<float>(0,0);
        serv.request.pose[4]=wRdesired_vec.at<float>(1,0);
        serv.request.pose[5]=wRdesired_vec.at<float>(2,0);

        // Setting R vector for transformation
        Mat tcpRvecmarker;
        Rodrigues(tcpRmarker,tcpRvecmarker);
        serv.request.tcpRmarker[0]=tcpRvecmarker.at<float>(0,0);
        serv.request.tcpRmarker[1]=tcpRvecmarker.at<float>(1,0);
        serv.request.tcpRmarker[2]=tcpRvecmarker.at<float>(2,0);
        // setting P for transformation
        serv.request.tcpPmarker[0]=tcpTmarker.at<float>(0,3);
        serv.request.tcpPmarker[1]=tcpTmarker.at<float>(1,3);
        serv.request.tcpPmarker[2]=tcpTmarker.at<float>(2,3);


        cout << "wTc is\n" << wTc << endl;
        cout << "tcpTmarker is\n" << tcpTmarker << endl;
        cout << "wRdesired is: \n" << wRdesired << endl;
        cout << "Entire positional error is: " << positional_error/error_correction << endl;
        cout << "Corrected positional error is: " << positional_error << endl;
//        cout << "Tip location: : \n" << wPtip << endl;
//        cout << "Marker 1 location world: \n" << wPmarker1 << endl;

//        while(true)
//        {
////            Mat wowoesa = (Mat_<float>(3,1) << 0.6, 0.8, 0.6);
////            wowoesa.at<float>(0, 0)=wTc.at<float>(0, 3);
////            wowoesa.at<float>(1, 0)=wTc.at<float>(1, 3);
////            wowoesa.at<float>(2, 0)=wTc.at<float>(2, 3);
////
////            sendTransformTf(wowoesa,wTc(Rect(0,0,3,3))/determinant(wTc(Rect(0,0,3,3))), "Camera", "map");
//////            sendTransformTf_PoseStamped(current_robot_marker_position.tcp,"Marker1","map");
////            wPmarker2=wTc*cPmarker2;
////            Mat wRmarker2=wTc(Rect(0,0,3,3))*convert_geomsg_to_R(marker2_position);
////            sendTransformTf(wPmarker2,wRmarker2/determinant(wRmarker2),"Marker2", "map");
////            sendTransformTf(wPmarker1_origin_corrected,_1Rdesired/determinant(_1Rdesired), "Desired pose", "map");
//            ros::spinOnce();
//        }

        bool done= false;
        while(done==false) {
            serv_move_robot_pose_marker_.call(serv);
            if (serv.response.ok == 1) {
                done = true;
            } else if (serv.response.ok == 2) {
                cout << "In collision" << endl;
                while (true) {
                    ros::spinOnce();
                }
            } else if (serv.response.ok == 3) {
                cout << "No solution to jacobian solver" << endl;
                while (true) {
                    ros::spinOnce();
                }
            } else {
                cout << "No error code" << endl;
                while (true) {
                    ros::spinOnce();
                }
            }
        }
//        }

        // change this to TCP view

        desired_robot_position.pose.position.x=serv.request.pose[0];
        desired_robot_position.pose.position.y=serv.request.pose[1];
        desired_robot_position.pose.position.z=serv.request.pose[2];
        desired_robot_position.pose.orientation.x=serv.request.pose[3];
        desired_robot_position.pose.orientation.y=serv.request.pose[4];
        desired_robot_position.pose.orientation.z=serv.request.pose[5];
        cout << "Going out of iteratively loop " << endl;

        // Update transformation in mp_mini_picker
        mp_mini_picker::changeTcpTMarker serv_update_tcpTmarker;
        Mat tcpRmarker_vec;
        Rodrigues(tcpTmarker(Rect(0,0,3,3)), tcpRmarker_vec);
        // Setting R vector for transformation
        serv_update_tcpTmarker.request.tcpRmarker[0] = tcpRmarker_vec.at<float>(0, 0);
        serv_update_tcpTmarker.request.tcpRmarker[1] = tcpRmarker_vec.at<float>(1, 0);
        serv_update_tcpTmarker.request.tcpRmarker[2] = tcpRmarker_vec.at<float>(2, 0);
        // setting P for transformation
        serv_update_tcpTmarker.request.tcpPmarker[0] = tcpTmarker.at<float>(0, 3);
        serv_update_tcpTmarker.request.tcpPmarker[1] = tcpTmarker.at<float>(1, 3);
        serv_update_tcpTmarker.request.tcpPmarker[2] = tcpTmarker.at<float>(2, 3);
        serv_change_tcpTmarker_.call(serv_update_tcpTmarker);
//        while(true)
//        {
//            ros::spinOnce();
//        }
    }

    message_package::currentToolPosition2 get_current_robot_position()
    {
        return current_robot_position;
    }

    bool saw_marker2=false;
    bool seen_marker1()
    {
        if(saw_marker1==true && saw_marker2==true)
        {
            saw_marker1=false;
            return true;
        }
        return false;
    }

    void cheat(Mat wTc_est, Mat tcpTmarker_est)
    {

        wTc=wTc_est;
        tcpTmarker=tcpTmarker_est;

        dataset_size=31; // bad fix for signal .. delete

        // Update transformation in mp_mini_picker
        Mat tcpRmarker=tcpTmarker_est(Rect(0,0,3,3));
        mp_mini_picker::changeTcpTMarker serv;
        Mat tcpRvecmarker;
        Rodrigues(tcpRmarker, tcpRvecmarker);
        // Setting R vector for transformation
        serv.request.tcpRmarker[0] = tcpRvecmarker.at<float>(0, 0);
        serv.request.tcpRmarker[1] = tcpRvecmarker.at<float>(1, 0);
        serv.request.tcpRmarker[2] = tcpRvecmarker.at<float>(2, 0);
        // setting P for transformation
        serv.request.tcpPmarker[0] = tcpTmarker.at<float>(0, 3);
        serv.request.tcpPmarker[1] = tcpTmarker.at<float>(1, 3);
        serv.request.tcpPmarker[2] = tcpTmarker.at<float>(2, 3);

        bool done = false;
//        while (done ==false)
//        {
        serv_change_tcpTmarker_.call(serv);
        if (serv.response.ok == 1) {
            done = true;
        }
        else
        {
            cout << "No error code" << endl;
        }

    }
    void print_dR()
    {
        Mat cR1=convert_geomsg_to_R(marker1_position);
        Mat cR2=convert_geomsg_to_R(marker2_position),_2Rc;
        transpose(cR2,_2Rc);
        Mat result=_2Rc*cR1;
        cout << result << endl;
    }

    void update_transformations()
    {
//        cout << "JEG GÅR IND" << wTc_ChosenMatchedPoints.size() << endl;
        if(wTc_ChosenMatchedPoints.size()!=0) {
            auto latest_point = wTc_ChosenMatchedPoints[wTc_ChosenMatchedPoints.size() - 1];
            wTc_ChosenMatchedPoints.clear();
            Mat wTc_update = Mat::eye(4, 4, CV_32F), tcpTmarker_update = Mat::eye(4, 4, CV_32F);


            Mat cTw = inverse_T(wTc);
            Mat wTt = convert_geomsg_to_trans(latest_point.wTtcp);
            Mat cTmarker = convert_geomsg_to_trans(latest_point.camera);
            // Mat find_gradient_wrapper(Mat wTc_orig, Mat wTt_kth, Mat tcpTmarker_orig, Mat wTc_update, Mat tcpTmarker_update, Mat cTmarker)
            Mat gradient = find_gradient_wrapper(wTc, wTt, tcpTmarker, wTc_update, tcpTmarker_update, cTmarker);
            gradient = -gradient * 0.05; // Correct 5 percent

            wTc_update.at<float>(0, 3) = wTc_update.at<float>(0, 3) + gradient.at<float>(0, 0);
            wTc_update.at<float>(1, 3) = wTc_update.at<float>(1, 3) + gradient.at<float>(1, 0);
            wTc_update.at<float>(2, 3) = wTc_update.at<float>(2, 3) + gradient.at<float>(2, 0);
            Mat wRc_vec = (Mat_<float>(3, 1) << gradient.at<float>(3, 0), gradient.at<float>(4, 0), gradient.at<float>(
                    5, 0));
            Rodrigues(wRc_vec, wTc_update(Rect(0, 0, 3, 3)));
//            // update alpha:
//            wTc_update.at<float>(2,1)=wTc_update.at<float>(2,1)+gradient.at<float>(3,0);
//            wTc_update.at<float>(1,2)=wTc_update.at<float>(1,2)-gradient.at<float>(3,0);
//            // update beta:
//            wTc_update.at<float>(2,0)=wTc_update.at<float>(2,0)-gradient.at<float>(4,0);
//            wTc_update.at<float>(0,2)=wTc_update.at<float>(0,2)+gradient.at<float>(4,0);
//            // update gamma:
//            wTc_update.at<float>(0,1)=wTc_update.at<float>(0,1)-gradient.at<float>(5,0);
//            wTc_update.at<float>(1,0)=wTc_update.at<float>(1,0)+gradient.at<float>(5,0);

            wTc = wTc * wTc_update;

            tcpTmarker_update.at<float>(0, 3) = tcpTmarker_update.at<float>(0, 3) + gradient.at<float>(6, 0);
            tcpTmarker_update.at<float>(1, 3) = tcpTmarker_update.at<float>(1, 3) + gradient.at<float>(7, 0);
            tcpTmarker_update.at<float>(2, 3) = tcpTmarker_update.at<float>(2, 3) + gradient.at<float>(8, 0);

//            // update alpha:
//            tcpTmarker_update.at<float>(2,1)=tcpTmarker_update.at<float>(2,1)+gradient.at<float>(9,0);
//            tcpTmarker_update.at<float>(1,2)=tcpTmarker_update.at<float>(1,2)-gradient.at<float>(9,0);
//            // update beta:
//            tcpTmarker_update.at<float>(2,0)=tcpTmarker_update.at<float>(2,0)-gradient.at<float>(10,0);
//            tcpTmarker_update.at<float>(0,2)=tcpTmarker_update.at<float>(0,2)+gradient.at<float>(10,0);
//            // update gamma:
//            tcpTmarker_update.at<float>(0,1)=tcpTmarker_update.at<float>(0,1)-gradient.at<float>(11,0);
//            tcpTmarker_update.at<float>(1,0)=tcpTmarker_update.at<float>(1,0)+gradient.at<float>(11,0);
            Mat tcpRmarker_vec = (Mat_<float>(3, 1) << gradient.at<float>(9, 0), gradient.at<float>(10,
                                                                                                    0), gradient.at<float>(
                    11, 0));
            Rodrigues(tcpRmarker_vec, tcpTmarker_update(Rect(0, 0, 3, 3)));
            tcpTmarker = tcpTmarker * tcpTmarker_update;


        }
//        // update alpha:
//        tcpTmarker_update.at<float>(2,1)=tcpTmarker_update.at<float>(2,1)+gradient.at<float>(9,0);
//        tcpTmarker_update.at<float>(1,2)=tcpTmarker_update.at<float>(1,2)-gradient.at<float>(9,0);
//        // update beta:
//        tcpTmarker_update.at<float>(2,0)=tcpTmarker_update.at<float>(2,0)-gradient.at<float>(10,0);
//        tcpTmarker_update.at<float>(0,2)=tcpTmarker_update.at<float>(0,2)+gradient.at<float>(10,0);
//        // update gamma:
//        tcpTmarker_update.at<float>(0,1)=tcpTmarker_update.at<float>(0,1)-gradient.at<float>(11,0);
//        tcpTmarker_update.at<float>(1,0)=tcpTmarker_update.at<float>(1,0)+gradient.at<float>(11,0);



//        cout << "cTw_update \n" << cTw_update << endl;
//        cout << "Updated cTw: \n" << cTw*cTw_update << endl;
//        cout << "Updated tcpTmarker: \n" << tcpTmarker*tcpTmarker_update << endl;

//        cout << "tcpTmarker_update is after correction: \n" << tcpTmarker_update << endl;
    }


private:

    bool saw_marker1=false,robot_initialized=false;
    
    bool gotMatchedPoint=false;
    
    bool move_robot_randomly=true,getNextMatchingPoint=false;
    
    geometry_msgs::PoseStamped desired_robot_position,marker2_position,marker1_position;
    message_package::currentToolPosition2 current_robot_position,old_robot_position,current_robot_marker_position ;

    vector<message_package::matched_points> wTc_ChosenMatchedPoints,tcpTmarker_ChosenMatchedPoints;

//    message_package::matched_points latest_desired_matched_point;

    mp_mini_picker::moveToQ srv_home;

    Mat wTc_estimate,tcpTmarker_estimate;

    int dataset_size = 30;

    Mat wTb,wRc,wtc,tcpTmarker,wTc, _2R1;

};

int main(int argc, char** argv)
{
    bool use_incremental_correction=true;
    ros::init(argc, argv, "test matched points");
    TestClass ic(use_incremental_correction);
    ROS_INFO("Starting test node up");
    
    vector<Mat> sphere_coordinates;

    message_package::currentToolPosition2 rotation_start_pose;

    Mat x_world,y_world,z_world,tool_start, wRc;
    bool first_iteration=true, found_wRc=false, asked_for_points=false, first_iteration_angleMinMax=true, positive_direction=false;
    bool first_minimum=true;
    int find_wRc_round=0;
    double length=0.1;
    int total_number_of_tcp_rotations=15;
    int find_minMax_round=0;
    double total_angle=0;
    Mat Rmin,Rmax,Pmin,Pmax,Ymin,Ymax;

    bool wTc_dataset_collected=false,tcpTmarker_dataset_collected=false, wTc_calculated=false, changed_pose=false, got_sphere=false, found_minMax_angles=false, big_error_correction;

    ros::Time last_publish=ros::Time::now();
    ros::Time last_marker1=ros::Time::now();
    ros::Duration start_sequence(3);
    ros::Duration debug_sequence(10);
    ros::Duration time_inverval_after_transformation_is_settled(0.5);
    
    ros::Duration time_difference;

    ros::Duration debug_time_difference;
    ros::Time debug_last_publish;
    bool cheat=true;
    if(cheat)
    {
        Mat wTc = (Mat_<float>(4, 4) << 0.85251993, 0.10858911, -0.50973761, 1.0800163,
                                        0.55342996, -0.10745841, 0.82497406, -1.1997528,
                                        0.033956207, -0.98651284, -0.15509245, 0.062517919,
                                        0, 0, 0, 1);
        
        Mat R=wTc(Rect(0,0,3,3));
        Mat wTc_vec;
        Rodrigues(R,wTc_vec);
        cout << "HERE: \n" << wTc_vec << endl;
        Mat tcpTmarker = (Mat_<float>(4, 4) <<  0.0069673657, -0.027343974, 0.9938972, 0.028517187,
                                                -0.00068785436, -1.0042343, -0.016774893, -0.11282735,
                                                1.0005599, -0.012419891, -0.012510687, 0.063874424,
                                                0, 0, 0, 1);
        // Bad version of original start guess
//        Mat tcpTmarker = (Mat_<float>(4, 4) <<  0.0069673657, -0.027343974, 0.9938972, 0.020517187,
//                                                -0.00068785436, -1.0042343, -0.016774893, -0.07282735,
//                                                1.0005599, -0.012419891, -0.012510687, 0.043874424,
//                                                0, 0, 0, 1);

        ic.cheat(wTc,tcpTmarker);
        while (!ic.robot_at_home_position()) {
            // wait
            usleep(100000);
            ros::spinOnce();
        }

        first_iteration=false;
        found_minMax_angles=false;
        found_wRc = true;
        find_minMax_round=4;

//        ros::Time wow=ros::Time::now();
//        while(ros::Time::now()-wow<ros::Duration(5));
//        {
//            ros::spinOnce();
//        }
//        Mat point=(Mat_<double>(3, 1) << 0.2, 0.2, 1.1);
//        ic.move_tcp_absolute(point);
//        while (!ic.robot_at_asked_position()) {
//            // wait
//            usleep(100000);
//            ros::spinOnce();
//        }
//        while(true)
//        {
//            usleep(500000);
//            ic.print_dR();
//            ros::spinOnce();
//        }

    }

    while(true)
    {
        if(first_iteration) {
            while (!ic.robot_at_home_position()) {
                // wait
                usleep(100000);
                ros::spinOnce();
            }

//            Mat point=Mat::zeros(3,1,CV_64F);
//            point.at<double>(2,0)=0.765;
//            ic.move_tcp_absolute(point);
//            while (!ic.robot_at_asked_position()) {
//                // wait
//                usleep(100000);
//                ros::spinOnce();
//            }

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
            if(ic.robot_at_asked_position(0.001))
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
        if(found_minMax_angles==false && found_wRc == true)
        {
            Mat sphere,result;
            switch (find_minMax_round) {
                case 0:
                    if (first_iteration_angleMinMax == true) {
                        first_iteration_angleMinMax = false;
                        rotation_start_pose = ic.get_current_robot_position();
                        ic.change_tcp_pose(0, 0, 0, -M_PI / 4, 0, 0);
                        last_marker1 = ros::Time::now();
                        cout << "####################################\nStarted rotations in Roll" << endl;
                    }
                    if (ic.robot_at_asked_pose(0.001, M_PI / (4 * 90))) {
//                        Mat direction=(Mat_<float>(3,1) <<M_PI/90,0,0);
                        if (asked_for_points == true && ic.got_marker1_positions()) {
//                            sphere_coordinates.push_back(ic.calc_marker1_positions_mean());
                            vector<Mat> hep = ic.return_marker1_list();
                            sphere_coordinates.insert(sphere_coordinates.end(), hep.begin(), hep.end());
//                            sphere_coordinates.push_back(ic.return_marker1_list());
                            asked_for_points = false;
                            if (total_angle < M_PI / 4) {
                                ic.change_tcp_pose(0, 0, 0, M_PI / 45, 0, 0); // 4 degree
                                total_angle = total_angle + M_PI / 45;
                                cout << "Got a new point and list is " << sphere_coordinates.size() << " long" << endl;
                            } else {
                                find_minMax_round++;
                                first_iteration_angleMinMax = true;
                                total_angle = 0;
                            }
                        } else if (asked_for_points == true && ros::Time::now() - last_marker1 > ros::Duration(6)) {
                            asked_for_points = false;
                            ic.set_get_marker_positions(false);
                            if (total_angle < M_PI / 4) {
                                ic.change_tcp_pose(0, 0, 0, M_PI / 45, 0, 0); // 4 degree
                                total_angle = total_angle + M_PI / 45;
                                cout << "Moved on. Could not get 10 cam points in 6 seconds." << endl;
                            } else {
                                find_minMax_round++;
                                first_iteration_angleMinMax = true;
                                total_angle = 0;
                            }
                        } else if (asked_for_points == false) {
                            ic.set_get_marker_positions();
                            cout << "Asked for marker coordinates" << endl;
                            asked_for_points = true;
                            last_marker1 = ros::Time::now();
                        }
                    }

                    break;
                case 1:
                    if (first_iteration_angleMinMax == true) {
                        first_iteration_angleMinMax = false;
                        rotation_start_pose = ic.get_current_robot_position();
                        ic.change_tcp_pose(0, 0, 0, 0, -M_PI / 4, 0);
                        last_marker1 = ros::Time::now();
                        cout << "####################################\nStarted rotations in Pitch" << endl;
                    }
                    if (ic.robot_at_asked_pose(0.001, M_PI / (4 * 90))) {
//                        Mat direction=(Mat_<float>(3,1) <<M_PI/90,0,0);
                        if (asked_for_points == true && ic.got_marker1_positions()) {
                            vector<Mat> hep = ic.return_marker1_list();
                            sphere_coordinates.insert(sphere_coordinates.end(), hep.begin(), hep.end());
//                            sphere_coordinates.push_back(ic.calc_marker1_positions_mean());
                            asked_for_points = false;
                            if (total_angle < M_PI / 4) {
                                ic.change_tcp_pose(0, 0, 0, 0, M_PI / 45, 0); // 4 degree
                                total_angle = total_angle + M_PI / 45;
                                cout << "Got a new point and list is " << sphere_coordinates.size() << " long" << endl;
                            } else {
                                find_minMax_round++;
                                first_iteration_angleMinMax = true;
                                total_angle = 0;
                            }
                        } else if (asked_for_points == true && ros::Time::now() - last_marker1 > ros::Duration(6)) {
                            asked_for_points = false;
                            ic.set_get_marker_positions(false);
                            if (total_angle < M_PI / 4) {
                                ic.change_tcp_pose(0, 0, 0, 0, M_PI / 45, 0); // 4 degree
                                total_angle = total_angle + M_PI / 45;
                                cout << "Moved on. Could not get 10 cam points in 6 seconds." << endl;
                            } else {
                                find_minMax_round++;
                                first_iteration_angleMinMax = true;
                                total_angle = 0;
                            }
                        } else if (asked_for_points == false) {
                            ic.set_get_marker_positions();
                            cout << "Asked for marker coordinates" << endl;
                            asked_for_points = true;
                            last_marker1 = ros::Time::now();
                        }
                    }
                    break;
                case 2:
                    if (first_iteration_angleMinMax == true) {
                        first_iteration_angleMinMax = false;
                        rotation_start_pose = ic.get_current_robot_position();
                        ic.change_tcp_pose(0, 0, 0, M_PI / 4, 0, 0);
                        last_marker1 = ros::Time::now();
                        cout << "####################################\nStarted rotations in Pitch" << endl;
                    }
                    if (ic.robot_at_asked_pose(0.001, M_PI / (4 * 90))) {
//                        Mat direction=(Mat_<float>(3,1) <<M_PI/90,0,0);
                        if (asked_for_points == true && ic.got_marker1_positions()) {
                            vector<Mat> hep = ic.return_marker1_list();
                            sphere_coordinates.insert(sphere_coordinates.end(), hep.begin(), hep.end());
//                            sphere_coordinates.push_back(ic.calc_marker1_positions_mean());
                            asked_for_points = false;
                            if (total_angle < M_PI / 4) {
                                ic.change_tcp_pose(0, 0, 0, -M_PI / 45, 0, 0); // 4 degree
                                total_angle = total_angle + M_PI / 45;
                                cout << "Got a new point and list is " << sphere_coordinates.size() << " long" << endl;
                            } else {
                                find_minMax_round++;
                                first_iteration_angleMinMax = true;
                                total_angle = 0;
                            }
                        } else if (asked_for_points == true && ros::Time::now() - last_marker1 > ros::Duration(6)) {
                            asked_for_points = false;
                            ic.set_get_marker_positions(false);
                            if (total_angle < M_PI / 4) {
                                ic.change_tcp_pose(0, 0, 0, -M_PI / 45, 0, 0); // 4 degree
                                total_angle = total_angle + M_PI / 45;
                                cout << "Moved on. Could not get 10 cam points in 6 seconds." << endl;
                            } else {
                                find_minMax_round++;
                                first_iteration_angleMinMax = true;
                                total_angle = 0;
                            }
                        } else if (asked_for_points == false) {
                            ic.set_get_marker_positions();
                            cout << "Asked for marker coordinates" << endl;
                            asked_for_points = true;
                            last_marker1 = ros::Time::now();
                        }
                    }
                    break;
                case 3:
                    find_minMax_round++;
                    sphere = fit_sphere(sphere_coordinates);
                    cout << "Our sphere is: \n" << sphere << endl;
                    cout << "Our translation is: \n" << ic.calculate_wtc(sphere) << endl;
                    result = ic.estimate_tcpTmarker();
                    cout << "MEASUREMENTS:" << endl;
//                    for (Mat measurement:sphere_coordinates) {
//                        cout << measurement.at<double>(0, 0) << " , " << measurement.at<double>(1, 0) << " , "
//                             << measurement.at<double>(2, 0) << endl;
//                    }

                    break;
                case 4:
//                    ic.move_to_marker2();
                    ic.get_all_matching_points(true);
                    find_minMax_round++;
                    break;
                case 5:
                    //code for visual servoing:
                    if (ic.robot_at_asked_pose(0.005, M_PI / (90*2),false) && big_error_correction==false || ic.robot_at_asked_pose(0.05, M_PI / (45),false) && big_error_correction==true || first_iteration_angleMinMax == true) {
                        if(ic.seen_marker1())
                        {
//                            usleep(500000);
                            if(norm(ic.calc_positional_iterative_error())>0.05)
                            {
                                ic.move_iteratively_to_marker2(0.35);
                                big_error_correction=true;
                            }
                            else
                            {
                                ic.move_iteratively_to_marker2(0.1);
                                big_error_correction=false;
                            }
                            first_iteration_angleMinMax = false;
                        }
                    }
//                    if (ic.robot_at_asked_position(0.01) || first_iteration_angleMinMax == true) {
////                        if (ic.seen_marker1())
////                        {
//                        usleep(500000);
//                        bool succeeded = false;
////                        ic.print_current_robot_position();
//                        while (succeeded == false) {
//                            Mat direction = generate_random_vector(1.0);
////                            cout << "Direction is: "<< direction << endl;
//                            succeeded = ic.move_tcp(direction, 0.1);
//                        }
//                        first_iteration_angleMinMax = false;
////                        }
//                    }
                    if(ros::Time::now()-last_publish>ros::Duration(0.3))
                    {
                        last_publish=ros::Time::now();
                        ic.update_transformations();
                    }
                    break;

            }

        }



        ros::spinOnce();
        
    }

    return 0;
}

