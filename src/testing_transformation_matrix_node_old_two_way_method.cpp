


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
#include "mp_mini_picker/moveToPoint.h"
#include "mp_mini_picker/moveToPose.h"
#include "mp_mini_picker/moveToQ.h"
#include "mp_mini_picker/currentQ.h"

#include "match_points/stopMatching.h"
#include "match_points/getNextMatchingPoint.h"
#include "match_points/setDistBetwChosenPoints.h"

#include <stdlib.h>
#include <stdio.h>
#include <time.h>

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

    ros::ServiceClient serv_move_robot_point_;

    ros::ServiceClient serv_move_robot_pose_;
    
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
//        marker2_sub_ = nh_.subscribe("/kalman_filter/marker2",1,&TestClass::update_marker2_position, this);
        marker1_sub_ = nh_.subscribe("/kalman_filter/marker1",1,&TestClass::update_marker1_position, this);
        
        
        
        // Publishers
        difference_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/matched_points/difference", 1);
        marker_dist_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/test/euclidian_distance_markers", 1);
        
        // services
        serv_move_robot_point_ = nh_.serviceClient<mp_mini_picker::moveToPoint>("/robot/MoveToPoint");
        serv_move_robot_pose_ = nh_.serviceClient<mp_mini_picker::moveToPose>("/robot/MoveToPose");
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
        srv_home.request.Q[0]=0.24905504286289215;
        srv_home.request.Q[1]=-1.2299278418170374;
        srv_home.request.Q[2]=-1.6500452200519007;
        srv_home.request.Q[3]=-1.8819416205035608;
        srv_home.request.Q[4]=1.6246130466461182;
        srv_home.request.Q[5]=-0.15994674364198858;
        
        
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

    void calculate_tcpTmarker()
    {
////          cout << "Calculation tcpTmarker" << endl;
//         vector<message_package::matched_points> tcpTmarker_copy(tcpTmarker_ChosenMatchedPoints);
////         for(int i=0; i<tcpTmarker_ChosenMatchedPoints.size() ; i++)
////            tcpTmarker_copy.push_back(tcpTmarker_ChosenMatchedPoints[i]);
//         cout << tcpTmarker_copy.size() << endl;
//
//        cout << tcpTmarker_copy[0].camera << endl;
//
//        for(int i=0 ; i<tcpTmarker_copy.size() ; i++)
//        {
//            cout << "######################### iteration " << i << " #######" << endl;
//            Mat cPmarker=convert_geomsg_to_hommat(tcpTmarker_copy[i].camera);
////            cout << cPmarker << endl;
//            Mat wPmarker=wTc_estimate*cPmarker;
////            cout << wPmarker << endl;
//            tcpTmarker_copy[i].camera.pose.position.x=wPmarker.at<float>(0,0);
//            tcpTmarker_copy[i].camera.pose.position.y=wPmarker.at<float>(1,0);
//            tcpTmarker_copy[i].camera.pose.position.z=wPmarker.at<float>(2,0);
//            print_point_pair(tcpTmarker_copy[i]);
////            cout << tcpTmarker_copy[i] << endl;
//        }
//        cout << tcpTmarker_copy[0].camera << endl;
//        cout << "Her er jeg" << endl;
//        tcpTmarker_estimate=find_transformation(tcpTmarker_copy);
//        cout << "tcpTmarker_estimate \n" << tcpTmarker_estimate << endl;


        cout << "Calculation tcpTmarker" << endl;
        vector<message_package::matched_points> tcpTmarker_copy(tcpTmarker_ChosenMatchedPoints);
//         for(int i=0; i<tcpTmarker_ChosenMatchedPoints.size() ; i++)
//            tcpTmarker_copy.push_back(tcpTmarker_ChosenMatchedPoints[i]);
        cout << tcpTmarker_copy.size() << endl;

        cout << tcpTmarker_copy[0].camera << endl;

        for(int i=0 ; i<tcpTmarker_copy.size() ; i++)
        {
            cout << "######################### iteration " << i << " #######" << endl;
            Mat cPmarker=convert_geomsg_to_hommat(tcpTmarker_copy[i].camera);
//            cout << cPmarker << endl;
            Mat tcpPmarker=inverse_T(convert_geomsg_to_trans(tcpTmarker_copy[i].wTtcp))*inverse_T(wTb)*wTc_estimate*cPmarker;
//            cout << wPmarker << endl;
            tcpTmarker_copy[i].camera.pose.position.x=tcpPmarker.at<float>(0,0);
            tcpTmarker_copy[i].camera.pose.position.y=tcpPmarker.at<float>(1,0);
            tcpTmarker_copy[i].camera.pose.position.z=tcpPmarker.at<float>(2,0);

            Mat wPtcp=convert_geomsg_to_hommat(tcpTmarker_copy[i].robot);
            Mat tcpPtcp=inverse_T(convert_geomsg_to_trans(tcpTmarker_copy[i].wTtcp))*inverse_T(wTb)*wPtcp;
            tcpTmarker_copy[i].robot.pose.position.x=tcpPtcp.at<float>(0,0);
            tcpTmarker_copy[i].robot.pose.position.y=tcpPtcp.at<float>(1,0);
            tcpTmarker_copy[i].robot.pose.position.z=tcpPtcp.at<float>(2,0);

            print_point_pair(tcpTmarker_copy[i]);
//            cout << tcpTmarker_copy[i] << endl;
        }
        cout << tcpTmarker_copy[0].camera << endl;
//        cout << "Her er jeg" << endl;
        tcpTmarker_estimate=find_transformation(tcpTmarker_copy);
        cout << "tcpTmarker_estimate \n" << tcpTmarker_estimate << endl;

//         cout << "Før " << endl;
//         tcpTmarker_estimate=find_transformation(tcpTmarker_copy);
//         cout << "efter " << endl;
//         cout << "tcpTmarker_estimate \n" << tcpTmarker_estimate << endl;
//         vector<message_package::matched_points> wTc_MatchedPoints_copy;
//         cout << " Done " << endl;
        
        
//        wTc_MatchedPoints_copy=wTc_ChosenMatchedPoints;
//        // take robot positions and add newly found transformation to them and transform them back to world
//        Mat markerT
//        for(int i=0 ; i<wTc_MatchedPoints_copy.size() ; i++)
//        {
//            Mat wPtcp=convert_geomsg_to_hommat(wTc_MatchedPoints_copy[i].robot);
//            Mat wTtcp=convert_geomsg_to_trans(wTc_MatchedPoints_copy[i].wTtcp);
//            Mat tcpTw=inverse_T(wTtcp);
//            Mat Ptcp=tcpTw*wPtcp;
//
//
//        }
    }
    
    void update_marker1_position(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
        marker1_position=*msg;
        Mat cam_marker1 = convert_geomsg_to_mat(marker1_position);
        Mat cam_marker2 = convert_geomsg_to_mat(marker2_position);
        
    }

    void update_robot_position(const message_package::currentToolPosition2::ConstPtr msg)
    {
        current_robot_position=*msg;
//        sendTransformTf_PoseStamped(current_robot_position.tcp,"world","marker1_robot");

    }
    
    void new_chosen_point(const message_package::matched_points::ConstPtr msg)
    {
        if(wTc_ChosenMatchedPoints.size()<dataset_size)
        {
            wTc_ChosenMatchedPoints.push_back(*msg);
            cout << "We have " << wTc_ChosenMatchedPoints.size() << " matched points" << endl;
        }
        else if(getNextMatchingPoint)
        {
            tcpTmarker_ChosenMatchedPoints.push_back(*msg);
            cout << "We have reached " << tcpTmarker_ChosenMatchedPoints.size() << " out of "<<wTc_ChosenMatchedPoints.size()<< " matched points" << endl;
            getNextMatchingPoint=false;
            gotMatchedPoint=true;
        }

    }
    int count=0;
    bool move_to_next_point_wTc_dataset()
    {
        if (count < dataset_size)
        {
            Mat cPmarker = convert_geomsg_to_hommat(wTc_ChosenMatchedPoints[count].camera);
            count++;
            Mat wPmarker = wTc_estimate * cPmarker;
            mp_mini_picker::moveToPoint serv;
            serv.request.point[0] = wPmarker.at<float>(0, 0);
            serv.request.point[1] = wPmarker.at<float>(1, 0);
            serv.request.point[2] = wPmarker.at<float>(2, 0);
            bool done = false;
            while (done == false) {
                serv_move_robot_point_.call(serv);
                if (serv.response.ok == true) {
                    done = true;
                }
            }
            desired_robot_position.pose.position.x = serv.request.point[0];
            desired_robot_position.pose.position.y = serv.request.point[1];
            desired_robot_position.pose.position.z = serv.request.point[2];
            gotMatchedPoint=false;
            return true;
        } else
            return false;


    }

    
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
    

    
    bool robot_at_asked_position()
    {
        // compare_3D_points checks if the distance is greater than expected_distance. If yes then return true. Since we want these two variables to be close to eachother, then we invert the result. 
        if(compare_3D_points(desired_robot_position.pose,current_robot_position.tcp.pose,0.005) == true)
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
    void move_robot_random_direction()
    {
        srand(time(&counter));
        counter++;
        if(move_robot_randomly==true)
        {
             
            mp_mini_picker::moveToPoint srv=generate_random_point(6.5);
//             if (serv_move_robot_point_.call(srv));
            bool got_valid_point=false;
            while(got_valid_point==false)
            {
//                 cout <<"Jeg laver nyt kald (y)";
                srv=generate_random_point(6.5);
                serv_move_robot_point_.call(srv);
                if(srv.response.ok==1)
                {
//                      cout << "Jeg blev accepteret" << endl;
                    got_valid_point=true;
                }
            }
            desired_robot_position.pose.position.x=srv.request.point[0];
            desired_robot_position.pose.position.y=srv.request.point[1];
            desired_robot_position.pose.position.z=srv.request.point[2];

//            cout << "New desired is: " << srv.request.point[0] << " , " << srv.request.point[1] << " , " << srv.request.point[2] << endl;
//             cout << "####################### \n GOT NEW DESIRED" << endl;
//             print_current_robot_position();
        }
    }
    
    mp_mini_picker::moveToPoint generate_random_point(double distance)
    {
        int x_dir=rand()%100+1;
        int y_dir=rand()%100+1;
        int z_dir=rand()%100+1;
        
        double reduc_fac=sqrt(pow(x_dir,2)+pow(y_dir,2)+pow(z_dir,2));        
        double cor_x_dir = distance*((double)x_dir)/reduc_fac/100;
        double cor_y_dir = distance*((double)y_dir)/reduc_fac/100;
        double cor_z_dir = distance*((double)z_dir)/reduc_fac/100;
        
        double cor_reduc_fac=sqrt(pow(cor_x_dir,2)+pow(cor_y_dir,2)+pow(cor_z_dir,2));
        
        mp_mini_picker::moveToPoint srv;
        if (rand()%2==1)
            srv.request.point[0]=current_robot_position.tcp.pose.position.x+cor_x_dir;
        else
            srv.request.point[0]=current_robot_position.tcp.pose.position.x-cor_x_dir;
        if (rand()%2==1)
            srv.request.point[1]=current_robot_position.tcp.pose.position.y+cor_y_dir;
        else
            srv.request.point[1]=current_robot_position.tcp.pose.position.y-cor_y_dir;
        if (rand()%2==1)
            srv.request.point[2]=current_robot_position.tcp.pose.position.z+cor_z_dir;
        else
            srv.request.point[2]=current_robot_position.tcp.pose.position.z-cor_z_dir;
        
        return srv;
    }
    
    bool program_finished()
    {
        return !(move_robot_randomly);
    }
    
    bool wtc_dataset_ready()
    {
        return (wTc_ChosenMatchedPoints.size()>=dataset_size);
    }
    

    
private:

    bool got_trans_mat=false,robot_initialized=false;
    
    bool gotMatchedPoint=false;
    
    bool move_robot_randomly=true,getNextMatchingPoint=false;
    
    geometry_msgs::PoseStamped desired_robot_position,marker2_position,marker1_position;
    message_package::currentToolPosition2 current_robot_position;

    vector<message_package::matched_points> wTc_ChosenMatchedPoints,tcpTmarker_ChosenMatchedPoints;

    mp_mini_picker::moveToQ srv_home;

    Mat wTc_estimate,tcpTmarker_estimate;

    int dataset_size = 30;

    Mat wTb;

};

int main(int argc, char** argv)
{
    bool use_incremental_correction=true;
    ros::init(argc, argv, "test matched points");
    TestClass ic(use_incremental_correction);
    ROS_INFO("Starting test node up");
    

    
    bool first_iteration=true;
    bool wTc_dataset_collected=false,tcpTmarker_dataset_collected=false, wTc_calculated=false;

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
            ic.start_matching_points();
            usleep(1000000); // half a seconds
            ic.move_robot_random_direction();
            first_iteration = false;
        }

        if(ic.wtc_dataset_ready()&& wTc_calculated==false)
        {
            ic.stop_matching_points();
            wTc_dataset_collected=true;
            ic.calculate_wTc();
            wTc_calculated=true;
            // ensure random move is done
            while (!ic.robot_at_asked_position()) {
                // wait
                usleep(100000);
                ros::spinOnce();
            }
            // Make move towards first location of list
            ic.move_to_next_point_wTc_dataset();
        }


        ros::Time current_time=ros::Time::now();
        time_difference=current_time-last_publish;
        debug_time_difference=current_time-debug_last_publish;
        if(time_difference>=start_sequence && wTc_dataset_collected==false)
        {
            if(ic.robot_at_asked_position())
            {
                ic.move_robot_random_direction();
                last_publish=ros::Time::now();
            }
        }
        else if(time_difference>=start_sequence && wTc_dataset_collected==true && tcpTmarker_dataset_collected==false)
        {
            if(ic.robot_at_asked_position())
            {
                ic.get_a_matching_point();
                while(ic.got_matching_point()==false)
                {
                    ros::spinOnce();
                }
                bool delivered_new_request=ic.move_to_next_point_wTc_dataset();
                if(delivered_new_request)
                {
                    last_publish=ros::Time::now();
                } else{
                    ic.calculate_tcpTmarker();
                    tcpTmarker_dataset_collected=true;
                }
            }
        }
        if(tcpTmarker_dataset_collected && wTc_dataset_collected)
        {
//            cout << "JEG GEMMER mig i main-loop" << endl;
        }
        
        ros::spinOnce();
        
    }

    return 0;
}

