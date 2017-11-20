


#include <string>
#include <sstream>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}


#include <ros/ros.h>
#include <ros/time.h>
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <match_points/matched_points.h>
#include "mp_mini_picker/moveToPoint.h"
#include "mp_mini_picker/moveToQ.h"

#include <stdlib.h>
#include <stdio.h>
#include <time.h>


using namespace std;
using namespace cv;


// struct matchedPoint {
//   geometry_msgs::PoseStamped camera;
//   geometry_msgs::PoseStamped robot;
// } ;

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
  
  ros::ServiceClient serv_move_robot_;
  
  ros::ServiceClient serv_move_robotQ_;

public:
    TestClass()
    {
        // Subscrive to input video feed and publish output video feed
        transformation_sub_ = nh_.subscribe("/matched_points/transformation_matrix", 1,&TestClass::update_tranformation,this);
        matched_points_sub_ = nh_.subscribe("/matched_points/all_matched_points", 1, &TestClass::newest_point, this);
        robot_position_sub_ = nh_.subscribe("/robot/moved", 1, &TestClass::update_robot_position, this);
        marker2_sub_ = nh_.subscribe("/kalman_filter/marker2",1,&TestClass::update_marker2_position, this);
        marker1_sub_ = nh_.subscribe("/kalman_filter/marker1",1,&TestClass::update_marker1_position, this);
        
        
        serv_move_robot_ = nh_.serviceClient<mp_mini_picker::moveToPoint>("/robot/MoveToPoint");
        serv_move_robotQ_ = nh_.serviceClient<mp_mini_picker::moveToQ>("/robot/MoveToQ");
        
        difference_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/matched_points/difference", 1);
        
        marker_dist_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/test/euclidian_distance_markers", 1);
        
        trans_mat = Mat::zeros(4,4,CV_32F);
        trans_mat.at<float>(3,3)=1;
        
        srand(time(NULL));
        
        
        // Set robot to starting position
        mp_mini_picker::moveToQ srv;
//         srv.request.Q[0]=-0.11449700990785772;
//         srv.request.Q[1]=-1.161553208027975;
//         srv.request.Q[2]=-1.9493358770953577;
//         srv.request.Q[3]=-0.8611777464496058;
//         srv.request.Q[4]=2.149153470993042;
//         srv.request.Q[5]=-0.33657676378359014;
        srv.request.Q[0]=-0.227210823689596;
        srv.request.Q[1]=-1.2569144407855433;
        srv.request.Q[2]=-1.6682723204242151;
        srv.request.Q[3]=-1.3121197859393519;
        srv.request.Q[4]=2.0358359813690186;
        srv.request.Q[5]=-0.44079381624330694;
        
    
       
        ros::Time last_publish=ros::Time::now();
        ros::Duration time_between_publishing(10); // Start up time
        ros::Duration time_difference;
        bool command_send=false;
        ROS_INFO("Test node moving robot in 10 seconds");
        while(command_send == false)
        {
            ros::Time current_time=ros::Time::now();
            time_difference=current_time-last_publish;
            if(time_difference>=time_between_publishing)
            {
                if (serv_move_robotQ_.call(srv))
                {
                    ROS_INFO("Moving to start position");
                    command_send=true;
                }
                else
                {
                    ROS_INFO("Did not succeed in moving robot to start position");
                }
            }
            ros::spinOnce();
        }
        
        
    }

    ~TestClass()
    {
        //     cv::destroyWindow(OPENCV_WINDOW);
    }
    
    void update_marker2_position(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
        marker2_position=*msg;
    }
    
    void update_marker1_position(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
        marker1_position=*msg;
        
        Mat cam_marker1 = convert_geomsg_to_mat(marker1_position);
        Mat cam_marker2 = convert_geomsg_to_mat(marker2_position);
        
        geometry_msgs::PoseStamped dist_msg;
        dist_msg.header.stamp=ros::Time::now();
        dist_msg.pose.position.x=norm(cam_marker2-cam_marker1);
        marker_dist_pub_.publish(dist_msg);
        
    }
    
    void update_robot_position(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
        current_robot_position=*msg;
    }
    
    void newest_point(const match_points::matched_points::ConstPtr msg)
    {
        Mat cam = convert_geomsg_to_hommat((msg->camera));
        Mat robot = convert_geomsg_to_hommat((msg->robot));
        
        if(got_trans_mat==true)
        {
            Mat rob_cam = trans_mat*cam;
            Mat result = robot-rob_cam;
            
            geometry_msgs::PoseStamped msg;
            
            msg.pose.position.x=result.at<float>(0,0);
            msg.pose.position.y=result.at<float>(1,0);
            msg.pose.position.z=result.at<float>(2,0);
            
            difference_pub_.publish(msg);
            
        }
    }
    
    void update_tranformation(const geometry_msgs::Transform::ConstPtr msg)
    {
        cout << "###############Test node updating its transformation matrix #################### " << endl;
        
        trans_mat_copy=trans_mat.clone();
        
        Mat R;
        Mat R_vector=Mat::zeros(3,1,CV_32F);
        
        R_vector.at<float>(0,0)=msg->rotation.x;
        R_vector.at<float>(1,0)=msg->rotation.y;
        R_vector.at<float>(2,0)=msg->rotation.z;
        
        Rodrigues(R_vector,R);
        
        R.copyTo(trans_mat(Rect(0,0,R.cols,R.rows)));
        trans_mat.at<float>(0,3)=msg->translation.x;
        trans_mat.at<float>(1,3)=msg->translation.y;
        trans_mat.at<float>(2,3)=msg->translation.z;
        
        //cout << "Transformation_matrix at test node:" << endl << trans_mat << endl;
        got_trans_mat=true;
        
        
        Mat correct_estimate = (Mat_<float>(4,1) <<    0.229208,0.368607,0.84233, 1);
        Mat cam_marker2 = convert_geomsg_to_hommat(marker2_position);
        Mat rob_marker2 = trans_mat*cam_marker2;
        
        if( norm(correct_estimate-rob_marker2) < 0.1 )
        {
            cout << "Estimate of marker 2 position in world frame using current transformation matrix is within 10 cm of correct result" << endl;
        }
        
        Mat cam_marker1 = convert_geomsg_to_hommat(marker1_position);
        Mat rob_marker1 = trans_mat*cam_marker1;
        
        cout << "World coordinate of marker 1" << endl << rob_marker1 << endl;
        cout << "World coordinate of marker 2" << endl << rob_marker2 << endl;
        cout << "Euclidian distance between the two markers: " << norm(rob_marker2-rob_marker1) << endl;
        

        
        cout << "Change in translation compared to last iteration: " << norm(trans_mat.col(3)-trans_mat_copy.col(3)) << " - current transformation matrix is: " << endl << trans_mat << endl;
        
        if(trans_mat_copy.cols == 4 && trans_mat_copy.rows == 4 && trans_mat.cols == 4 && trans_mat.rows == 4 && move_robot_randomly == true)
        {
            //ROS_INFO("Have a transformation_matrix");
            bool significant_difference=false;
            for(int i = 0 ; i<4 ; i++)
            {
                for(int j=0 ; j<4 ; j++)
                {
                    double comparison=abs((trans_mat.at<float>(i,j)-trans_mat_copy.at<float>(i,j))/trans_mat_copy.at<float>(i,j));
                    if(comparison>0.10)
                    {
//                         cout << i << "," << j << " : " << trans_mat.at<float>(i,j) << " , " << trans_mat_copy.at<float>(i,j) << endl;
                        significant_difference = true;
                    }
                }
            }
            if(significant_difference==true)
            {
                ROS_INFO("Got rejected due to too large change compared to last iteration");
            }
            else
            {
                ROS_INFO("Less than 10 percent change for each element compared to last iteration \n ");
                //move_robot_randomly=false;
                
                mp_mini_picker::moveToPoint srv;
                
                Mat marker2_cam = convert_geomsg_to_hommat(marker2_position);
                Mat rob_cam = trans_mat*marker2_cam;
                
                srv.request.point[0]=(double)(rob_cam.at<float>(0,0));
                srv.request.point[1]=(double)(rob_cam.at<float>(1,0));
                srv.request.point[2]=(double)(rob_cam.at<float>(2,0));
                cout << "Calculated position of Marker 2 in world frame: "  <<srv.request.point[0]  << " , " << srv.request.point[1]  << " , " << srv.request.point[2]  << endl;
                
//                 if (serv_move_robot_.call(srv))
//                 {
//                     ROS_INFO ("Succesful call to robot");
//                 }
//                 else
//                 {
//                     ROS_INFO("Houston we got a problem");
//                 }
            }
            
            cout << "############### Test node finished updating #################### " << endl;
            
        }
        
        
    }
    
    
    Mat convert_geomsg_to_mat(geometry_msgs::PoseStamped msg)
    {
        Mat point=Mat::zeros(3,1,CV_32F);
        point.at<float>(0,0)=(float)msg.pose.position.x;
        point.at<float>(1,0)=(float)msg.pose.position.y;
        point.at<float>(2,0)=(float)msg.pose.position.z;
        
        return point;
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
    
    void move_robot()
    {
        if(move_robot_randomly==true)
        {
             
            int x_dir=rand()%100+1;
            int y_dir=rand()%100+1;
            int z_dir=rand()%100+1;
            
            double reduc_fac=sqrt(pow(x_dir,2)+pow(y_dir,2)+pow(z_dir,2));
//             cout << x_dir << " , " << y_dir << " , "<< z_dir  << endl; 
//             cout << "Factor is: " << reduc_fac << endl;
            
            double cor_x_dir = 5.1*((double)x_dir)/reduc_fac/100;
            double cor_y_dir = 5.1*((double)y_dir)/reduc_fac/100;
            double cor_z_dir = 5.1*((double)z_dir)/reduc_fac/100;
            
            double cor_reduc_fac=sqrt(pow(cor_x_dir,2)+pow(cor_y_dir,2)+pow(cor_z_dir,2));
//             cout << cor_x_dir << " , " << cor_y_dir << " , "<< cor_z_dir  << endl; 
//             cout << "Factor is: " << cor_reduc_fac << endl;
            
            mp_mini_picker::moveToPoint srv;
            if (rand()%2==1)
                srv.request.point[0]=current_robot_position.pose.position.x+cor_x_dir;
            else
                srv.request.point[0]=current_robot_position.pose.position.x-cor_x_dir;
            if (rand()%2==1)
                srv.request.point[1]=current_robot_position.pose.position.y+cor_y_dir;
            else
                srv.request.point[1]=current_robot_position.pose.position.y-cor_y_dir;
            if (rand()%2==1)
                srv.request.point[2]=current_robot_position.pose.position.z+cor_z_dir;
            else
                srv.request.point[2]=current_robot_position.pose.position.z-cor_z_dir;
            
            
//             cout << "Current robot position: " << current_robot_position.pose.position.x << " , " << current_robot_position.pose.position.y << " , " << current_robot_position.pose.position.z  << endl;
//             cout << "Requested position: "  <<srv.request.point[0]  << " , " << srv.request.point[1]  << " , " << srv.request.point[2]  << endl;
            
            
//             if (serv_move_robot_.call(srv));
            
//                 cout << "Succesful call" << endl;
            
        }
    }
    
    
private:
    
    bool compare_3D_points(geometry_msgs::Pose new_position, geometry_msgs::Pose old_position, double expected_distance)
    {
        double old_dist=sqrt(pow(old_position.position.x,2)+pow(old_position.position.y,2)+pow(old_position.position.z,2));
        double new_dist=sqrt(pow(new_position.position.x,2)+pow(new_position.position.y,2)+pow(new_position.position.z,2));
        
        if(abs(new_dist-old_dist)>=expected_distance)
            return true;
        else 
            return false;
    }
    double dist_between_points(geometry_msgs::Pose new_position, geometry_msgs::Pose old_position)
    {
        double old_dist=sqrt(pow(old_position.position.x,2)+pow(old_position.position.y,2)+pow(old_position.position.z,2));
        double new_dist=sqrt(pow(new_position.position.x,2)+pow(new_position.position.y,2)+pow(new_position.position.z,2));
        return abs(new_dist-old_dist);
    }
    bool got_trans_mat=false;

    Mat trans_mat,trans_mat_copy;
    
    bool move_robot_randomly=true;
    
    geometry_msgs::PoseStamped current_robot_position,marker2_position,marker1_position;
    
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test matched points");
    TestClass ic;
    ROS_INFO("Starting test node up");
    
    ros::Time last_publish=ros::Time::now();
    ros::Duration time_between_publishing(6); // camera has framerate of 7 hz
    ros::Duration time_difference;
    while(true)
    {
        ros::Time current_time=ros::Time::now();
        time_difference=current_time-last_publish;
        if(time_difference>=time_between_publishing)
        {
    //             ROS_INFO("I'm inside");
            last_publish=ros::Time::now();
             ic.move_robot();
        }
        
        ros::spinOnce();
    }

    return 0;
}

