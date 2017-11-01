


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
  
  ros::Publisher difference_pub_;
  
  
  

public:
    TestClass()
    {
        // Subscrive to input video feed and publish output video feed
        transformation_sub_ = nh_.subscribe("/matched_points/transformation_matrix", 1,&TestClass::update_tranformation,this);
        matched_points_sub_ = nh_.subscribe("/matched_points/all_matched_points", 1, &TestClass::newest_point, this);
        
        difference_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/matched_points/difference", 1);
        trans_mat = Mat::zeros(4,4,CV_32F);
        trans_mat.at<float>(3,3)=1;
    }

    ~TestClass()
    {
        //     cv::destroyWindow(OPENCV_WINDOW);
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
        
        cout << "Transformation_matrix at test node:" << endl << trans_mat << endl;
        got_trans_mat=true;
        
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

    Mat trans_mat;
    
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test matched points");
  TestClass ic;
  ROS_INFO("Starting test node up");
  ros::spin();
  return 0;
}

