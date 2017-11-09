


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


struct matchedPoint {
  geometry_msgs::PoseStamped camera;
  geometry_msgs::PoseStamped robot;
} ;

class MatchPoints
{
  ros::NodeHandle nh_;
  ros::Subscriber camera_point_sub_;
  ros::Subscriber robot_sub_;
  
  ros::Publisher transformation_pub_;
  ros::Publisher matched_points_pub_;
  
  

public:
    MatchPoints()
    {
        // Subscrive to input video feed and publish output video feed
        transformation_pub_ = nh_.advertise<geometry_msgs::Transform>("/matched_points/transformation_matrix", 1);
        matched_points_pub_ = nh_.advertise<match_points::matched_points>("/matched_points/all_matched_points", 1);
        
        robot_sub_ = nh_.subscribe("/robot/moved", 1, &MatchPoints::robot_has_moved,this);
        camera_point_sub_ = nh_.subscribe("/aruco/marker1", 1, &MatchPoints::camera_3D_points,this);
        
    }

    ~MatchPoints()
    {
        //     cv::destroyWindow(OPENCV_WINDOW);
    }
    
    void camera_3D_points(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
        camera_points.insert(camera_points.begin(),*msg);
        if(camera_points.size() > 7) // only one second of data is stored
        {
            camera_points.pop_back();
        }
        match_points();
    }
    
    void robot_has_moved(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
        robot_positions.insert(robot_positions.begin(),*msg);
        if(robot_positions.size() > 7) // only one second of data is stored
        {
            robot_positions.pop_back();
        }
        match_points();
    }
    
    void match_points()
    {
//         ROS_INFO("MATCHING POINTS: Number of robot points: %d  -  Camera points: %d ", robot_positions.size(), camera_points.size());
        for( int i = 0 ; i < robot_positions.size() ; i++)
        {
            for( int j = 0 ; j < camera_points.size() ; j++)
            {
                ros::Duration possible_match(0.0714); // 7 Hz * 2 = 14 HZ
                ros::Time low_bound = camera_points[j].header.stamp - possible_match;
                ros::Time high_bound = camera_points[j].header.stamp + possible_match;
                if(robot_positions[i].header.stamp>low_bound && robot_positions[i].header.stamp<high_bound)
                {
//                     cout << camera_points[j].stamp << " ~ " << robot_positions[i].header.stamp << endl;
//                     cout << robot_positions[i].pose << endl;
//                     cout << camera_points[j].point3D << endl;
//                     ROS_INFO("In a match");
                    matchedPoint current;
                    current.camera=camera_points[j];
                    current.robot=robot_positions[i];
                    
                    match_points::matched_points msg;
                    msg.robot=robot_positions[i];
                    msg.camera=camera_points[j];
                    
                    matched_points_pub_.publish(msg);
                    
                    matchedPoints.insert(matchedPoints.begin(),current);
                    
                    camera_points.erase(camera_points.begin()+j);
                    robot_positions.erase(robot_positions.begin()+i);
                }
            }
        }
        if(matchedPoints.size()>0)
        {
//             ROS_INFO("GOING IN");
            choosePoints();
//             ROS_INFO("GOING OUT");
        }
    }
    
    void choosePoints()
    {
        // if no points has been choosen, then just take the first matched point
        if(chosenMatchedPoints.size() == 0)
        {
            chosenMatchedPoints.insert(chosenMatchedPoints.begin(),matchedPoints[0]);
            matchedPoints.erase(matchedPoints.begin());
        }
        int old_array_size=chosenMatchedPoints.size();
        // choosing points based on, whether the robot has moved position 
            while(matchedPoints.size()>0)
            {
//                 ROS_INFO("Hej %f",dist_between_points(matchedPoints[0].robot.pose,chosenMatchedPoints[0].robot.pose));
                int i = matchedPoints.size()-1;
                if(compare_3D_points(matchedPoints[i].robot.pose,chosenMatchedPoints[0].robot.pose,0.05))
                {
                    ROS_INFO("Distance between chosen poinsts:  %f",dist_between_points(matchedPoints[0].robot.pose,chosenMatchedPoints[0].robot.pose));
                    
                    chosenMatchedPoints.insert(chosenMatchedPoints.begin(),matchedPoints[i]);
                    matchedPoints.erase(matchedPoints.end());

                }
                else
                {
//                     float x=chosenMatchedPoints[0].robot.pose.position.x;
//                     float y=chosenMatchedPoints[0].robot.pose.position.y;
//                     float z=chosenMatchedPoints[0].robot.pose.position.z;
//                     
//                     float x2=matchedPoints[0].robot.pose.position.x;
//                     float y2=matchedPoints[0].robot.pose.position.y;
//                     float z2=matchedPoints[0].robot.pose.position.z;
//                     
//                     ROS_INFO("Rej. Dist:  %f and chosen point is: ( %f, %f, %f) \n While compared point is: ( %f, %f, %f) ",dist_between_points(matchedPoints[0].robot.pose,chosenMatchedPoints[0].robot.pose),x,y,z,x2,y2,z2);
                    
                    matchedPoints.erase(matchedPoints.end());
                }
            }
        if(chosenMatchedPoints.size() > old_array_size && chosenMatchedPoints.size() > 3)
        {
//             for(int i = 0 ; i < chosenMatchedPoints.size() ; i++)
//             {
//                 cout << "Chosen point " << i << ": ";
//                 cout << chosenMatchedPoints[i].robot.pose.position.x << " , ";
//                 cout << chosenMatchedPoints[i].robot.pose.position.y << " , ";
//                 cout << chosenMatchedPoints[i].robot.pose.position.z << endl;
//                 
//             }
//             cout << chosenMatchedPoints.size() << endl;
//             
//             for(int i = 0; i<chosenMatchedPoints.size() ; i++)
//             {
//                 cout << "########################################" << endl;
//                 cout << chosenMatchedPoints[i].robot.pose.position.x << " , " << chosenMatchedPoints[i].robot.pose.position.y << " , " << chosenMatchedPoints[i].robot.pose.position.z << endl;
//                 cout << chosenMatchedPoints[i].camera.pose.position.x << " , " << chosenMatchedPoints[i].camera.pose.position.y << " , " << chosenMatchedPoints[i].camera.pose.position.z << endl;
//             }
            
            find_transformation();
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
    
    void find_transformation()
    {
        // finding centroids
        // frame 1 is camera 
        Mat frame1_sum=Mat::zeros(3,1,CV_32F);
        Mat frame2_sum=Mat::zeros(3,1,CV_32F);
        for(int i=0 ; i<chosenMatchedPoints.size(); i++)
        {
            Mat cam_point=convert_geomsg_to_mat(chosenMatchedPoints[i].camera);
//             cout <<"Camera: " << cam_point << endl;
            Mat rob_point=convert_geomsg_to_mat(chosenMatchedPoints[i].robot);
//             cout << "Robot: " << rob_point << endl;
            
            frame1_sum=frame1_sum+cam_point;
            frame2_sum=frame2_sum+rob_point;
        }
        
//         cout << "Total for frame 1: " << frame1_sum << endl;
//         cout << "Total for frame 2: " << frame2_sum << endl;
        
        frame1_sum=frame1_sum/((float)chosenMatchedPoints.size());
        frame2_sum=frame2_sum/((float)chosenMatchedPoints.size());

//         cout << "Centroids: " << endl << frame1_sum << endl << frame2_sum << endl;
//         cout << "Centroids: " << endl << frame1_sum-frame2_sum << endl;


        // Using SVD - create H matrix
        Mat H = Mat::zeros(3,3,CV_32F);
        for(int i=0 ; i<chosenMatchedPoints.size(); i++)
        {
            Mat transposed;
            transpose(convert_geomsg_to_mat(chosenMatchedPoints[i].robot)-frame2_sum,transposed);
            H=H+(convert_geomsg_to_mat(chosenMatchedPoints[i].camera)-frame1_sum)*transposed;
        }

        Mat w,u,vt,v,ut;
        SVD::compute(H,w,u,vt);
        transpose(vt,v);
        transpose(u,ut);

        Mat R_computed=v*ut;

//         cout << R_computed << endl;

        // getting the translation

        Mat t_computed= -R_computed*frame1_sum+frame2_sum;
        
//         cout << "########### tranformation: ###############" << endl;
//         cout << R_computed << t_computed << endl;
        
        
        Mat R_vector;
        Rodrigues(R_computed,R_vector);
        
        geometry_msgs::Transform msg;
        msg.translation.x=t_computed.at<float>(0,0);
        msg.translation.y=t_computed.at<float>(1,0);
        msg.translation.z=t_computed.at<float>(2,0);
                
        msg.rotation.x=R_vector.at<float>(0,0);
        msg.rotation.y=R_vector.at<float>(1,0);
        msg.rotation.z=R_vector.at<float>(2,0);
        
        transformation_pub_.publish(msg);
//         cout << t_computed << endl;

//         cout << "Difference: " << endl << (R_computed-R) << endl;
    }
    
    
private:
    
    bool compare_3D_points(geometry_msgs::Pose new_position, geometry_msgs::Pose old_position, double expected_distance)
    {
        float x=old_position.position.x;
        float y=old_position.position.y;
        float z=old_position.position.z;
        
        float x2=new_position.position.x;
        float y2=new_position.position.y;
        float z2=new_position.position.z;
        
        double dist=sqrt(pow((x-x2),2)+pow((y-y2),2)+pow((z-z2),2));
        
        if(dist>=expected_distance)
            return true;
        else 
            return false;
    }
    double dist_between_points(geometry_msgs::Pose new_position, geometry_msgs::Pose old_position)
    {
        float x=old_position.position.x;
        float y=old_position.position.y;
        float z=old_position.position.z;
        
        float x2=new_position.position.x;
        float y2=new_position.position.y;
        float z2=new_position.position.z;
        
        double dist=sqrt(pow((x-x2),2)+pow((y-y2),2)+pow((z-z2),2));
        return dist;
    }

    vector<geometry_msgs::PoseStamped> camera_points;
    vector<geometry_msgs::PoseStamped> robot_positions;
    vector<matchedPoint> matchedPoints;
    vector<matchedPoint> chosenMatchedPoints;
    
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "match_points");
  MatchPoints ic;
  ROS_INFO("Starting camera node up");
  ros::spin();
  return 0;
}

