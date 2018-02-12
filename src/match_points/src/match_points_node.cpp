


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

#include <tf/transform_broadcaster.h>
// #include <Matrix3x3.h>

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

#include <message_package/matched_points.h>
#include "message_package/currentToolPosition2.h"
#include "match_points/setDistBetwChosenPoints.h"
#include "match_points/stopMatching.h"
#include "match_points/getNextMatchingPoint.h"
// #include "mp_mini_picker/currentToolPosition.h"

// #include "/home/jepod13/catkin_ws/devel/include/mp_mini_picker/currentToolPosition.h"

#include <iostream>
#include <fstream>
#include <string> 

using namespace std;
using namespace cv;


struct matchedPoint {
  geometry_msgs::PoseStamped camera;
//   geometry_msgs::PoseStamped robot;
  message_package::currentToolPosition2 robot;
} ;

class Response;

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
        use_cheat=false;
        if(use_cheat==true)
        {
            string chosenPoint;
            ifstream infile;
            infile.open("/home/rovi2/Jesper/data_for_wTc.txt");
            string::size_type sz;
            int current_case=0;
            int last_delimiter=0;
            int count=0;
            while(getline(infile,chosenPoint)) // To get you all the lines.
            {

                cout << chosenPoint << endl;
                matchedPoint hej;
                for(int i=0; i<chosenPoint.length(); i++)
                {
                    
                    double number;
                    if(chosenPoint.at(i)==',')
                    {
                        string hep=chosenPoint.substr(last_delimiter,(i-last_delimiter));
//                         cout << hep << endl;
                        number = stod(hep,&sz);
                        last_delimiter=i+1;
                        
//                             cout << last_delimiter << " " << number << " " << current_case << endl;
                        switch(current_case)
                        {
                            case 0:
                                hej.camera.pose.position.x=number;
                                break;
                            case 1:
                                hej.camera.pose.position.y=number;
                                break;
                            case 2:
                                hej.camera.pose.position.z=number;
                                break;
                            case 3:
                                hej.robot.tcp.pose.position.x=number;
                                break;
                            case 4:
                                hej.robot.tcp.pose.position.y=number;
                                break;
                            case 5:
                                hej.robot.tcp.pose.position.z=number;
                                break;
                        }
                        current_case++;
                        
                    }
                    else if(i==chosenPoint.length()-1)
                    {
                        string hep=chosenPoint.substr(last_delimiter,(i-last_delimiter));
//                         cout << hep << endl;
                        number = stod(hep,&sz);
//                             cout << last_delimiter << " " << number << " " << current_case << endl;
                        switch(current_case)
                        {
                            case 0:
                                hej.camera.pose.position.x=number;
                                break;
                            case 1:
                                hej.camera.pose.position.y=number;
                                break;
                            case 2:
                                hej.camera.pose.position.z=number;
                                break;
                            case 3:
                                hej.robot.tcp.pose.position.x=number;
                                break;
                            case 4:
                                hej.robot.tcp.pose.position.y=number;
                                break;
                            case 5:
                                hej.robot.tcp.pose.position.z=number;
                                break;
                        }
                        current_case++;
                    }
                }
                chosenMatchedPoints.push_back(hej);
                last_delimiter=0;
                current_case=0;
                count++;
            }
            infile.close();
            gain_first_element=true;
            for(int i=0 ; i<chosenMatchedPoints.size() ; i++)
            {
                cout << chosenMatchedPoints[i].camera.pose.position.x << ",";
                cout << chosenMatchedPoints[i].camera.pose.position.y << ",";
                cout << chosenMatchedPoints[i].camera.pose.position.z << ",";
                cout << chosenMatchedPoints[i].robot.tcp.pose.position.x << ",";
                cout << chosenMatchedPoints[i].robot.tcp.pose.position.y << ",";
                cout << chosenMatchedPoints[i].robot.tcp.pose.position.z << "\n";
                
            }
            
        }
        
        
        // Subscrive to input video feed and publish output video feed
        transformation_pub_ = nh_.advertise<geometry_msgs::Transform>("/matched_points/transformation_matrix", 1);
        matched_points_pub_ = nh_.advertise<message_package::matched_points>("/matched_points/chosenMatchedPoints", 1);
        
        robot_sub_ = nh_.subscribe("/robot/moved", 1, &MatchPoints::robot_has_moved,this);
        camera_point_sub_ = nh_.subscribe("/kalman_filter/marker1", 1, &MatchPoints::camera_3D_points,this);
        
    }

    ~MatchPoints()
    {
        //     cv::destroyWindow(OPENCV_WINDOW);
    }
    
    void camera_3D_points(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
        if(do_match_points==true)
        {
            camera_points.insert(camera_points.begin(),*msg);
            if(camera_points.size() > 7) // only one second of data is stored
            {
                camera_points.pop_back();
            }
            match_points();
        }
        else if(temp_match_point==true)
        {
            camera_points.insert(camera_points.begin(),*msg);
            match_a_point_pair();
        } else
        {
            camera_points.clear();
            chosenMatchedPoints.clear();
        }
    }

    void robot_has_moved(const message_package::currentToolPosition2::ConstPtr msg)
    {
        if(do_match_points==true)
        {
            robot_positions.insert(robot_positions.begin(),*msg);
            if(robot_positions.size() > 7) // only one second of data is stored
            {
                robot_positions.pop_back();
            }
            match_points();
        }
        else if(temp_match_point==true)
        {
            robot_positions.insert(robot_positions.begin(),*msg);
            match_a_point_pair();
        } else
        {
            robot_positions.clear();
            chosenMatchedPoints.clear();
        }
    }

    void match_a_point_pair()
    {
        for( int i = 0 ; i < robot_positions.size() ; i++) {
            for (int j = 0; j < camera_points.size(); j++) {
                ros::Duration possible_match(0.0714); // 7 Hz * 2 = 14 HZ
                ros::Time low_bound = camera_points[j].header.stamp - possible_match;
                ros::Time high_bound = camera_points[j].header.stamp + possible_match;
                if (robot_positions[i].tcp.header.stamp > low_bound &&
                    robot_positions[i].tcp.header.stamp < high_bound)
                {
                    message_package::matched_points msg;
                    msg.robot=robot_positions[i].tcp;
                    msg.wTtcp=robot_positions[i].wTtcp;
                    msg.camera=camera_points[i];

                    matched_points_pub_.publish(msg);
                    temp_match_point=false;
                    cout << msg.camera.pose.position.x << ",";
                    cout << msg.camera.pose.position.y << ",";
                    cout << msg.camera.pose.position.z << ",";
                    cout << msg.robot.pose.position.x << ",";
                    cout << msg.robot.pose.position.y << ",";
                    cout << msg.robot.pose.position.z << "\n";
                    cout << msg.wTtcp.pose.position.x << ",";
                    cout << msg.wTtcp.pose.position.y << ",";
                    cout << msg.wTtcp.pose.position.z << ",";
                    cout << msg.wTtcp.pose.orientation.x << ",";
                    cout << msg.wTtcp.pose.orientation.y << ",";
                    cout << msg.wTtcp.pose.orientation.z << "\n";
                }
            }
        }
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
                if(robot_positions[i].tcp.header.stamp>low_bound && robot_positions[i].tcp.header.stamp<high_bound)
                {
//                     cout << camera_points[j].stamp << " ~ " << robot_positions[i].header.stamp << endl;
//                     cout << robot_positions[i].pose << endl;
//                     cout << camera_points[j].point3D << endl;
//                     ROS_INFO("In a match");
                    matchedPoint current;
                    current.camera=camera_points[j];
                    current.robot=robot_positions[i];
                    
                    
                    
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
        if(chosenMatchedPoints.size() == 0 || gain_first_element==true)
        {
            chosenMatchedPoints.insert(chosenMatchedPoints.begin(),matchedPoints[0]);
            matchedPoints.erase(matchedPoints.begin());
            gain_first_element=false;
        }
//         int old_array_size=chosenMatchedPoints.size();
        bool added_points=false;
        // choosing points based on, whether the robot has moved position 
            while(matchedPoints.size()>0)
            {
//                 ROS_INFO("Hej %f",dist_between_points(matchedPoints[0].robot.tcp.pose,chosenMatchedPoints[0].robot.tcp.pose));
                int i = matchedPoints.size()-1;
                if(compare_3D_points(matchedPoints[i].robot.tcp.pose,chosenMatchedPoints[0].robot.tcp.pose,0.02))
                {
                    float rob_dist=dist_between_points(matchedPoints[i].robot.tcp.pose,chosenMatchedPoints[0].robot.tcp.pose);
                    float cam_dist=dist_between_points(matchedPoints[i].camera.pose,chosenMatchedPoints[0].camera.pose);
                    if(abs(cam_dist-rob_dist)/rob_dist < 0.15)
                    {
                        added_points = true;
//                         ROS_INFO("Distance between chosen poinsts:  %f",dist_between_points(matchedPoints[0].robot.tcp.pose,chosenMatchedPoints[0].robot.tcp.pose));
                        cout << chosenMatchedPoints[0].camera.pose.position.x << ",";
                        cout << chosenMatchedPoints[0].camera.pose.position.y << ",";
                        cout << chosenMatchedPoints[0].camera.pose.position.z << ",";
                        cout << chosenMatchedPoints[0].camera.pose.orientation.x << ",";
                        cout << chosenMatchedPoints[0].camera.pose.orientation.y << ",";
                        cout << chosenMatchedPoints[0].camera.pose.orientation.z << "\n";
                        
                        cout << chosenMatchedPoints[0].robot.Q[0] << ",";
                        cout << chosenMatchedPoints[0].robot.Q[1] << ",";
                        cout << chosenMatchedPoints[0].robot.Q[2] << ",";
                        cout << chosenMatchedPoints[0].robot.Q[3] << ",";
                        cout << chosenMatchedPoints[0].robot.Q[4] << ",";
                        cout << chosenMatchedPoints[0].robot.Q[5] << "\n";
                        
                        
//                         cout << chosenMatchedPoints[0].robot.tcp.pose.position.x << ",";
//                         cout << chosenMatchedPoints[0].robot.tcp.pose.position.y << ",";
//                         cout << chosenMatchedPoints[0].robot.tcp.pose.position.z << ",";
//                         cout << chosenMatchedPoints[0].robot.tcp.pose.orientation.x << ",";
//                         cout << chosenMatchedPoints[0].robot.tcp.pose.orientation.y << ",";
//                         cout << chosenMatchedPoints[0].robot.tcp.pose.orientation.z << "\n";
                        
//                         cout << chosenMatchedPoints[0].robot.wTtcp.pose.position.x << ",";
//                         cout << chosenMatchedPoints[0].robot.wTtcp.pose.position.y << ",";
//                         cout << chosenMatchedPoints[0].robot.wTtcp.pose.position.z << ",";
//                         cout << chosenMatchedPoints[0].robot.wTtcp.pose.orientation.x << ",";
//                         cout << chosenMatchedPoints[0].robot.wTtcp.pose.orientation.y << ",";
//                         cout << chosenMatchedPoints[0].robot.wTtcp.pose.orientation.z << "\n";
//                         
                        chosenMatchedPoints.insert(chosenMatchedPoints.begin(),matchedPoints[i]);
                        matchedPoints.erase(matchedPoints.end());
                        
                        message_package::matched_points msg;
                        msg.robot=chosenMatchedPoints[0].robot.tcp;
                        msg.wTtcp=chosenMatchedPoints[0].robot.wTtcp;
                        msg.camera=chosenMatchedPoints[0].camera;
                    
                        matched_points_pub_.publish(msg);
                    }
                    else
                    {
                        matchedPoints.erase(matchedPoints.end());
                    }
                    
                    
                    
//                     if(chosenMatchedPoints.size() > 15)
//                     {
//                         chosenMatchedPoints.erase(chosenMatchedPoints.end());
//                     }

                }
                else
                {
                    
                    matchedPoints.erase(matchedPoints.end());
                }
            }
        if(added_points)
        {
            chosenMatchedPoints.erase(chosenMatchedPoints.end()-(chosenMatchedPoints.size()-2),chosenMatchedPoints.end());
            
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
    
    
    
    void sendTransformTf(Mat translation, Mat rotation, string to_frame, string from_frame)
    {
        tf::Matrix3x3 rot;
        rot.setValue(rotation.at<float>(0,0),rotation.at<float>(0,1),rotation.at<float>(0,2),
                     rotation.at<float>(1,0),rotation.at<float>(1,1),rotation.at<float>(1,2),
                     rotation.at<float>(2,0),rotation.at<float>(2,1),rotation.at<float>(2,2));
//         cout << rotation << endl;
//         cout << rot.determinant() << endl;
//         cout << rot.getColumn(0).getX() << "," << rot.getColumn(0).getY() << "," << rot.getColumn(0).getZ() << endl;
        tf::Vector3 t;
        t.setValue(translation.at<float>(0,0),translation.at<float>(1,0),translation.at<float>(2,0));
        tf::Transform wTc(rot,t);
//         wTc.setOrigin( tf::Vector3(0, 0, 0) );
        
        tf::StampedTransform msg(wTc,ros::Time::now(),to_frame,from_frame);
        static tf::TransformBroadcaster br;
        br.sendTransform(msg);
        
        
    }
    bool stop_matching(match_points::stopMatching::Request  &req,
                       match_points::stopMatching::Response &res)
    {
        do_match_points=!(req.stop);
        if (req.stop==0)
            cout << "###################  Started Matching ###################" << endl;
        else
            cout << "###################  stopped Matching ###################" << endl;
        
        res.done=true;
    }

    bool distBetwPoints(match_points::setDistBetwChosenPoints::Request &req,
                        match_points::setDistBetwChosenPoints::Response &res)
    {
        dist_accepted_points=req.dist;
        res.done=true;
    }

    bool getNextMatchingPoint(match_points::getNextMatchingPoint::Request &req,
                              match_points::getNextMatchingPoint::Response &res)
    {
        temp_match_point=req.getMatchedPoint;
        res.done=true;
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
    
    Mat t_computed_old; 
    
    bool posted_tranformation =false;
    float euc_translation_dif_old;
    bool use_cheat,gain_first_element=false;
    
    vector<geometry_msgs::PoseStamped> camera_points;
//     vector<geometry_msgs::PoseStamped> robot_positions;
    vector<message_package::currentToolPosition2> robot_positions;
    vector<matchedPoint> matchedPoints;
    vector<matchedPoint> chosenMatchedPoints;
    
    bool do_match_points=false;
    double dist_accepted_points=0.02;
    bool temp_match_point=false;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "match_points");
    ros::NodeHandle n;
    MatchPoints ic;
    ROS_INFO("Starting camera node up");
//    ros::ServiceServer service = n.advertiseService("/robot/GetCurrentQ", &UrTest::get_current_Q, &ur_test);
    ros::ServiceServer service = n.advertiseService("/match_points/stopMatching", &MatchPoints::stop_matching, &ic);
    ros::ServiceServer service1 = n.advertiseService("/match_points/getNextMatchedPoint", &MatchPoints::getNextMatchingPoint, &ic);
    ros::ServiceServer service2 = n.advertiseService("/match_points/distanceBetweenAcceptedPoints", &MatchPoints::distBetwPoints, &ic);
    ros::spin();
    return 0;
}

