

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

struct cameraPoint {
  Mat point3D;
  ros::Time stamp;
} ;

struct matchedPoint {
  cameraPoint camera;
  geometry_msgs::PoseStamped robot;
} ;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  ros::Subscriber robot_sub_;

public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image_raw", 1,&ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        robot_sub_ = nh_.subscribe("/robot/moved", 1, &ImageConverter::robot_has_moved,this);
        
        // For using solvePNP on aruco Marker
//         marker_3D.push_back(Point3f(0,0,0));
//         marker_3D.push_back(Point3f(0.169,0,0));
//         marker_3D.push_back(Point3f(0.169,-0.169,0));
//         marker_3D.push_back(Point3f(0,-0.169,0));
        marker_3D.push_back(Point3f(0,0,0));
        marker_3D.push_back(Point3f(0.044,0,0));
        marker_3D.push_back(Point3f(0.044,-0.044,0));
        marker_3D.push_back(Point3f(0,-0.044,0));
        
        
        distortion.push_back(-0.228385);
        distortion.push_back(0.266082);
        distortion.push_back(-0.001812);
        distortion.push_back(0.000035);
        distortion.push_back(0.000000);
        
        mark_dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
        
        camera_matrix = (Mat_<float>(3,3) <<    1348.715676, 0.000000, 722.486120,
                                                0.000000, 1347.386029, 495.012476,
                                                0.000000, 0.000000, 1.000000);
        
    }

    ~ImageConverter()
    {
        //     cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        vector< vector<Point2f> > markerCorners; // So, the first corner is the top left corner, followed by the top right, bottom right and bottom left.
        vector<int> markerIds;
        aruco::detectMarkers(cv_ptr->image, mark_dict, markerCorners, markerIds);
        if(!markerCorners.empty())
        {
            Mat rvec, tvec;
            solvePnP(marker_3D,markerCorners[0],camera_matrix,distortion,rvec,tvec);
            cameraPoint current;
            current.stamp=msg->header.stamp;
            current.point3D=tvec.clone();
            camera_points.insert(camera_points.begin(),current);
            if(camera_points.size() > 4) // only one second of data is stored
            {
                camera_points.pop_back();
            }
        }  
        match_points();
    }
    void robot_has_moved(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
//         ROS_INFO("We are brilliant");
//         cout << *msg << endl;
//         
//         ROS_INFO("We are brilliant2");
        robot_positions.insert(robot_positions.begin(),*msg);
        if(robot_positions.size() > 2) // only one second of data is stored
        {
            robot_positions.pop_back();
        }
        match_points();
    }
    
    void match_points()
    {
//         ROS_INFO("MATCHING POINTS");
        for( int i = 0 ; i < robot_positions.size() ; i++)
        {
            for( int j = 0 ; j < camera_points.size() ; j++)
            {
                ros::Duration possible_match(0.0714); // 7 Hz / 2 
                ros::Time low_bound = camera_points[j].stamp - possible_match;
                ros::Time high_bound = camera_points[j].stamp + possible_match;
                if(robot_positions[i].header.stamp>low_bound && robot_positions[i].header.stamp<high_bound)
                {
//                     cout << camera_points[j].stamp << " ~ " << robot_positions[i].header.stamp << endl;
//                     cout << robot_positions[i].pose << endl;
//                     cout << camera_points[j].point3D << endl;
//                     ROS_INFO("In a match");
                    matchedPoint current;
                    current.camera=camera_points[j];
                    current.robot=robot_positions[i];
                    matchedPoints.insert(matchedPoints.begin(),current);
//                     /*camera_points.erase(camera_points.begin()+j);
//                     robot_positions.erase(robot_positions.begin()+i)*/;
                    
//                     if(matchedPoints.size() > 15) // Allowing more data to be stored here
//                     {
//                         matchedPoints.pop_back();
//                     }
                }
            }
        }
        if(matchedPoints.size()>0)
            choosePoints();
        
    }
    
    void choosePoints()
    {
        // if no points has been choosen, then just take the first matched point
        if(chosenMatchedPoints.size() == 0)
        {
            chosenMatchedPoints.insert(chosenMatchedPoints.begin(),matchedPoints[0]);
            matchedPoints.erase(matchedPoints.begin());
        }
//         ROS_INFO("RUNNING CODE AGAIN %d", matchedPoints.size());
        // choosing points based on, whether the robot has moved position 
//         if(matchedPoints.size()>0)
//         {
            while(matchedPoints.size()>0)
            {
//                 ROS_INFO("Hej %f",dist_between_points(matchedPoints[0].robot.pose,chosenMatchedPoints[0].robot.pose));
//                 ROS_INFO("Wow wow wo");
                if(compare_3D_points(matchedPoints[0].robot.pose,chosenMatchedPoints[0].robot.pose,0.05))
                {
                    ROS_INFO("Hej %d",dist_between_points(matchedPoints[0].robot.pose,chosenMatchedPoints[0].robot.pose));
                    chosenMatchedPoints.insert(chosenMatchedPoints.begin(),matchedPoints[0]);
    //                 cout << "hej" << endl;
                    
                    matchedPoints.erase(matchedPoints.begin());

                }
                else
                    matchedPoints.erase(matchedPoints.begin());
            }
//         }
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
    double dist_between_points(geometry_msgs::Pose new_position, geometry_msgs::Pose old_position)
    {
        double old_dist=sqrt(pow(old_position.position.x,2)+pow(old_position.position.y,2)+pow(old_position.position.z,2));
        double new_dist=sqrt(pow(new_position.position.x,2)+pow(new_position.position.y,2)+pow(new_position.position.z,2));
        return abs(new_dist-old_dist);
    }
    
private:
    cv::Mat current_image;
    vector<Point3f> marker_3D;
    vector<double> distortion;
    Ptr<aruco::Dictionary> mark_dict;
    Mat_<float> camera_matrix;
    vector<cameraPoint> camera_points;
    vector<geometry_msgs::PoseStamped> robot_positions;
    vector<matchedPoint> matchedPoints;
    vector<matchedPoint> chosenMatchedPoints;
    
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ROS_INFO("Starting camera node up");
  ros::spin();
  return 0;
}




/*
void robot_has_moved(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "aruco_3D_node");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/robot/moved", 2, robot_has_moved);

    ros::spin();


    return 0;
}*/