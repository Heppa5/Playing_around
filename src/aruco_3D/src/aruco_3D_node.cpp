

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

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


        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());

        current_image = cv_ptr->image.clone();
    }
    void robot_has_moved(const geometry_msgs::Pose::ConstPtr& msg)
    {
        ROS_INFO("We are brilliant");
        cout << *msg << endl;
    }
private:
    cv::Mat current_image;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
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