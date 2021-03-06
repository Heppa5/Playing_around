


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
#include "geometry_msgs/Pose.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <opencv2/aruco.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>



using namespace std;
using namespace cv;


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher image_pub2_;
  
  ros::Publisher aruco_point_pub2_;
  ros::Publisher aruco_point_pub3_;

public:
    ImageConverter(string ID)
        : it_(nh_)
    {
        Marker_ID=atoi(ID.c_str());
        
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image_raw", 1,&ImageConverter::imageCb, this);
        if(Marker_ID==2)
        {
            image_pub_ = it_.advertise("/image_converter/output_video", 1);
            aruco_point_pub2_ = nh_.advertise<geometry_msgs::PoseStamped>("/aruco/marker1", 1);
            marker2_3D.push_back(Point3f(0,0,0));
            marker2_3D.push_back(Point3f(0.066,0,0));
            marker2_3D.push_back(Point3f(0.066,-0.066,0));
            marker2_3D.push_back(Point3f(0,-0.066,0));
            
            marker3_3D.push_back(Point3f(0,0,0));
            marker3_3D.push_back(Point3f(0.055,0,0));
            marker3_3D.push_back(Point3f(0.055,-0.055,0));
            marker3_3D.push_back(Point3f(0,-0.055,0));
            marker2_size=0.066;
            marker3_size=0.055;
        }
        else if(Marker_ID==3)
        {
            image_pub2_ = it_.advertise("/image_converter/output_video2", 1);
            aruco_point_pub3_ = nh_.advertise<geometry_msgs::PoseStamped>("/aruco/marker2", 1);
            marker2_3D.push_back(Point3f(0,0,0));
            marker2_3D.push_back(Point3f(0.066,0,0));
            marker2_3D.push_back(Point3f(0.066,-0.066,0));
            marker2_3D.push_back(Point3f(0,-0.066,0));
            
            marker3_3D.push_back(Point3f(0,0,0));
            marker3_3D.push_back(Point3f(0.055,0,0));
            marker3_3D.push_back(Point3f(0.055,-0.055,0));
            marker3_3D.push_back(Point3f(0,-0.055,0));
            marker2_size=0.066;
            marker3_size=0.055;
        }
        if(Marker_ID==4)
        {
            image_pub_ = it_.advertise("/image_converter/output_video", 1);
            aruco_point_pub2_ = nh_.advertise<geometry_msgs::PoseStamped>("/aruco/marker1", 1);
            marker2_3D.push_back(Point3f(0,0,0));
            marker2_3D.push_back(Point3f(0.024,0,0));
            marker2_3D.push_back(Point3f(0.024,-0.024,0));
            marker2_3D.push_back(Point3f(0,-0.024,0));
            
            marker3_3D.push_back(Point3f(0,0,0));
            marker3_3D.push_back(Point3f(0.024,0,0));
            marker3_3D.push_back(Point3f(0.024,-0.024,0));
            marker3_3D.push_back(Point3f(0,-0.024,0));
            cout << Marker_ID << endl;
            marker2_size=0.024;
            marker3_size=0.024;
        }
        else if(Marker_ID==5)
        {
            image_pub2_ = it_.advertise("/image_converter/output_video2", 1);
            aruco_point_pub3_ = nh_.advertise<geometry_msgs::PoseStamped>("/aruco/marker2", 1);
            marker2_3D.push_back(Point3f(0,0,0));
            marker2_3D.push_back(Point3f(0.024,0,0));
            marker2_3D.push_back(Point3f(0.024,-0.024,0));
            marker2_3D.push_back(Point3f(0,-0.024,0));
            
            marker3_3D.push_back(Point3f(0,0,0));
            marker3_3D.push_back(Point3f(0.024,0,0));
            marker3_3D.push_back(Point3f(0.024,-0.024,0));
            marker3_3D.push_back(Point3f(0,-0.024,0));
            cout << Marker_ID << endl;
            marker2_size=0.024;
            marker3_size=0.024;
        }

        

        
        



//         distortion.push_back(-0.228385);
//         distortion.push_back(0.266082);
//         distortion.push_back(-0.001812);
//         distortion.push_back(0.000035);
//         distortion.push_back(0.000000);
//         distortion_mat=(Mat_<float>(5,1) <<    -0.228385,0.266082,-0.001812,0.000035,0.000000);
        
        distortion_mat=(Mat_<float>(5,1) <<    -0.21475579816108,0.202105909302347,-0.00070360488190,0.000066847865518,0.000000);
//         distortion_mat=(Mat_<float>(4,1) <<    -0.21475579816108,0.202105909302347,-0.00070360488190,0.000066847865518);
        distortion.push_back(-0.21475579816108);
        distortion.push_back(0.202105909302347);
        distortion.push_back(-0.00070360488190);
        distortion.push_back(0.000066847865518);
        distortion.push_back(0.000000);
        
        
        
        print_pixel_values=true;
        
        
        mark_dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
        
//         camera_matrix = (Mat_<float>(3,3) <<    1348.715676, 0.000000, 722.486120,
//                                                 0.000000, 1347.386029, 495.012476,
//                                                 0.000000, 0.000000, 1.000000);
        camera_matrix = (Mat_<float>(3,3) <<    1323.53603931762, 0.000000, 717.013056698467,
                                                0.000000, 1323.10476760428, 487.627723543151,
                                                0.000000, 0.000000, 1.000000);
        
        
        
//         filtering_constant=3.14/6.0;
//         filtering_constant=3.14/4.0;
        filtering_constant=3.14/6.0;
        
//         Mat rvec_marker2,rvec_marker3;
//        rvec_marker2.create(3,1,CV_64F);
//        rvec_marker3.create(3,1,CV_64F);
        
    }

    ~ImageConverter()
    {
        //     cv::destroyWindow(OPENCV_WINDOW);
    }
    
    Mat mean(vector<Mat> data)
    {
        Mat mean=Mat::zeros(3,1,CV_64F);
        for (int i = 0; i<data.size() ; i++)
        {
            mean=mean+data[i];
        }
        mean=mean/data.size();
        return mean;
    }
    int count2=0;
    vector<Point2f> last_marker4;
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        vector<double> empty_array;
        Mat empty_mat;

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
        Mat image=cv_ptr->image;
//        cv::undistort(cv_ptr->image,image, camera_matrix,distortion_mat,camera_matrix_updated);


        aruco::detectMarkers(image, mark_dict, markerCorners, markerIds,aruco::DetectorParameters::create(),rejectedImagePoints);
        aruco::drawDetectedMarkers(image, markerCorners, markerIds);
        
        Mat rvec3(3,1,CV_64F), tvec3(3,1,CV_64F);
        Mat rvec2(3,1,CV_64F), tvec2(3,1,CV_64F);
        vector<Mat> rvecs,tvecs;

//        bool its_there=false;
//        for(int i = 0 ; i < markerCorners.size() ; i++)
//        {
//            if(4 == markerIds[i] && Marker_ID==4) {
//                its_there=true;
//            }
//        }
//        if(its_there==false)
//        {
//            cout << "¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤" << endl;
//            for(int i=0; i<rejectedImagePoints.size() ; i++)
//            {
//                cout << "Pixel coordinates: \n" << endl;
//                for(int j=0; j<rejectedImagePoints[i].size() ; j++)
//                {
//                    cout  << rejectedImagePoints[i][j].x << " , " << rejectedImagePoints[i][j].y << "\t";
//
//                }
//                cout << endl;
//            }
//            cout << "Last pixel coordinate for marker 4: \n" << endl;
//            for(int j=0; j<last_marker4.size() ; j++)
//            {
//                cout  << last_marker4[j].x << " , " << last_marker4[j].y << "\t";
//            }
//            cout << endl;
//            cout << "¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤" << endl;
//        }

        
        for(int i = 0 ; i < markerCorners.size() ; i++)
        {
            if(4 == markerIds[i] && Marker_ID==4)
            {
//                solvePnP(marker3_3D,markerCorners[i],camera_matrix,empty_array,rvec,tvec);
                solvePnP(marker2_3D,markerCorners[i],camera_matrix,distortion,rvec2,tvec2);

//                 solvePnP(marker2_3D,markerCorners[i],camera_matrix,empty_array,rvec,tvec);
                if(initialized_marker2==true)
                {
                    last_marker4.clear();
                    for(int j=0; j<markerCorners[i].size() ; j++) {
                        last_marker4.push_back(markerCorners[i][j]);
                    }

                    Mat rot_dif=rvec_marker2-rvec2;

                    if(norm(rot_dif)<filtering_constant)
                    {
//                        cout << "##############################\nMarker 1 position\n" << tvec2 << endl;
                      aruco::drawAxis(image, camera_matrix, distortion_mat, rvec2, tvec2,marker2_size * 0.5f);
//                        aruco::drawAxis(image, camera_matrix, empty_mat, rvec, tvec,marker2_size * 0.5f);

                        geometry_msgs::PoseStamped aruco3D_msg;
                        aruco3D_msg.header.stamp=msg->header.stamp;
                        aruco3D_msg.pose.position.x=tvec2.at<double>(0,0);
                        aruco3D_msg.pose.position.y=tvec2.at<double>(1,0);
                        aruco3D_msg.pose.position.z=tvec2.at<double>(2,0);
                        aruco3D_msg.pose.orientation.x=rvec2.at<double>(0,0);
                        aruco3D_msg.pose.orientation.y=rvec2.at<double>(1,0);
                        aruco3D_msg.pose.orientation.z=rvec2.at<double>(2,0);
                        aruco_point_pub2_.publish(aruco3D_msg);
                        rvec_marker2=rvec2.clone();

                    }
                    else {
//                         cout << "¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤" << endl;
//                         cout << "Size of rot dif: " << norm(rot_dif) << endl;
//                         cout << "Rot difference is : \n" << rot_dif << endl;
//                         cout << "Rvec is: \n " << rvec << endl;
//                         cout << "Rvec_marker2 is: " <<rvec_marker2 << endl;
//                         cout << "¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤" << endl;

                             aruco::drawAxis(image, camera_matrix, distortion_mat, rvec2, tvec2,marker2_size * 1.0f);
//                        aruco::drawAxis(image, camera_matrix, empty_mat, rvec, tvec, marker2_size * 1.0f);

                    }
                    if(rvec_marker2.rows!=3)
                    {
                        cout << "LIFE IS A BITCH AND YOUR MATRICE ISN'T THE RIGHT SIZE!" << endl;
                    }
//                    else
//                    {
//                        rvec_marker2=rvec;
//                    }
                }
                else
                {
                    if(initialize_marker2.size()>20)
                    {
                        rvec_marker2=select_rotation(&initialize_marker2);
                        initialized_marker2=true;
//                         cout << "First guess: \n" << rvec_marker2 << endl;
                    }
                    else
                    {
//                         sort_vector(&initialize_marker2,rvec.clone());
                        initialize_marker2.push_back(rvec2.clone());
                    }
                }
                
            }
            else if(5 == markerIds[i] && Marker_ID==5)
            {
                solvePnP(marker3_3D,markerCorners[i],camera_matrix,distortion,rvec3,tvec3);
//                solvePnP(marker3_3D,markerCorners[i],camera_matrix,empty_array,rvec,tvec);
//                 solvePnP(marker3_3D,markerCorners[i],camera_matrix,empty_array,rvec,tvec);
                if(initialized_marker3==true)
                {
                    if(rvec_marker3.rows==3) // initialized or not
                    {
                        Mat rot_dif=rvec_marker3-rvec3;
                        if(norm(rot_dif)<filtering_constant)
                        {
//                            cout << "##############################\nMarker 2 position\n" << tvec3 << endl;
                             aruco::drawAxis(image, camera_matrix, distortion_mat, rvec3, tvec3,marker3_size * 0.5f);
//                            aruco::drawAxis(image, camera_matrix, empty_mat, rvec, tvec,marker3_size * 0.5f);
                            geometry_msgs::PoseStamped aruco3D_msg;
                            aruco3D_msg.header.stamp=msg->header.stamp;
                            aruco3D_msg.pose.position.x=tvec3.at<double>(0,0);
                            aruco3D_msg.pose.position.y=tvec3.at<double>(1,0);
                            aruco3D_msg.pose.position.z=tvec3.at<double>(2,0);
                            aruco3D_msg.pose.orientation.x=rvec3.at<double>(0,0);
                            aruco3D_msg.pose.orientation.y=rvec3.at<double>(1,0);
                            aruco3D_msg.pose.orientation.z=rvec3.at<double>(2,0);
                            aruco_point_pub3_.publish(aruco3D_msg);
                            rvec_marker3=rvec3.clone();
                        }
                        else
                        {
                             aruco::drawAxis(image, camera_matrix, distortion_mat, rvec3, tvec3,marker3_size * 1.0f);
//                             aruco::drawAxis(image, camera_matrix, empty_mat, rvec, tvec,marker3_size * 1.0f);
                        }
                    }
                }
                else{
                    if(initialize_marker3.size()>20)
                    {    
                        rvec_marker3=select_rotation(&initialize_marker3);
                        initialized_marker3=true;
                    }
                    else
                    {
//                        if(rvec3.at<double>(0,0)>0 && rvec3.at<double>(1,0)>0 && rvec3.at<double>(2,0)<0)
                            initialize_marker3.push_back(rvec3.clone());
//                         sort_vector(&initialize_marker3,rvec.clone());
//                        1.7276578531
//                        y: 0.638452777045
//                        z: -0.492245440404
                    }
                }
            }
        }
        
        if(print_pixel_values && Marker_ID==4 )
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            image_pub_.publish(msg);
//             msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
//             image_pub2_.publish(msg);
        }
        if(print_pixel_values && Marker_ID==5 )
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            image_pub2_.publish(msg);
//             msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
//             image_pub2_.publish(msg);
        }
        

    }
    
    Mat select_rotation(vector<Mat>* data)
    {
//         cout << "################################# \n SELECT ROTATION" << endl;
        double lowest_error=9999999;
        int index=0;
        
        double current_error=0;
        Mat ith(3,1,CV_64F);
        Mat jth(3,1,CV_64F);
        
        for(int i=0 ; i < data->size() ; i++)
        {
            for(int j=0 ; j < data->size() ; j++)
            {
                ith.at<double>(0,0)=data->at(i).at<double>(0,0);
                ith.at<double>(1,0)=data->at(i).at<double>(1,0);
                ith.at<double>(2,0)=data->at(i).at<double>(2,0);
                
                jth.at<double>(0,0)=data->at(j).at<double>(0,0);
                jth.at<double>(1,0)=data->at(j).at<double>(1,0);
                jth.at<double>(2,0)=data->at(j).at<double>(2,0);
//                 cout << ith << endl;
//                 cout << jth << endl;
//                 cout <<"Error being calculated" << endl;
//                 Mat error=data->at(j)-data->at(i);
                Mat error=jth-ith;
//                 cout <<"Error calculated" << endl;
                current_error=current_error+norm(error);
//                 cout <<"_currentError calculated" << endl;
            }
            if(current_error<lowest_error)
            {
                lowest_error=current_error;
                index=i;
            }
            current_error=0;
        }
        cout << lowest_error << endl;
        return data->at(index);
    }
    


    
private:
    cv::Mat current_image;
    vector<Point3f> marker3_3D, marker2_3D;
    vector<double> distortion;
    Ptr<aruco::Dictionary> mark_dict;
    Mat_<float> camera_matrix, camera_matrix_updated;

    vector< vector<Point2f> > markerCorners, rejectedImagePoints; // So, the first corner is the top left corner, followed by the top right, bottom right and bottom left.
    vector<int> markerIds;

    int count=0;
    int Marker_ID;
    bool print_pixel_values;
    
    double marker2_size, marker3_size;
    
    bool chose_a_rotation=false;
    int count_of_rotatations=0;
    
    vector<Mat> initialize_marker2,initialize_marker3;
    bool initialized_marker2=false,initialized_marker3=false;
    
    Mat distortion_mat;
    
    Mat rvec_marker2,rvec_marker3;
    
    float filtering_constant;
    
};

int main(int argc, char** argv)
{
    if (argv[1] == "2" ) 
        ros::init(argc, argv, "image_converter");
    else 
        ros::init(argc, argv, "image_converter2");
    ImageConverter ic(argv[1]);
  ROS_INFO("Starting camera node up");
  ros::spin();
  return 0;
}

