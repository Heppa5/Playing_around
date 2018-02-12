#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream> 


#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace cv;

struct Quaternion {
    double x;
    double y;
    double z;
    double w;
};


struct Point_ros{
    double x;
    double y;
    double z;
};

struct Pose{
    Point_ros position;
    Quaternion orientation;
};

namespace geometry_msgs
{
    struct PoseStamped{
        Pose pose;
    };
}


struct matchedPoint {
    geometry_msgs::PoseStamped camera;
    geometry_msgs::PoseStamped robot;
} ;

vector <matchedPoint> chosenMatchedPoints, updatedChosenMatchedPoints;


void load_dataset()
{
    bool gain_first_element=false;
    string chosenPoint;
        ifstream infile;
        infile.open("/home/jepod13/Jesper/Estimating_tcp_to_marker/data.txt");
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
                            hej.robot.pose.position.x=number;
                            break;
                        case 4:
                            hej.robot.pose.position.y=number;
                            break;
                        case 5:
                            hej.robot.pose.position.z=number;
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
                            hej.robot.pose.position.x=number;
                            break;
                        case 4:
                            hej.robot.pose.position.y=number;
                            break;
                        case 5:
                            hej.robot.pose.position.z=number;
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
            cout << chosenMatchedPoints[i].robot.pose.position.x << ",";
            cout << chosenMatchedPoints[i].robot.pose.position.y << ",";
            cout << chosenMatchedPoints[i].robot.pose.position.z << "\n";
            
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

geometry_msgs::PoseStamped convert_mat_to_geomsg(Mat point)
{
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x=point.at<float>(0,0);
    msg.pose.position.y=point.at<float>(1,0);
    msg.pose.position.z=point.at<float>(2,0);
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

Mat find_transformation(vector<matchedPoint> &dataMatchedPoints)
{
    // finding centroids
    // frame 1 is camera
    Mat frame1_sum=Mat::zeros(3,1,CV_32F);
    Mat frame2_sum=Mat::zeros(3,1,CV_32F);
    for(int i=0 ; i<dataMatchedPoints.size(); i++)
    {
        Mat cam_point=convert_geomsg_to_mat(dataMatchedPoints[i].camera);
        Mat rob_point=convert_geomsg_to_mat(dataMatchedPoints[i].robot);

        frame1_sum=frame1_sum+cam_point;
        frame2_sum=frame2_sum+rob_point;
    }



    frame1_sum=frame1_sum/((float)dataMatchedPoints.size());
    frame2_sum=frame2_sum/((float)dataMatchedPoints.size());



    // Using SVD - create H matrix
    Mat H = Mat::zeros(3,3,CV_32F);
    for(int i=0 ; i<dataMatchedPoints.size(); i++)
    {
        Mat transposed;
        transpose(convert_geomsg_to_mat(dataMatchedPoints[i].robot)-frame2_sum,transposed);
        H=H+(convert_geomsg_to_mat(dataMatchedPoints[i].camera)-frame1_sum)*transposed;
    }

    Mat w,u,vt,v,ut;
    SVD::compute(H,w,u,vt);
    transpose(vt,v);
    transpose(u,ut);

    Mat R_computed=v*ut;

    if(cv::determinant(R_computed) < 0)
    {
//             cout << "----------------------------"  << " Reflection special case " << " ----------------------------" << endl;
//             cout << "Before "  << R_computed << endl;
        for(int i=0; i<R_computed.rows ; i++)
        {
            R_computed.at<float>(i,2)=R_computed.at<float>(i,2)*(-1);
        }
//             cout << "After "  << R_computed << endl;
//
//             cout << "----------------------------"  << " END: Reflection special case " << " ----------------------------" << endl;
    }

    // getting the translation

    Mat t_computed= -R_computed*frame1_sum+frame2_sum;

    cout <<R_computed << endl;
    cout << t_computed << endl;

    Mat T = Mat::zeros(4,4,CV_32F);
    T.at<float>(3,3)=1;

    R_computed.copyTo(T(Rect(0,0,R_computed.cols,R_computed.rows)));

    T.at<float>(0,3)=t_computed.at<float>(0,0);
    T.at<float>(1,3)=t_computed.at<float>(1,0);
    T.at<float>(2,3)=t_computed.at<float>(2,0);
    cout << T << endl;
    return T;
}



int main(int argc, char** argv )
{
    load_dataset();
    Mat current_T=find_transformation(chosenMatchedPoints);

    for(int i=0 ; i<chosenMatchedPoints.size() ;i++)
    {
        Mat P=current_T*convert_geomsg_to_hommat(chosenMatchedPoints[i].camera);
        Mat P_robot=convert_geomsg_to_hommat(chosenMatchedPoints[i].robot);
        cout << "############\n"<<(P-P_robot) << endl;
        matchedPoint copy=chosenMatchedPoints[i];
        copy.camera=convert_mat_to_geomsg(P);
        updatedChosenMatchedPoints.push_back(copy);
    }

    cout << "################################3"<< endl;
    find_transformation(updatedChosenMatchedPoints);

    
    return 0;
}
