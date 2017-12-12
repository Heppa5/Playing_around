#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream> 


using namespace std;
using namespace cv;


int main(int argc, char** argv )
{
//     // Example for Marker 2 where the sign changes
//     Mat rvec = Mat::zeros(3, 1, CV_32F);
//     rvec.at<float>(0,0)=2.34380435621;
//     rvec.at<float>(1,0)=2.21257691282;
//     rvec.at<float>(2,0)=0.361833057989;
//     Mat R1;
//     Rodrigues(rvec,R1);
//     rvec.at<float>(0,0)=-2.19502895874;
//     rvec.at<float>(1,0)=-2.06967660715;
//     rvec.at<float>(2,0)=-0.340373051424;
    
//     // Marker 1
//     Mat rvec = Mat::zeros(3, 1, CV_32F);
//     rvec.at<float>(0,0)=-2.18159525612;
//     rvec.at<float>(1,0)=2.01906042229;
//     rvec.at<float>(2,0)=-1.19363963406;
//     Mat R1;
//     Rodrigues(rvec,R1);
//     rvec.at<float>(0,0)=2.23957072942;
//     rvec.at<float>(1,0)=-2.08362111228;
//     rvec.at<float>(2,0)=-0.550177695098;
    
    // Marker 1
    Mat rvec = Mat::zeros(3, 1, CV_32F);
    rvec.at<float>(0,0)=1;
    rvec.at<float>(1,0)=0;
    rvec.at<float>(2,0)=0;
    Mat R1;
    Rodrigues(rvec,R1);
    rvec.at<float>(0,0)=0;
    rvec.at<float>(1,0)=0;
    rvec.at<float>(2,0)=0;
    
    
    Mat R2;
    Rodrigues(rvec,R2);
    
    Mat R1t;
    Mat R2t;
    
    transpose(R1,R1t);
    transpose(R2,R2t);
    
    Mat R21t=R2*R1t;
    Mat R12t=R1*R2t;
    
    Mat R21t_vec, R12t_vec;
    
    Rodrigues(R21t,R21t_vec);
    Rodrigues(R12t,R12t_vec);
    
    cout << "R1: " << R1 << endl;
    cout << "R2: " << R2 << endl;
    
    
    cout << "R21t:" << endl;
    cout << R21t << endl;
    cout << R21t_vec << endl;
    
    cout << "R12t:" << endl;
    cout << R12t << endl;
    cout << R12t_vec << endl;
    



    
    // http://nghiaho.com/?page_id=671
    
//     vector<Point3f> Points_frame1;
//     Points_frame1.push_back(Point3f(1,2,4));
//     Points_frame1.push_back(Point3f(3,6,7));
//     Points_frame1.push_back(Point3f(5,0,9));
//     Points_frame1.push_back(Point3f(4,7,7));
//     Points_frame1.push_back(Point3f(8,3,1));
    
//     vector<Mat> Points_frame1;
//     Points_frame1.push_back(Point4d(1,2,4,1));
//     vector<Mat> Points_frame1;
// 
//     Mat_<float> point = (Mat_<float>(4,1) <<    1, 2, 3,1);
//     Points_frame1.push_back(point);
//     point = (Mat_<float>(4,1) <<    2, 8, 4,1);
//     Points_frame1.push_back(point);
//     point = (Mat_<float>(4,1) <<    6, 7,1,1);
//     Points_frame1.push_back(point);
//        
//     
//     vector<Mat> Points_frame2;
// 
//     Mat rvec = Mat::zeros(3, 1, CV_32F);
//     rvec.at<float>(1,0)=3.14/4;
//     rvec.at<float>(2,0)=3.14/8;
//     Mat R;
//     Rodrigues(rvec,R);
// 
//     cout << "Rotation matrix: " << endl << R << endl;
// 
//     Mat trans = Mat::eye(4, 4, CV_32F);
// 
//     R.copyTo(trans(Rect(0,0,R.cols,R.rows))); // Rect(x,y,R.cols,R.rows)
//     trans.at<float>(0,3)=1.2;
//     trans.at<float>(1,3)=5;
//     trans.at<float>(2,3)=3.7;
// 
//     cout << trans << endl;
// 
//     for(int i=0 ; i<Points_frame1.size(); i++)
//     {
//         Points_frame2.push_back(trans*Points_frame1[i]);
//         cout << Points_frame2[i] << endl;
//     }
//     
//     // Resizing Points from homogeneous to normal.
//     vector<Mat> Points_frame1_3;
//     vector<Mat> Points_frame2_3;
//     for(int i=0 ; i<Points_frame1.size(); i++)
//     {
//         Mat shit=Mat::zeros(3,1,CV_32F);
//         Mat shit2=Mat::zeros(3,1,CV_32F);
//         Points_frame1[i](Rect(0,0,1,3)).copyTo(shit);
//         Points_frame1_3.push_back(shit);
//         
//         Points_frame2[i](Rect(0,0,1,3)).copyTo(shit2);
//         Points_frame2_3.push_back(shit2);
//         
//     }
//     
//     
//     // finding centroids
//     Mat frame1_sum=Mat::zeros(3,1,CV_32F);
//     Mat frame2_sum=Mat::zeros(3,1,CV_32F);
//     for(int i=0 ; i<Points_frame1_3.size(); i++)
//     {
//         frame1_sum=frame1_sum+Points_frame1_3[i];
//         frame2_sum=frame2_sum+Points_frame2_3[i];
//     }
//     frame1_sum=frame1_sum/((float)Points_frame1_3.size());
//     frame2_sum=frame2_sum/((float)Points_frame2_3.size());
//     
//     cout << "Centroids: " << endl << frame1_sum << endl << frame2_sum << endl;
//     cout << "Centroids: " << endl << frame1_sum-frame2_sum << endl;
//     
//     
//     
//     
// 
//     
//     // Using SVD - create H matrix
//     Mat H = Mat::zeros(3,3,CV_32F);
//     for(int i=0 ; i<Points_frame1.size(); i++)
//     {
//         Mat transposed;
//         transpose(Points_frame2_3[i]-frame2_sum,transposed);
//         H=H+(Points_frame1_3[i]-frame1_sum)*transposed;
//     }
//     
//     Mat w,u,vt,v,ut;
//     SVD::compute(H,w,u,vt);
//     transpose(vt,v);
//     transpose(u,ut);
// 
//     Mat R_computed=v*ut;
//     
//     cout << R_computed << endl;
//     
//     // getting the translation
//     
//     Mat t_computed= -R_computed*frame1_sum+frame2_sum;
//     
//     cout << t_computed << endl;
//     
//     cout << "Difference: " << endl << (R_computed-R) << endl;
//     
//     return 0;
}
