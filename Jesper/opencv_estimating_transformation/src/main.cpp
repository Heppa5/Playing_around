#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream> 


using namespace std;
using namespace cv;


Mat inverse_T(Mat T)
{
    Mat T_inverse= Mat::eye(4, 4, CV_32F);
    Mat translation=Mat::zeros(3, 1, CV_32F);
    Mat R= Mat::zeros(3, 3, CV_32F);
    T(Rect(0,0,3,3)).copyTo(R);
    cout << "##########################################\n" << R << endl;
    
    Mat R_trans;
    transpose(R,R_trans);
    cout << R_trans << endl;
    R_trans.copyTo(T_inverse(Rect(0,0,R_trans.cols,R_trans.rows)));
    cout << T_inverse << endl;
    Mat P=T(Rect(3,0,1,3));
    Mat P_inv=-(R_trans*P);
    P_inv.copyTo(T_inverse(Rect(3,0,P_inv.cols,P_inv.rows)));
    cout << T_inverse << endl;
    return T_inverse;
}

//     template <class T>
//     const Transform3D<T> inverse(const Transform3D<T>& aTb)
//     {
//         return Transform3D<T>(
//             -(inverse(aTb.R()) * aTb.P()),
//             inverse(aTb.R()));
//     }

int main(int argc, char** argv )
{
    // http://nghiaho.com/?page_id=671
    
//     vector<Point3f> Points_frame1;
//     Points_frame1.push_back(Point3f(1,2,4));
//     Points_frame1.push_back(Point3f(3,6,7));
//     Points_frame1.push_back(Point3f(5,0,9));
//     Points_frame1.push_back(Point3f(4,7,7));
//     Points_frame1.push_back(Point3f(8,3,1));
    
//     vector<Mat> Points_frame1;
//     Points_frame1.push_back(Point4d(1,2,4,1));
    vector<Mat> Points_frame1;

    Mat_<float> point = (Mat_<float>(4,1) <<    1, 2, 3,1);
//     Points_frame1.push_back(point);
//     point = (Mat_<float>(4,1) <<    2, 8, 4,1);
//     Points_frame1.push_back(point);
//     point = (Mat_<float>(4,1) <<    6, 7,1,1);
//     Points_frame1.push_back(point);
       
    cout << "Original point: " << point << endl;
    vector<Mat> Points_frame2;

    Mat rvec = Mat::zeros(3, 1, CV_32F);
    rvec.at<float>(1,0)=3.14/4;
    rvec.at<float>(2,0)=3.14/8;
    rvec.at<float>(3,0)=0.07;
    Mat R;
    Rodrigues(rvec,R);

//    cout << "Rotation matrix: " << endl << R << endl;

    Mat trans = Mat::eye(4, 4, CV_32F);

    R.copyTo(trans(Rect(0,0,R.cols,R.rows))); // Rect(x,y,R.cols,R.rows)
    trans.at<float>(0,3)=1.2;
    trans.at<float>(1,3)=5;
    trans.at<float>(2,3)=3.7;

//    cout << trans << endl;
//    cout << inverse_T(trans) <<endl;
    Mat point_transformed=trans*point;
    cout << inverse_T(trans)*trans << endl;
    cout << inverse_T(trans)*trans*point << endl;
    cout << inverse_T(trans)*point_transformed << endl;
//     cout << "Point transformed:" << point_transformed << endl;
//     
//     
//     Mat R_trans;
//     transpose(R,R_trans);
//     Mat Tinv=Mat::eye(4, 4, CV_32F);
//     Mat tvec=Mat::zeros(3, 1, CV_32F);
//     Mat tvec2=Mat::zeros(3, 1, CV_32F);
//     Mat tvec3=Mat::zeros(3, 1, CV_32F);
//     tvec.at<float>(0,3)=1.2;
//     tvec.at<float>(1,3)=5;
//     tvec.at<float>(2,3)=3.7;
    
}




    
    
//     R_trans.copyTo(Tinv(Rect(0,0,R_trans.cols,R_trans.rows)));
//     tvec2=-R_trans*tvec;
//     tvec3=-R_trans*(tvec*(-1));
//     
//     Tinv.at<float>(0,3)=tvec2.at<float>(0,0);
//     Tinv.at<float>(1,3)=tvec2.at<float>(1,0);
//     Tinv.at<float>(2,3)=tvec2.at<float>(2,0);
//     Mat point1=Tinv*point_transformed;
//     cout << "Attempt 1: " << point1 << endl;
//     
//     Tinv.at<float>(0,3)=tvec3.at<float>(0,0);
//     Tinv.at<float>(1,3)=tvec3.at<float>(1,0);
//     Tinv.at<float>(2,3)=tvec3.at<float>(2,0);
//     point1=Tinv*point_transformed;
//     cout << "Attempt 2: " << point1 << endl;
    
    

    
    
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
    
//     return 0;
// }
