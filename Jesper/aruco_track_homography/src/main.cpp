#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream> 


using namespace std;
using namespace cv;

Mat get_A_i(Mat projection,Mat point)
{
    Mat A_i;
    Mat Q1=projection.row(0);
    Q1=Q1.colRange(0,3);
    Mat Q2=projection.row(1);
    Q2=Q2.colRange(0,3);
    Mat Q3=projection.row(2);
    Q3=Q3.colRange(0,3);
    
    A_i = Mat::zeros(2, 3, CV_64F);
    
    Mat first=Q1-point.at<Vec2d>(0)[0]*Q3;
    Mat second=Q2-point.at<Vec2d>(0)[1]*Q3;

    first.copyTo(A_i(Rect(0, 0, first.cols, first.rows)));
    second.copyTo(A_i(Rect(0, 1, second.cols, second.rows)));
    

    
    return A_i;
}

Mat get_b_i(Mat projection,Mat point)
{
    Mat b_i;
    b_i = Mat::zeros(2, 1, CV_64F);
    b_i.at<double>(0,0) = point.at<Vec2d>(0)[0]*projection.at<double>(2,3)-projection.at<double>(0,3);
    b_i.at<double>(1,0) = point.at<Vec2d>(0)[1]*projection.at<double>(2,3)-projection.at<double>(1,3);
    
    
    //cout << b_i << endl;
    return b_i;
}

Mat calc_3d(Mat projection_l, Mat projection_r, Point2f left, Point2f right)
{
    Mat cam0pnts(1, 1, CV_64FC2);
    Mat cam1pnts(1, 1, CV_64FC2);
    cam0pnts.at<Vec2d>(0)[0] = left.x;
    cam0pnts.at<Vec2d>(0)[1] = left.y;
    cam1pnts.at<Vec2d>(0)[0] = right.x;
    cam1pnts.at<Vec2d>(0)[1] = right.y;
    
    Mat A1=get_A_i(projection_l,cam0pnts);
    Mat A2=get_A_i(projection_r,cam1pnts);
    
    Mat A;
    A = Mat::zeros(4, 3, CV_64F);
    A1.copyTo(A(Rect(0, 0, A1.cols, A1.rows)));
    A2.copyTo(A(Rect(0, 2, A2.cols, A2.rows)));
    
    
    Mat b1=get_b_i(projection_l,cam0pnts);
    Mat b2=get_b_i(projection_r,cam1pnts);
    Mat b;
    b = Mat::zeros(4, 1, CV_64F);
    b1.copyTo(b(Rect(0, 0, b1.cols, b1.rows)));
    b2.copyTo(b(Rect(0, 2, b2.cols, b2.rows)));
    
    Mat part=A.t()*A;
    Mat M=part.inv()*A.t()*b;
    
    cout << M << endl;
    
    return M;
}

int main(int argc, char** argv )
{
    Mat_<float> camera_matrix = (Mat_<float>(3,3) <<    1348.715676, 0.000000, 722.486120,
                                            0.000000, 1347.386029, 495.012476,
                                            0.000000, 0.000000, 1.000000);
    
    // Destination image = desired
    // source image = what you have now. 
    
    Mat image2=imread("../test_images/5cmf.jpg",1);
    Mat image1=imread("../test_images/20cmf.jpg",1);
    Ptr<aruco::Dictionary> mark_dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

    // image 1
    vector<int> markerIds1;
    vector< vector<Point2f> > markerCorners1;
    aruco::detectMarkers(image1, mark_dict, markerCorners1, markerIds1);
//     aruco::drawDetectedMarkers(image1, markerCorners1, markerIds1);
    
    // image 2
    vector<int> markerIds2;
    vector< vector<Point2f> > markerCorners2;
    aruco::detectMarkers(image2, mark_dict, markerCorners2, markerIds2);
//     aruco::drawDetectedMarkers(image2, markerCorners2, markerIds2);
    
    
    Mat H1=findHomography   ( 	markerCorners1[0],
                                    markerCorners2[0]
                            );	
    cout << H1 << endl;
    
    
    // Output image
    Mat im_out;
    // Warp source image to destination based on homography
    warpPerspective(image1, im_out, H1, image2.size());
 
    // Display images
//     imshow("Source Image", image1);
//     imshow("Destination Image", image2);
//     imshow("Warped Source Image", im_out);
//  
//     waitKey(0);
    
    vector<Mat> R, t,n;
    decomposeHomographyMat(H1,camera_matrix,R,t,n);
    
    
    // get Euclidian distance estimate
    vector<Point3f> marker_3D;
    marker_3D.push_back(Point3f(0,0,0));
    marker_3D.push_back(Point3f(0.169,0,0));
    marker_3D.push_back(Point3f(0.169,-0.169,0));
    marker_3D.push_back(Point3f(0,-0.169,0));
    
    Mat rvec, tvec,rvec2, tvec2;
    vector<double> distortion;
    distortion.push_back(-0.228385);
    distortion.push_back(0.266082);
    distortion.push_back(-0.001812);
    distortion.push_back(0.000035);
    distortion.push_back(0.000000);
    
    solvePnP(marker_3D,markerCorners1[0],camera_matrix,distortion,rvec,tvec);
    solvePnP(marker_3D,markerCorners2[0],camera_matrix,distortion,rvec2,tvec2);
    
    
    
     for( int i=0; i<R.size() ; i++)
    {
        if( t[i].at<double>(2,0)>0) // points must be in front of camera
        {
            cout << endl << "Solution " << i << endl; 
            cout << "Rotation: " << endl << R[i] << endl;
            cout << "Translation: " << endl << t[i] << endl;
            cout << "Estimated distance to camera in desired view " << norm(tvec2) << endl;
            cout << "Euclidian distance: " << norm(t[i])/**norm(tvec2)*/<< endl;
            cout <<"Normal vector: " << endl <<  n[i] << endl; 
        }
    }
    cout << "SolvePnP Euclidian distance: " << norm(tvec2-tvec) << endl;
    
//     
//     
//     Mat camera_matrix_extended = Mat::zeros(3, 4, CV_64F);
//     camera_matrix.copyTo(camera_matrix_extended(cv::Rect(0,0,camera_matrix.cols, camera_matrix.rows)));
//     
//     Mat trans1=Mat::eye(4, 4, CV_64F);
//     Mat trans2=Mat::eye(4, 4, CV_64F);
//     Mat trans3=Mat::eye(4, 4, CV_64F);
// 
//     Mat projection1=camera_matrix_extended*trans1;
//     Mat projection2=Mat::zeros(3, 4, CV_64F);
//     Mat projection3=Mat::zeros(3, 4, CV_64F);
//     
//     int count=0;
//     for( int i=0; i<R.size() ; i++)
//     {
//         if( t[i].at<double>(2,0)>0) // points must be in front of camera
//         {
//             cout << endl << "Solution " << i << endl; 
//             cout << "Rotation: " << endl << R[i] << endl;
//             cout << "Translation: " << endl << t[i] << endl;
//             cout << "Euclidian distance: " << sqrt(pow(t[i].at<double>(0,0),2)+pow(t[i].at<double>(1,0),2)+pow(t[i].at<double>(2,0),2)) << endl;
//             cout <<"Normal vector: " << endl <<  n[i] << endl; 
//             if(count==0)
//             {
//                 cout << "Jeg er her" << endl;
//                 R[i].copyTo(trans2(cv::Rect(0,0,R[i].cols, R[i].rows)));
//                 t[i].copyTo(trans2(cv::Rect(3,0,t[i].cols, t[i].rows)));
//                 projection2=camera_matrix_extended*trans2;
//             }
//             else
//             {
//                 cout << "Jeg er her2" << endl;
//                 R[i].copyTo(trans3(cv::Rect(0,0,R[i].cols, R[i].rows)));
//                 t[i].copyTo(trans3(cv::Rect(3,0,t[i].cols, t[i].rows)));
//                 projection3=camera_matrix_extended*trans3;
//             }
//             count++;
//         }
//     }
//     cout << "projection1:" << projection1 << endl;
//     cout << "projection2:" << projection2<< endl;
//     cout << "projection3:" << projection3 << endl;
//     
//     
//     // point1
//     Mat M1_2=calc_3d(projection1,projection2,markerCorners1[0][0],markerCorners2[0][0]);
//     Mat M1_3=calc_3d(projection1,projection3,markerCorners1[0][0],markerCorners2[0][0]);
//     
//     // point2
//     Mat M2_2=calc_3d(projection1,projection2,markerCorners1[0][1],markerCorners2[0][1]);
//     Mat M2_3=calc_3d(projection1,projection3,markerCorners1[0][1],markerCorners2[0][1]);
//     
// //     // point1
// //     Mat M1_2=calc_3d(projection1,projection2,markerCorners2[0][0],markerCorners1[0][0]);
// //     Mat M1_3=calc_3d(projection1,projection3,markerCorners2[0][0],markerCorners1[0][0]);
// //     
// //     // point2
// //     Mat M2_2=calc_3d(projection1,projection2,markerCorners2[0][1],markerCorners1[0][1]);
// //     Mat M2_3=calc_3d(projection1,projection3,markerCorners2[0][1],markerCorners1[0][1]);
//     
//     Mat dist1=(M1_2-M2_2);
//     Mat dist2=(M1_3-M2_3);
//      
//     cout << sqrt(pow(dist1.at<double>(0,0),2)+pow(dist1.at<double>(1,0),2)+pow(dist1.at<double>(2,0),2)) << endl;
//     cout << sqrt(pow(dist2.at<double>(0,0),2)+pow(dist2.at<double>(1,0),2)+pow(dist2.at<double>(2,0),2)) << endl;
//     
//     cout << "3d koordinater mellem punkt 1 og 2 i de to pose løsninger" << endl;
//     cout << (M1_2-M2_2) << endl;
//     
//     cout << (M1_3-M2_3) << endl;
//     
//     
//     namedWindow("Display Image_green", WINDOW_AUTOSIZE );
//     imshow("Display Image_green", image2);
//     
//     namedWindow("Display Image_green3", WINDOW_AUTOSIZE );
//     imshow("Display Image_green3", image1);
//     
//     waitKey(0);
    
    
    return 0;
}
