#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream> 
#include <find_gradient.h>
#include "../include/find_gradient.h"


using namespace std;
using namespace cv;

double dot_product(Mat vec1, Mat vec2)
{
    if(vec1.rows==vec2.rows && vec1.cols == vec2.cols && vec1.cols==1)
    {
        double result=0;
        for(int i=0; i<vec1.rows ; i++)
        {
            result=result+vec1.at<double>(i,0)*vec2.at<double>(i,0);
        }
        return result;
    }
    else
    {
        cout << "Weird dimensions of your vectors" << endl;
    }
}


int main(int argc, char** argv )
{
     // One datapoint to test
    Mat hej;

    Mat wTc = (Mat_<float>(4, 4) << 0.85251993, 0.10858911, -0.50973761, 1.0800163,
                                        0.55342996, -0.10745841, 0.82497406, -1.1997528,
                                        0.033956207, -0.98651284, -0.15509245, 0.062517919,
                                        0, 0, 0, 1);
    Mat tcpTmarker = (Mat_<float>(4, 4) <<  0.0069673657, -0.027343974, 0.9938972, 0.028517187,
                                            -0.00068785436, -1.0042343, -0.016774893, -0.11282735,
                                            1.0005599, -0.012419891, -0.012510687, 0.063874424,
                                            0, 0, 0, 1);
    
    
    Mat wRt_vec = (Mat_<float>(3, 1) <<  2.60355 , -1.71225 , -0.156108);
    Mat wTt_tvec = (Mat_<float>(3, 1) << 0.29124 , -0.368471 , 0.0416441);
    Mat wTt=Mat::zeros(4,4,CV_32F);
    wTt.at<float>(3,3) = 1;
    Rodrigues(wRt_vec,wTt(Rect(0,0,3,3)));
    wTt_tvec.copyTo(wTt(Rect(3,0,1,3)));
    cout << wTt << endl;
    
    Mat cTw_update=Mat::eye(4,4,CV_32F);
    Mat tcpTmarker_update=Mat::eye(4,4,CV_32F);
    
    cout << "JESPER" << endl;
    Mat cTm=Mat::zeros(4,4,CV_32F);
    cTm.at<float>(3,3) = 1;
    Mat cTm_tvec = (Mat_<float>(3, 1) << -0.137337 , -0.0774432 , 1.11488);
    Mat cTm_rvec = (Mat_<float>(3, 1) << -1.92436 , -2.13001 , -0.0766579);
    Rodrigues(cTm_rvec,cTm(Rect(0,0,3,3)));
    cTm_tvec.copyTo(cTm(Rect(3,0,1,3)));
    cout << cTm << endl;
    
    
    // Mat find_gradient_wrapper(Mat wTc_orig, Mat wTt_kth, Mat tcpTmarker_orig, Mat wTc_update, Mat tcpTmarker_update, Mat cTmarker)
    cout << find_gradient_wrapper(wTc, wTt, tcpTmarker, cTw_update, tcpTmarker_update, cTm) << endl;
    
    
//     cout << "OpenCV version : " << CV_VERSION << endl;
//     
//     Mat wTc = (Mat_<float>(4, 4) << 0.85251993, 0.10858911, -0.50973761, 1.0800163,
//                                         0.55342996, -0.10745841, 0.82497406, -1.1997528,
//                                         0.033956207, -0.98651284, -0.15509245, 0.062517919,
//                                         0, 0, 0, 1);
//         
//     Mat tcpTmarker = (Mat_<float>(4, 4) <<  0.0069673657, -0.027343974, 0.9938972, 0.028517187,
//                                             -0.00068785436, -1.0042343, -0.016774893, -0.11282735,
//                                             1.0005599, -0.012419891, -0.012510687, 0.063874424,
//                                             0, 0, 0, 1);
//     
//     Mat cPmarker=(Mat_<float>(3, 1) <<-0.0245988,-0.0767337,1.01616);
//     Mat cRmarker_vec=(Mat_<float>(3, 1) <<0.428997,-0.383085,0.0578148);
//     Mat cRmarker;
//     Rodrigues(cRmarker_vec,cRmarker);
//     Mat cTmarker=Mat::eye(4,4,CV_32F);
//     cRmarker.copyTo(cTmarker(Rect(0,0,3,3)));
//     cPmarker.copyTo(cTmarker(Rect(3,0,1,3)));
//     
//     Mat wPtcp=(Mat_<float>(3, 1) <<0.428997,-0.383085,0.0578148);
//     Mat wRtcp_vec=(Mat_<float>(3, 1) <<2.60342,-1.71274,-0.15769);
//     Mat wRtcp;
//     Rodrigues(wRtcp_vec,wRtcp);
//     Mat wTtcp=Mat::eye(4,4,CV_32F);
//     wRtcp.copyTo(wTtcp(Rect(0,0,3,3)));
//     wPtcp.copyTo(wTtcp(Rect(3,0,1,3)));
//     
//     cout << "wTmarker: \n" << cTmarker << endl;
//     cout << "forward kinematics: \n " <<wTc*wTtcp*tcpTmarker << endl;
//     wRtcp_vec=(Mat_<float>(3, 1) <<-1.21554 , -2.44584 , -0.388837);
//     Rodrigues(wRtcp_vec,wRtcp);
//     cout << wRtcp << endl;
}
