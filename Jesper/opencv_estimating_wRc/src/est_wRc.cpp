#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream> 


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
    
    // From here: https://stackoverflow.com/questions/34391968/how-to-find-the-rotation-matrix-between-two-coordinate-systems 
    Mat Xcam = (Mat_<double>(3, 1) <<  1, 0, 0);
    Mat Ycam = (Mat_<double>(3, 1) <<  0, 1, 0);
    Mat Zcam = (Mat_<double>(3, 1) <<  0, 0, 1);
    
    Mat tool_start = (Mat_<double>(3, 1) <<  0, 0, 1);
    Mat Xtool = (Mat_<double>(3, 1) <<  -0.075, -0.006, 0.933);
    Mat Ytool = (Mat_<double>(3, 1) <<  0.065, -0.014, 0.929);
    Mat Ztool = (Mat_<double>(3, 1) <<  -0.008, -0.098, 1.017);
    
    Mat Xtool_updated=Xtool-tool_start;
    Mat Xtool_updated_unit=Xtool_updated/norm(Xtool_updated);
    cout << Xtool_updated << " , " << norm(Xtool_updated) << endl;
    Mat Ytool_updated=Ytool-tool_start;
    Mat Ytool_updated_unit=Ytool_updated/norm(Ytool_updated);
    cout << Ytool_updated << " , " << norm(Ytool_updated) << endl;
    Mat Ztool_updated=Ztool-tool_start;
    Mat Ztool_updated_unit=Ztool_updated/norm(Ztool_updated);
    cout << Ztool_updated << " , " << norm(Ztool_updated) << endl;
    
    cout << dot_product(Xtool_updated_unit,Ytool_updated_unit) << " , " << dot_product(Xtool_updated_unit,Ztool_updated_unit)  << " , " << dot_product(Ytool_updated_unit,Ztool_updated_unit) << endl;
    
    Mat R = (Mat_<double>(3, 3) <<  dot_product(Xtool_updated_unit,Xcam), dot_product(Xtool_updated_unit,Ycam), dot_product(Xtool_updated_unit,Zcam),
                                    dot_product(Ytool_updated_unit,Xcam), dot_product(Ytool_updated_unit,Ycam), dot_product(Ytool_updated_unit,Zcam),
                                    dot_product(Ztool_updated_unit,Xcam), dot_product(Ztool_updated_unit,Ycam), dot_product(Ztool_updated_unit,Zcam));
    
    Mat R_correct = (Mat_<double>(3, 3) <<  -0.73812979, -0.057609037, -0.6721946,
                                            0.67215085, -0.14862442, -0.72534406,
                                            -0.058118165, -0.98721433, 0.14842606);
    
    cout << endl << "#############################################" << endl;
    cout << "Estimated R: " << endl << R << endl;
    cout << "correct R from Kabsch: " << endl << R_correct << endl;
    
    // Next finding translation of wTc
    //  TCP_camera is at "tool_start"
    Mat wPtcp = (Mat_<double>(3, 1) <<  0.031, 0.267, 0.916);
    
    Mat t_eRot=-R*tool_start+wPtcp;
    Mat t_cRot=-R_correct*tool_start+wPtcp;
    cout << "Translation using the estimated rotation: " << t_eRot << endl;
    cout << "Translation using the Kabsch rotation: " << t_cRot << endl;
    
    
    
    
}
