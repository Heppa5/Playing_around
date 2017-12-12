#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream> 


using namespace std;
using namespace cv;



int main(int argc, char** argv )
{
    Mat Marker;
    
    Ptr<aruco::Dictionary> mark_dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
//     aruco::drawMarker(mark_dict, 2, 500, Marker, 1);
    aruco::drawMarker(mark_dict, 3, 500, Marker, 1);
    imwrite("Arucomarker.png",Marker);
    
    return 0;
}
