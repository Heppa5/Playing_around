#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream> 

#include <random>

using namespace std;
using namespace cv;

// Mat RPY_to_rotation(float a, float b, float c) // should be RPY -> zyx
// {
//     float  ca = cos(a);
//     float  sa = sin(a);
//     float  cb = cos(b);
//     float  sb = sin(b);
//     float  cc = cos(c);
//     float  sc = sin(c);
// 
//     Mat rotation=(Mat_<float>(3, 3) << ca * cb, ca * sb * sc-sa * cc, ca * sb * cc+sa * sc,
//                                         sa * cb, sa * sb * sc+ca * cc, sa * sb * cc-ca * sc,
//                                         -sb, cb * sc, cb * cc);
//     return rotation;
// }

Mat roll(float angle)
{
    float  ca = cos(angle);
    float  sa = sin(angle);
    Mat rotation=(Mat_<float>(3, 3) << ca,-sa,0,
                                        sa,ca,0,
                                        0,0,1);
    return rotation;
}

Mat pitch(float angle)
{
    float  ca = cos(angle);
    float  sa = sin(angle);
    Mat rotation=(Mat_<float>(3, 3) << ca,0,sa,
                                        0,1,0,
                                        -sa,0,ca);
    return rotation;
}  

Mat yaw(float angle)
{
    float  ca = cos(angle);
    float  sa = sin(angle);
    Mat rotation=(Mat_<float>(3, 3) << 1,0,0,
                                        0,ca,-sa,
                                        0,sa,ca);
    return rotation;
}  
Mat RPY_to_rotation(float r, float p, float y) 
{
    return (roll(r)*pitch(p)*yaw(y));
}


double get_sum(vector<Mat> data, string variable1="0", string variable2="0", string variable3="0")
{
    double sum=0;
    for(Mat data_point : data)
    {
        if(variable1=="x"&&variable2=="0"&&variable3=="0")
        {
            sum=sum+data_point.at<double>(0,0);
        }
        else if(variable1=="y"&&variable2=="0"&&variable3=="0")
        {
            sum=sum+data_point.at<double>(1,0);
        }
        else if(variable1=="z"&&variable2=="0"&&variable3=="0")
        {
            sum=sum+data_point.at<double>(2,0);
        }
        else if(variable1=="x"&&variable2=="x"&&variable3=="0")
        {
            sum=sum+data_point.at<double>(0,0)*data_point.at<double>(0,0);
        }
        else if(variable1=="y"&&variable2=="y"&&variable3=="0")
        {
            sum=sum+data_point.at<double>(1,0)*data_point.at<double>(1,0);
        }
        else if(variable1=="z"&&variable2=="z"&&variable3=="0")
        {
            sum=sum+data_point.at<double>(2,0)*data_point.at<double>(2,0);
        }
        else if(variable1=="x"&&variable2=="y"&&variable3=="0")
        {
            sum=sum+data_point.at<double>(0,0)*data_point.at<double>(1,0);
        }
        else if(variable1=="x"&&variable2=="z"&&variable3=="0")
        {
            sum=sum+data_point.at<double>(0,0)*data_point.at<double>(2,0);
        }
        else if(variable1=="y"&&variable2=="z"&&variable3=="0")
        {
            sum=sum+data_point.at<double>(1,0)*data_point.at<double>(2,0);
        }
        else if(variable1=="x"&&variable2=="x"&&variable3=="x")
        {
            sum=sum+data_point.at<double>(0,0)*data_point.at<double>(0,0)*data_point.at<double>(0,0);
        }
        else if(variable1=="y"&&variable2=="y"&&variable3=="y")
        {
            sum=sum+data_point.at<double>(1,0)*data_point.at<double>(1,0)*data_point.at<double>(1,0);
        }
        else if(variable1=="z"&&variable2=="z"&&variable3=="z")
        {
            sum=sum+data_point.at<double>(2,0)*data_point.at<double>(2,0)*data_point.at<double>(2,0);
        }
        else if(variable1=="x"&&variable2=="y"&&variable3=="y")
        {
            sum=sum+data_point.at<double>(0,0)*data_point.at<double>(1,0)*data_point.at<double>(1,0);
        }
        else if(variable1=="x"&&variable2=="z"&&variable3=="z")
        {
            sum=sum+data_point.at<double>(0,0)*data_point.at<double>(2,0)*data_point.at<double>(2,0);
        }
        else if(variable1=="x"&&variable2=="x"&&variable3=="y")
        {
            sum=sum+data_point.at<double>(0,0)*data_point.at<double>(0,0)*data_point.at<double>(1,0);
        }
        else if(variable1=="x"&&variable2=="x"&&variable3=="z")
        {
            sum=sum+data_point.at<double>(0,0)*data_point.at<double>(0,0)*data_point.at<double>(2,0);
        }
        else if(variable1=="y"&&variable2=="y"&&variable3=="z")
        {
            sum=sum+data_point.at<double>(1,0)*data_point.at<double>(1,0)*data_point.at<double>(2,0);
        }
        else if(variable1=="y"&&variable2=="z"&&variable3=="z")
        {
            sum=sum+data_point.at<double>(1,0)*data_point.at<double>(2,0)*data_point.at<double>(2,0);
        }
    }
}

// int random_in_range(int min, int max) {
//     std::random_device rd;
//     std::uniform_int_distribution<int> uni(min, max);
//     return uni(rd());
// }
Mat generate_random_vector(double length,double mean=0.0, double std_dev=10.0)
{
//     std::default_random_engine generator(random_in_range(0,99999999999));
    std::random_device generator;
    std::normal_distribution<double> distribution(mean,std_dev); // mean, std. dev

    double x=distribution(generator);
    double y=distribution(generator);
    double z=distribution(generator);
    double factor=sqrt(pow(x,2)+pow(y,2)+pow(z,2));
    Mat result=Mat::zeros(3,1,CV_64F);
    result.at<double>(0,0)=x/factor*length;
    result.at<double>(1,0)=y/factor*length;
    result.at<double>(2,0)=z/factor*length;

    return result;
}


int main(int argc, char** argv )
{

    
    
    // inspired by https://math.stackexchange.com/questions/1585975/how-to-generate-random-points-on-a-sphere
    // and formulaes from "Fast Geometric Fit Algorithm for Sphere Using Exact Solution"
    const int nrolls=20;  // number of experiments
    int N=nrolls;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(5.0,2.0); // mean, std. dev
    std::normal_distribution<double> error(0,0.002); // mean, std. dev

    vector<Mat> points;
    double x_origin=2;
    double y_origin=3;
    double z_origin=4;
    double r=6.5;
    double x,y,z;
    for (int i=0; i<nrolls; ++i) {
        x = distribution(generator);
        y = distribution(generator);
        z = distribution(generator);
        if(!(x==0 && y==0 && z==0))
        {
            double x_error = error(generator);
            double y_error = error(generator);
            double z_error = error(generator);
            cout << x_error << " , " << y_error << " , " << z_error << endl;
            
            double factor=sqrt(pow(x,2)+pow(y,2)+pow(z,2));
            Mat point=Mat::zeros(3,1,CV_64F);
            point.at<double>(0,0)=x/factor*r+x_origin+x_error;
            point.at<double>(1,0)=y/factor*r+y_origin+y_error;
            point.at<double>(2,0)=z/factor*r+z_origin+z_error;
            points.push_back(point);
            cout << point << endl;
            cout << norm(point) << endl;
        }
    }
//     int N=6;
//     vector<Mat> points;
//     
//     Mat point1 = Mat::zeros(3,1,CV_64F);
//     point1.at<double>(0,0)=1;
//     point1.at<double>(1,0)=5;
//     point1.at<double>(2,0)=9;
//     points.push_back(point1);
//     
//     Mat point2 = Mat::zeros(3,1,CV_64F);
//     point2.at<double>(0,0)=2;
//     point2.at<double>(1,0)=6;
//     point2.at<double>(2,0)=10;
//     points.push_back(point2);
//     Mat point3 = Mat::zeros(3,1,CV_64F);
//     point3.at<double>(0,0)=3;
//     point3.at<double>(1,0)=7;
//     point3.at<double>(2,0)=11;
//     points.push_back(point3);
//     Mat point4 = Mat::zeros(3,1,CV_64F);
//     point4.at<double>(0,0)=4;
//     point4.at<double>(1,0)=8;
//     point4.at<double>(2,0)=12;
//     points.push_back(point4);
//     Mat point5 = Mat::zeros(3,1,CV_64F);
//     point5.at<double>(0,0)=5;
//     point5.at<double>(1,0)=9;
//     point5.at<double>(2,0)=13;
//     points.push_back(point5);
//     Mat point6 = Mat::zeros(3,1,CV_64F);
//     point6.at<double>(0,0)=6;
//     point6.at<double>(1,0)=10;
//     point6.at<double>(2,0)=14;
//     points.push_back(point6);
    
//     double Sx=get_sum(points,"x");
//     double Sy=get_sum(points,"y");
//     double Sz=get_sum(points,"z");
//     cout << Sx << " , " << Sy << " , " << Sz << endl;
//     
//     double Sxx=get_sum(points,"x","x");
//     double Syy=get_sum(points,"y","y");
//     double Szz=get_sum(points,"z","z");
//     double Sxy=get_sum(points,"x","y");
//     double Sxz=get_sum(points,"x","z");
//     double Syz=get_sum(points,"y","z");
//     cout << endl;
//     cout << Sxx << " , " << Syy << " , " << Szz << endl;
//     cout << Sxy << " , " << Sxz << " , " << Syz << endl;
//     
//     double Sxxx=get_sum(points,"x","x","x");
//     double Syyy=get_sum(points,"y","y","y");
//     double Szzz=get_sum(points,"z","z","z");
//     double Sxyy=get_sum(points,"x","y","y");
//     double Sxzz=get_sum(points,"x","z","z");
//     double Sxxy=get_sum(points,"x","x","y");
//     double Sxxz=get_sum(points,"x","x","z");
//     double Syyz=get_sum(points,"y","y","z");:
//     double Syzz=get_sum(points,"y","z","z");
//     
//     cout << endl;
//     cout << Sxxx << " , " << Syyy << " , " << Szzz << endl;
//     cout << Sxyy << " , " << Syzz << " , " << Sxxy << endl;
//     cout << Sxxz << " , " << Syyz << " , " << Syzz << endl;
//     
//     double A1 = Sxx +Syy +Szz;
// 
//     double a=2*Sx*Sx-2*N*Sxx;
//     double b=2*Sx*Sy-2*N*Sxy;
//     double c=2*Sx*Sz-2*N*Sxz;
//     double d=-N*(Sxxx +Sxyy +Sxzz)+A1*Sx;
//     
//     double e=2*Sx*Sy-2*N*Sxy;
//     double f=2*Sy*Sy-2*N*Syy;
//     double g=2*Sy*Sz-2*N*Syz;
//     double h=-N*(Sxxy +Syyy +Syzz)+A1*Sy;
// 
//     double j=2*Sx*Sz-2*N*Sxz;
//     double k=2*Sy*Sz-2*N*Syz;
//     double l=2*Sz*Sz-2*N*Szz;
//     double m=-N*(Sxxz +Syyz + Szzz)+A1*Sz;
//     double delta = a*(f*l - g*k)-e*(b*l-c*k) + j*(b*g-c*f);
//     
//     double xc = (d*(f*l-g*k) -h*(b*l-c*k) +m*(b*g-c*f))/delta;
//     double yc = (a*(h*l-m*g) -e*(d*l-m*c) +j*(d*g-h*c))/delta;
//     double zc = (a*(f*m-h*k) -e*(b*m-d*k) +j*(b*h-d*f))/delta;
//     double R = sqrt(pow(xc,2)+pow(yc,2)+pow(zc,2)+(A1-2*(xc*Sx+yc*Sy+zc*Sz))/N);
//     
//     cout << endl << "######################################" << endl;
//     cout << "True origin values: " << x_origin << " , " << y_origin << " , " << z_origin << endl;
//     cout << "Estimated origin values from noisy input " <<xc << " , " << yc << " , " << zc << endl;
//     cout << "True radius: " << r << endl;
//     cout << "Estimated radius: " << R << endl; 
    
    cout << RPY_to_rotation(0,0,0) << endl;
    
    cout << RPY_to_rotation(M_PI/2,0,0) << endl;
    
    cout << RPY_to_rotation(0,0,M_PI_2) << endl;
    
    cout << RPY_to_rotation(3.14,-M_PI/2,0) << endl;
    cout << RPY_to_rotation(0,-M_PI/2,3.14) << endl;
}
