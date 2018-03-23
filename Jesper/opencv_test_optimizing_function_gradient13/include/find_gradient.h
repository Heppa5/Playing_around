#ifndef FIND_GRADIENT3_H
#define FIND_GRADIENT3_H
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

void find_gradient13_v2(const double wTc0[16], const double wTt[16], const
  double tTm0[16], const double dtTm[16], const double cTm[16], const double
  tTm_vec[3], double gradient_error[6])
{
  double error[12];
  double dv0[72];
  int i0;
  int i1;

  /* function gradient_error = find_gradient(x1,y1,z1,alpha1,beta1,gamma1,x2,y2,z2,alpha2,beta2,gamma2,x3,y3,z3,robot_trans) */
  /*  Based on fullRotation_stillEuler_T0_wCoordinates_only_tcpTmarker.m  */
  /* a1,b1, g1 -> EAA of tTm */
  /*  x1, y1, z1 -> translation of tTm */
  /*  x3, y3, z3 -> camera coordinate for m 1 - the measured model.  */
  /*  robot_trans -> wTt -> robots own configuration  */
  /*  extracting data from dwTc  */
  /*  tTm_vec=Rodrigues(dtTm(1:3,1:3)); */
  /* extracting camera coordinate */
  /* x3=cam_point(1,1);y3=cam_point(2,1);z3=cam_point(3,1); */
  /*  Only 9 parameters gets updated; */
  error[0] = ((((((tTm_vec[1] * ((tTm0[8] * wTt[0] + tTm0[9] * wTt[4]) + tTm0[10]
    * wTt[8]) - tTm_vec[2] * ((tTm0[4] * wTt[0] + tTm0[5] * wTt[4]) + tTm0[6] *
    wTt[8])) + cTm[0] * wTc0[0]) + cTm[1] * wTc0[4]) + cTm[2] * wTc0[8]) - tTm0
               [0] * wTt[0]) - tTm0[1] * wTt[4]) - tTm0[2] * wTt[8];
  error[1] = ((((((tTm_vec[2] * ((tTm0[0] * wTt[0] + tTm0[1] * wTt[4]) + tTm0[2]
    * wTt[8]) - tTm_vec[0] * ((tTm0[8] * wTt[0] + tTm0[9] * wTt[4]) + tTm0[10] *
    wTt[8])) + cTm[4] * wTc0[0]) + cTm[5] * wTc0[4]) + cTm[6] * wTc0[8]) - tTm0
               [4] * wTt[0]) - tTm0[5] * wTt[4]) - tTm0[6] * wTt[8];
  error[2] = ((((((tTm_vec[0] * ((tTm0[4] * wTt[0] + tTm0[5] * wTt[4]) + tTm0[6]
    * wTt[8]) - tTm_vec[1] * ((tTm0[0] * wTt[0] + tTm0[1] * wTt[4]) + tTm0[2] *
    wTt[8])) + cTm[8] * wTc0[0]) + cTm[9] * wTc0[4]) + cTm[10] * wTc0[8]) -
               tTm0[8] * wTt[0]) - tTm0[9] * wTt[4]) - tTm0[10] * wTt[8];
  error[3] = (((((((((wTc0[12] - wTt[12]) - dtTm[12] * ((tTm0[0] * wTt[0] +
    tTm0[1] * wTt[4]) + tTm0[2] * wTt[8])) - dtTm[13] * ((tTm0[4] * wTt[0] +
    tTm0[5] * wTt[4]) + tTm0[6] * wTt[8])) - dtTm[14] * ((tTm0[8] * wTt[0] +
    tTm0[9] * wTt[4]) + tTm0[10] * wTt[8])) - tTm0[12] * wTt[0]) - tTm0[13] *
                 wTt[4]) - tTm0[14] * wTt[8]) + wTc0[0] * cTm[12]) + wTc0[4] *
              cTm[13]) + wTc0[8] * cTm[14];
  error[4] = ((((((tTm_vec[1] * ((tTm0[8] * wTt[1] + tTm0[9] * wTt[5]) + tTm0[10]
    * wTt[9]) - tTm_vec[2] * ((tTm0[4] * wTt[1] + tTm0[5] * wTt[5]) + tTm0[6] *
    wTt[9])) + cTm[0] * wTc0[1]) + cTm[1] * wTc0[5]) + cTm[2] * wTc0[9]) - tTm0
               [0] * wTt[1]) - tTm0[1] * wTt[5]) - tTm0[2] * wTt[9];
  error[5] = ((((((tTm_vec[2] * ((tTm0[0] * wTt[1] + tTm0[1] * wTt[5]) + tTm0[2]
    * wTt[9]) - tTm_vec[0] * ((tTm0[8] * wTt[1] + tTm0[9] * wTt[5]) + tTm0[10] *
    wTt[9])) + cTm[4] * wTc0[1]) + cTm[5] * wTc0[5]) + cTm[6] * wTc0[9]) - tTm0
               [4] * wTt[1]) - tTm0[5] * wTt[5]) - tTm0[6] * wTt[9];
  error[6] = ((((((tTm_vec[0] * ((tTm0[4] * wTt[1] + tTm0[5] * wTt[5]) + tTm0[6]
    * wTt[9]) - tTm_vec[1] * ((tTm0[0] * wTt[1] + tTm0[1] * wTt[5]) + tTm0[2] *
    wTt[9])) + cTm[8] * wTc0[1]) + cTm[9] * wTc0[5]) + cTm[10] * wTc0[9]) -
               tTm0[8] * wTt[1]) - tTm0[9] * wTt[5]) - tTm0[10] * wTt[9];
  error[7] = (((((((((wTc0[13] - wTt[13]) - dtTm[12] * ((tTm0[0] * wTt[1] +
    tTm0[1] * wTt[5]) + tTm0[2] * wTt[9])) - dtTm[13] * ((tTm0[4] * wTt[1] +
    tTm0[5] * wTt[5]) + tTm0[6] * wTt[9])) - dtTm[14] * ((tTm0[8] * wTt[1] +
    tTm0[9] * wTt[5]) + tTm0[10] * wTt[9])) - tTm0[12] * wTt[1]) - tTm0[13] *
                 wTt[5]) - tTm0[14] * wTt[9]) + wTc0[1] * cTm[12]) + wTc0[5] *
              cTm[13]) + wTc0[9] * cTm[14];
  error[8] = ((((((tTm_vec[1] * ((tTm0[8] * wTt[2] + tTm0[9] * wTt[6]) + tTm0[10]
    * wTt[10]) - tTm_vec[2] * ((tTm0[4] * wTt[2] + tTm0[5] * wTt[6]) + tTm0[6] *
    wTt[10])) + cTm[0] * wTc0[2]) + cTm[1] * wTc0[6]) + cTm[2] * wTc0[10]) -
               tTm0[0] * wTt[2]) - tTm0[1] * wTt[6]) - tTm0[2] * wTt[10];
  error[9] = ((((((tTm_vec[2] * ((tTm0[0] * wTt[2] + tTm0[1] * wTt[6]) + tTm0[2]
    * wTt[10]) - tTm_vec[0] * ((tTm0[8] * wTt[2] + tTm0[9] * wTt[6]) + tTm0[10] *
    wTt[10])) + cTm[4] * wTc0[2]) + cTm[5] * wTc0[6]) + cTm[6] * wTc0[10]) -
               tTm0[4] * wTt[2]) - tTm0[5] * wTt[6]) - tTm0[6] * wTt[10];
  error[10] = ((((((tTm_vec[0] * ((tTm0[4] * wTt[2] + tTm0[5] * wTt[6]) + tTm0[6]
    * wTt[10]) - tTm_vec[1] * ((tTm0[0] * wTt[2] + tTm0[1] * wTt[6]) + tTm0[2] *
    wTt[10])) + cTm[8] * wTc0[2]) + cTm[9] * wTc0[6]) + cTm[10] * wTc0[10]) -
                tTm0[8] * wTt[2]) - tTm0[9] * wTt[6]) - tTm0[10] * wTt[10];
  error[11] = (((((((((wTc0[14] - wTt[14]) - dtTm[12] * ((tTm0[0] * wTt[2] +
    tTm0[1] * wTt[6]) + tTm0[2] * wTt[10])) - dtTm[13] * ((tTm0[4] * wTt[2] +
    tTm0[5] * wTt[6]) + tTm0[6] * wTt[10])) - dtTm[14] * ((tTm0[8] * wTt[2] +
    tTm0[9] * wTt[6]) + tTm0[10] * wTt[10])) - tTm0[12] * wTt[2]) - tTm0[13] *
                  wTt[6]) - tTm0[14] * wTt[10]) + wTc0[2] * cTm[12]) + wTc0[6] *
               cTm[13]) + wTc0[10] * cTm[14];
  dv0[0] = 0.0;
  dv0[1] = 0.0;
  dv0[2] = 0.0;
  dv0[3] = 0.0;
  dv0[4] = (tTm0[8] * wTt[0] + tTm0[9] * wTt[4]) + tTm0[10] * wTt[8];
  dv0[5] = (-tTm0[4] * wTt[0] - tTm0[5] * wTt[4]) - tTm0[6] * wTt[8];
  dv0[6] = 0.0;
  dv0[7] = 0.0;
  dv0[8] = 0.0;
  dv0[9] = (-tTm0[8] * wTt[0] - tTm0[9] * wTt[4]) - tTm0[10] * wTt[8];
  dv0[10] = 0.0;
  dv0[11] = (tTm0[0] * wTt[0] + tTm0[1] * wTt[4]) + tTm0[2] * wTt[8];
  dv0[12] = 0.0;
  dv0[13] = 0.0;
  dv0[14] = 0.0;
  dv0[15] = (tTm0[4] * wTt[0] + tTm0[5] * wTt[4]) + tTm0[6] * wTt[8];
  dv0[16] = (-tTm0[0] * wTt[0] - tTm0[1] * wTt[4]) - tTm0[2] * wTt[8];
  dv0[17] = 0.0;
  dv0[18] = (-tTm0[0] * wTt[0] - tTm0[1] * wTt[4]) - tTm0[2] * wTt[8];
  dv0[19] = (-tTm0[4] * wTt[0] - tTm0[5] * wTt[4]) - tTm0[6] * wTt[8];
  dv0[20] = (-tTm0[8] * wTt[0] - tTm0[9] * wTt[4]) - tTm0[10] * wTt[8];
  dv0[21] = 0.0;
  dv0[22] = 0.0;
  dv0[23] = 0.0;
  dv0[24] = 0.0;
  dv0[25] = 0.0;
  dv0[26] = 0.0;
  dv0[27] = 0.0;
  dv0[28] = (tTm0[8] * wTt[1] + tTm0[9] * wTt[5]) + tTm0[10] * wTt[9];
  dv0[29] = (-tTm0[4] * wTt[1] - tTm0[5] * wTt[5]) - tTm0[6] * wTt[9];
  dv0[30] = 0.0;
  dv0[31] = 0.0;
  dv0[32] = 0.0;
  dv0[33] = (-tTm0[8] * wTt[1] - tTm0[9] * wTt[5]) - tTm0[10] * wTt[9];
  dv0[34] = 0.0;
  dv0[35] = (tTm0[0] * wTt[1] + tTm0[1] * wTt[5]) + tTm0[2] * wTt[9];
  dv0[36] = 0.0;
  dv0[37] = 0.0;
  dv0[38] = 0.0;
  dv0[39] = (tTm0[4] * wTt[1] + tTm0[5] * wTt[5]) + tTm0[6] * wTt[9];
  dv0[40] = (-tTm0[0] * wTt[1] - tTm0[1] * wTt[5]) - tTm0[2] * wTt[9];
  dv0[41] = 0.0;
  dv0[42] = (-tTm0[0] * wTt[1] - tTm0[1] * wTt[5]) - tTm0[2] * wTt[9];
  dv0[43] = (-tTm0[4] * wTt[1] - tTm0[5] * wTt[5]) - tTm0[6] * wTt[9];
  dv0[44] = (-tTm0[8] * wTt[1] - tTm0[9] * wTt[5]) - tTm0[10] * wTt[9];
  dv0[45] = 0.0;
  dv0[46] = 0.0;
  dv0[47] = 0.0;
  dv0[48] = 0.0;
  dv0[49] = 0.0;
  dv0[50] = 0.0;
  dv0[51] = 0.0;
  dv0[52] = (tTm0[8] * wTt[2] + tTm0[9] * wTt[6]) + tTm0[10] * wTt[10];
  dv0[53] = (-tTm0[4] * wTt[2] - tTm0[5] * wTt[6]) - tTm0[6] * wTt[10];
  dv0[54] = 0.0;
  dv0[55] = 0.0;
  dv0[56] = 0.0;
  dv0[57] = (-tTm0[8] * wTt[2] - tTm0[9] * wTt[6]) - tTm0[10] * wTt[10];
  dv0[58] = 0.0;
  dv0[59] = (tTm0[0] * wTt[2] + tTm0[1] * wTt[6]) + tTm0[2] * wTt[10];
  dv0[60] = 0.0;
  dv0[61] = 0.0;
  dv0[62] = 0.0;
  dv0[63] = (tTm0[4] * wTt[2] + tTm0[5] * wTt[6]) + tTm0[6] * wTt[10];
  dv0[64] = (-tTm0[0] * wTt[2] - tTm0[1] * wTt[6]) - tTm0[2] * wTt[10];
  dv0[65] = 0.0;
  dv0[66] = (-tTm0[0] * wTt[2] - tTm0[1] * wTt[6]) - tTm0[2] * wTt[10];
  dv0[67] = (-tTm0[4] * wTt[2] - tTm0[5] * wTt[6]) - tTm0[6] * wTt[10];
  dv0[68] = (-tTm0[8] * wTt[2] - tTm0[9] * wTt[6]) - tTm0[10] * wTt[10];
  dv0[69] = 0.0;
  dv0[70] = 0.0;
  dv0[71] = 0.0;
  for (i0 = 0; i0 < 6; i0++) {
    gradient_error[i0] = 0.0;
    for (i1 = 0; i1 < 12; i1++) {
      gradient_error[i0] += dv0[i0 + 6 * i1] * error[i1];
    }
  }

  /* gradient_error = zeros(9,1); */
}


// void find_gradient11_v2(const double wTc0[16], const double wTt[16], const
//   double tTm0[16], const double dwTc[16], const double dtTm[16], const double
//   cTm[16], const double tTm_vec[3], const double dcRw_vec[3], double
//   gradient_error[12])

Mat find_gradient_wrapper_tcpTmarker(Mat wTc_orig, Mat wTt_kth, Mat tcpTmarker_orig, Mat tcpTmarker_update, Mat cTmarker)
{
  Mat result_gradient=Mat::zeros(6,1,CV_32F);

  double wTc0[4][4];
  double wTt[4][4], tTm0[4][4], dtTm[4][4],cTm[4][4], cam_point[3], dwRc_vec[3],dtTm_vec[3], gradient_error[6];
  for(int i=0; i<4 ; i++)
  {
    for(int j=0; j<4 ; j++)
    {
      // apparently the one-dimensional array intreprets the 2 dimension array from top to bottom, then switches column
      // that's why I switch index
      wTc0[j][i]=(double)wTc_orig.at<float>(i,j);
      wTt[j][i]=(double)wTt_kth.at<float>(i,j);
      tTm0[j][i]=(double)tcpTmarker_orig.at<float>(i,j);
      dtTm[j][i]=(double)tcpTmarker_update.at<float>(i,j);
      cTm[j][i]=(double)cTmarker.at<float>(i,j);
    }
  }

  dtTm_vec[0]=(double)tcpTmarker_update.at<float>(2,1);
  dtTm_vec[1]=(double)tcpTmarker_update.at<float>(0,2);
  dtTm_vec[2]=(double)tcpTmarker_update.at<float>(1,0);



  find_gradient13_v2( *wTc0,   *wTt,   *tTm0,   *dtTm,  *cTm, dtTm_vec,  gradient_error);

  for(int i=0; i<6 ;i++)
  {
    result_gradient.at<float>(i,0)=(float)gradient_error[i];
  }
  return result_gradient;
}

#endif