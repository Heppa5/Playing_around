#ifndef FIND_GRADIENT3_H
#define FIND_GRADIENT3_H
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

void find_gradient_error(double cTw0[16],  double wTt[16],  double
                    tTm0[16],  double dcTw[16],  double dtTm[16],
                     double cam_point[3],  double dcRw_vec[3], double gradient_error[9])
{
  /*  Based on only_translation_skew_T0.m */
  /* a1,b1, g1 -> EAA of cTw */
  /*  x1, y1, z1 -> translation of cTw */
  /* a2,b2, g2 -> EAA of tTm */
  /*  x2, y2, z2 -> translation of tTm */
  /*  x3, y3, z3 -> camera coordinate for m 1 - the measured model.  */
  /*  robot_trans -> wTt -> robots own configuration  */
  /*  extracting data from dcTw  */
  /* dcRw_vec=[0,0,0];%rotationMatrixToVector(dcTw(1:3,1:3)); */
  /*  extracting data from dtTm */
  /* tTm_vec=[0,0,0];%rotationMatrixToVector(dtTm(1:3,1:3)); */
  /* extracting camera coordinate */
  /*  Only 9 parameters gets updated; */
  gradient_error[0] = (cTw0[0] * (((((((((((((cTw0[12] - cam_point[0]) + cTw0[0]
    * dcTw[12]) + cTw0[4] * dcTw[13]) + cTw0[8] * dcTw[14]) + wTt[14] * ((cTw0[8]
    - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0])) + dtTm[12] * ((tTm0[0] *
    ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0]) +
      wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec[2])) +
     wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])) +
    tTm0[1] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[2] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2]))))
    + tTm0[12] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[13] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[14] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])))) + wTt[13] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) -
    cTw0[0] * dcRw_vec[2])) + wTt[12] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) +
    cTw0[4] * dcRw_vec[2])) + cTw0[1] * (((((((((((((cTw0[13] - cam_point[1]) +
    cTw0[1] * dcTw[12]) + cTw0[5] * dcTw[13]) + cTw0[9] * dcTw[14]) + wTt[14] *
    ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1])) + dtTm[12] *
    ((tTm0[0] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2]))
      + tTm0[1] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
     + tTm0[2] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2]))))
    + tTm0[12] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[13] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[14] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])))) + wTt[13] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) -
    cTw0[1] * dcRw_vec[2])) + wTt[12] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) +
    cTw0[5] * dcRw_vec[2]))) + cTw0[2] * (((((((((((((cTw0[14] - cam_point[2]) +
    cTw0[2] * dcTw[12]) + cTw0[6] * dcTw[13]) + cTw0[10] * dcTw[14]) + wTt[14] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2])) + dtTm[12] *
    ((tTm0[0] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] *
    cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] *
    dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] *
    dcRw_vec[2])) + tTm0[1] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
    dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) -
    cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2]))) + tTm0[2] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])))) + tTm0[12] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0]
    * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] *
    cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] *
    cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[13] * ((wTt[6] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[14] * ((wTt[10] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + dtTm[13] *
    ((tTm0[4] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] *
    cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] *
    dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
    dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) -
    cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[10] *
    ((wTt[10] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) +
      wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
     wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])))) +
    wTt[13] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
    wTt[12] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]));
  gradient_error[1] = (cTw0[4] * (((((((((((((cTw0[12] - cam_point[0]) + cTw0[0]
    * dcTw[12]) + cTw0[4] * dcTw[13]) + cTw0[8] * dcTw[14]) + wTt[14] * ((cTw0[8]
    - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0])) + dtTm[12] * ((tTm0[0] *
    ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0]) +
      wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec[2])) +
     wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])) +
    tTm0[1] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[2] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2]))))
    + tTm0[12] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[13] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[14] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])))) + wTt[13] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) -
    cTw0[0] * dcRw_vec[2])) + wTt[12] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) +
    cTw0[4] * dcRw_vec[2])) + cTw0[5] * (((((((((((((cTw0[13] - cam_point[1]) +
    cTw0[1] * dcTw[12]) + cTw0[5] * dcTw[13]) + cTw0[9] * dcTw[14]) + wTt[14] *
    ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1])) + dtTm[12] *
    ((tTm0[0] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2]))
      + tTm0[1] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
     + tTm0[2] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2]))))
    + tTm0[12] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[13] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[14] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])))) + wTt[13] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) -
    cTw0[1] * dcRw_vec[2])) + wTt[12] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) +
    cTw0[5] * dcRw_vec[2]))) + cTw0[6] * (((((((((((((cTw0[14] - cam_point[2]) +
    cTw0[2] * dcTw[12]) + cTw0[6] * dcTw[13]) + cTw0[10] * dcTw[14]) + wTt[14] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2])) + dtTm[12] *
    ((tTm0[0] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] *
    cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] *
    dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] *
    dcRw_vec[2])) + tTm0[1] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
    dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) -
    cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2]))) + tTm0[2] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])))) + tTm0[12] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0]
    * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] *
    cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] *
    cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[13] * ((wTt[6] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[14] * ((wTt[10] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + dtTm[13] *
    ((tTm0[4] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] *
    cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] *
    dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
    dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) -
    cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[10] *
    ((wTt[10] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) +
      wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
     wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])))) +
    wTt[13] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
    wTt[12] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]));
  gradient_error[2] = (cTw0[8] * (((((((((((((cTw0[12] - cam_point[0]) + cTw0[0]
    * dcTw[12]) + cTw0[4] * dcTw[13]) + cTw0[8] * dcTw[14]) + wTt[14] * ((cTw0[8]
    - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0])) + dtTm[12] * ((tTm0[0] *
    ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0]) +
      wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec[2])) +
     wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])) +
    tTm0[1] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[2] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2]))))
    + tTm0[12] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[13] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[14] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])))) + wTt[13] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) -
    cTw0[0] * dcRw_vec[2])) + wTt[12] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) +
    cTw0[4] * dcRw_vec[2])) + cTw0[9] * (((((((((((((cTw0[13] - cam_point[1]) +
    cTw0[1] * dcTw[12]) + cTw0[5] * dcTw[13]) + cTw0[9] * dcTw[14]) + wTt[14] *
    ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1])) + dtTm[12] *
    ((tTm0[0] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2]))
      + tTm0[1] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
     + tTm0[2] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2]))))
    + tTm0[12] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[13] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[14] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])))) + wTt[13] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) -
    cTw0[1] * dcRw_vec[2])) + wTt[12] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) +
    cTw0[5] * dcRw_vec[2]))) + cTw0[10] * (((((((((((((cTw0[14] - cam_point[2])
    + cTw0[2] * dcTw[12]) + cTw0[6] * dcTw[13]) + cTw0[10] * dcTw[14]) + wTt[14]
    * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2])) + dtTm[12] *
    ((tTm0[0] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] *
    cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] *
    dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] *
    dcRw_vec[2])) + tTm0[1] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
    dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) -
    cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2]))) + tTm0[2] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])))) + tTm0[12] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0]
    * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] *
    cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] *
    cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[13] * ((wTt[6] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[14] * ((wTt[10] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + dtTm[13] *
    ((tTm0[4] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] *
    cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] *
    dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
    dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) -
    cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[10] *
    ((wTt[10] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) +
      wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
     wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])))) +
    wTt[13] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
    wTt[12] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]));
  gradient_error[3] = ((((((((dtTm[12] * ((tTm0[0] * (cTw0[8] * wTt[1] - cTw0[4]
    * wTt[2]) + tTm0[1] * (cTw0[8] * wTt[5] - cTw0[4] * wTt[6])) + tTm0[2] *
    (cTw0[8] * wTt[9] - cTw0[4] * wTt[10])) + dtTm[13] * ((tTm0[4] * (cTw0[8] *
    wTt[1] - cTw0[4] * wTt[2]) + tTm0[5] * (cTw0[8] * wTt[5] - cTw0[4] * wTt[6]))
    + tTm0[6] * (cTw0[8] * wTt[9] - cTw0[4] * wTt[10]))) + dtTm[14] * ((tTm0[8] *
    (cTw0[8] * wTt[1] - cTw0[4] * wTt[2]) + tTm0[9] * (cTw0[8] * wTt[5] - cTw0[4]
    * wTt[6])) + tTm0[10] * (cTw0[8] * wTt[9] - cTw0[4] * wTt[10]))) + cTw0[8] *
    wTt[13]) - cTw0[4] * wTt[14]) + tTm0[12] * (cTw0[8] * wTt[1] - cTw0[4] *
    wTt[2])) + tTm0[13] * (cTw0[8] * wTt[5] - cTw0[4] * wTt[6])) + tTm0[14] *
                        (cTw0[8] * wTt[9] - cTw0[4] * wTt[10])) *
                       (((((((((((((cTw0[12] - cam_point[0]) + cTw0[0] * dcTw[12])
    + cTw0[4] * dcTw[13]) + cTw0[8] * dcTw[14]) + wTt[14] * ((cTw0[8] -
    dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0])) + dtTm[12] * ((tTm0[0] *
    ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0]) +
      wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec[2])) +
     wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])) +
    tTm0[1] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[2] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2]))))
    + tTm0[12] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[13] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[14] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])))) + wTt[13] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) -
    cTw0[0] * dcRw_vec[2])) + wTt[12] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) +
    cTw0[4] * dcRw_vec[2])) + (((((((dtTm[12] * ((tTm0[0] * (cTw0[9] * wTt[1] -
    cTw0[5] * wTt[2]) + tTm0[1] * (cTw0[9] * wTt[5] - cTw0[5] * wTt[6])) + tTm0
    [2] * (cTw0[9] * wTt[9] - cTw0[5] * wTt[10])) + dtTm[13] * ((tTm0[4] *
    (cTw0[9] * wTt[1] - cTw0[5] * wTt[2]) + tTm0[5] * (cTw0[9] * wTt[5] - cTw0[5]
    * wTt[6])) + tTm0[6] * (cTw0[9] * wTt[9] - cTw0[5] * wTt[10]))) + dtTm[14] *
    ((tTm0[8] * (cTw0[9] * wTt[1] - cTw0[5] * wTt[2]) + tTm0[9] * (cTw0[9] *
    wTt[5] - cTw0[5] * wTt[6])) + tTm0[10] * (cTw0[9] * wTt[9] - cTw0[5] * wTt
    [10]))) + cTw0[9] * wTt[13]) - cTw0[5] * wTt[14]) + tTm0[12] * (cTw0[9] *
    wTt[1] - cTw0[5] * wTt[2])) + tTm0[13] * (cTw0[9] * wTt[5] - cTw0[5] * wTt[6]))
    + tTm0[14] * (cTw0[9] * wTt[9] - cTw0[5] * wTt[10])) * (((((((((((((cTw0[13]
    - cam_point[1]) + cTw0[1] * dcTw[12]) + cTw0[5] * dcTw[13]) + cTw0[9] *
    dcTw[14]) + wTt[14] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1])) + dtTm[12] * ((tTm0[0] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])) + tTm0[1] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) + tTm0[2] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])))) + tTm0[12] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) + tTm0[13] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) + tTm0[14] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[9] -
    dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] +
    dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] -
    dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])) + tTm0[5] * ((wTt[6] *
    ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[5] *
    ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec[2])) + wTt[4] *
    ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2]))) + tTm0[6] *
    ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1]) +
      wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec[2])) +
     wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))) +
    dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])))) + wTt[13] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] *
    dcRw_vec[2])) + wTt[12] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2]))) + (((((((dtTm[12] * ((tTm0[0] * (cTw0[10] * wTt[1] - cTw0[6] *
    wTt[2]) + tTm0[1] * (cTw0[10] * wTt[5] - cTw0[6] * wTt[6])) + tTm0[2] *
    (cTw0[10] * wTt[9] - cTw0[6] * wTt[10])) + dtTm[13] * ((tTm0[4] * (cTw0[10] *
    wTt[1] - cTw0[6] * wTt[2]) + tTm0[5] * (cTw0[10] * wTt[5] - cTw0[6] * wTt[6]))
    + tTm0[6] * (cTw0[10] * wTt[9] - cTw0[6] * wTt[10]))) + dtTm[14] * ((tTm0[8]
    * (cTw0[10] * wTt[1] - cTw0[6] * wTt[2]) + tTm0[9] * (cTw0[10] * wTt[5] -
    cTw0[6] * wTt[6])) + tTm0[10] * (cTw0[10] * wTt[9] - cTw0[6] * wTt[10]))) +
    cTw0[10] * wTt[13]) - cTw0[6] * wTt[14]) + tTm0[12] * (cTw0[10] * wTt[1] -
    cTw0[6] * wTt[2])) + tTm0[13] * (cTw0[10] * wTt[5] - cTw0[6] * wTt[6])) +
                      tTm0[14] * (cTw0[10] * wTt[9] - cTw0[6] * wTt[10])) *
    (((((((((((((cTw0[14] - cam_point[2]) + cTw0[2] * dcTw[12]) + cTw0[6] *
               dcTw[13]) + cTw0[10] * dcTw[14]) + wTt[14] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2])) + dtTm[12] * ((tTm0[0] *
              ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] *
    cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] *
    dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] *
    dcRw_vec[2])) + tTm0[1] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
    dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) -
    cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2]))) + tTm0[2] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])))) + tTm0[12] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0]
    * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] *
    cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] *
              cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[13] * ((wTt[6] *
            ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5]
            * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
           wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])))
         + tTm0[14] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
            dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0[10])
            - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0
            [10]) + cTw0[6] * dcRw_vec[2]))) + dtTm[13] * ((tTm0[4] * ((wTt[2] *
            ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1]
            * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
           wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))
          + tTm0[5] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec
             [1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) -
             cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0
             [10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[10]
             - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] *
           ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[8]
          * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])))) +
       dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
            dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10])
            - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0
            [10]) + cTw0[6] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[10] -
             dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6]
             + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] *
          ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0
                   [10] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
           dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0[10])
           - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0
           [10]) + cTw0[6] * dcRw_vec[2])))) + wTt[13] * ((cTw0[6] + dcRw_vec[0]
        * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[12] * ((cTw0[2] - dcRw_vec[1]
       * cTw0[10]) + cTw0[6] * dcRw_vec[2]));
  gradient_error[4] = (-(((((((dtTm[12] * ((tTm0[0] * (cTw0[8] * wTt[0] - cTw0[0]
    * wTt[2]) + tTm0[1] * (cTw0[8] * wTt[4] - cTw0[0] * wTt[6])) + tTm0[2] *
    (cTw0[8] * wTt[8] - cTw0[0] * wTt[10])) + dtTm[13] * ((tTm0[4] * (cTw0[8] *
    wTt[0] - cTw0[0] * wTt[2]) + tTm0[5] * (cTw0[8] * wTt[4] - cTw0[0] * wTt[6]))
    + tTm0[6] * (cTw0[8] * wTt[8] - cTw0[0] * wTt[10]))) + dtTm[14] * ((tTm0[8] *
    (cTw0[8] * wTt[0] - cTw0[0] * wTt[2]) + tTm0[9] * (cTw0[8] * wTt[4] - cTw0[0]
    * wTt[6])) + tTm0[10] * (cTw0[8] * wTt[8] - cTw0[0] * wTt[10]))) + cTw0[8] *
    wTt[12]) - cTw0[0] * wTt[14]) + tTm0[12] * (cTw0[8] * wTt[0] - cTw0[0] *
    wTt[2])) + tTm0[13] * (cTw0[8] * wTt[4] - cTw0[0] * wTt[6])) + tTm0[14] *
    (cTw0[8] * wTt[8] - cTw0[0] * wTt[10])) * (((((((((((((cTw0[12] - cam_point
    [0]) + cTw0[0] * dcTw[12]) + cTw0[4] * dcTw[13]) + cTw0[8] * dcTw[14]) +
    wTt[14] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0])) +
    dtTm[12] * ((tTm0[0] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])) + tTm0[1] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2]))) + tTm0[2] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])))) + tTm0[12] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2]))) + tTm0[13] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2]))) + tTm0[14] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2]))) + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[8] -
    dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] +
    dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] -
    dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] *
    ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[5] *
    ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec[2])) + wTt[4] *
    ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2]))) + tTm0[10] *
    ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0]) +
      wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec[2])) +
     wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))) +
    wTt[13] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec[2])) +
    wTt[12] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])) -
                       (((((((dtTm[12] * ((tTm0[0] * (cTw0[9] * wTt[0] - cTw0[1]
    * wTt[2]) + tTm0[1] * (cTw0[9] * wTt[4] - cTw0[1] * wTt[6])) + tTm0[2] *
    (cTw0[9] * wTt[8] - cTw0[1] * wTt[10])) + dtTm[13] * ((tTm0[4] * (cTw0[9] *
    wTt[0] - cTw0[1] * wTt[2]) + tTm0[5] * (cTw0[9] * wTt[4] - cTw0[1] * wTt[6]))
    + tTm0[6] * (cTw0[9] * wTt[8] - cTw0[1] * wTt[10]))) + dtTm[14] * ((tTm0[8] *
    (cTw0[9] * wTt[0] - cTw0[1] * wTt[2]) + tTm0[9] * (cTw0[9] * wTt[4] - cTw0[1]
    * wTt[6])) + tTm0[10] * (cTw0[9] * wTt[8] - cTw0[1] * wTt[10]))) + cTw0[9] *
    wTt[12]) - cTw0[1] * wTt[14]) + tTm0[12] * (cTw0[9] * wTt[0] - cTw0[1] *
    wTt[2])) + tTm0[13] * (cTw0[9] * wTt[4] - cTw0[1] * wTt[6])) + tTm0[14] *
                        (cTw0[9] * wTt[8] - cTw0[1] * wTt[10])) *
                       (((((((((((((cTw0[13] - cam_point[1]) + cTw0[1] * dcTw[12])
    + cTw0[5] * dcTw[13]) + cTw0[9] * dcTw[14]) + wTt[14] * ((cTw0[9] -
    dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1])) + dtTm[12] * ((tTm0[0] *
    ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1]) +
      wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec[2])) +
     wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])) +
    tTm0[1] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[2] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2]))))
    + tTm0[12] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[13] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[14] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])))) + wTt[13] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) -
    cTw0[1] * dcRw_vec[2])) + wTt[12] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) +
    cTw0[5] * dcRw_vec[2]))) - (((((((dtTm[12] * ((tTm0[0] * (cTw0[10] * wTt[0]
    - cTw0[2] * wTt[2]) + tTm0[1] * (cTw0[10] * wTt[4] - cTw0[2] * wTt[6])) +
    tTm0[2] * (cTw0[10] * wTt[8] - cTw0[2] * wTt[10])) + dtTm[13] * ((tTm0[4] *
    (cTw0[10] * wTt[0] - cTw0[2] * wTt[2]) + tTm0[5] * (cTw0[10] * wTt[4] -
    cTw0[2] * wTt[6])) + tTm0[6] * (cTw0[10] * wTt[8] - cTw0[2] * wTt[10]))) +
    dtTm[14] * ((tTm0[8] * (cTw0[10] * wTt[0] - cTw0[2] * wTt[2]) + tTm0[9] *
                 (cTw0[10] * wTt[4] - cTw0[2] * wTt[6])) + tTm0[10] * (cTw0[10] *
    wTt[8] - cTw0[2] * wTt[10]))) + cTw0[10] * wTt[12]) - cTw0[2] * wTt[14]) +
    tTm0[12] * (cTw0[10] * wTt[0] - cTw0[2] * wTt[2])) + tTm0[13] * (cTw0[10] *
    wTt[4] - cTw0[2] * wTt[6])) + tTm0[14] * (cTw0[10] * wTt[8] - cTw0[2] * wTt
    [10])) * (((((((((((((cTw0[14] - cam_point[2]) + cTw0[2] * dcTw[12]) + cTw0
                        [6] * dcTw[13]) + cTw0[10] * dcTw[14]) + wTt[14] *
                      ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]))
                     + dtTm[12] * ((tTm0[0] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0]
    * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] *
    cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] *
    cTw0[10]) + cTw0[6] * dcRw_vec[2])) + tTm0[1] * ((wTt[6] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[2] * ((wTt[10] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])))) + tTm0[12] *
                    ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1]
    * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] *
    dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] *
    dcRw_vec[2]))) + tTm0[13] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
    dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) -
    cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2]))) + tTm0[14] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2]))) + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])) + tTm0[5] * ((wTt[6] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[6] *
    ((wTt[10] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) +
      wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
     wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])))) +
                dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0]
    * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] *
    cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] *
    cTw0[10]) + cTw0[6] * dcRw_vec[2])))) + wTt[13] * ((cTw0[6] + dcRw_vec[0] *
    cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[12] * ((cTw0[2] - dcRw_vec[1] *
    cTw0[10]) + cTw0[6] * dcRw_vec[2]));
  gradient_error[5] = ((((((((dtTm[12] * ((tTm0[0] * (cTw0[4] * wTt[0] - cTw0[0]
    * wTt[1]) + tTm0[1] * (cTw0[4] * wTt[4] - cTw0[0] * wTt[5])) + tTm0[2] *
    (cTw0[4] * wTt[8] - cTw0[0] * wTt[9])) + dtTm[13] * ((tTm0[4] * (cTw0[4] *
    wTt[0] - cTw0[0] * wTt[1]) + tTm0[5] * (cTw0[4] * wTt[4] - cTw0[0] * wTt[5]))
    + tTm0[6] * (cTw0[4] * wTt[8] - cTw0[0] * wTt[9]))) + dtTm[14] * ((tTm0[8] *
    (cTw0[4] * wTt[0] - cTw0[0] * wTt[1]) + tTm0[9] * (cTw0[4] * wTt[4] - cTw0[0]
    * wTt[5])) + tTm0[10] * (cTw0[4] * wTt[8] - cTw0[0] * wTt[9]))) + cTw0[4] *
    wTt[12]) - cTw0[0] * wTt[13]) + tTm0[12] * (cTw0[4] * wTt[0] - cTw0[0] *
    wTt[1])) + tTm0[13] * (cTw0[4] * wTt[4] - cTw0[0] * wTt[5])) + tTm0[14] *
                        (cTw0[4] * wTt[8] - cTw0[0] * wTt[9])) *
                       (((((((((((((cTw0[12] - cam_point[0]) + cTw0[0] * dcTw[12])
    + cTw0[4] * dcTw[13]) + cTw0[8] * dcTw[14]) + wTt[14] * ((cTw0[8] -
    dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0])) + dtTm[12] * ((tTm0[0] *
    ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0]) +
      wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec[2])) +
     wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])) +
    tTm0[1] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[2] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2]))))
    + tTm0[12] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[13] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[14] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])))) + wTt[13] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) -
    cTw0[0] * dcRw_vec[2])) + wTt[12] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) +
    cTw0[4] * dcRw_vec[2])) + (((((((dtTm[12] * ((tTm0[0] * (cTw0[5] * wTt[0] -
    cTw0[1] * wTt[1]) + tTm0[1] * (cTw0[5] * wTt[4] - cTw0[1] * wTt[5])) + tTm0
    [2] * (cTw0[5] * wTt[8] - cTw0[1] * wTt[9])) + dtTm[13] * ((tTm0[4] * (cTw0
    [5] * wTt[0] - cTw0[1] * wTt[1]) + tTm0[5] * (cTw0[5] * wTt[4] - cTw0[1] *
    wTt[5])) + tTm0[6] * (cTw0[5] * wTt[8] - cTw0[1] * wTt[9]))) + dtTm[14] *
    ((tTm0[8] * (cTw0[5] * wTt[0] - cTw0[1] * wTt[1]) + tTm0[9] * (cTw0[5] *
    wTt[4] - cTw0[1] * wTt[5])) + tTm0[10] * (cTw0[5] * wTt[8] - cTw0[1] * wTt[9])))
    + cTw0[5] * wTt[12]) - cTw0[1] * wTt[13]) + tTm0[12] * (cTw0[5] * wTt[0] -
    cTw0[1] * wTt[1])) + tTm0[13] * (cTw0[5] * wTt[4] - cTw0[1] * wTt[5])) +
    tTm0[14] * (cTw0[5] * wTt[8] - cTw0[1] * wTt[9])) * (((((((((((((cTw0[13] -
    cam_point[1]) + cTw0[1] * dcTw[12]) + cTw0[5] * dcTw[13]) + cTw0[9] * dcTw
    [14]) + wTt[14] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1]))
    + dtTm[12] * ((tTm0[0] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])) + tTm0[1] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2]))) + tTm0[2] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])))) + tTm0[12] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2]))) + tTm0[13] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2]))) + tTm0[14] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2]))) + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[9] -
    dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] +
    dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] -
    dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] *
    ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[5] *
    ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec[2])) + wTt[4] *
    ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2]))) + tTm0[10] *
    ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1]) +
      wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec[2])) +
     wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))) +
    wTt[13] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec[2])) +
    wTt[12] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2]))) +
    (((((((dtTm[12] * ((tTm0[0] * (cTw0[6] * wTt[0] - cTw0[2] * wTt[1]) + tTm0[1]
                        * (cTw0[6] * wTt[4] - cTw0[2] * wTt[5])) + tTm0[2] *
                       (cTw0[6] * wTt[8] - cTw0[2] * wTt[9])) + dtTm[13] *
           ((tTm0[4] * (cTw0[6] * wTt[0] - cTw0[2] * wTt[1]) + tTm0[5] * (cTw0[6]
              * wTt[4] - cTw0[2] * wTt[5])) + tTm0[6] * (cTw0[6] * wTt[8] -
             cTw0[2] * wTt[9]))) + dtTm[14] * ((tTm0[8] * (cTw0[6] * wTt[0] -
             cTw0[2] * wTt[1]) + tTm0[9] * (cTw0[6] * wTt[4] - cTw0[2] * wTt[5]))
           + tTm0[10] * (cTw0[6] * wTt[8] - cTw0[2] * wTt[9]))) + cTw0[6] * wTt
         [12]) - cTw0[2] * wTt[13]) + tTm0[12] * (cTw0[6] * wTt[0] - cTw0[2] *
        wTt[1])) + tTm0[13] * (cTw0[6] * wTt[4] - cTw0[2] * wTt[5])) + tTm0[14] *
     (cTw0[6] * wTt[8] - cTw0[2] * wTt[9])) * (((((((((((((cTw0[14] - cam_point
    [2]) + cTw0[2] * dcTw[12]) + cTw0[6] * dcTw[13]) + cTw0[10] * dcTw[14]) +
    wTt[14] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2])) +
    dtTm[12] * ((tTm0[0] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
    dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) -
    cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2])) + tTm0[1] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2]))) + tTm0[2] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])))) + tTm0[12] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0]
    * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] *
    cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] *
    cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[13] * ((wTt[6] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[14] * ((wTt[10] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + dtTm[13] *
    ((tTm0[4] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] *
    cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] *
    dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
    dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) -
    cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[10] *
    ((wTt[10] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) +
      wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
     wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])))) +
    wTt[13] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
    wTt[12] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]));
  gradient_error[6] = (((tTm0[0] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4])
    + dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) -
    cTw0[0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) +
    cTw0[4] * dcRw_vec[2])) + tTm0[1] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2]))) + tTm0[2] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2]))) * (((((((((((((cTw0[12] - cam_point[0]) + cTw0[0]
    * dcTw[12]) + cTw0[4] * dcTw[13]) + cTw0[8] * dcTw[14]) + wTt[14] * ((cTw0[8]
    - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0])) + dtTm[12] * ((tTm0[0] *
    ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0]) +
      wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec[2])) +
     wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])) +
    tTm0[1] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[2] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2]))))
    + tTm0[12] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[13] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[14] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])))) + wTt[13] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) -
    cTw0[0] * dcRw_vec[2])) + wTt[12] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) +
    cTw0[4] * dcRw_vec[2])) + ((tTm0[0] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])) + tTm0[1] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) + tTm0[2] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) * (((((((((((((cTw0[13] - cam_point[1]) + cTw0[1]
    * dcTw[12]) + cTw0[5] * dcTw[13]) + cTw0[9] * dcTw[14]) + wTt[14] * ((cTw0[9]
    - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1])) + dtTm[12] * ((tTm0[0] *
    ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1]) +
      wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec[2])) +
     wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])) +
    tTm0[1] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[2] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2]))))
    + tTm0[12] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[13] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[14] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])))) + wTt[13] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) -
    cTw0[1] * dcRw_vec[2])) + wTt[12] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) +
    cTw0[5] * dcRw_vec[2]))) + ((tTm0[0] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])) + tTm0[1] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2]))) + tTm0[2] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2]))) * (((((((((((((cTw0[14] - cam_point[2]) + cTw0[2]
    * dcTw[12]) + cTw0[6] * dcTw[13]) + cTw0[10] * dcTw[14]) + wTt[14] * ((cTw0
    [10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2])) + dtTm[12] * ((tTm0
    [0] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2])
            + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec
                        [2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2])) + tTm0[1] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2]))) + tTm0[2] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])))) + tTm0[12] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0]
    * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] *
    cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] *
    cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[13] * ((wTt[6] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[14] * ((wTt[10] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + dtTm[13] *
    ((tTm0[4] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] *
    cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] *
    dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
    dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) -
    cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[10] *
    ((wTt[10] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) +
      wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
     wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])))) +
    wTt[13] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
    wTt[12] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]));
  gradient_error[7] = (((tTm0[4] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4])
    + dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) -
    cTw0[0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) +
    cTw0[4] * dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2]))) * (((((((((((((cTw0[12] - cam_point[0]) + cTw0[0]
    * dcTw[12]) + cTw0[4] * dcTw[13]) + cTw0[8] * dcTw[14]) + wTt[14] * ((cTw0[8]
    - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0])) + dtTm[12] * ((tTm0[0] *
    ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0]) +
      wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec[2])) +
     wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])) +
    tTm0[1] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[2] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2]))))
    + tTm0[12] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[13] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[14] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])))) + wTt[13] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) -
    cTw0[0] * dcRw_vec[2])) + wTt[12] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) +
    cTw0[4] * dcRw_vec[2])) + ((tTm0[4] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) * (((((((((((((cTw0[13] - cam_point[1]) + cTw0[1]
    * dcTw[12]) + cTw0[5] * dcTw[13]) + cTw0[9] * dcTw[14]) + wTt[14] * ((cTw0[9]
    - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1])) + dtTm[12] * ((tTm0[0] *
    ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1]) +
      wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec[2])) +
     wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])) +
    tTm0[1] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[2] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2]))))
    + tTm0[12] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[13] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[14] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])))) + wTt[13] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) -
    cTw0[1] * dcRw_vec[2])) + wTt[12] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) +
    cTw0[5] * dcRw_vec[2]))) + ((tTm0[4] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2]))) * (((((((((((((cTw0[14] - cam_point[2]) + cTw0[2]
    * dcTw[12]) + cTw0[6] * dcTw[13]) + cTw0[10] * dcTw[14]) + wTt[14] * ((cTw0
    [10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2])) + dtTm[12] * ((tTm0
    [0] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2])
            + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec
                        [2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2])) + tTm0[1] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2]))) + tTm0[2] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])))) + tTm0[12] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0]
    * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] *
    cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] *
    cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[13] * ((wTt[6] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[14] * ((wTt[10] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + dtTm[13] *
    ((tTm0[4] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] *
    cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] *
    dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
    dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) -
    cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[10] *
    ((wTt[10] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) +
      wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
     wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])))) +
    wTt[13] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
    wTt[12] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]));
  gradient_error[8] = (((tTm0[8] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4])
    + dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) -
    cTw0[0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) +
    cTw0[4] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2]))) * (((((((((((((cTw0[12] - cam_point[0]) + cTw0[0]
    * dcTw[12]) + cTw0[4] * dcTw[13]) + cTw0[8] * dcTw[14]) + wTt[14] * ((cTw0[8]
    - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0])) + dtTm[12] * ((tTm0[0] *
    ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] * cTw0[0]) +
      wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec[2])) +
     wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])) +
    tTm0[1] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[2] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2]))))
    + tTm0[12] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[13] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + tTm0[14] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) + dcRw_vec[1] *
    cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0[0] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] * dcRw_vec[2])))
    + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] * cTw0[4]) +
    dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) - cTw0
    [0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) + cTw0[4] *
    dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[1] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[0] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[5] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[4] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[8] - dcRw_vec[0] *
    cTw0[4]) + dcRw_vec[1] * cTw0[0]) + wTt[9] * ((cTw0[4] + dcRw_vec[0] * cTw0
    [8]) - cTw0[0] * dcRw_vec[2])) + wTt[8] * ((cTw0[0] - dcRw_vec[1] * cTw0[8])
    + cTw0[4] * dcRw_vec[2])))) + wTt[13] * ((cTw0[4] + dcRw_vec[0] * cTw0[8]) -
    cTw0[0] * dcRw_vec[2])) + wTt[12] * ((cTw0[0] - dcRw_vec[1] * cTw0[8]) +
    cTw0[4] * dcRw_vec[2])) + ((tTm0[8] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) * (((((((((((((cTw0[13] - cam_point[1]) + cTw0[1]
    * dcTw[12]) + cTw0[5] * dcTw[13]) + cTw0[9] * dcTw[14]) + wTt[14] * ((cTw0[9]
    - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1])) + dtTm[12] * ((tTm0[0] *
    ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] * cTw0[1]) +
      wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec[2])) +
     wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])) +
    tTm0[1] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[2] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2]))))
    + tTm0[12] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[13] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + tTm0[14] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) + dcRw_vec[1] *
    cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0[1] * dcRw_vec
    [2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] * dcRw_vec[2])))
    + dtTm[13] * ((tTm0[4] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] * cTw0[5]) +
    dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) - cTw0
    [1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) + cTw0[5] *
    dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[1] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[0] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[5] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[4] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[9] - dcRw_vec[0] *
    cTw0[5]) + dcRw_vec[1] * cTw0[1]) + wTt[9] * ((cTw0[5] + dcRw_vec[0] * cTw0
    [9]) - cTw0[1] * dcRw_vec[2])) + wTt[8] * ((cTw0[1] - dcRw_vec[1] * cTw0[9])
    + cTw0[5] * dcRw_vec[2])))) + wTt[13] * ((cTw0[5] + dcRw_vec[0] * cTw0[9]) -
    cTw0[1] * dcRw_vec[2])) + wTt[12] * ((cTw0[1] - dcRw_vec[1] * cTw0[9]) +
    cTw0[5] * dcRw_vec[2]))) + ((tTm0[8] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2]))) + tTm0[10] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0]
    * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] *
    cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] *
    cTw0[10]) + cTw0[6] * dcRw_vec[2]))) * (((((((((((((cTw0[14] - cam_point[2])
    + cTw0[2] * dcTw[12]) + cTw0[6] * dcTw[13]) + cTw0[10] * dcTw[14]) + wTt[14]
    * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2])) + dtTm[12] *
    ((tTm0[0] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] *
    cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] *
    dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] *
    dcRw_vec[2])) + tTm0[1] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
    dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) -
    cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2]))) + tTm0[2] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])))) + tTm0[12] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0]
    * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] *
    cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] *
    cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[13] * ((wTt[6] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[14] * ((wTt[10] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + dtTm[13] *
    ((tTm0[4] * ((wTt[2] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] *
    cTw0[2]) + wTt[1] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] *
    dcRw_vec[2])) + wTt[0] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] *
    dcRw_vec[2])) + tTm0[5] * ((wTt[6] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) +
    dcRw_vec[1] * cTw0[2]) + wTt[5] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) -
    cTw0[2] * dcRw_vec[2])) + wTt[4] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) +
    cTw0[6] * dcRw_vec[2]))) + tTm0[6] * ((wTt[10] * ((cTw0[10] - dcRw_vec[0] *
    cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0
    [10]) - cTw0[2] * dcRw_vec[2])) + wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10])
    + cTw0[6] * dcRw_vec[2])))) + dtTm[14] * ((tTm0[8] * ((wTt[2] * ((cTw0[10] -
    dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[1] * ((cTw0[6] +
    dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[0] * ((cTw0[2] -
    dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])) + tTm0[9] * ((wTt[6] *
    ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) + wTt[5] *
    ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) + wTt[4] *
    ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]))) + tTm0[10] *
    ((wTt[10] * ((cTw0[10] - dcRw_vec[0] * cTw0[6]) + dcRw_vec[1] * cTw0[2]) +
      wTt[9] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
     wTt[8] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2])))) +
    wTt[13] * ((cTw0[6] + dcRw_vec[0] * cTw0[10]) - cTw0[2] * dcRw_vec[2])) +
    wTt[12] * ((cTw0[2] - dcRw_vec[1] * cTw0[10]) + cTw0[6] * dcRw_vec[2]));



}

// find_gradient_error(double cTw0[16],  double wTt[16],  double
//                     tTm0[16],  double dcTw[16],  double dtTm[16],
//                      double cam_point[3],  double dcRw_vec[3], 
//                     double [3], double gradient_error[9])
Mat find_gradient_wrapper(Mat cTw_orig, Mat wTt_kth, Mat tcpTmarker_orig, Mat cTw_update, Mat tcpTmarker_update, Mat cPmarker)
{
  Mat result_gradient=Mat::zeros(9,1,CV_32F);

  double cTw0[4][4];
  double wTt[4][4], tTm0[4][4], dcTw[4][4], dtTm[4][4], cam_point[3], dcRw_vec[3], gradient_error[9];
  for(int i=0; i<4 ; i++)
  {
    for(int j=0; j<4 ; j++)
    {
      // apparently the one-dimensional array intreprets the 2 dimension array from top to bottom, then switches column
      // that's why I switch index
      cTw0[j][i]=(double)cTw_orig.at<float>(i,j);
      wTt[j][i]=(double)wTt_kth.at<float>(i,j);
      tTm0[j][i]=(double)tcpTmarker_orig.at<float>(i,j);
      dcTw[j][i]=(double)cTw_update.at<float>(i,j);
      dtTm[j][i]=(double)tcpTmarker_update.at<float>(i,j);
    }
  }
  Mat cRw_update_vec;
  Rodrigues(cTw_update(Rect(0,0,3,3)),cRw_update_vec);
  for(int i=0; i<3 ;i++)
  {
    dcRw_vec[i]=(double)cRw_update_vec.at<float>(i,0);
    cam_point[i]=(double)cPmarker.at<float>(i,0);
  }

  //    find_gradient_error( &cTw0,   wTt,   tTm0,   dcTw,   dtTm,  cam_point,   dcRw_vec,  &gradient_error);
  find_gradient_error( *cTw0,   *wTt,   *tTm0,   *dcTw,   *dtTm,  cam_point,   dcRw_vec,  gradient_error);

  for(int i=0; i<9 ;i++)
  {
    result_gradient.at<float>(i,0)=(float)gradient_error[i];
  }
  return result_gradient;
}

#endif