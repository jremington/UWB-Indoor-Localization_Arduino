#include <stdio.h>
#include <stdlib.h>
#include "m33v3.h"

// for method see technical paper at
// https://www.th-luebeck.de/fileadmin/media_cosa/Dateien/Veroeffentlichungen/Sammlung/TR-2-2015-least-sqaures-with-ToA.pdf
// 3D, 4 Anchor test code
// MATLAB test code
/*
%%clean up
clear all; close all;
%% define coordinates
anchor_matrix=[0 0 0; 10 0 1; 0 10 0; 10 10 2];
r = [2 3 1];
r0 = [0 0 0];
for j = 1:10
%% determine distances
for i = 1:length(anchor_matrix)
vec = anchor_matrix(i,:) - r;
% add some noise for tests. Here, 0.1 m rms distance error
d(i) = sqrt(sum(vec.^2)) + random('Normal',0,0.1);
end
disp(d);
%%generate matrix
x=anchor_matrix(:,1);
y=anchor_matrix(:,2);
z=anchor_matrix(:,3);
k=x.^2 + y.^2 + z.^2;
for i=2:length(d)
A(i-1,:)=[x(i) y(i) z(i)] - [x(1) y(1) z(1)];
b(i-1)=d(1)^2 - d(i)^2 + k(i) - k(1);
end
%% compute position
r0=pinv(A)*b'/2;
disp(r0)
*/

// C translation
// coordinates of at least four anchors. It is straightforward to generalize this method to five or more.
// S. James Remington 1/2022

float anchor_matrix[4][3]=
{
    {0., 0., 0.},  //origin anchor. coordinates are relative to this (arbitrary) point
    {10., 0., 1.},
    {0., 10., 0.},
    {10., 10., 2.}
};

float r[3]={2.0, 3.0, 1.0};  //test position
float rc[3]={0, 0, 0};  //result

int main()
{
    int i;
    float d[4] ={0}; //distances from anchors
    float k[4], x[4], y[4], z[4]; //temporary vectors
    float A[3][3], Ainv[3][3], b[3];  //the system of equations to solve

    printf("trilat 3D test\n");

    // as test, calculate distances from position to anchors
    float tmp;
    for(i=0; i<4; i++) {
    VEC_DIFF(x,anchor_matrix[i],r);
    VEC_LENGTH(tmp, x);
    d[i]=tmp;
//debug
//    printf("%5.1f %5.1f %5.1f d= %5.1f\n",x[0],x[1],x[2], d);
    }
    for (i=0; i<4; i++) {
    x[i] = anchor_matrix[i][0];
    y[i] = anchor_matrix[i][1];
    z[i] = anchor_matrix[i][2];
    k[i] = x[i]*x[i] + y[i]*y[i] + z[i]*z[i];
    }
//debug
//    for (i=0; i<4; i++)
//        printf(" %5.1f %5.1f %5.1f %5.2f \n",x[i],y[i],z[i],k[i]);

// set up least squares equation

    for (i=1; i<4; i++) {
    A[i-1][0] = x[i] - x[0];
    A[i-1][1] = y[i] - y[0];
    A[i-1][2] = z[i] - z[0];
    b[i-1] = d[0]*d[0] - d[i]*d[i] + k[i] - k[0];
    }

// solve:  2 A rc = b
    float det;
    DETERMINANT_3X3 (det, A);
//    check solution stability (small or zero det) here
//    printf(" det = %8.3e\n",tmp);
    det = 1.0 / (det);
    SCALE_ADJOINT_3X3 (Ainv, det, A);

    MAT_DOT_VEC_3X3(rc,Ainv,b);
    for (i=0; i<3; i++) rc[i] *= 0.5;  //remove factor of 2

    printf("rc: %6.2f %6.2f %6.2f\n",rc[0],rc[1],rc[2]);
    return 0;
}
