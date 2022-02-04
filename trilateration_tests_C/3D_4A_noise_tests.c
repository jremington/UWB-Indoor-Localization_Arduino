#include <stdio.h>
#include <stdlib.h>
#include "m33v3.h"
#include "Mersenne.h"  //random number generator

    MTRand Random;  //required object for MTRand

//
// coordinates of at least four anchors. It is straightforward to generalize this method to five or more.

float anchor_matrix[4][3]=
{
    {0., 0., 0.},  //origin anchor. coordinates are relative to this (arbitrary) point
    {10., 0., 3.},
    {0., 10., 5.},
    {10., 10., 9.}
};

float r[3]={0.0};  //input test position
float rc[3]={0.0};  //calculated result

// Gaussian with mean zero, S.D. depends on n
float Gauss (int n) {
    int i;
    float t=0.0;
    for(i=0; i<n; i++) {
    t += genRand(&Random)-0.5;;
    }
    return t/n;
}


int main()
{
    int i,j;
    int N_trials = 1000;
    int N_ANCHORS = 4;
    Random = seedRand(1337);

    float tmp;
    float d[4] ={0}; //distances from anchors
    float k[4], x[4], y[4], z[4], v[3]; //temporary vectors
    float A[3][3], Ainv[3][3], b[3];  //the system of equations to solve

    float dn, dnbar=0.0, dn2=0.0;  //noise parameters

    printf("trilat 3D test: %d runs\n", N_trials);

    for (j=1; j<=N_trials; j++) {

// generate positions
    r[0] = 10.0*genRand(&Random);
    r[1] = 10.0*genRand(&Random);
    r[2] =  3.0*genRand(&Random);

    //calculate distances from position to anchors

    for(i=0; i<4; i++) {
    VEC_DIFF(v,anchor_matrix[i],r);
    VEC_LENGTH(tmp, v);
    dn = 0.5*Gauss(7);
    d[i] = tmp + dn; //add noise to calculated distances
    dnbar += dn;  //for noise statistics
    dn2 += dn*dn;
    }

    for (i=0; i<4; i++) {
    x[i] = anchor_matrix[i][0];
    y[i] = anchor_matrix[i][1];
    z[i] = anchor_matrix[i][2];
    k[i] = x[i]*x[i] + y[i]*y[i] + z[i]*z[i];
    }

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
//    if (fabs(det) < 0.1)
//           printf(" det = %8.3e\n",det);
    det = 1.0 / (det);
    SCALE_ADJOINT_3X3 (Ainv, det, A);

    MAT_DOT_VEC_3X3(rc,Ainv,b);
    for (i=0; i<3; i++) rc[i] *= 0.5;  //remove factor of 2

// data for Excel .csv file
    printf("%6.2f, %6.2f, %6.2f, ",r[0],r[1],r[2]);  //input r
    printf("%6.2f, %6.2f, %6.2f, ",rc[0],rc[1],rc[2]); //calculated r, given noisy measurements
    VEC_DIFF(x,r,rc);
    VEC_LENGTH(tmp, x);
    printf("%6.2f, ", tmp);  //rms diff r versus rc

// error in distances from calculated position

    float dc, rmse=0.0;
    for(i=0; i<N_ANCHORS; i++) {
    VEC_DIFF(x,anchor_matrix[i],rc);
    VEC_LENGTH(dc, x);
    rmse += (d[i]-dc)*(d[i]-dc);
     }
    printf("%6.2f\n",sqrt(rmse/((float)N_ANCHORS)));
    }  // end for j

    dnbar /= (4*N_trials);  //four distances per test
    dn2 /= (4*N_trials);
    dn = sqrt(dn2 - dnbar*dnbar); //standard deviation of added noise
    printf("noise statistics: mean %6.4f, sd %6.4f\n",dnbar,dn); //noise added to distances (m)
    return 0;
}
