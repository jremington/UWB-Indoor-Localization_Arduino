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
    int i,j,k;  //loop variables

    int nk = 10; //number of solutions to average
    int N_trials = 1000;
    int N_ANCHORS = 4;
    Random = seedRand(1337);

    float tmp;
    float d[4] ={0}; //distances from anchors
    float kv[4], x[4], y[4], z[4]; //temporary vectors
    float A[3][3], Ainv[3][3], b[3];  //the system of equations to solve

    float dn, dnbar=0.0, dn2=0.0;  //noise parameters

    printf("trilat 3D test: %d runs\n", N_trials);
    printf(" averaging over %d position estimates\n",nk);
    //calculate Ainv
    // It depends on anchors configurations, and needs to be determined only once

    for (i=0; i<4; i++) {
    x[i] = anchor_matrix[i][0];
    y[i] = anchor_matrix[i][1];
    z[i] = anchor_matrix[i][2];
    kv[i] = x[i]*x[i] + y[i]*y[i] + z[i]*z[i];
    }

// set up least squares equation

    for (i=1; i<4; i++) {
    A[i-1][0] = x[i] - x[0];
    A[i-1][1] = y[i] - y[0];
    A[i-1][2] = z[i] - z[0];
    }

// solve:  2 A rc = b
    float det;
    DETERMINANT_3X3 (det, A);
//    check solution stability (small or zero det) here
//    if (fabs(det) < 0.1)
//           printf(" det = %8.3e\n",det);
    det = 1.0 / (det);
    SCALE_ADJOINT_3X3 (Ainv, det, A);

    for (j=1; j<=N_trials; j++) {

// generate positions
    r[0] = 10.0*genRand(&Random);
    r[1] = 10.0*genRand(&Random);
    r[2] =  3.0*genRand(&Random);

    float rcavg[3]={0}, rcavg2[3]={0};

    for(k=0; k<nk; k++) {//average several position estimates

    //calculate distances from position to anchors

    for(i=0; i<4; i++) {
    VEC_DIFF(x,anchor_matrix[i],r);  //reuse x vector
    VEC_LENGTH(tmp, x);
    dn = Gauss(7);  //generate +/- 0.1 rms noise
    d[i] = tmp + dn; //add noise to calculated distances
    dnbar += dn;  //for noise statistics
    dn2 += dn*dn;
    }

// set up least squares equation

    for (i=1; i<4; i++) {
    b[i-1] = d[0]*d[0] - d[i]*d[i] + kv[i] - kv[0];
    }
// solve for rc[]

    MAT_DOT_VEC_3X3(rc,Ainv,b);
    for (i=0; i<3; i++) {
      rc[i] *= 0.5;  //remove factor of 2
      rcavg[i] += rc[i];  //average the position data
      rcavg2[i] += rc[i]*rc[i]; //for stats
      }
   } // end for k

    for (i=0; i<3; i++) {
    rc[i]=rcavg[i]/nk;
    rcavg2[i] = sqrt(rcavg2[i]/nk - rc[i]*rc[i]);  //axial SDs of cluster average
    }

// data for Excel .csv file
    printf("%6.2f, %6.2f, %6.2f, ",r[0],r[1],r[2]);  //input r
    printf("%6.2f, %6.2f, %6.2f, ",rc[0],rc[1],rc[2]); //calculated and averaged r, given noisy measurements
    printf("%6.2f, %6.2f, %6.2f, ",rcavg2[0],rcavg2[1],rcavg2[2]); //point cluster axial SDs
    // the ninth column above shows that the Z values are very poorly determined

    VEC_LENGTH(tmp,rcavg2);
    printf("%6.2f, ", tmp);  //SD for calculate rc point cluster


    VEC_DIFF(x,r,rc);
    VEC_LENGTH(tmp, x);
    printf("%6.2f, ", tmp);  //rms diff r versus rc

// error in distances from calculated position (averaged)

    float dc, rmse=0.0;
    for(i=0; i<N_ANCHORS; i++) {
    VEC_DIFF(x,anchor_matrix[i],rc);
    VEC_LENGTH(dc, x);
    rmse += (d[i]-dc)*(d[i]-dc);
     }
    printf("%6.2f\n",sqrt(rmse/((float)N_ANCHORS)));
    }  // end for j

    dnbar /= (nk*4*N_trials);  //four distances per test
    dn2 /= (nk*4*N_trials);
    dn = sqrt(dn2 - dnbar*dnbar); //standard deviation of added noise
    printf("noise statistics: mean %6.4f, sd %6.4f\n",dnbar,dn); //noise added to distances (m)
    return 0;
}
