#include <stdio.h>
#include <stdlib.h>
#include "m33v3.h"   //matrix and vector operations, loops unrolled.
#include "Mersenne.h"  //random number generator
#define N_ANCHORS 6

    MTRand Random;  //required object for MTRand

//
// coordinates of at least four anchors.
float anchor_matrix[N_ANCHORS][3]=
{
    {0., 0., 0.},  //origin anchor. coordinates are relative to this (arbitrary) point
    {10., 0., 3.},
    {0., 10., 5.},
    {10., 10., 1.},
    {5., 5., 2.},
    {3., 3., 3.}
};

float r[3]={0.0};  //input test position
float rc[3]={0.0};  //calculated result

// Gaussian noise with mean zero, S.D. depends inversely on n
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
    int i,j,k,l;  //loop variables

    int N_est = 3; //number of trial solutions to average
    int N_trials = 5;  //number of generated position trials
    int N_EQN = N_ANCHORS-1;  //number of equations
    Random = seedRand(1337);

    float tmp;
    float d[N_ANCHORS] ={0}; //vector of distances from anchors to position
    float kv[N_ANCHORS], x[N_ANCHORS], y[N_ANCHORS], z[N_ANCHORS]; //temporary vectors
    float A[N_EQN][3], ATA[3][3], ATAinv[3][3], b[N_EQN], ATb[3];  //the system of equations to solve

    float dn, dnbar=0.0, dn2=0.0;  //noise parameters

    printf("trilat 3D test: %d anchors, %d position trials\n", N_ANCHORS, N_trials);
    printf("Averaging over %d individual position estimates\n",N_est);
    //calculate ATAinv
    // It depends on anchor configurations, and needs to be determined only once

    for (i=0; i<N_ANCHORS; i++) {
    x[i] = anchor_matrix[i][0];
    y[i] = anchor_matrix[i][1];
    z[i] = anchor_matrix[i][2];
    kv[i] = x[i]*x[i] + y[i]*y[i] + z[i]*z[i];
    }

// set up least squares equation
    printf("\nSet up A matrix\n");
    for (i=1; i<N_ANCHORS; i++) {
    A[i-1][0] = x[i] - x[0];
    A[i-1][1] = y[i] - y[0];
    A[i-1][2] = z[i] - z[0];
    printf("A    %7.4f %7.4f %7.4f\n",A[i-1][0],A[i-1][1],A[i-1][2]);
    }

    printf("\nATA\n");  //calculate ATA
    for (i=0; i<3; i++) {
        for (j=0; j<3; j++) {
            ATA[i][j]=0.0;
            for (k=0; k<N_EQN; k++) {
                ATA[i][j] += A[k][i]*A[k][j];
            }
            printf("%7.4f  ",ATA[i][j]);
        }
        printf("\n");
    }

// solve:  2 A rc = b
    float det;
    DETERMINANT_3X3 (det, ATA);
//    check solution stability (small or zero det) here
 //   if (fabs(det) < 0.1)
           printf("  det = %8.3e\n",det);
    det = 1.0 / (det);
    SCALE_ADJOINT_3X3 (ATAinv, det, ATA);

        printf("\nATAinv\n   %7.4f %7.4f %7.4f\n   %7.4f %7.4f %7.4f\n   %7.4f %7.4f %7.4f\n\n",
               ATAinv[0][0],ATAinv[0][1],ATAinv[0][2],ATAinv[1][0],ATAinv[1][1],ATAinv[1][2],ATAinv[2][0],ATAinv[2][1],ATAinv[2][2]);

    for (j=1; j<=N_trials; j++) {

// generate positions
    r[0] = 10.0*genRand(&Random);
    r[1] = 10.0*genRand(&Random);
    r[2] =  3.0*genRand(&Random);
    printf("P  %d %5.2f %5.2f %5.2f\n",j, r[0],r[1],r[2]);  //generated position for this trial
    float rcavg[3]={0}, rcavg2[3]={0};

    for(l=0; l<N_est; l++) {//average several position estimates with different values of distance noise added

    //calculate distances from position to anchors

    for(i=0; i<N_EQN; i++) {
    VEC_DIFF(x,anchor_matrix[i],r);  //reuse x vector
    VEC_LENGTH(tmp, x);
    dn = Gauss(7);  //generate +/- 0.1 rms noise
    d[i] = tmp + dn; //add noise to calculated distances
    dnbar += dn;  //for noise statistics
    dn2 += dn*dn;
    }

// finish setting up least squares equation

    for (i=1; i<N_ANCHORS; i++) {
    b[i-1] = d[0]*d[0] - d[i]*d[i] + kv[i] - kv[0];
    }
        for (i=0; i<3; i++) {
        ATb[i]=0;
        for (k=0; k<N_EQN; k++) {
            ATb[i] += A[k][i]*b[k];
        }
    }
// solve for rc[]

    MAT_DOT_VEC_3X3(rc,ATAinv,ATb); //position scaled by 2

    for (i=0; i<3; i++) {
      rc[i] *= 0.5;  //remove factor of 2
      rcavg[i] += rc[i];  //average the position data
      rcavg2[i] += rc[i]*rc[i]; //for stats
      }
    printf("Pi %d %5.2f %5.2f %5.2f\n", l, rc[0],rc[1],rc[2]);  //current position estimate
   } // end for l (N_est)

   // average position and accumulate errors

    for (i=0; i<3; i++) {
    rc[i] = rcavg[i]/N_est;
    rcavg2[i] = sqrt(rcavg2[i]/N_est - rc[i]*rc[i]);  //to calc sd over cluster average along each axis
    }
    printf("PC %d %5.2f %5.2f %5.2f\n",j, rc[0],rc[1],rc[2]);  //average of N_est position estimates

// data for Excel .csv file
    printf("%6.2f, %6.2f, %6.2f, ",r[0],r[1],r[2]);  //input r
    printf("%6.2f, %6.2f, %6.2f, ",rc[0],rc[1],rc[2]); //calculated and averaged r, given noisy measurements
    printf("%6.2f, %6.2f, %6.2f, ",rcavg2[0],rcavg2[1],rcavg2[2]); //point cluster axial VARIANCES

    // the ninth column above shows that the Z values are poorly determined ^^^^

    VEC_LENGTH(tmp,rcavg2);
    printf("%6.2f, ", tmp);  //SD for rc point cluster


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

    dnbar /= (N_est*N_ANCHORS*N_trials);  // N_ANCHORS distances per individual test
    dn2 /= (N_est*N_ANCHORS*N_trials);
    dn = sqrt(dn2 - dnbar*dnbar); //standard deviation of added noise
    printf("noise statistics: mean %6.4f, sd %6.4f\n",dnbar,dn); //noise added to distances (m)
    return 0;
}
