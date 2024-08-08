#include <stdio.h>
#include <stdlib.h>
#include "Mersenne.h"  //random number generator

// This code implements an exponential filter for the calculated positions. Tests for the 2D, 4 anchor
// (overdetermined case) show that averaging 3 independent measurements is sufficient to achieve
//  position accuracy equivalent to the input noise in distance measurements (+/- 10 cm).

    MTRand Random;  //required object for MTRand

#define N_ANCHORS 4
//
// coordinates of at least four anchors. It is straightforward to generalize this method to five or more.

float anchor_matrix[N_ANCHORS][3]=
{
    {0., 0., 0.},  //origin anchor. coordinates are relative to this (arbitrary) point
    {10., 0., 1.},  //Z coordinates of anchors should comparable to rover Z
    {0., 10., 2.},
    {10., 10., 0.}
};

float r[3]={0.0};  //input test position
float rc[3]={0.0};  //calculated result
float alpha = 0.3; //exponential filter weight


// Gaussian with mean zero, S.D. depends on n
// for n=7, rms spread is ~ 0.1

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

    int Nk = 3; //number of solutions to average
    int N_trials = 100;
    float alpha1 = 1.0-alpha;

    Random = seedRand(1337);


    float tmp;
    float d[N_ANCHORS] ={0}; //distances from anchors
    float kv[N_ANCHORS], x[N_ANCHORS], y[N_ANCHORS]; //temporary vectors
    float A[N_ANCHORS-1][2], ATA[2][2], Ainv[2][2], b[N_ANCHORS-1],ATb[2];  //the system of equations to solve

    printf("trilat 2D noise test: %d runs\n", N_trials);
    printf(" averaging over %d position estimates\n",Nk);
    printf(" filter alpha = %5.2f\n",alpha);

    //calculate Ainv
    //depends on anchors configurations, and needs to be determined only once

   for (i=0; i<N_ANCHORS; i++) {
    x[i] = anchor_matrix[i][0];
    y[i] = anchor_matrix[i][1];
    kv[i] = x[i]*x[i] + y[i]*y[i];
    }

// set up least squares equation

    for (i=1; i<N_ANCHORS; i++) {
    A[i-1][0] = x[i] - x[0];
    A[i-1][1] = y[i] - y[0];
    }

    //calculate Atranspose A
    // Cij = sum(k) (Aki*Akj)
    for (i=0; i<2; i++) {
      for(j=0; j<2; j++) {
        ATA[i][j]=0.0;
        for(k=0; k<N_ANCHORS-1; k++) ATA[i][j] += A[k][i]*A[k][j];
      }
    }
 //    printf("ATA %5.2f %5.2f\n    %5.2f %5.2f\n",ATA[0][0],ATA[0][1],ATA[1][0],ATA[1][1]);

    //invert ATA
    float det = ATA[0][0]*ATA[1][1] - ATA[1][0]*ATA[0][1];
    printf("det ATA %8.3e\n",det);
    det = 1.0/det;

   //scale adjoint
    Ainv[0][0] =  det*ATA[1][1];
    Ainv[0][1] = -det*ATA[0][1];
    Ainv[1][0] = -det*ATA[1][0];
    Ainv[1][1] =  det*ATA[0][0];
    printf("Ainv %7.4f %7.4f\n     %7.4f %7.4f\n",Ainv[0][0],Ainv[0][1],Ainv[1][0],Ainv[1][1]);

// generate positions and test the algorithm

    float dn=0.0, dnbar=0.0, dn2=0.0;  //noise statistics
    int Ndn=0;

    for (j=1; j<=N_trials; j++) {

// generate positions
    r[0] = 10.0*genRand(&Random);
    r[1] = 10.0*genRand(&Random);
    r[2] =  3.0*genRand(&Random);

    float rcavg[2]={0}, rcavg2[2]={0};

    for(k=0; k<Nk; k++) {//MOVING average of several position estimates


    //calculate distances from known 3D position to anchors

    for(i=0; i<N_ANCHORS; i++) {
    x[0] = anchor_matrix[i][0]-r[0];
    x[1] = anchor_matrix[i][1]-r[1];
    x[2] = anchor_matrix[i][2]-r[2];
    d[i] = sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]);
    dn = Gauss(7);  //generate +/- 0.1 rms noise
    d[i] = d[i] + dn; //add noise to calculated distances
    dnbar += dn;  //for noise statistics
    dn2 += dn*dn;
    Ndn++;
    }

// set up least squares equation

    for (i=1; i<N_ANCHORS; i++) {
    b[i-1] = d[0]*d[0] - d[i]*d[i] + kv[i] - kv[0];
    }

    //A transpose b
    ATb[0] = A[0][0]*b[0] + A[1][0]*b[1] + A[2][0]*b[2];
    ATb[1] = A[0][1]*b[0] + A[1][1]*b[1] + A[2][1]*b[2];

// solve for rc[]

    //least squares solution
    rc[0] = 0.5*(Ainv[0][0]*ATb[0] + Ainv[0][1]*ATb[1]);
    rc[1] = 0.5*(Ainv[1][0]*ATb[0] + Ainv[1][1]*ATb[1]);

// moving average (exponential) filter
    if (k==0) { //first time through?
      rcavg[0] = rc[0];
      rcavg[1] = rc[1];  //initialize position data
    }
    else {
 //     rcavg[0] = alpha1*rcavg[0] + alpha*rc[0];  //exp. average
 //     rcavg[1] = alpha1*rcavg[1] + alpha*rc[1];
      rcavg[0] += alpha*(rc[0] - rcavg[0]);  //exp. average
      rcavg[1] += alpha*(rc[1] - rcavg[1]);  //optimized, equivalent formulation

    }
    } // end for k


// data for Excel .csv file
    printf("%6.2f, %6.2f, %6.2f, ",r[0],r[1],r[2]);  //input r
    printf("%6.2f, %6.2f, ",rcavg[0],rcavg[1]); //calculated, averaged r, given noisy measurements

    x[0]=rcavg[0]-r[0];
    x[1]=rcavg[1]-r[1];
    tmp = sqrt(x[0]*x[0]+x[1]*x[1]);
    printf("%6.2f, ", tmp);  //|r - rc|  2D

// error in distances from calculated position (averaged)
// note Z coordinate is ignored, so get offset

    float rmse=0.0;
    for(i=0; i<N_ANCHORS; i++) {
    x[0]=anchor_matrix[i][0]-rc[0];
    x[1]=anchor_matrix[i][1]-rc[1];
    tmp = sqrt(x[0]*x[0]+x[1]*x[1]);
    rmse += (d[i]-tmp)*(d[i]-tmp);
     }
    printf("%6.2f\n",sqrt(rmse/((float)N_ANCHORS)));
    }  // end for j

    dnbar /= (float) Ndn;
    dn2 /= (float) Ndn;
    dn = sqrt(dn2 - dnbar*dnbar); //standard deviation of added noise
    printf("noise statistics: mean %6.4f, sd %6.4f\n",dnbar,dn); //noise added to distances (m)
    return 0;
}
