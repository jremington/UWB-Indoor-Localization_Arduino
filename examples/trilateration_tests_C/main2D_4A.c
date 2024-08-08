#include <stdio.h>
#include <stdlib.h>
#include "m33v3.h"

// for method see technical paper at
// https://www.th-luebeck.de/fileadmin/media_cosa/Dateien/Veroeffentlichungen/Sammlung/TR-2-2015-least-sqaures-with-ToA.pdf

// 2D (x,y) position, 4 Anchor test code (overdetermined case)
// coordinates of four anchors. It is straightforward to generalize this method to more anchors

#define N_ANCHORS 4

float anchor_matrix[N_ANCHORS][3]=
{
  {0.0, 0.0, 0.97},  //Z coordinates are ignored
  {3.99, 5.44, 1.14},
  {3.71, -0.3, 0.6},
  {-0.56, 4.88, 0.15}
};


float r[3]={2.0, 3.0, 1.0};  //test position
float rc[2]={0};  //result

int main()
{
    int i,j,k;
    float d[N_ANCHORS]; //distances from anchors
    float kv[N_ANCHORS], x[N_ANCHORS], y[N_ANCHORS]; //temporary vectors
    float A[N_ANCHORS-1][2], Ainv[2][2], b[N_ANCHORS-1];  //overdetermined system of equations to solve

    printf("trilat 2D 4 Anchor test\n");
    printf(" test point %5.2f %5.2f \n",r[0],r[1]);

    // for testing, calculate distances from known position to anchors
    float tmp;
    for(i=0; i<N_ANCHORS; i++) {
    x[0] = anchor_matrix[i][0]-r[0];
    x[1] = anchor_matrix[i][1]-r[1];
    x[2] = anchor_matrix[i][2]-r[2];
    d[i] = sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]); // + 0.02*(rand()%10 - 5); //add noise
//    printf("d= %5.2f\n", d[i]);
    }
    for (i=0; i<N_ANCHORS; i++) {
    x[i] = anchor_matrix[i][0];
    y[i] = anchor_matrix[i][1];
    kv[i] = x[i]*x[i] + y[i]*y[i];
    }

// set up least squares equation

    for (i=1; i<N_ANCHORS; i++) {
    A[i-1][0] = x[i] - x[0];
    A[i-1][1] = y[i] - y[0];
    b[i-1] = d[0]*d[0] - d[i]*d[i] + kv[i] - kv[0];
 //   printf("A %5.2f %5.2f b %5.2f\n",A[i-1][0],A[i-1][0],b[i-1]);
    }

    float ATA[2][2];  //calculate A transpose A
    // Cij = sum(k) (Aki*Akj)
    for (i=0; i<2; i++) {
      for(j=0; j<2; j++) {
        ATA[i][j]=0.0;
        for(k=0; k<N_ANCHORS-1; k++) ATA[i][j] += A[k][i]*A[k][j];
      }
    }
    printf("ATA %5.2f %5.2f\n    %5.2f %5.2f\n",ATA[0][0],ATA[0][1],ATA[1][0],ATA[1][1]);

    float ATb[2];  //A transpose b
    ATb[0] = A[0][0]*b[0] + A[1][0]*b[1] + A[2][0]*b[2];
    ATb[1] = A[0][1]*b[0] + A[1][1]*b[1] + A[2][1]*b[2];

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
    //least squares solution
    rc[0] = 0.5*(Ainv[0][0]*ATb[0] + Ainv[0][1]*ATb[1]);
    rc[1] = 0.5*(Ainv[1][0]*ATb[0] + Ainv[1][1]*ATb[1]);

    printf("r calc: %6.2f %6.2f\n",rc[0],rc[1]);

    float dc, rmse=0.0;
    for(i=0; i<N_ANCHORS; i++) {
    x[0] = anchor_matrix[i][0]-rc[0];
    x[1] = anchor_matrix[i][1]-rc[1];
    dc = sqrt(x[0]*x[0] + x[1]*x[1]);
    rmse += (d[i]-dc)*(d[i]-dc);
    printf ("d, dc %6.2f %6.2f\n",d[i],dc);
    }
    printf("rmse = %6.3f\n",sqrt(rmse/((float)N_ANCHORS)));
    return 0;
}
