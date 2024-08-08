#include <stdio.h>
#include <stdlib.h>

// for method see technical paper at
// https://www.th-luebeck.de/fileadmin/media_cosa/Dateien/Veroeffentlichungen/Sammlung/TR-2-2015-least-sqaures-with-ToA.pdf
// 2D, 3 Anchor test code
// Z axis is ignored, which introduces error into (x,y) position
// coordinates of three anchors required

float anchor_matrix[3][3]=
{
  {0.0, 0.0, 0.97},
  {3.99, 5.44, 1.14},
  {3.71, -0.3, 0.6},
};

float r[3]={1.0, 4.0, 0.};  //test position
float rc[2];  //result

int main()
{
    int i,j;
 //  for (j=0; j<20; j++) {
//    for (i=0; i<2; i++) r[i] = (rand()%10-5);

    float d[3] ={0}; //distances from anchors
    float k[3], x[3], y[3]; //temporary vectors
    float A[2][2], Ainv[2][2], b[2];  //the system of equations to solve

    printf("trilat 2D 3Anchor test\n");
    printf("r   %6.2f %6.2f %6.2f\n",r[0],r[1],r[2]);

    // as test, calculate distances from position to anchors
    for(i=0; i<3; i++) {
    x[0] = anchor_matrix[i][0]-r[0];
    x[1] = anchor_matrix[i][1]-r[1];
    x[2] = anchor_matrix[i][2]-r[2];  //add in Z to test
     d[i] = sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]); // + 0.04*(rand()%10 - 5); //add noise
    }
// Z ignored in solution
    for (i=0; i<3; i++) {
    x[i] = anchor_matrix[i][0];
    y[i] = anchor_matrix[i][1];
    k[i] = x[i]*x[i] + y[i]*y[i];
//    printf("x, y, k: %5.1f %5.1f %5.1f \n",x[i],y[i],k[i]);
    }

// set up least squares equation

    for (i=1; i<3; i++) {
    A[i-1][0] = x[i] - x[0];
    A[i-1][1] = y[i] - y[0];
    b[i-1] = d[0]*d[0] - d[i]*d[i] + k[i] - k[0];
//    printf("A, b  %5.2f %5.2f %5.2f \n",A[i-1][0],A[i-1][1],b[i-1]);
    }

// solve:  2 A rc = b
    //invert A
    float det = A[0][0]*A[1][1] - A[1][0]*A[0][1];

//    printf("det A %8.3e\n",det);
    det = 1.0/det;
    //scale adjoint
    Ainv[0][0] =  det*A[1][1];
    Ainv[0][1] = -det*A[0][1];
    Ainv[1][0] = -det*A[1][0];
    Ainv[1][1] =  det*A[0][0];
    printf("Ainv %5.2f %5.2f\n     %5.2f %5.2f\n",Ainv[0][0],Ainv[0][1],Ainv[1][0],Ainv[1][1]);
    //least squares solution
    rc[0] = 0.5*(Ainv[0][0]*b[0] + Ainv[0][1]*b[1]);
    rc[1] = 0.5*(Ainv[1][0]*b[0] + Ainv[1][1]*b[1]);

    printf("rc: %6.2f %6.2f\n",rc[0],rc[1]);
    // calculate distances from solution position to anchors
    float dc[3], rmse=0.0;
    for(i=0; i<3; i++) {
    x[0] = anchor_matrix[i][0]-r[0];
    x[1] = anchor_matrix[i][1]-r[1];
    dc[i] = sqrt(x[0]*x[0] + x[1]*x[1]);
    rmse += (d[i]-dc[i])*(d[i]-dc[i]);
    printf ("d, dc %6.2f %6.2f\n",d[i],dc[i]);
    }
    printf("rmse = %6.3f\n",sqrt(rmse/3.0));
//    } //j
    return 0;
}
