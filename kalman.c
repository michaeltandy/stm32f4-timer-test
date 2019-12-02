#include "matrix.h"

int main()
{
    SimpleMatrix x = {{{0},{0}}, 2, 1}; // Initial state (time error and rate of error increase)
    SimpleMatrix P = {{{10000000,0},{0,10000000}}, 2, 2}; // Initial uncertainty
    SimpleMatrix u = {{{0},{0}}, 2, 1}; // Control input
    SimpleMatrix F = {{{1,1},{0,1}}, 2, 2}; // Next state function (assumes 1 second time step)
    SimpleMatrix H = {{{1,0}}, 1, 2}; // Measurement function
    SimpleMatrix R = {{{50000}}, 1, 1}; // Measurement uncertainty
    SimpleMatrix I = {{{1,0},{0,1}}, 2, 2}; // Identity matrix
    SimpleMatrix Q = {{{1,0.007},{0.007,0.000044}}, 2, 2}; // Process noise covariance

  for (int i=0 ; i<100 ; i++) {
    if (i>0) {
      // Update prediction:
      { // x = F*x+u
        SimpleMatrix result1;
        matrixMultiply(&result1, &F, &x);
        matrixAdd(&x, &result1, &u);
      }
      {// P = F*P*F'
        SimpleMatrix F_t;
        matrixTranspose(&F_t, &F);
        SimpleMatrix result1;
        matrixMultiply(&result1, &F, &P);
        matrixMultiply(&P, &result1, &F_t);
      }
    }

    // Measurement update
    // z = matrix([[measurements[n]]])
    SimpleMatrix z = {{{i+3}}, 1, 1}; // New measurement

    SimpleMatrix y;
    { // y = z-H*x
        SimpleMatrix result1;
        matrixMultiply(&result1, &H, &x);
        matrixSubtract(&y, &z, &result1);
    }

    SimpleMatrix S;
    { // S = H*P*H'+R
        SimpleMatrix H_t;
        matrixTranspose(&H_t, &H);
        SimpleMatrix result1;
        matrixMultiply(&result1, &H, &P);
        SimpleMatrix result2;
        matrixMultiply(&result2, &result1, &H_t);
        matrixAdd(&S, &result2, &R);
    }

    SimpleMatrix K;
    { // K = P*H.transpose()*S.inverse()
        SimpleMatrix H_t;
        matrixTranspose(&H_t, &H);
        SimpleMatrix S_inv;
        matrixInvert(&S_inv, &S);
        SimpleMatrix result1;
        matrixMultiply(&result1, &P, &H_t);
        matrixMultiply(&K, &result1, &S_inv);
    }

    { // x = x+K*y
        SimpleMatrix result1;
        matrixMultiply(&result1, &K, &y);
        SimpleMatrix result2;
        matrixAdd(&result2, &x, &result1);
        matrixCopy(&x, &result2);
    }

    {// P = (I-K*H)*P
        SimpleMatrix result1;
        matrixMultiply(&result1, &K, &H);
        SimpleMatrix result2;
        matrixSubtract(&result2, &I, &result1);
        SimpleMatrix result3;
        matrixMultiply(&result3, &result2, &P);
        matrixCopy(&P, &result3);
    }

    matrixPrint(&x);
  }
}
