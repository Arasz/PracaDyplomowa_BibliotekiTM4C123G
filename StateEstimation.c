/*
 * StateEstimation.c
 *
 *  Created on: 21 sty 2016
 *      Author: Rafal
 */


#include "StateEstimation.h"

static double a[] = {0.003072, -0.02395, 2.208e-5, 1}; /// A (state) matrix elements
static double b[] = {0.2848, 3.03e-5}; /// B (input) matrix elements
static double c[] = {1, 0}; /// C (output) matrix elements
static double p0[4] = {0}; /// Input kalman covariance
static double q[4] = {0}; /// Measurment covariance matrix elements
static double r[4] = {0}; /// Proces covariance matrix elements
static double x0[4] = {0}; /// Proces covariance matrix elements


static KalmanFilter stateEstimator;

/**
 * @brief Initialization of Kalman Filter alogrithm matrices
 */
void InitKalmanFilter()
{
	InitMatrix(&stateEstimator.A, 2, 2, a);
	InitMatrix(&stateEstimator.B, 2, 1, b);
	InitMatrix(&stateEstimator.C, 1, 2, c);
	InitMatrix(&stateEstimator.P0, 2, 2, p0);
	InitMatrix(&stateEstimator.x0, 2, 1, x0);
	InitMatrix(&stateEstimator.R, 2, 1, r);
	InitMatrix(&stateEstimator.Q, 1, 2, q);

	DeepMatrixCopy(&stateEstimator.P_post, &stateEstimator.P0);
	DeepMatrixCopy(&stateEstimator.x_pri, &stateEstimator.x0);
}


/**
 * @brief Calculates one step of Kalman filter algorithm
 */
void KalmanFilterAlgorithm(double u, double y, Matrix* estimatedState)
{
	 Matrix Ax, Bu, Cx, Atran, Ctran, Keps, Ktran;
	  // Aktualizacja czasu

	  //x_pri = A*x_post +B*u; Predykcja stanu
	  Multiply(&stateEstimator.A, &stateEstimator.x_post,& Ax);
	  MultiplyByScalar(&stateEstimator.B, u, &Bu);
	  Add(&Ax, &Bu, &stateEstimator.x_pri);

	  //P_pri = A*P_post*(A') + R; Predykcja macierzy kowariancji
	  Multiply(&stateEstimator.A,& stateEstimator.P_post, &stateEstimator.P_pri);
	  Transpose(&stateEstimator.A, &Atran);
	  Multiply(&stateEstimator.P_pri, &Atran, &stateEstimator.P_pri);
	  Add(&stateEstimator.P_pri, &stateEstimator.R, &stateEstimator.P_pri);

	  // Aktualizacja pomiarów

	  //eps = y - C*x_pri;  Wyliczenie b³êdu predykcji
	  Multiply(&stateEstimator.C, &stateEstimator.x_pri, &Cx);
	  stateEstimator.eps = y - GetElement(&Cx, 1, 1);

	  //S = C*P_pri*C' + Q;
	  Multiply(&stateEstimator.C, &stateEstimator.P_pri, &stateEstimator.S);
	  Transpose(&stateEstimator.C, &Ctran);
	  Multiply(&stateEstimator.S, &Ctran, &stateEstimator.S);
	  Add(&stateEstimator.S, &stateEstimator.Q, &stateEstimator.S);

	  //K = P_pri*C'*(S^(-1)); % Wyliczenie wzmocnienia kalmana
	  Multiply(&stateEstimator.P_pri, &Ctran, &stateEstimator.K);
	  MultiplyByScalar(&stateEstimator.K,(1/GetElement(&stateEstimator.S,1,1)),&stateEstimator.K);

	  //x_post = x_pri + K*eps(i);  Korekcja stanu
	  MultiplyByScalar(&stateEstimator.K, stateEstimator.eps, &Keps);
	  Add(&stateEstimator.x_pri, &Keps, &stateEstimator.x_post);

	  //P_post = P_pri-K*S*K'; Korekcja macierzy kowariancji
	  Transpose(&stateEstimator.K, &Ktran);
	  Multiply(&stateEstimator.K, &Ktran, &stateEstimator.P_post);
	  MultiplyByScalar(&stateEstimator.P_post, -GetElement(&stateEstimator.S,1,1),
			  &stateEstimator.P_post);
	  Add(&stateEstimator.P_pri, &stateEstimator.P_post, &stateEstimator.P_post);

	  // Return estimated state
	  DeepMatrixCopy(&stateEstimator.x_post, estimatedState);
}

/**
 * @brief Calculates estimeated given input and output of object
 */
void CalculateEstimatedState(double u, double y, Matrix* estimatedState)
{
	KalmanFilterAlgorithm(u, y, estimatedState);
}
