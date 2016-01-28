/*
 * StateEstimation.h
 *
 *  Created on: 21 sty 2016
 *      Author: Rafal
 */

#ifndef STATEESTIMATION_H_
#define STATEESTIMATION_H_

#include "MatrixCalculation.h"

#define DIV(x,y) (x/y)


/// Motor parameters
#define Lt  0.000605 // H
#define Rt  3.5 // Ohm
/// This 2 should be equal
#define ke  0.0841 // Vs/rad
#define km 0.06625 // Nm/A
//TODO Change this moment of inertia for shaft moment of inertia
#define J  0.0000007656// kg*m^2

extern int Tp; // Sampling time

double a[] = {DIV(-Rt,Lt), DIV(-ke,Lt), DIV(km,J), 0}; /// A (state) matrix elements
double b[] = {DIV(1.0, Lt), 0}; /// B (input) matrix elements
double c[] = {1, 0}; /// C (output) matrix elements
double z[] = {0, DIV(-1.0,J)}; // Z (error) matrix elements
double l[] = {DIV(-Rt,Lt)+5.6568, (16.0/DIV(-ke,Lt))+ DIV(km,J)};

const uint n = 2; // state vector height
const uint m = 1; // inputs vector height
const uint q = 1; // error vector height
const uint p = 1; // output vector height


Matrix A, B, C, Z;
Matrix L; /// Observer gain matrix
Matrix xp;
Matrix yp;
double u;
double d; // Distribuance
double eps; // Estimation error


// Kalman matrices

Matrix P0, x0. Ppri, Ppost, V;
double stdDevV, stdDevW, W, S;

/**
 * @brief Initialize state space motor model with pre-configured parameters
 */
void InitStateObserver()
{
	double zeros[4] = {0};

	InitMatrix(&A, n, n, a);

	InitMatrix(&B,n, m, b);

	InitMatrix(&Z, n, q, z);

	InitMatrix(&C, p, n, c);

	InitMatrix(&L, n, p, l);

	InitMatrix(&xp, n, 1, zeros);

	InitMatrix(&yp, m, 1, zeros);
}

void InitKalmanFilter()
{

}

void CalculateEstimatedState(double u, double y, Matrix* estimatedState)
{
	StateObserver(double u, double y, Matrix* estimatedState);
}

void StateObserver(double u, double y, Matrix* estimatedState)
{
	Matrix Ax, Bu, Cx, Leps;

	// Time update
	Multiply(&A, &xp, &Ax); // Ax'
	MultiplyByScalar(&B, u, &Bu); // Bu
	Add(&Ax,&Bu,estimatedState); // Ax' + Bu

	// Measuremnt update
	Multiply(&C, estimatedState, &Cx);// Cx'
	eps = y - GetElement(&Cx, 0, 0); // eps = y - y' estimation error

	MultiplyByScalar(&L, eps, &Leps); // L*eps
	Add(estimatedState, &Leps, &xp); // Update state
}

void KalmanFilter(double u, double y, Matrix* estimatedState)
{

}



#endif /* STATEESTIMATION_H_ */
