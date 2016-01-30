/*
 * StateEstimation.h
 *
 *  Created on: 21 sty 2016
 *      Author: Rafal
 */

#ifndef STATEESTIMATION_H_
#define STATEESTIMATION_H_

#include "MatrixCalculation.h"

/**
 * Kalman filter definition
 */
typedef struct kalman
{
	Matrix A, B, C; /// Macierze dyskretnego modelu uk³adu
	Matrix P0; /// Pocz¹tkowa wartoœc macierzy kowariancji uk³du
	Matrix P_pri; /// Predykcja jednokrokowa macierzy kowariancji
	Matrix P_post; /// Macierz kowariancji po korekcji
	Matrix Q; /// Macierz kowariancji szumu pomiarowego
	Matrix R; /// Macierz kowariancji procesu
	Matrix S;
	Matrix K; /// Wzmocnienie kalmana

	Matrix x_init; // Oczekiwany poczatkowy stan uk³adu
	Matrix x_pri; // Predykcja jednokrokowa stanu uk³adu
	Matrix x_post; // Predykcja stanu uk³adu po korekcji

	double eps; // B³¹d predykcji

} KalmanFilter;


void InitKalmanFilter();
void KalmanFilterAlgorithm(double u, double y, Matrix* estimatedState);

void CalculateEstimatedState(double u, double y, Matrix* estimatedState);






#endif /* STATEESTIMATION_H_ */
