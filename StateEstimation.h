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
	Matrix A, B, C; /// Macierze dyskretnego modelu uk�adu
	Matrix P0; /// Pocz�tkowa warto�c macierzy kowariancji uk�du
	Matrix P_pri; /// Predykcja jednokrokowa macierzy kowariancji
	Matrix P_post; /// Macierz kowariancji po korekcji
	Matrix Q; /// Macierz kowariancji szumu pomiarowego
	Matrix R; /// Macierz kowariancji procesu
	Matrix S;
	Matrix K; /// Wzmocnienie kalmana

	Matrix x_init; // Oczekiwany poczatkowy stan uk�adu
	Matrix x_pri; // Predykcja jednokrokowa stanu uk�adu
	Matrix x_post; // Predykcja stanu uk�adu po korekcji

	double eps; // B��d predykcji

} KalmanFilter;


void InitKalmanFilter();
void KalmanFilterAlgorithm(double u, double y, Matrix* estimatedState);

void CalculateEstimatedState(double u, double y, Matrix* estimatedState);






#endif /* STATEESTIMATION_H_ */
