// MECH534 - HW5: Solving non-linear differential equations using numerical methods.
// Berke Ataseven
// 54326

#include <iostream>
#include "nr.h"
#include "nrutil.h"

using namespace std;

float** y, *xx; // Outputs need to be stored in a global variable

const float b1 = 1; 
const float b2 = 1;
const float c1 = 1;
const float c2 = 1;

void derivs(float x, float y[], float dydx[]) {
	dydx[1] = y[1] * (b1 - c1 * y[2]);
	dydx[2] = y[2] * (-b2 + c2 * y[1]);
}


void rk4(float y[], float dydx[], int n, float x, float h, float yout[],
	void (*derivs)(float, float[], float[]))
	/*	Given values for the variables y[1..n] and their derivatives dydx[1..n] known at x, use the
		fourth - order Runge - Kutta method to advance the solution over an interval h and return the
		incremented variables as yout[1..n], which need not be a distinct array from y.The user
		supplies the routine derivs(x, y, dydx), which returns derivatives dydx at x. */
{

	int i;
	float xh, hh, h6, * dym, * dyt, * yt;
	dym = vector(1, n);
	dyt = vector(1, n);
	yt = vector(1, n);
	hh = h * 0.5;
	h6 = h / 6.0;
	xh = x + hh;
	for (i = 1; i <= n; i++) yt[i] = y[i] + hh * dydx[i];	// First step.
	(*derivs)(xh, yt, dyt);									// Second step.
	for (i = 1; i <= n; i++) yt[i] = y[i] + hh * dyt[i];
	(*derivs)(xh, yt, dym);									// Third step.
	for (i = 1; i <= n; i++) {
		yt[i] = y[i] + h * dym[i];
		dym[i] += dyt[i];
	}
	(*derivs)(x + h, yt, dyt);								// Fourth step.
	for (i = 1; i <= n; i++)								//Accumulate increments with proper weights.
		yout[i] = y[i] + h6 * (dydx[i] + dyt[i] + 2.0 * dym[i]);
	free_vector(yt, 1, n);
	free_vector(dyt, 1, n);
	free_vector(dym, 1, n);
}


void rkdumb(float vstart[], int nvar, float x1, float x2, int nstep,
	void (*derivs)(float, float[], float[]))
	/*	Starting from initial values vstart[1..nvar] known at x1 use fourth - order Runge - Kutta
		to advance nstep equal increments to x2.The user - supplied routine derivs(x, v, dvdx)
		evaluates derivatives.Results are stored in the global variables y[1..nvar][1..nstep + 1]
		and xx[1..nstep + 1]. */
{
	void rk4(float y[], float dydx[], int n, float x, float h, float yout[],
		void (*derivs)(float, float[], float[]));

	int i, k;
	float x, h;
	float* v, * vout, * dv;
	v = vector(1, nvar);
	vout = vector(1, nvar);
	dv = vector(1, nvar);
	for (i = 1; i <= nvar; i++) { // Load starting values.		
		v[i] = vstart[i];
		y[i][1] = v[i];
	}
	xx[1] = x1;
	x = x1;
	h = (x2 - x1) / nstep;
	for (k = 1; k <= nstep; k++) { // Take nstep steps.		
		(*derivs)(x, v, dv);
		rk4(v, dv, nvar, x, h, vout, derivs);
		if ((float)(x + h) == x) nrerror("Step size too small in routine rkdumb");
		x += h;
		xx[k + 1] = x; // Store intermediate steps.
		for (i = 1; i <= nvar; i++) {
			v[i] = vout[i];
			y[i][k + 1] = v[i];
		}
	}
	free_vector(dv, 1, nvar);
	free_vector(vout, 1, nvar);
	free_vector(v, 1, nvar);
}

int main()
{
	float n = 2; // Number of variables

	float t_start = 0;
	float t_final = 10;

	float nstep = 200; // Number of steps

	float *initial_values = vector(1, n); // Values given at t=0
	initial_values[1] = 0.5;
	initial_values[2] = 0.5;

	y = matrix(1, n, 1, nstep + 1); // Output will be stored here
	xx = vector(1, nstep + 1); // Output time will be stored here

	rkdumb(initial_values, n, t_start, t_final, nstep, &derivs); // Call the function

	cout << "Prey		Predator	Time" << endl;
	cout << "====		========	====" << endl << endl;
	for (int i = 1; i <= nstep + 1; i++)
		printf_s("%.2f		%.2f		%.2f\n", y[1][i], y[2][i], xx[i]); // Print the values on the console
}