// MECH534 - COMPUTER BASED MODELING AND SIMULATION - Homework #2
// This program calculates the volume of a box defined by 3 vectors.
// Written by:
// Berke Atasseven
// 0054326

#include <iostream>
#include "nr.h"
#include "nrutil.h"

using namespace std;

double dot_product(double *a, double *b, int n) {
    double sum = 0;
    for (int i = 0; i < n; i++) 
        sum += a[i] * b[i];
    
    return sum;
}

double *cross_product(double* a, double* b) {
    double *result = dvector(1, 3);

    result[0] = (a[1] * b[2] - a[2] * b[1]);
    result[1] = -(a[0] * b[2] - a[2] * b[0]);
    result[2] = (a[0] * b[1] - a[1] * b[0]);

    return result;
}

int main()
{
    int n = 3;

    double* a;
    a = dvector(1, n);
    a[0] = 2.0;
    a[1] = 1.0;
    a[2] = 0.1;

    double* b;
    b = dvector(1, n);
    b[0] = 0.0;
    b[1] = 0.3;
    b[2] = -3.6;

    double* c;
    c = dvector(1, n);
    c[0] = 0.2;
    c[1] = 4.6;
    c[2] = -0.1;

    double volume = abs(dot_product(cross_product(a, b), c, n));    

    printf_s("A vector is: [%.2f, % .2f, % .2f]\n",a[0], a[1], a[2]);
    printf_s("B vector is: [%.2f, % .2f, % .2f]\n", b[0], b[1], b[2]);
    printf_s("C vector is: [%.2f, % .2f, % .2f]\n", c[0], c[1], c[2]);    

    free_dvector(a, 1, n);
    free_dvector(b, 1, n);
    free_dvector(c, 1, n);

    printf_s("Volume of a box defined by the vectors A, B, C is: %.3f \n", volume);
    printf_s("End of program. Press enter to exit.");
    cin.get();
}