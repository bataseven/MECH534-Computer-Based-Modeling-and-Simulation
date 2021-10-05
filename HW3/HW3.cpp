// MECH544 - HW3 LU Decomposition
// Berke Ataseven
// 0054326

#include <iostream>
#include "nrutil.h"
#include "nr.h"

using namespace std;

int main()
{
	float** A = matrix(1, 6, 1, 6);
	memset(A[1], 0, 37 * sizeof(float));

	A[1][1] = 11;
	A[1][2] = -5;
	A[1][6] = -1;
    A[2][1] = 20;
    A[2][2] = -41;
    A[2][3] = 15;
    A[2][5] = 6;
    A[3][2] = 3;
    A[3][3] = -7;
    A[3][4] = 4;
    A[4][3] = 1;
    A[4][4] = -2;
    A[4][5] = 1;
    A[5][2] = 3;
    A[5][4] = 10;
    A[5][5] = -28;
    A[5][6] = 15;
    A[6][1] = 2;
    A[6][5] = 15;
    A[6][6] = -47;

    cout << "A matrix is:\n";

	for (int r = 1; r <= 6; r++) {
		for (int c = 1; c <= 6; c++)
		{
			cout << A[r][c] << "\t";
		}
		cout << endl;
	}
	
    float *b = vector(1, 6);
    memset(b, 0, 7 * sizeof(float));
   
    b[1] = 500;

    int* indx = ivector(1, 6);
    float d;

    ludcmp(A, 6, indx, &d);
    lubksb(A, 6, indx, b);

    cout << endl << "The result is: " << endl;
    for (int i = 1; i <= 6; i++) cout << b[i] << endl;    

    free_matrix(A, 1, 6, 1, 6);
    free_vector(b, 1, 6);
}

