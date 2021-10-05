// Mech534 - HW4: Eigen Value Problem
// Berke Ataseven
// 54326

#include <iostream>
#include "nrutil.h"
#include "nr.h"

using namespace std;

int main()
{
    int n = 2;
	float** A = matrix(1, n, 1, n);
	float** v = matrix(1, n, 1, n);

	// Values of "A" matrix are found from the equations of motions
	A[1][1] = -2;
	A[1][2] = 1;
	A[2][1] = 1;
	A[2][2] = -2;

	cout << "A matrix is:\n";

	for (int r = 1; r <= n; r++) {
		for (int c = 1; c <= n; c++)
		{
			cout << A[r][c] << "\t";
		}
		cout << endl;
	}

	int nrot;
	float* d = vector(1, n);

	jacobi(A, n, d, v, &nrot);

	cout << endl << "The eigen values are: " << endl;
	for (int i = 1; i <= n; i++) cout << d[i] << endl;

	cout << endl << "Eigen vector 1 is: " << endl;
	for (int r = 1; r <= n; r++) cout << v[r][1] << "\n";

	cout << endl << "Eigen vector 2 is: " << endl;
	for (int r = 1; r <= n; r++) cout << v[r][2] << "\n";
		
	free_matrix(A, 1, n, 1, n);
	free_matrix(v, 1, n, 1, n);
	free_vector(d, 1, n);

}