#include <math.h>
#include <stdio.h>
#include "nrutil.h"
#include "nr.h"
#define TINY 1.0e-20 //A small number.
#define ROTATE(a,i,j,k,l) g=a[i][j];h=a[k][l];a[i][j]=g-s*(h+g*tau);\
	a[k][l]=h+s*(g-h*tau);

float *vector(long nl, long nh);
void free_vector(float *v, long nl, long nh);
void nrerror(const char* error_text);

void jacobi(float **a, int n, float d[], float **v, int *nrot)
/*********************************************************************************************

Computes all eigenvalues and eigenvectors of a real symmetric matrix a[1..n][1..n].On
output, elements of a above the diagonal are destroyed. d[1..n] returns the eigenvalues of a.
v[1..n][1..n] is a matrix whose columns contain, on output, the normalized eigenvectors of
a. nrot returns the number of Jacobi rotations that were required.

*********************************************************************************************/

{
	int j,iq,ip,i;
	float tresh,theta,tau,t,sm,s,h,g,c,*b,*z;
	b=vector(1,n);
	z=vector(1,n);
	for (ip=1;ip<=n;ip++) { //Initialize to the identity matrix.
		for (iq=1;iq<=n;iq++) 
			v[ip][iq]=0.0;
		v[ip][ip]=1.0;
	}
	for (ip=1;ip<=n;ip++) { // Initialize b and d to the diagonal of a. 
		b[ip]=d[ip]=a[ip][ip];
		z[ip]=0.0; /*This vector will accumulate terms of the form 
				   tapq as in equation (11.1.14).*/
	}
	*nrot=0;
	for (i=1;i<=50;i++) {
		sm=0.0;
		for (ip=1;ip<=n-1;ip++) { // Sum off-diagonal elements
			for (iq=ip+1;iq<=n;iq++)
				sm += fabs(a[ip][iq]);
		}
		if (sm == 0.0) {//The normal return, which relies
			//on quadratic convergence to
			//machine underﬂow.
			free_vector(z,1,n);
			free_vector(b,1,n);
			return;
		}
		if (i < 4)
			tresh=0.2*sm/(n*n); //...on the ﬁrst three sweeps.
		else
			tresh=0.0; //...thereafter.
		for (ip=1;ip<=n-1;ip++) {
			for (iq=ip+1;iq<=n;iq++) {
				g=100.0*fabs(a[ip][iq]);
				//After four sweeps, skip the rotation if the off-diagonal element is small.
				if (i > 4 && (float)(fabs(d[ip])+g) == (float)fabs(d[ip])
					&& (float)(fabs(d[iq])+g) == (float)fabs(d[iq]))
					a[ip][iq]=0.0;
				else if (fabs(a[ip][iq]) > tresh) {
					h=d[iq]-d[ip];
					if ((float)(fabs(h)+g) == (float)fabs(h))
						t=(a[ip][iq])/h; //t =1/(2θ)
					else {
						theta=0.5*h/(a[ip][iq]); //Equation (11.1.10).
						t=1.0/(fabs(theta)+sqrt(1.0+theta*theta));
						if (theta < 0.0) t = -t;
					}
					c=1.0/sqrt(1+t*t);
					s=t*c;
					tau=s/(1.0+c);
					h=t*a[ip][iq];
					z[ip] -= h;
					z[iq] += h;
					d[ip] -= h;
					d[iq] += h;
					a[ip][iq]=0.0;
					for (j=1;j<=ip-1;j++) { // Case of rotations 1 ≤ j<p.
						ROTATE(a,j,ip,j,iq)
					}
					for (j=ip+1;j<=iq-1;j++) { // Case of rotations p<j<q.
						ROTATE(a,ip,j,j,iq)
					}
					for (j=iq+1;j<=n;j++) { // Case of rotations q<j ≤ n.
						ROTATE(a,ip,j,iq,j)
					}
					for (j=1;j<=n;j++) {
						ROTATE(v,j,ip,j,iq)
					}
					++(*nrot);
				}
			}
		}
		for (ip=1;ip<=n;ip++) {
			b[ip] += z[ip];
			d[ip]=b[ip]; //Update d with the sum of tapq,
			z[ip]=0.0; //and reinitialize z.
		}
	}
	nrerror("Too many iterations in routine jacobi");
}


void ludcmp(float **a, int n, int *indx, float *d)
/*********************************************************************************************

Given a matrix a[1..n][1..n], this routine replaces it by the LU decomposition of a rowwise
permutation of itself. a and n are input. a is output, arranged as in equation (2.3.14) above;
indx[1..n] is an output vector that records the row permutation eﬀected by the partial
pivoting; d is output as ±1 depending on whether the number of row interchanges was even
or odd, respectively. This routine is used in combination with lubksb to solve linear equations
or invert a matrix.

*********************************************************************************************/
{
	int i,imax,j,k;
	float big,dum,sum,temp;
	float *vv; //vv stores the implicit scaling of each row.
	vv=vector(1,n);
	*d=1.0; //No row interchanges yet.
	for (i=1;i<=n;i++) { //Loop over rows to get the implicit scaling information. 
		big=0.0;
		for (j=1;j<=n;j++)
			if ((temp=fabs(a[i][j])) > big) 
				big=temp;
		if (big == 0.0) nrerror("Singular matrix in routine ludcmp");
		//No nonzero largest element.
		vv[i]=1.0/big; //Save the scaling.
	}
	for (j=1;j<=n;j++) { //This is the loop over columns of Crout’s method.
		for (i=1;i<j;i++) { //This is equation (2.3.12) except for i = j.
			sum=a[i][j];
			for (k=1;k<i;k++) 
				sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
		}
		big=0.0; //Initialize for the search for largest pivot element.
		for (i=j;i<=n;i++) { //This is i = j of equation (2.3.12) and i = j+1 ...N
			//of equation (2.3.13). 
			sum=a[i][j];
			for (k=1;k<j;k++)
				sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
			if ( (dum=vv[i]*fabs(sum)) >= big) {
				//Is the ﬁgure of merit for the pivot better than the best so far?
				big=dum;
				imax=i;
			}
		}
		if (j != imax) { //Do we need to interchange rows?
			for (k=1;k<=n;k++) { //Yes, do so...
				dum=a[imax][k];
				a[imax][k]=a[j][k];
				a[j][k]=dum;
			}
			*d = -(*d); //...and change the parity of d.
			vv[imax]=vv[j]; //Also interchange the scale factor.
		}
		indx[j]=imax;
		if (a[j][j] == 0.0) 
			a[j][j]=TINY;
		//If the pivot element is zero the matrix is singular (at least to the precision of the
		//	algorithm). For some applications on singular matrices, it is desirable to substitute
		//	TINY for zero.
		if (j != n) { //Now, ﬁnally, divide by the pivot element.
			dum=1.0/(a[j][j]);
			for (i=j+1;i<=n;i++) a[i][j] *= dum;
		}
	} //Go back for the next column in the reduction.
	free_vector(vv,1,n);
}



void lubksb(float **a, int n, int *indx, float b[])
/*********************************************************************************************

Solves the set of n linear equations A·X = B.Here a[1..n][1..n] is input, not as the matrix
A but rather as its LU decomposition, determined by the routine ludcmp. indx[1..n] is input
as the permutation vector returned by ludcmp. b[1..n] is input as the right-hand side vector
B, and returns with the solution vector X. a, n,and indx are not modiﬁed by this routine
and can be left in place for successive calls with diﬀerent right-hand sides b. This routine takes
into account the possibility that b will begin with many zero elements, so it is eﬃcient for use
in matrix inversion.

*********************************************************************************************/
{
	int i,ii=0,ip,j;
	float sum;
	for (i=1;i<=n;i++) { //When ii is set to a positive value, it will become the
		//index of the ﬁrst nonvanishing element of b.Wenow
		//do the forward substitution, equation (2.3.6). The
		//	only new wrinkle is to unscramble the permutation
		//	as we go.
		ip=indx[i];
		sum=b[ip];
		b[ip]=b[i];
		if (ii)
			for (j=ii;j<=i-1;j++) sum -= a[i][j]*b[j];
		else if (sum) 
			ii=i; //A nonzero element was encountered, so from now on we
		//will have to do the sums in the loop above. 
		b[i]=sum;
	}
	for (i=n;i>=1;i--) { //Now we do the backsubstitution, equation (2.3.7).
		sum=b[i];
		for (j=i+1;j<=n;j++) sum -= a[i][j]*b[j];
		b[i]=sum/a[i][i]; //Store a component of the solution vector X.
	} //All done!
}