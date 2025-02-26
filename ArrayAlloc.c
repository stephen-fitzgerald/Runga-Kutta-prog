/*------------------------------------------------------------------------
	Stephen Fitzgerald, July 12, 1988

	This file contains a memory allocater for 2 dimensional arrays.
	Get2dMatrix returns a pointer to an array of pointers to doubles.
	This handle can be doubly subcripted just like a 2 dimensional
	array. The members of the array are all set to 0.0.  If there is
	not enough memory to fullfill a request a NULL pointer is returned.

	dispos2dMatrix() frees the memory used by a previously allocated
	array.  It returns (int) 0 if the array was indeed allocated and
	the de-allocation went OK; otherwise it returns -1 . To get rid of
	all alocated matrices call disposAll2dMatrix(), which returns the
	number of matrices that were de-allocated.

	example ;

		double **a, get2dMatrix() ;
		int 	i, j, nrows, ncols ;

		a = get2dMatrix( nrows, ncols ) ;

		if ( a != NULL ) {
			a[i][j] = 45.0 ;

			ect...

			Dispos2dMatrix( a ) ;
		}

	A small advantage to the double indirect pointer scheme is that rows
	of the matrix can be swaped by simply swapping poiters...

	double **a, *temp ;

	|* swap rows i & j *|

	temp = a[i] ;
	a[i] = a[j] ;
	a[j] = temp ;

	NOTE :

		1. You must declare the array as a handle to double as shown, NOT
			as a 2 dimensional array.

		2. If you use the same array twice, be sure to free the memory
			of the first call before the second, or the memory will be
			lost for ever !. i.e.

				a = Get2dMatrix( 3, 10 ) ;

				\* free up memory for re-use *\
				dispos2dMatrix( a ) ;

				a = get2dMatrix( 5, 9 ) ;
		3. There is a limit on the number of arrays that you can have
			allocated at the same time. This limit is set with the
			#define _Max_Arrays at the start of this file.


-------------------------------------------------------------------------*/

/*--------------------------------------------------------------
	the _Max_Arrays # of arrays you can have allocated at once
-----------------------------------------------------------------*/
#define _Max_Arrays 10

#include <stdio.h>
#include <stdlib.h>
#include "ArrayAlloc.h"

#define EMPTY NULL

static int size[_Max_Arrays], firstcall = 1;
static double **a[_Max_Arrays];

double **Get2dMatrix(int nrow, int ncol){
	double **mat;
	int err, i, j, found;

	if (nrow == 0 || ncol == 0)
	{
		return (NULL);
	}

	if (firstcall)
	{
		for (i = 1; i < _Max_Arrays; i++)
		{
			a[i] = (double **)NULL;
			size[i] = 0;
		}
		firstcall = 0;
	}

	// mat = (double **) NewPtr( nrow * sizeof(double *) ) ;
	mat = (double **)malloc(nrow * sizeof(double *));

	if (mat != NULL)
	{
		err = nrow;
		for (i = 0; i < nrow; i++)
		{

			// mat[i] = (double *) NewPtr( ncol * sizeof(double) ) ;
			mat[i] = (double *)malloc(ncol * sizeof(double));

			if (mat[i] == NULL)
			{
				err = i;
				break;
			}
		}

		if (err != nrow)
		{
			for (i = 0; i < err; i++)
			{
				// DisposPtr( mat[i] ) ;
				free(mat[i]);
			}
			free(mat);
			mat = (double **)NULL;
		}
	} /* if mat != NULL */

	if (mat != NULL)
	{
		found = 0;

		for (i = 0; i < _Max_Arrays; i++)
		{
			if (a[i] == EMPTY)
			{
				found = 1;
				break;
			}
		}

		if (found == 1)
		{
			a[i] = mat;
			size[i] = nrow;

			for (i = 0; i < nrow; i++)
			{
				for (j = 0; j < ncol; j++)
				{
					mat[i][j] = 0.0;
				}
			}
		}
		else
		{

			for (i = 0; i < nrow; i++)
			{
				// DisposPtr( mat[i] ) ;
				free(mat[i]);
			}
			// DisposPtr( mat ) ;
			free(mat);
			mat = (double **)NULL;
		}

	} /* if mat != NULL */

	return (mat);
}

int Dispos2dMatrix(double **mat)
{
	int i, j;
	double **b;
	int found;

	if (mat == NULL)
	{
		return (-1);
	}

	found = 0;

	for (i = 0; i < _Max_Arrays; i++)
	{
		if (a[i] == mat)
		{
			found = 1;
			break;
		}
	}

	if (found == 1 && a[i] != NULL)
	{
		b = a[i];

		for (j = 0; j < size[i]; j++)
		{
			// DisposPtr( b[j] ) ;
			free(b[j]);
		}
		// DisposPtr( b ) ;
		free(b);
		a[i] = EMPTY;
		size[i] = 0;
		return (0);

	} /* if found == 1 */

	return (-1);
}

int disposAll2dMatrix()

{
	double **b;
	int i, j, count = 0;

	for (i = 0; i < _Max_Arrays; i++)
	{
		if (a[i] != EMPTY)
		{
			b = a[i];

			for (j = 0; j < size[i]; j++)
			{
				free(b[j]);
			}
			free(b);
			a[i] = EMPTY;
			size[i] = 0;
			count++;
		}
	}
	return (count);
}
