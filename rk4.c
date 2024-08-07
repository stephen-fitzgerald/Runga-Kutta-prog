
#include <stdio.h>
#include <math.h>
#include "constants.h"

static int ERR_FLAG = 0 ;
static char ERR_MESS[255] ;

set_err_flag( int val, char *err_mess ) {
	if ( val ) {
		ERR_FLAG = TRUE ;
		strcpy( ERR_MESS, err_mess ) ;
	}
	else {
		ERR_MESS[0] = '\0' ;
		ERR_FLAG = FALSE ;
	}
}

int error(){
	return( ERR_FLAG ) ;
}

char *error_mess(){
	return( ERR_MESS ) ;
}

void nrerror( char *text ){
	fprintf( stderr, "\n %s \n", text ) ;
	fprintf( stderr, "\n In short, YOU'RE FUCKED !!\n\n\n I QUIT !!" ) ;
	ExitToShell();
}

/*----------------------------------------------------
	Results are stored, at intervals of dxsav. 
----------------------------------------------------*/
double dxsav = 0 ;
double yscal_vals[] = { .001,.1,.01,1,.01,.01,.01,1,.01,1,.01,.0005,.01 } ;

/*-------------------------------------------------------------------------

	Given values for n variables, y[1..n] and their derivatives dydx[1..n]
	known at x, use a forth-order Runge - Kutta method to advance the 
	solution over an interval h and return the incremented variables as
	yout[1..n], which need not be a distinct array from y[].  The user
	supplies the routine derivs( x, y, dydx ) which returns derivatives
	dydx at x.
	
	from : 'Numerical Recipes in C'
-------------------------------------------------------------------------*/

rk4 ( double y[], double dydx[], int nvar, double x, double h, double yout[] ) {
    
	int 	i;
	double 	xh, hh, h6, dym[NVAR+2], dyt[NVAR+2], yt[NVAR+2] ;
	void derivs() ;
	double  Vsource() ;
		
	hh = h * 0.5 ;
	h6 = h / 6.0 ;
	
	xh = x + hh ;
	
	for ( i=1; i<=nvar; i++ ) {
		yt[i] = y[i] + hh * dydx[i] ;
	}
	
	derivs( xh, yt, dyt ) ;
	
	for ( i=1; i<=nvar; i++ ) {
		yt[i] = y[i] + hh * dyt[i] ;
	}
	
	derivs( xh, yt, dym ) ;
	
	for ( i=1; i<=nvar; i++ ) {
		yt[i] = y[i] + h * dym[i] ;
		dym[i] += dyt[i] ;
	}

	derivs( x + h, yt, dyt ) ;
	
	for ( i=1; i<=nvar; i++ ) {
		yout[i] = y[i] + h6 * (dydx[i] + dyt[i] + 2.0 * dym[i] ) ;
	}

}


#define	PGROW	-0.20
#define	PSHRNK	-0.25
#define	FCOR	 0.06666666
#define	SAFETY	 0.9
#define	ERRCON	 6.0e-4



void rkqc( double y[], double dydx[], int n, double *x, double htry, double eps, double *yscal, double *hdid, double *hnext, double hmin ){
    
	int 	i, last_try = 0 ;
	double	xsav, hh, h, temp, errmax ;
	double	dysav[NVAR+2], ysav[NVAR+2], ytemp[NVAR+2] ;
	void	nrerror() ;
	void    derivs() ;
	double  Vsource() ;

	D( printf("Enter rkqc() \n"); )
	D( printf("\t x = %10.4e \n", *x );)
	D( printf("\t htry = %10.4e \n", htry );)
	D( printf("\t eps = %10.4e \n", eps );)

	/*-------------------------
		Save starting values.
	---------------------------*/
	xsav = (*x) ;
	for ( i=1; i<=n; i++ ) {
		ysav[i] = y[i] ;
		dysav[i] = dydx[i] ;
	}
	
	h = htry;
	
	for ( ; ; ) {
	
		/*----------------------
			Take 2 small steps
		------------------------*/
		hh = 0.5 * h ;
		rk4( ysav, dysav, n, xsav, hh, ytemp ) ;
		*x = xsav + hh ;
		derivs( *x, ytemp, dydx ) ;
		rk4( ytemp, dydx, n, *x, hh, y ) ;
		*x = xsav + h ;

		D( printf("\t current step size is %10.4e \n", h );)
		
		if ( *x == xsav ) {
			printf("Step size too small in rkqc()") ;
			printf ( " x = %g\n h= %g \n xsav = %g", *x, h, xsav ) ;
			printf ( " last try = %d \n", last_try) ;
			printf ( " hmin = %g \n", hmin) ;
			printf ( " htry = %g \n", htry) ;
			nrerror("Step size too small in rkqc()") ;
		}

		/*----------------------
			Take 1 large step
		------------------------*/
		rk4( ysav, dysav, n, xsav, h, ytemp ) ;
		
		/*---------------------------
			Put difference in ytemp
			& find max error
		----------------------------*/
		errmax = 0.0 ;
		for ( i=1; i<=n; i++ ) {
			ytemp[i] = y[i] - ytemp[i] ; 
			temp = fabs( ytemp[i]/yscal[i] ) ;
			if ( errmax < temp ) 
				errmax = temp ;
		}
		
		/*-----------------------------
			Compare max error to the
			allowable err. If its OK
			then break out of loop
			and return.
		------------------------------*/
		errmax /= eps ;
		
		if ( errmax <= 1.0 ) {
			*hdid = h ;
			*hnext = ( errmax > ERRCON ?
				SAFETY * h * exp( PGROW * log(errmax) ) : 4.0 * h ) ;			
			break ;
		}
		
		/*---------------------------
			error too large
			try smaller step
		h = SAFETY * h * exp( PSHRNK * log( errmax ) ) ;
		----------------------------*/
		h /= 2.0 ;
		
		/*----------------------------------------------------
			If it was our last attemp at meeting the tolerance
			asked for, then set hnext to the value that would
			be tried next.  The calling routine will know
			the error conditions have not been met if
			hdid > hnext.
		-----------------------------------------------------*/
		if ( last_try ) {
			*hdid = hmin ;
			*hnext = h ;
			break ;
		}
		
		/*----------------------------------------
			If the step grows too small use
			the min sized step and integrate one
			last time before returning.
		------------------------------------------*/
		if ( h < hmin ) {
			h = hmin ;
			last_try = 1 ;
		}

	}
	
	for ( i=1; i<=n; i++ ) 
		y[i] += ytemp[i] * FCOR ;

}


#define TINY	0.50



void	odeint( double ystart[], int nvar, double x1, double x2, double eps, double h1, double hmin, int *nok, int *nbad ){
    
	int 	nstp, i ;
	double 	xsav, x, hnext, hdid, h ;
	double	yscal[NVAR+2], y[NVAR+2], dydx[NVAR+2] ;
	void	nrerror() ;
	void    derivs() ;
	double  Vsource() ;
	extern double BrkTrq, Brk_Trq() ;
	
	
	D( printf("Enter odeint() \n");)
	D( printf("\t x1 = %10.4e \n", x1 );)
	D( printf("\t x2 = %10.4e \n", x2 );)
	D( printf("\t eps = %10.4e \n", eps );)
	D( printf("\t h1 = %10.4e \n", h1 );)
	D( printf("\t hmin = %10.4e \n", hmin );)
	
	y[0] = xsav = x = x1 ;
	(*nok) = (*nbad)  = 0 ;

	/*-------------------------------------------------
		Insure that time step takes us from x1 to x2.
	--------------------------------------------------*/
	h = (x2 > x1) ? fabs(h1) : -fabs( h1) ;
	
	for ( i=1; i<=nvar; i++ )
		yscal[i] = 1.0 ;

	/*--------------------------------------------------
		Set initial values for integrator.
	---------------------------------------------------*/
	for ( i=1; i<=nvar; i++ ) 
		y[i] = ystart[i] ;
	
	/*--------------------------------------------------
		Output initial values 
	---------------------------------------------------*/
	print_data( x, y, Vsource(x) ) ;
		
	for ( ; ; ) {

		derivs( x, y, dydx ) ;

		/*-------------------------------------------------------
			Call controller routine with current state variable
			values, and thier derivatives.
		--------------------------------------------------------*/			
		BrkTrq = Brk_Trq( x, y, dydx ) ;

		/*----------------------------------------------------
			yscal is set to 1 earlier, so we get absolute
			tolerances.
		for ( i=1; i<=nvar; i++ )
			yscal[i] = fabs(y[i]) + fabs(dydx[i]*h) + TINY ;
		-----------------------------------------------------*/

		/*--------------------------------------------
			print the current data if its time
		---------------------------------------------*/
		if ( fabs(x-xsav) >= fabs(dxsav) ) {
			print_data( x, y, Vsource(x) ) ;
			xsav = x ;
		}
		
		/*-------------------------------------------
			Cut down step if it will overstep end,
			and allign it to save times.
		--------------------------------------------*/
		if ( (x+h-(xsav+dxsav)) * (x+h-x1) >= 0.0 ) h = (xsav+dxsav) - x ;
		if ( (x+h-x2) * (x+h-x1) > 0.0 ) h = x2 - x ;
		if ( h < hmin ) h = hmin ;
		
		/*----------------------------------------------
			Integrate for a single time step.
		----------------------------------------------*/		
		rkqc( y, dydx, nvar, &x, h, eps, yscal, &hdid, &hnext, hmin ) ;
						
		if ( hdid == h ) ++(*nok); else ++(*nbad) ;
		
		/*--------------------------------
			Are we done ? 
		---------------------------------*/
		if ( (x-x2)*(x2-x1) >= 0.0 || user_quits( x, hdid ) ) {

			/*------------------------------------------------
				Set ystart to final values. This is used when
				the user doesnt want all the data, just the 
				final results.
			-----------------------------------------------*/
			for ( i=1; i<=nvar; i++ ) 
				ystart[i] = y[i] ;
				
			print_data( x, y, Vsource(x) ) ;
			return ;
		}
		
		/*------------------------------------------------------
			If the step size grows too small then use the min.
			step size provided by the user and press on.
		-------------------------------------------------------*/
		if ( fabs(hnext) <= fabs(hmin) ) {
			hnext = hmin ;
			/*-----------------------------------------------------------------
			printf(" Step size smaller than %g in odeint() at t= %g",hmin,x) ;
			-------------------------------------------------------------*/
		}
		
		h = hnext ;
	}
	nrerror(" Too mant steps in odeint()" ) ;
}

/*-----------------------------------------------------------------------
	This routine displays the time during integration and asks the user
	if he wants to quit. If he does it returns 1.
------------------------------------------------------------------------*/
int	user_quits( double t , double h){
    
	char c[255] ;
	register int ret = 0, i = 0;
	
	if ( Button() ) {
		printf( "\n\n\n\tCurrent time is : %g \n", t ) ;
		printf( "\n\tCurrent step size is : %g \n\n \tDo You Want to QUIT ? : (y/n) ", h ) ;
		c[0] = getchar() ;
		/*
		fscanf(stdin, " %s ", c ) ;
		*/
		printf("\n");
		
		i=0 ;
		while ( c[i] == ' ' || c[i] == '\t' )
			i++ ;
			
		if ( c[i] == 'y' || c[i] == 'Y' )
			ret = 1 ;
	}
	return ( ret ) ;
}

