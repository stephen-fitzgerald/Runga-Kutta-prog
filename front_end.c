#include <stdio.h>
#include <math.h>
#include "constants.h"
#include "rk4.h"
#include "derivs.h"

void change_param();
void instl_dflt_V();
void change_param() ;
void change_pid() ;
void get_vel_func() ;
void run_sim() ;
int main_menu();
int get_choice( int lo, int hi, int dflt );
void calc_all_params() ;
void integrate( double t, int n, double step1, double hmin, double eps );
void print_header();

struct	PARAMETER	{
    char	*name ;
    double	value ;
    char	*units ;
} ;

/*---------------------------------------------------------------------------
 * The params array is the interface to the user.  The values used in the
 * differential equations are derived from these.
 * -----------------------------------------------------------------------------*/
#define NUM_PARAMS 	15
struct PARAMETER params[] = {
    { "Roll Diameter"	, 1.5		, "m" 		} ,
    { "Core Diameter"	, 0.1		, "m" 		} ,
    { "Core Mass    "	, 10.0		, "kg" 		} ,
    { "Roll Friction"	, 30.0		, "N-m" 	} ,
    { "Web Density  "	, 700.0		, "kg/m^3" 	} ,
    { "Web Width    "	, 1.0		, "m" 		} ,
    { "B of Web     "	, 1000.0	, "N/(m/s)"     } ,
    { "Web Thickness"	, 0.0008	, "m" 		} ,
    { "K of Web     "	, 3000.0	, "N/m" 	} ,
    { "roller Diam  "	, 0.20		, "m" 		} ,
    { "roller Mass  "	, 15.0		, "kg" 		} ,
    { "roller fric. "	, 0.002         , "N-m/(rad/s)" } ,
    { "Dancer arm M"	, 10.0		, "kg" 		} ,
    { "L of Dancer Arm"	, 1.00		, "m" 		} ,
    { "Ten. Setting "	, 350.0		, "N"           }
} ;


double  Roll_Diam 	= 0.0 ;     /* Initial diameter of roll           */
double 	Jc 		= 0.0 ;		/* Inertia of core in kg-m^2 		*/
double 	Density 	= 0.0 ;		/* Density of web in kg/m^3 		*/
double 	W  		= 0.0 ;		/* Width of the roll in meters 		*/
double 	Rc4 		= 0.0 ; 	/* Core radius ^4 in m^4 			*/
double 	K1 		= 0.0 ;		/* Spring constant of web sections in N/m */
double 	K2 		= 0.0 ;
double 	K3 		= 0.0 ;
double 	K4 		= 0.0 ;
double 	J1 		= 0.0 ;		/* Inertia of rollers in kg-m^2 	*/
double 	J2 		= 0.0 ;
double 	J3 		= 0.0 ;
double 	TH 		= 0.0 ;		/* Web thickness in meters  		*/
double 	B1 		= 0.0 ;		/* Friction of rollers N/(rad/s) 		*/
double 	B2 		= 0.0 ;		/* 	*/
double 	B3 		= 0.0 ;
double 	BK1 		= 0.0 ;		/* Friction Parallel to Spring	N/(rad/s) 	*/
double 	BK2 		= 0.0 ;		/* Friction Parallel to Spring		*/
double 	BK3 		= 0.0 ;		/* Friction Parallel to Spring		*/
double 	BK4 		= 0.0 ;		/* Friction Parallel to Spring		*/
double 	Broll 		= 0.0 ;		/* Guess at Roll friction ( N )		*/
double 	Fd 		= 0.0 ;
double 	rr 		= 0.0 ;
double 	l_arm 		= 0.0 ;
double 	JDancer 	= 0.0 ;
double 	Tset 		= 0.0 ;

/*-------------------------------
 * string for file title.
 * --------------------------------*/
char  title_str[255] ;

int main()

{
    int choice = 0 ;
    
    /*--------------------------------------
     * Install default v(t) to save the
     * user some typing.
     * ---------------------------------------*/
    instl_dflt_V() ;
    
    while ( (choice = main_menu() )  ) {
        switch ( choice ) {
            case 1 :
                change_param() ;
                break ;
            case 2 :
                change_pid() ;
                break ;
            case 3 :
                get_vel_func() ;
                break ;
            case 4 :
                run_sim() ;
                break ;
        }
    }
}

/*---------------------------------------------------------------------------
 * clear_screen() clears the screen
 * ---------------------------------------------------------------------------*/

void	clear_screen()

{
    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n" ) ;
}


/*------------------------------------------------------------------------
 * This routine gets an integer between hi & lo from the keyboard.
 * If the user just hits 'return' then the default value is returned.
 * ------------------------------------------------------------------------*/
int get_choice( int lo, int hi, int dflt ) {
    char			buf[255];
    register char	*p ;
    register int	hhi, llo ;
    int				ret ;
    
    if ( lo > hi )
    {	llo = hi ;
        hhi = lo ;	}
    else {	llo = lo ;
    hhi = hi ;	}
    
    *buf = '\0' ;
    
    ret = llo - 1 ;
    
    while ( ret < llo || ret > hhi ) {
        
        p = buf ;
        
        fgets( p, 254, stdin ) ;
        
        /*----------------------------------------
         * Skip white space.
         * ----------------------------------------*/
        while ( *p == ' ' || *p == '\t' )
            p++ ;
        
        /*----------------------------------------
         * If RETURN then return default value.
         * -----------------------------------------*/
        if ( *p == '\n' ) {
            ret = dflt ;
            break ;
        }
        
        if ( sscanf( p, "%d", &ret ) != 1 ) ret = llo - 1 ;
    }
    
    return( ret ) ;
}


/*------------------------------------------------------------------------
 * This routine gets a double floating point number from the keyboard.
 * The number is assigned to x and a 1 is returned to signal OK. If the
 * user types a blank line the default value is returned.
 *
 * This is used when the user is promted for new parameter values, by
 * hitting return he can signal that he wants to keep the original
 * value after all.
 * ------------------------------------------------------------------------*/

double get_double( double dflt ) {
    char	buf[255], *p ;
    double	ret = 0 ;
    
    *buf = '\0' ;
    p = buf ;
    
    while ( *buf == '\0' ) {
        fgets( buf, 254, stdin ) ;
        
        while ( *p == ' ' || *p == '\t' )
            p++ ;
        
        if ( *p == '\n' ) {
            ret = dflt ;
            break ;
        }
        
        if ( sscanf( p, "%lf", &ret ) != 1 )
            *buf = '\0' ;
    }
    
    return( ret );
}

/*---------------------------------------------------------------------------
 * This function returns a menu choice from the main menu.
 *
 * 0. Quit Program
 * 1. Change a Parameter
 * 2. Change PID coefficients
 * 3. Enter a Velocity Function
 * 4. Run Simulation
 * ----------------------------------------------------------------------------*/

static char *m1 = "MAIN MENU" ;
static char *m2 = "Press the number of your choice," ;
static char *m3 = "0. Quit Program" ;
static char *m4 = "1. Change a Parameter" ;
static char *m5 = "2. Change PID coefficients " ;
static char *m6 = "3. Enter a Velocity Function" ;
static char *m7 = "4. Run Simulation" ;
static char *m8 = "Enter Your Choice, Then RETURN :" ;

int	main_menu()

{
    int  c = 0 ;
    
    clear_screen() ;
    
    printf("\t\t\t %s \n\n\t\t %s \n\n\t\t %s \n\t\t %s", m1, m2, m3, m4 );
    printf("\n\t\t %s \n\t\t %s \n\t\t %s \n\n\t\t %s", m5, m6, m7, m8 ) ;
    
    c = get_choice( 0, 4, 5 ) ;
    return( c ) ;
    
}
/*----------------------------------------------------------------------------
 * This function allows the user to change the physical parameters of
 * the model.
 * ----------------------------------------------------------------------------*/
void change_param() {
    int i, c ;
    double temp = 0 ;
    
    for(;;) {
        clear_screen() ;
        
        printf("\t %d \t FOR MAIN MENU \n", 0 ) ;
        
        for ( i=0; i<NUM_PARAMS; i++ ) {
            printf("\t %d ", i+1 ) ;
            if ( i < 9 )
                printf("\t") ;
            printf("\t %s \t %4.3e \t %s \n", params[i].name, params[i].value, params[i].units ) ;
        }
        printf("\t Enter # of Parameter, then RETURN \n\n") ;
        printf("\t Your Choice ?  ") ;
        
        c = get_choice( 0, NUM_PARAMS, 0 ) ;
        
        if ( c > 0 ) {
            c-- ;
            printf(" %s = %g Enter New Value (in %s ), Or RETURN To Cancel :", params[c].name , params[c].value,  params[c].units ) ;
            params[c].value = get_double( params[c].value )  ;
        }
        else
            break ;
    }
    
}

/*------------------------------------------------------------------------
 * This function actually sets up the model and calls the integrator.
 * -------------------------------------------------------------------------*/
FILE	*out_file ;

void run_sim() {
    
    double	temp = 0.0, time = 0.0, min_step = 0.0, eps = 0.0, step1 = 0.0 ;
    int		num_pts = 0, v_is_defined() ;
    FILE	*get_out_file() ;
    void 	reset_pid() ;
    
    clear_screen() ;
    
    reset_pid() ;
    
    if ( !v_is_defined() ) {
        get_vel_func() ;
    }
    
    calc_all_params() ;
    
    while ( time <= 0.0 ) {
        printf("\n\n\t Enter Length of Simulation in Seconds  ( >= 0 ) [10] :" ) ;
        time = get_double( 10.0 ) ;
    }
    
    while ( step1 <= 0.0 || step1 > 0.05 ) {
        printf("\n\n\t Enter first time step in Seconds  ( 0 - 0.05 ) [0.0001] :" ) ;
        step1 = get_double( 0.0001 ) ;
        if ( step1 > 0.05 )
            printf("\n\t  Initial time step is too large \n") ;
        if ( step1 <= 0.0 )
            printf("\n\t  Initial time step is too small \n") ;
    }
    
    while ( min_step <= 0.0 || min_step > 0.05 ) {
        printf("\n\n\t Enter min. time step in Seconds  ( 0 - 0.05 ) [1e-6] :" ) ;
        min_step = get_double( 0.000001 ) ;
        if ( min_step > 0.05 )
            printf("\n\t  Minimum time step is too large \n") ;
        if ( min_step <= 0.0 )
            printf("\n\t  Minimum time step is too small \n") ;
    }
    
    while ( eps <= 0.0 ) {
        printf("\n\n\t Enter epsilon  ( > 0 ) [0.01] :" ) ;
        eps = get_double( 0.01 ) ;
        if ( eps <= 0.0 )
            printf("\n\t  epsilon can not be negative \n") ;
    }
    
    
    while ( num_pts < 5 ) {
        printf("\n\n\t Enter # of Data Pts You Want(  > 5 ) [100]  :") ;
        num_pts = get_choice( 5, 5000, 100 ) ;
    }
    
    out_file = get_out_file() ;
    
    fprintf( stderr, "\nWorking." ) ;
    
    integrate( time, num_pts, step1, min_step, eps ) ;
    
    if ( out_file != stdout ) {
        fclose( out_file ) ;
    }
    
}

FILE	*get_out_file() {
    char fn[255], *fp ;
    FILE *ret ;
    
    fp = fn ;
    
    printf("\n\t Enter Name of Output File, OR RETURN for screen output\n");
    printf("\n\tfilename : ");
    
    fgets( fn, 254, stdin ) ;
    
    while ( *fp=='\t' || *fp==' ' )
        fp++ ;
    
    if ( *fp != '\0' && *fp != '\n' ) {
        if ( (ret = fopen( fp, "w") ) <= (FILE *) NULL) {
            //SysBeep( 5L );
            fprintf( stderr, "Can not open file : %s \n\n", fp ) ;
            ret = stdout ;
        }
    }
    else {
        ret = stdout ;
    }
    
    if ( ret != stdout ) {
        printf("\n Enter a header line for the output file : " ) ;
        fgets( title_str , 254, stdin ) ;
    }
    
    return( ret ) ;
}


/*------------------------------------------------------------------------
 * This function calculates the parameters that the integrator needs
 * from the data that the user has input.
 * ------------------------------------------------------------------------*/

void calc_all_params() {
    int i ;
    
    Roll_Diam 	= params[0].value ;
    Jc 			= params[1].value * params[1].value * params[2].value / 4.0 ;
    Density 	= params[4].value ;
    W  			= params[5].value ;	/* Width of the roll in meters 		*/
    Rc4 		= params[1].value/2.0 ; 	/* Core radius ^4 in m^4 */
    for ( i=1; i<=3; i++ )
        Rc4 *= Rc4 ;
    
    K1 = K2 = K3 = K4 = params[8].value ;
    J1 = J2 = J3 = params[9].value * params[9].value * params[10].value / 4 ;
    TH 	= params[7].value ;			/* Web thickness in meters  */
    B1 = B2 = B3 = params[11].value ;
    BK1 = BK2 = BK3 = BK4 = params[6].value ; /* Friction Parallel to Spring	N/(rad/s) 	*/
    Broll = params[3].value ;
    Tset 	= params[14].value ;
    Fd 	= params[14].value * 2.0 ;
    rr = params[9].value / 2.0 ;
    l_arm = params[13].value ;
    JDancer = params[13].value * params[13].value *( params[12].value/3 + params[10].value );
    
}
/*------------------------------------------------------------------------
 * This is a driver for odeint. It sets the constants and the initial
 * conditions and then calls odeint to integrate the equations over
 * a given time interval .
 * -------------------------------------------------------------------------*/


void integrate( double t, int n, double step1, double hmin, double eps ) {
    
    int i, j, nbad, nok ;
    double x1, x2, ystart[NVAR+1], h1, dtsav;
    extern double  dxsav ;
    extern char	  V_func[] ;
    
    extern int kount, kmax, DANCER ;
    
    x1 = 0.0 ;  /* start time */
    x2 = t ; /* end time   */
    
    /*-----------------------------------------------
     * Set up the values the integrator will use :
     *
     * eps : 	err_max = eps*(abs(y)+abs(h*dydx))
     * see 'Numerical Recipes in C' for a
     * discussion on errors. p.576
     *
     * h1	: 	suggested first step size. In our
     * particular problem the step size
     * must start small (.00001 ), but it
     * can grow ( automatically ).
     *
     * hmin :	This is the smallest step size
     * allowed this helps detect when
     * a solution is not working.
     *
     * kmax :	the max # of steps to store
     *
     * dxsav : the min step size between stored
     * values.
     *
     * --------------------------------------------------*/
    
    dtsav = dxsav = t/((double) n) ;
    
    print_header() ;
    
    /*--------------------------------
     * Set initial values
     * --------------------------------*/
    for ( i=1; i<= NVAR; i++ ) {
        ystart[i] = 0 ;
    }
    
    ystart[11] = Roll_Diam/2.0 ;
    
    /*------------------------------------
     * Call the integrator.
     * -------------------------------------*/
    odeint( ystart, 11, x1, x2, eps, step1, hmin, &nok, &nbad ) ;
    
    
}

void print_header() {
    
    int i;
    extern int DANCER, cntr_type ;
    extern double Kp, Ti, Td ;
    extern char *cntr_descr, *f_type, *c_type ;
    
    fprintf( out_file, "%s\n", title_str) ;
    
    for ( i=0; i<NUM_PARAMS; i++ ) {
        fprintf( out_file, "%s \t %lf \t %s \n", params[i].name, params[i].value, params[i].units ) ;
    }
    
    fprintf( out_file, "\n V(t) = %s \n", V_func ) ;
    
    fprintf( out_file, "\n\n CONTROLLER DATA : \n"  ) ;
    
    if ( (cntr_type = PID) ) {
        fprintf( out_file, " Controller type :\t %s\n" , cntr_descr ) ;
        fprintf( out_file, " Controller Gain : \t%g\n" , Kp ) ;
        fprintf( out_file, " Td = \t%g\n", Td  ) ;
        fprintf( out_file, " Ti = \t%g\n\n", Ti  ) ;
    }
    
    fprintf( out_file, " Feedback Type %s \n", f_type ) ;
    fprintf( out_file, " Controlling a  %s \n\n", c_type ) ;
    
    fprintf( out_file, "\n\n %6s \t %6s \t %6s \t %6s \t %6s \t %6s \t %6s \t %6s \t %6s \t %6s \t %6s \t %9s \t",
            "time", "vel_source", "roll rot. spd.", "radius", "F_k_4", "w_3", "F_k_3", "w_2", "F_k_2", "w_1", "F_k_1", "Tension") ;
    
    if ( DANCER ) {
        fprintf( out_file, "%6s \t", "DANCER ANGLE" ) ;
    }
    
    fprintf( out_file, "\n" ) ;
}

void print_data( double x, double y[], double v ) {
    
    extern int DANCER ;
    double ten = 0 ;
    
    ten = ( y[2] - y[4] ) * rr * BK1 + y[1] ;
    
    fprintf( out_file, "%10.4f \t %10.4e \t %10.4e \t %10.4e \t %10.4e \t %10.4e \t %10.4e \t %10.4e \t %10.4e \t %10.4e \t %10.4e \t %10.4e \t ",
            x, v, y[10], y[11] , y[9], y[8] , y[7], y[4] , y[3], y[2], y[1], ten) ;
    
    if ( DANCER ) {
        fprintf( out_file, " %10.4e \t", y[6]*57.2958   ) ;
    }
    
    fprintf( out_file, "\n" ) ;
    
    if ( out_file != stdout )
        printf(".");
        }