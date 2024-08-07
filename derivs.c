#include <stdio.h>
#include <math.h>
#include <string.h>
#include"parse.h"
#include "constants.h"
#include "rk4.h"
#include "front_end.h"


extern double  Roll_Diam 	 ;     /* Initial diameter of roll           */
extern double 	Jc 		 ;  /* Inertia of core in kg-m^2 		*/
extern double 	Density 	 ; /* Density of web in kg/m^3 		*/
extern double 	W  		 ; /* Width of the roll in meters 		*/
extern double 	Rc4 		 ; /* Core radius ^4 in m^4 			*/
extern double 	K1 		 ;		/* Spring constant of web sections in N/m */
extern double 	K2 		 ;
extern double 	K3 		 ;
extern double 	K4 		 ;
extern double 	J1 		 ;		/* Inertia of rollers in kg-m^2 	*/
extern double 	J2 		 ;
extern double 	J3 		 ;
extern double 	TH 		 ;		/* Web thickness in meters  		*/
extern double 	B1 		 ;		/* Friction of rollers N/(rad/s) 		*/
extern double 	B2 		 ;		/* 	*/
extern double 	B3 		 ;
extern double 	BK1 		 ;		/* Friction Parallel to Spring	N/(rad/s) 	*/
extern double 	BK2 		 ;		/* Friction Parallel to Spring		*/
extern double 	BK3 		 ;		/* Friction Parallel to Spring		*/
extern double 	BK4 		 ;		/* Friction Parallel to Spring		*/
extern double 	Broll 		 ;		/* Guess at Roll friction ( N )		*/
extern double 	Fd 		 ;
extern double 	rr		 ;
extern double 	l_arm		 ;
extern double 	JDancer		 ;
extern double 	Tset		 ;


/*----------------------------------------------------------------------
 * This is the current velocity source function in string form
 * and its corrisponding parse tree.
 * ----------------------------------------------------------------------*/
char	V_func[255] = "(t*.5)*(t<=10)+(t>10)*5";
static PARSETREE	V_tree = NULL ;

/*----------------------------------------------------------------------
 * The controller parameters and other data.
 * -----------------------------------------------------------------------*/
double 		Td = 0.0 ;
double 		Ti = 10.0e12 ;
double 		Kp = 0.0 ;
static double 	Integral = 0.0 ;
static double 	last_time = 0.0 ;
static double 	last_err = 0.0 ;
double 		BrkTrq = 0.0 ;

/*----------------------------------------------------------
 * Flags to let outside routines know the controlled
 * hardware and the feedback type, and the controller type.
 * -----------------------------------------------------------*/
int DANCER = 0 ;
int Motor = 0 ;
int  cntr_type = PID ;

char *cntr_descr = "P.I.D. Controller" ;
/*-----------------------------------------------------------
 * Strings to print in menus and in the output file.
 * ------------------------------------------------------------*/
char *dan = "Dancer Feedback" ;
char *trans = "Transducer Feedback" ;

static char *mot = "Regenerative Motor" ;
static char *brk = "Brake" ;

char *f_type, *c_type ;



/*------------------------------------------------------------------------
 * This function installs a default velocity source function to save
 * the user some typing. It should only be called once, at the begining
 * of main().
 * -------------------------------------------------------------------------*/

int	instl_dflt_V()

{
    int		e_err = 0, p_err = 0 ;
    char 	*p, p_err_mess[80] ;
    PARSETREE 	tree = NULL ;
    double 	temp = 0.0;
    
    p = V_func ;
    
    /*---------------------------------------------
     * Set output string to describe controller.
     * ----------------------------------------------*/
    f_type = trans ;
    c_type = brk ;
    
    /*---------------------------------------------
     * Parse the function, save the parse tree.
     * ---------------------------------------------*/
    tree = parse( &p, &p_err, p_err_mess ) ;
    
    /*------------------------------------------------------
     * Check the function input for syntax errors.
     * If there are any, call get_vel_func() recursively
     * -------------------------------------------------------*/
    if ( p_err ) {
        printf("%s", p_err_mess ) ;
        printf("Press RETURN to go on...\n\n") ;
        return(0) ;
    }
    else {
        V_tree = tree ;
        tree = NULL ;
    }
    return(1);
}

/*---------------------=--------------------------------
 * returns TRUE if velocity source has been entered.
 * -------------------------------------------------------*/
int	v_is_defined( ) {
    if ( V_tree == NULL )
        return( FALSE ) ;
    else
        return( TRUE ) ;
}

/*----------------------------------------------------------------------
 * This function inputs the velocity source on the web.
 * -----------------------------------------------------------------------*/
int get_vel_func() {
    int		e_err = 0, p_err = 0 ;
    char 		buf[255], *p, p_err_mess[80] ;
    PARSETREE 	tree = NULL ;
    double 	temp = 0.0;
    
    *buf = '\0' ;
    
    
    /*-------------------------------------------------------
     * Show the user the current function and get the new
     * function, in a char string.
     * --------------------------------------------------------*/
    clear_screen() ;
    
    if ( V_tree != NULL ) {
        printf("The Current Velocity Function is,\n\n \t V(t) = %s\n\n", V_func ) ;
        printf("Press RETURN alone to keep current function\n\n") ;
    }
    
    printf("Enter the Velocity Function, \n\n\t V(t) = ") ;
    
    *buf = '\0' ;
    fgets( buf, 255, stdin ) ;
    
    /*------------------------------------------------
     * If the user just hits return then keep the
     * current function and return.
     * -------------------------------------------------*/
    if ( *buf == '\n' ) {
        return(0);
    }
    
    p = buf ;
    
    /*---------------------------------------------
     * Parse the function, save the parse tree.
     * ---------------------------------------------*/
    tree = parse( &p, &p_err, p_err_mess ) ;
    
    /*------------------------------------------------------
     * Check the function input for syntax errors.
     * If there are any, call get_vel_func() recursively
     * -------------------------------------------------------*/
    if ( p_err ) {
        printf("%s", p_err_mess ) ;
        printf("Press RETURN to go on...\n\n") ;
        fgets( buf, 254, stdin ) ;
        get_vel_func() ;
        return(0) ;
    }
    else {
        disposParseTree( V_tree ) ;
        V_tree = tree ;
        tree = NULL ;
        strcpy( V_func, buf ) ;
    }
    return(1);
}


double Vsource( double t ) {
    
    double  v = 0;
    int e_err ;
    
    
    if ( t < 0 ) {
        set_err_flag( TRUE , "neg. time in V(t)" ) ;
    }
    else if ( V_tree ) {
        setVariable( "t", t ) ;
        setVariable( "T", t ) ;
        v = eval( V_tree, &e_err ) ;
        if ( e_err )
            set_err_flag( TRUE , "trouble in V(t)" ) ;
    }
    return v ;
}

/*-----------------------------------------------------------------------
 *
 * This function calculates the inertia of the main roll as a function
 * of the outside radius.  Jc is a global variable representing the
 * inertia of the core. The global variable Rc4 is the radius of the core
 * to the 4th power.
 * -------------------------------------------------------------------------*/

double J( double r ){
    double j ;
    j = Jc + _2pi * Density * W * ( r * r * r * r - Rc4 ) / 4 ;
    return j ;
}

/*-------------------------------------------------------------------------
 * The controller settings,
 *
 * command = Kp ( 1 + Td S + 1/(Td S) ) * err
 *
 * If Motor is non zero the controller will send
 * a signal to a regenerative motor. Otherwise,
 * it assumes the system is being controlled with
 * a brake.
 *
 * double Td = 0.0 ;
 * double Ti = 10.0e12 ;
 * double Kp = 0.0 ;
 * int DANCER = 0 ;
 * char *dan = "Dancer Feedback" ;
 * char *trans = "Transducer Feedback" ;
 * char cntr_descr[40] = "P.I.D. Controller" ;
 * int  cntr_type = PID ;
 * int Motor = 0 ;
 * static char *mot = "Regenerative Motor" ;
 * static char *brk = "Brake" ;
 * char *f_type ;
 * static double Integral = 0.0 ;
 * static double last_time = 0.0 ;
 * static double last_err = 0.0 ;
 * --------------------------------------------------------------------------*/

void reset_pid() {
    Integral = 0 ;
    last_time = 0.0 ;
    last_err = 0.0 ;
}

int change_pid() {
    
    char 	*str, *str2 ;
    int 	ok, done = FALSE ;
    int 	choice = 0 ;
    double 	temp = 0.0 ;
    
    while ( !done ) {
        
        clear_screen() ;
        
        if ( Motor ) {
            str = brk ;
        }
        else {
            str = mot ;
        }
        
        if ( DANCER ) {
            str2 = trans ;
        }
        else {
            str2 = dan ;
        }
        
        printf("\n\t\tP.I.D. MENU\n") ;
        printf("\nYour Options Are :\n") ;
        printf("\n\t0.  Return To Main Menu ") ;
        printf("\n\t1.  Change Kp, ( = %g )", Kp ) ;
        printf("\n\t2.  Change Td, ( = %g )", Td ) ;
        printf("\n\t3.  Change Ti, ( = %g )", Ti ) ;
        printf("\n\t4.  Use %s Control ", str ) ;
        printf("\n\t5.  Use %s  ", str2 ) ;
        printf("\n\n Enter Your Choice , Then RETURN : ") ;
        
        choice = get_choice( 0, 5, 0 ) ;
        
        switch ( choice ) {
            case 0 :
                done = TRUE ;
                break ;
            case 1 :
                ok = FALSE ;
                while ( !ok ) {
                    ok = TRUE ;
                    printf("\n\nKp = %g , Enter New Value, or RETURN to cancel : ", Kp) ;
                    temp = get_double( Kp ) ;
                    if ( temp < 0.0 ) {
                        printf("\n\n Value Can Not Be Negative");
                        ok = FALSE ;
                    }
                    else {
                        Kp = temp ;
                    }
                }
                break ;
            case 2 :
                printf("\n\nTd = %g , Enter New Value, or RETURN to cancel : ", Td) ;
                Td = get_double( Td ) ;
                break ;
            case 3 :
                ok = FALSE ;
                while ( !ok ) {
                    printf("\n\nTi = %g , Enter New Value, or RETURN to cancel : ", Ti) ;
                    
                    temp = get_double( Ti ) ;
                    
                    if ( temp == 0.0 ) {
                        printf("\n\n Ti Can Not Be Zero");
                        ok = FALSE ;
                    }
                    else {
                        ok = TRUE ;
                    }
                }
                Ti = temp ;
                break ;
            case 4 :
                if ( Motor ) {
                    Motor = FALSE ;
                    c_type = brk ;
                }
                else {
                    Motor = TRUE ;
                    c_type = mot ;
                }
                break ;
            case 5 :
                if ( DANCER ) {
                    DANCER = FALSE ;
                    f_type = trans ;
                }
                else {
                    DANCER = TRUE ;
                    f_type = dan ;
                }
                break ;
        }
    }
    return(0) ;
}

/*-----------------------------------------------------------------------
 * This is the controller routine. It is passed the current time in t,
 * the value of each of the 11 variables in the array y[] and thier
 * derivatives in dydt.  It returns the braking tourque that will be
 * applied to the roll.  The global Tset is the tension setting
 * that we are trying to achieve.
 *
 * This function will change often during the project. Be sure to
 * include code that handles negative tensions ect.  If you want to save
 * information, such as the integral of the error terms, use a double
 * variable that has been declared as static.
 *
 * -------------------------------------------------------------------------*/
static double olderr[20] ;
int NF = 1 ;

double Brk_Trq( double t, double y[], double dydt[] ) {
    
    double err, v, dv1 ;
    double t_step 	= t - last_time ;
    double derr_dt = 0.0 ;
    double command 	= 0.0 ;
    static int n = 0 ;
    int i ;
    
    
    if ( DANCER ) {
        err = y[6] ;
    }
    else {
        v = Vsource(t) ;
        dv1 = ( v - y[2]*rr) ;
        err = Tset - (y[1] + BK1*dv1)  ;
    }
    
    
    /*-------------------------------------------------------------------
     * Average last this error with last NF-1 errors.
     * -------------------------------------------------------------------*/
    if ( n >= NF ) {
        
        n = NF ;
        for ( i=1; i<NF ; i++) {
            olderr[i-1] = olderr[i] ;
        }
        olderr[NF-1] = err ;
        
    }
    else {
        olderr[n++] = err ;
    }
    
    err = 0.0 ;
    
    for ( i=0; i<n; i++ ) {
        err += olderr[i] ;
    }
    err /= n ;
    
    
    if ( t_step > 0.0 )
        derr_dt = (err - last_err) / t_step ;
    
    if ( Ti == 0.0 ) {
        set_err_flag( TRUE, "\nCan't have Ti = 0.0" ) ;
        return( 0.0 ) ;
    }
    
    command = Kp * ( err + Td * derr_dt + Integral/Ti ) ;
    
    last_time = t ;
    last_err = err ;
    Integral += t_step * err ;
    
    return( command );
    
}

/*-----------------------------------------------------------------------
 *
 * This function returns the derivatives of each variable at a given
 * time t. It is also given the values of the variables at t.
 *
 * -----------------------------------------------------------------------*/

void derivs( double t, double y[], double dydt[] ) {
    
    double temp = 0.0, tension = 0.0, bt = 0.0;
    double v = 0.0 ;
    double dv1, dv2, dv3, dv4, f1, f2, f3, f4 ;
    int err = 0 ;
    char err_mess[255] ;
    
    /*--------------------------------
     * The variables are :
     *
     * y[0] = current time
     * y[1] = F in spring 1
     * y[2] = w of roller 1
     * y[3] = F in spring 2
     * y[4] = w of roller 2
     * y[5] = w of dancer arm
     * y[6] = angle of dancer arm
     * y[7] = F in spring 3
     * y[8] = w of roller 3
     * y[9] = F in spring 4
     * y[10] = w of roll
     * y[11] = radius of roll
     * y[12] = Brake Torque, not yet
     * -----------------------------------*/
    err_mess[0] = '\0' ;
    
    y[0] = t ;
    
    v = Vsource(t) ;
    
    if ( error() ) {
        sprintf( err_mess, "can't evaluate V(t) at t = %g seconds\n\n", t ) ;
        set_err_flag( TRUE, err_mess ) ;
        return ;
    }
    
    dv1 = ( v - y[2]*rr) ;
    dv2 = (y[2] - y[4])*rr - y[5]*l_arm ;
    dv3 = (y[4] - y[8])*rr - y[5]*l_arm ;
    dv4 = (y[8]*rr - y[10]*y[11] ) ;
    
    
    f1 = y[1] ;
    f1 += BK1*dv1 ;
    f2 = y[3] ;
    f2 += BK2*dv2 ;
    f3 = y[7] ;
    f3 += BK3*dv3 ;
    f4 = y[9] ;
    f4 += BK4*dv1 ;
    
    if ( f1 < 0.0 ) f1 = 0.0 ;
    if ( f2 < 0.0 ) f2 = 0.0 ;
    if ( f3 < 0.0 ) f3 = 0.0 ;
    if ( f4 < 0.0 ) f4 = 0.0 ;
    
    dydt[1] = K1 * dv1 ;
    dydt[2] = ((f1 - f2) * rr - B1 * y[2] ) / J1 ;
    dydt[3] = K2 * dv2 ;
    dydt[4] = ((f2 - f3) * rr - B2 * y[4] ) / J2 ;
    
    if ( DANCER )
        dydt[5] = ((f3 + f2 - Fd) * l_arm ) / JDancer ;
    else
        dydt[5] = 0 ;
    
    dydt[6] = y[5] ;
    dydt[7] = K3 * dv3 ;
    dydt[8] = ((f3 - f4) * rr - B3 * y[8] ) / J3 ;
    dydt[9] = K4 * dv4 ;
    
    temp = f4 * y[11] ;
    
    /*----------------------------------
     * Friction works against motion
     * -----------------------------------*/
    if ( y[10] > 0.02 ) {
        temp -= Broll ;
    }
    else if ( y[10] < 0.02 ) {
        temp += Broll ;
    }
    else {
        temp -= Broll * y[10]/0.02 ;
    }
    
    bt = BrkTrq ;
    
    if ( Motor ) {
        temp -= bt ;
    }
    else {
        /*--------------------------------------------------
         * Brakes only work against motion.
         * --------------------------------------------------*/
        
        if ( (y[10] * bt ) > 0.0 )
            temp -= bt ;
    }
    
    
    dydt[10] = temp  / J( y[11] ) ;
    
    dydt[11] = - y[10] * TH / _2pi ;
    
}

