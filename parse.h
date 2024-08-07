/*-------------------------------------------------------------------------
	this is only for version 1.00 and earlier
-------------------------------------------------------------------------*/
extern int ErrorCode ;


/*-------------
#define NoOp -1
#define BINOP 1
#define UNOP  2
#define NUM   3
#define FUNC  4
#define CONST -1
----------------*/

typedef struct nodeRecord {
	int    type ;  
	int	   opratorid ;
	struct nodeRecord *left, *right ;
	double oprand ;
	
} node, *PARSETREE ;

PARSETREE parse( char *expr_p[], int *err, char err_mess[] );
double eval( PARSETREE n, int *err_num );
void disposParseTree ( PARSETREE n );
int setVariable( char *s, double v );

