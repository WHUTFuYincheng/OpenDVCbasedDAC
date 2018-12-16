
/* STATE OF RANDOM NUMBER GENERATOR. */

#define N_tables 5		/* Number of tables of real random numbers */

typedef struct
{ int seed;			/* Seed state derives from */
  int ptr[N_tables];		/* Pointers for tables of real random numbers */
  unsigned short state48[3];	/* State of 'rand48' pseudo-random generator */
} rand_state;
struct drand48_data
  {
    unsigned short int __x[3];	/* Current state.  */
    unsigned short int __old_x[3]; /* Old state.  */
    unsigned short int __c;	/* Additive const. in congruential formula.  */
    unsigned short int __init;	/* Flag for initializing.  */
    unsigned long long int __a;	/* Factor in congruential formula.  */
  };
int nrand48_r (unsigned short int __xsubi[3],
		      struct drand48_data *__restrict __buffer,
		      long int *__restrict __result);

long int nrand48 (unsigned short int __xsubi[3]) ;

/* BASIC PSEUDO-RANDOM GENERATION PROCEDURES. */

void rand_seed (int);		/* Initialize current state structure by seed */

void rand_use_state (rand_state *); /* Start using given state structure */
rand_state *rand_get_state (void);  /* Return pointer to current state */

int rand_word (void);		/* Generate random 31-bit positive integer */


/* GENERATORS FOR VARIOUS DISTRIBUTIONS. */

double rand_uniform (void);	/* Uniform from [0,1) */
double rand_uniopen (void);	/* Uniform from (0,1) */

int rand_int (int);		/* Uniform from 0, 1, ... (n-1) */
int rand_pickd (double *, int);	/* From 0 ... (n-1), with given distribution */
int rand_pickf (float *, int);	/* Same as above, but with floats */

double rand_gaussian (void);	/* Gaussian with mean zero and unit variance */
double rand_logistic (void);	/* Logistic centred at zero with unit width */
double rand_cauchy (void);	/* Cauchy centred at zero with unit width */
double rand_gamma (double);	/* Gamma with given shape parameter */
double rand_exp (void);		/* Exponential with mean one */
double rand_beta (double, double); /* Beta with given parameters */
