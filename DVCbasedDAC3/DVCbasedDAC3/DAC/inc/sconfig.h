#include <stdlib.h>
#include <stdio.h>
#define SCR_KEY_MAXLEN 32
#define SCR_VAR_MAXLEN 960

#define SCL_PC 0
#define SCL_UNIX 1

#define SCW_APPEND 1
#define SCW_REPLACE 2

int sc_copy(const char *, const char *);
int sc_read(const char *, const char *, void *, int);
void sc_newline(int);
FILE * sc_open(const char *);
void sc_close(FILE *);
int sc_reset(FILE *);
int sc_readline(FILE *, char *, char *);
int sc_write(const char *, const char *, const char *, int);
int sc_read_int(const char * name, const char * pkey, int * pdes);
int sc_read_double(const char * name, const char * pkey, double * pdes);
unsigned long sc_read_long(const char * name, const char * pkey, unsigned long * pdes);