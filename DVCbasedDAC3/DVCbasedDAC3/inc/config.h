
#ifndef COMMON_INC_CONFIG_H
#define COMMON_INC_CONFIG_H

#define PI                      3.14159265
#define BLOCK_SIZE              4
#define MB_BLOCK_SIZE           16
#define DC_BITDEPTH             10
// Macros for encoder/decoder
#define WHOLEFLOW               1
#define AC_QSTEP                1
#define RESIDUAL_CODING         1
#define SKIP_MODE               1
#define MODE_DECISION           1
#define INTEGER_DCT             1
#define HARDWARE_FLOW           1
#define HARDWARE_QUANTIZATION   1
#define HARDWARE_CMS            1
#define HARDWARE_OPT            1
#define BIDIRECT_REFINEMENT     0
#define SI_REFINEMENT           1
#define ISADAPTIVE				0
#define HEAPSORT				0
#define CRC						1					//Whether CRC validation is used
#if CRC
#define CRCLENGTH				1584
#define CRCWRONGWEIGHT			-50
#endif // CRC
#define MYNODECOUNT				2048				//Use to set the size of nodecount for decoding
/*==============================The following parameters are adjusted for different video sequences===========*/
#define ISCHANGE				1					//Whether overlap is variable
#define PSOTION					2					//The size of overlap change, and the last X+1 bit is AC algorithm
#define ADAPTIVEDAC				0
#define OVERLAP					0.025				//For different sequence, overlap is different!!!

// Macros for encoder only
#ifdef ENCODER
# define TESTPATTERN            1
# define DEBUG                  0
#endif

// Macros for decoder only
#ifdef DECODER
# define INVERSE_MATRIX         1
#endif

#include "types.h"
#endif // COMMON_INC_CONFIG_H

